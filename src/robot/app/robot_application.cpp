#include "robot/app/robot_application.hpp"

#include <algorithm>
#include <cmath>

#include "robot/core/geometry.hpp"
#include "robot/platform/transform_tree.hpp"

namespace lost_signal::robot::app {

namespace {
platform::ChassisLimits autonomy_limits_from_description(
 const platform::RobotDescription& desc) {
 platform::ChassisLimits limits = desc.limits;
 if (desc.chassis_kind == "ackermann" &&
 desc.plant.wheel_base_m > 0.0 &&
 std::isfinite(desc.plant.wheel_base_m) &&
 std::isfinite(desc.limits.max_steer_rad)) {
 const double tan_steer = std::tan(desc.limits.max_steer_rad);
 if (std::isfinite(tan_steer)) {
 limits.max_curvature_inv_m =
 std::abs(tan_steer) / desc.plant.wheel_base_m;
 }
 }
 return limits;
}

class NullMapProvider : public iface::IMapProvider {
 public:
  iface::MapSnapshot active_snapshot() const override { return snapshot_; }
  bool select_map(const std::string& map_id) override {
    if (map_id.empty()) return false;
    snapshot_.map_id = map_id;
    snapshot_.revision += 1;
    return true;
  }

 private:
  iface::MapSnapshot snapshot_{.map_id = "void", .revision = 0};
};
}  // namespace

namespace {

autonomy::LidarPerceptionConfig perception_config_from(
    const platform::RobotDescription& desc) {
  (void)desc;  // extrinsics live in the TF tree, not here; thresholds only
  autonomy::LidarPerceptionConfig config;
  return config;
}

}  // namespace

RobotApplication::RobotApplication(std::unique_ptr<iface::IMapProvider> maps,
                                   platform::RobotDescription desc)
    : maps_(maps ? std::move(maps) : std::make_unique<NullMapProvider>()),
      desc_(std::move(desc)),
      chassis_(platform::make_chassis(desc_)),
 coordinator_(autonomy_limits_from_description(desc_), 100'000'000,
 perception_config_from(desc_), {}, desc_.plant.wheel_base_m) {
  // Declared plant loadout fans into the autonomy envelopes once, with
  // the machine file (validator rollover support/COM + survey ride).
  coordinator_.set_plant_geometry(desc_.plant);
  channel_.activate();
  // Bringup order: autonomy state first, executive policy last.
  bringup_wired_ =
      bringup_.add_module(&coordinator_) && bringup_.add_module(&executive_);
}

core::Status RobotApplication::configure() {
  if (!bringup_wired_) return core::Status::Fail("bringup not wired");
  if (!chassis_) return core::Status::Fail("unknown chassis kind");
  // Static TF from the description (mount extrinsics); the coordinator
  // refreshes the dynamic map->odom / odom->base edges every update.
  coordinator_.set_static_frames(
      platform::static_transform_tree(desc_));
  return bringup_.configure_all();
}

core::Status RobotApplication::activate() {
  if (!bringup_wired_) return core::Status::Fail("bringup not wired");
  return bringup_.activate_all();
}

void RobotApplication::shutdown() { bringup_.cleanup_all(); }

iface::GoalHandle RobotApplication::request_goal(
    const iface::TaskRequest& req) {
  if (!req.place_id.empty()) {
    // Semantic resolution through the ROBOT map provider: the place id
    // becomes an estimated-frame goal for the coordinator. Unknown ids
    // fail closed with an invalid handle; raw coordinates never cross
    // this boundary.
    const iface::MapSnapshot snap = maps_->active_snapshot();
    const iface::Place* found = nullptr;
    for (const auto& p : snap.places) {
      if (p.place_id == req.place_id) {
        found = &p;
        break;
      }
    }
    if (found == nullptr) {
      return iface::GoalHandle{};  // unknown place: nothing armed
    }
    // Authorization first: the executive accepts before anything plans.
    // A rejected request (inactive executive, empty request id) arms no
    // route — the coordinator never drives an unaccepted goal.
    const iface::GoalHandle handle =
        executive_.request_goal(req.place_id, req.request_id);
    if (!handle.valid()) {
      return handle;
    }
    autonomy::RoutePoint goal{found->x_m, found->y_m, found->yaw_rad};
    coordinator_.set_goal(goal);
    return handle;
  }
 // Empty place is a hold: acknowledge it and withdraw the route so the
 // coordinator cannot keep driving the previous goal.
 const iface::GoalHandle hold =
 executive_.request_goal(req.place_id, req.request_id);
 if (hold.valid()) coordinator_.clear_goal();
 return hold;
}

core::Status RobotApplication::request_map(const std::string& map_id) {
  if (!maps_->select_map(map_id)) return core::Status::Fail("unknown map");
  return core::Status::Ok();
}

bool RobotApplication::register_provider(const iface::EndpointDescriptor& desc) {
  return registry_.register_endpoint(desc);
}

bool RobotApplication::publish_observation(const iface::ObservationFrame& obs) {
  if (!registry_.owns(obs.source, iface::EndpointRole::kLidar) &&
      !registry_.owns(obs.source, iface::EndpointRole::kImu) &&
      !registry_.owns(obs.source, iface::EndpointRole::kEncoder)) {
    return false;  // unregistered source: never enters the decision path
  }
  return channel_.publish_observation(obs);
}

iface::MotionCommand RobotApplication::take_command(core::MonotonicNs now_ns) {
  return channel_.take_command(now_ns);
}

iface::ActuatorCommand RobotApplication::take_actuator(
    core::MonotonicNs now_ns) {
  return channel_.take_actuator(now_ns);
}

autonomy::diagnostics::DecisionEvidence RobotApplication::last_evidence()
    const {
  return coordinator_.last_evidence();
}

iface::MotionCommand RobotApplication::tick(core::MonotonicNs now_ns) {
  if (bringup_.state() != lifecycle::ModuleState::kActive || !chassis_) {
    return iface::MotionCommand::SafeStop(now_ns);  // not brought up: hold
  }
  for (const auto& obs : channel_.drain_observations()) {
    coordinator_.ingest_observation(obs);
 }
 iface::MotionCommand body = coordinator_.update(
      maps_->active_snapshot(), channel_.health(), now_ns);
 // Autonomy owns pose and settling policy; the application only sequences
 // its terminal result into the executive. The policy sees ROBOT estimates
 // and map goals only, never simulator truth.
 if (coordinator_.goal_arrived()) {
 executive_.note_arrival(true);
 coordinator_.clear_goal();
 body = iface::MotionCommand::SafeStop(now_ns);
 }
  if (!iface::valid_motion_command(body, now_ns)) {
    body = iface::MotionCommand::SafeStop(now_ns);
  }
  body = chassis_->clamp_command(body, now_ns);
  // Closed-loop translation with the latest joint feedback. The chassis
  // returns safe zero without valid feedback; loops never close on stale
  // joints.
  const iface::ObservationFrame& last = coordinator_.last_observation();
  iface::MotionFeedback fb;
  fb.drive_count = last.drive_count;
  fb.steer_count = last.steer_count;
  fb.drive_radps = last.drive_radps;
  fb.steer_rad = last.steer_rad;
  fb.steer_vel_radps = last.steer_vel_radps;
  fb.source = "app.joints";
  fb.frame = core::Frame::kBaseLink;
  fb.stamp_ns = last.stamp_ns;
  iface::ActuatorCommand act = chassis_->to_actuator(body, fb, now_ns);
  channel_.publish_command(body);
  channel_.publish_actuator(act);
  // Passive braking instrumentation: command vs realized motion.
  const autonomy::EstimatedState& est = coordinator_.estimate();
  measurement_.update(now_ns * 1e-9, {est.x_m, est.y_m},
                      std::abs(est.forward_mps), std::abs(body.forward_mps));
  return body;
}

iface::GoalStatus RobotApplication::current_goal() const {
  return executive_.current_goal();
}

iface::MapSnapshot RobotApplication::active_map() const {
  return maps_->active_snapshot();
}

iface::ChannelHealth RobotApplication::channel_health() const {
  return channel_.health();
}

RobotApplication::Diagnostics RobotApplication::diagnostics() const {
  Diagnostics d;
  d.has_observation = coordinator_.has_observation();
  if (d.has_observation) {
    d.last_obs_stamp_ns = coordinator_.last_observation().stamp_ns;
    d.last_obs_seq = coordinator_.last_observation().sequence;
    d.estimate = coordinator_.estimate();
  }
  d.bringup = bringup_.state();
  d.localized = coordinator_.localized();
  d.slam_keyframes = coordinator_.slam_keyframes();
  d.slam_loops_closed = coordinator_.slam_loops_closed();
 const auto& loop = coordinator_.last_slam_result();
 d.slam_loop_from_id = loop.from_id;
 d.slam_loop_to_id = loop.to_id;
 d.slam_loop_correction_m = loop.correction_m;
 d.slam_loop_correction_rad = loop.correction_rad;
 d.slam_loop_forward_score = loop.forward_score;
 d.slam_loop_reverse_score = loop.reverse_score;
 d.slam_loop_overlap = loop.overlap;
 d.slam_loop_forward_margin = loop.forward_margin;
 d.slam_loop_reverse_margin = loop.reverse_margin;
 d.slam_loop_diagnostic = loop.diagnostic;
  d.slam_tracks = coordinator_.slam_tracks();
 const auto& track = coordinator_.last_track_result();
 d.track_correction_m = track.correction_m;
 d.track_correction_rad = track.correction_rad;
 d.track_diagnostic = track.diagnostic;
  d.localization_action = coordinator_.last_localization_action();
  d.safety = coordinator_.last_safety().state;
  d.safety_reason = coordinator_.last_safety().reason;
  d.evidence = coordinator_.last_evidence();
  d.braking_active = measurement_.braking().active;
  d.braking_distance_m = measurement_.braking().distance_m;
  d.slip_quality = autonomy::slip_quality_name(coordinator_.last_slip().quality);
  d.slip_confidence = coordinator_.last_slip().confidence;
  d.grip_factor = coordinator_.grip_factor();
  d.goal = executive_.current_goal();
  d.health = channel_.health();
  return d;
}

}  // namespace lost_signal::robot::app
