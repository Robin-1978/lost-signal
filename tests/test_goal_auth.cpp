#include <cassert>
#include <cstdio>
#include <memory>

#include "robot/app/robot_application.hpp"

namespace {
class PlacesMap : public lost_signal::robot::iface::IMapProvider {
 public:
  explicit PlacesMap(double dock_x_m = 5.0) {
    snap_.map_id = "test";
    snap_.revision = 1;
    snap_.frame_id = "map";
    snap_.places.push_back({"dock", dock_x_m, 0.0, 0.0});
  }
 const lost_signal::robot::iface::MapSnapshot& active_snapshot() const override {
    return snap_;
  }
  bool select_map(const std::string& id) override {
    return id == "test" || id.empty();
  }

 private:
  lost_signal::robot::iface::MapSnapshot snap_;
};

void feed_obs(lost_signal::robot::app::RobotApplication& app,
              int seq, lost_signal::robot::core::MonotonicNs stamp_ns) {
  using namespace lost_signal::robot;
  iface::ObservationFrame obs;
  obs.sequence = static_cast<std::uint64_t>(seq);
  obs.stamp_ns = stamp_ns;
  obs.source = "t.lidar";
  obs.frame = core::Frame::kLidarLink;
  obs.scan_valid = true;
  obs.ranges_m.assign(72, 10.0f);
 obs.scan_header = {obs.sequence, stamp_ns, "t.lidar",
 core::Frame::kLidarLink};
 obs.odometry_valid = true;
 obs.motion_header = {obs.sequence, stamp_ns, "t.encoder",
 core::Frame::kBaseLink};
  assert(app.publish_observation(obs));
}

// Monotone feeder: sequence and stamp strictly increase across the whole
// test (the channel rejects stale per-source stamps).
struct Feeder {
  int seq = 1;
  lost_signal::robot::core::MonotonicNs t_ns = 1'000'000;
  void feed(lost_signal::robot::app::RobotApplication& app) {
    feed_obs(app, seq++, t_ns);
  }
  lost_signal::robot::iface::MotionCommand tick(
      lost_signal::robot::app::RobotApplication& app) {
    feed(app);
    const auto cmd = app.tick(t_ns);
    t_ns += 100'000'000;
    return cmd;
  }
};
}  // namespace

int main() {
  using namespace lost_signal::robot;

  app::RobotApplication app(std::make_unique<PlacesMap>());
  assert(app.configure().ok && app.activate().ok);
  app.register_provider({"t.lidar", iface::EndpointRole::kLidar,
                         iface::ProviderBackend::kNative, "lidar_link"});
 app.register_provider({"t.encoder", iface::EndpointRole::kEncoder,
 iface::ProviderBackend::kNative, "base_link"});

  // --- Pin 1: a task the Executive rejects never enters planning. ---
  // Valid place, but empty request id: the executive fails closed with an
  // invalid handle. The coordinator must arm nothing: ticks stay stopped.
  Feeder feed;
  {
    iface::TaskRequest rejected;
    rejected.request_id = "";  // executive rejects: no acceptance
    rejected.place_id = "dock";
    const iface::GoalHandle h = app.request_goal(rejected);
    assert(!h.valid());
    assert(app.current_goal().state != iface::GoalState::kActive);
    for (int i = 0; i < 6; ++i) {
      assert(feed.tick(app).is_safe_stop());
    }
  }

  // --- Accepted task plans and drives (positive control). ---
  iface::TaskRequest req;
  req.request_id = "t1";
  req.place_id = "dock";
  const iface::GoalHandle h = app.request_goal(req);
  assert(h.valid());
  assert(app.goal_status(h).state == iface::GoalState::kActive);
  bool driving = false;
  for (int i = 0; i < 8 && !driving; ++i) {
    driving = feed.tick(app).forward_mps > 0.0;
  }
  assert(driving);

  // --- Failed cancel leaves the route alone. ---
  assert(!app.cancel_goal(iface::GoalHandle{}));  // invalid handle
  assert(app.goal_status(h).state == iface::GoalState::kActive);
  assert(feed.tick(app).forward_mps > 0.0);  // still driving

  // --- Pin 2: successful cancel withdraws the coordinator's route. ---
  assert(app.cancel_goal(h));
  assert(app.goal_status(h).state == iface::GoalState::kCanceled);
  for (int i = 0; i < 6; ++i) {
    assert(feed.tick(app).is_safe_stop());  // dead goal drives nothing
  }

  // --- Hold withdraws the active route immediately. ---
 {
 iface::TaskRequest again;
 again.request_id = "t2";
 again.place_id = "dock";
 const iface::GoalHandle h2 = app.request_goal(again);
 assert(h2.valid());
 bool moving = false;
 for (int i = 0; i < 8 && !moving; ++i) {
 moving = feed.tick(app).forward_mps > 0.0;
 }
 assert(moving);

 // Manual authority cancels the autonomous task once and takes over the
 // command source.  A watchdog expiry must stop, never resume that route.
 app.set_operator_command(0.1, 0.0);
 assert(app.goal_status(h2).state == iface::GoalState::kCanceled);
 assert(app.goal_status(h2).detail == "canceled");
 const auto manual = feed.tick(app);
 assert(!manual.is_safe_stop());
 feed.tick(app);  // 100 ms after the manual refresh
 feed.tick(app);  // 200 ms boundary is still valid (strict watchdog)
 const auto manual_expired = feed.tick(app);  // past the 200 ms watchdog
 assert(manual_expired.is_safe_stop());
 assert(app.goal_status(h2).state == iface::GoalState::kCanceled);

 // A fresh goal is required to restore autonomous authority.
 const iface::GoalHandle h_manual_resume = app.request_goal(again);
 assert(h_manual_resume.valid());
 assert(app.goal_status(h_manual_resume).state == iface::GoalState::kActive);
 for (int i = 0; i < 3; ++i) feed.tick(app);

 iface::TaskRequest hold;
 hold.request_id = "hold1"; // empty place = hold
 const iface::GoalHandle hh = app.request_goal(hold);
 assert(hh.valid());
 assert(app.goal_status(hh).state == iface::GoalState::kDone);
 assert(app.goal_status(h_manual_resume).state == iface::GoalState::kCanceled);
 for (int i = 0; i < 6; ++i) {
 assert(feed.tick(app).is_safe_stop());
 }
 }

 // --- Arrival completes the goal and stops on the arrival tick. ---
 {
 app::RobotApplication near(std::make_unique<PlacesMap>(1.0));
 assert(near.configure().ok && near.activate().ok);
 // Arrival authority needs an observable pose. This fixture has no map
 // geometry (all laser returns are max range), so seed the documented known
 // dock pose and exercise the final 0.25 m approach without manufacturing
 // confidence from an unobservable scan.
 near.set_initial_pose(0.75, 0.0, 0.0, 0.02, 0.01);
 near.register_provider({"t.lidar", iface::EndpointRole::kLidar,
 iface::ProviderBackend::kNative, "lidar_link"});
 near.register_provider({"t.encoder", iface::EndpointRole::kEncoder,
 iface::ProviderBackend::kNative, "base_link"});
 iface::TaskRequest task;
 task.request_id = "arrive";
 task.place_id = "dock";
 const iface::GoalHandle h = near.request_goal(task);
 assert(h.valid());
 Feeder nf;
 bool arrived = false;
 bool settling_stop_seen = false;
 double measured_speed_mps = 0.5;
 for (int i = 0; i < 80; ++i) {
 iface::ObservationFrame obs;
 obs.sequence = static_cast<std::uint64_t>(nf.seq++);
 obs.stamp_ns = nf.t_ns;
 obs.source = "t.lidar";
 obs.frame = core::Frame::kLidarLink;
 obs.scan_valid = true;
 obs.ranges_m.assign(72, 10.0f);
 obs.scan_header = {obs.sequence, obs.stamp_ns, "t.lidar",
 core::Frame::kLidarLink};
 obs.odometry_valid = true;
 obs.forward_mps = measured_speed_mps;
 obs.motion_header = {obs.sequence, obs.stamp_ns, "t.encoder",
 core::Frame::kBaseLink};
 assert(near.publish_observation(obs));
 const auto cmd = near.tick(nf.t_ns);
 nf.t_ns += 100'000'000;
 // Model the plant response to the arrival hold. A moving observation must
 // never complete the task; once the command stops near the goal, publish
 // settled wheel motion on subsequent samples.
 if (near.diagnostics().estimate.x_m > 0.70 && cmd.is_safe_stop()) {
 settling_stop_seen = true;
 measured_speed_mps = 0.0;
 }
 if (near.current_goal().state == iface::GoalState::kDone) {
 assert(cmd.is_safe_stop());
 arrived = true;
 break;
 }
 }
 assert(settling_stop_seen);
 if (!arrived) {
 const auto diag = near.diagnostics();
 std::fprintf(stderr,
 "arrival final: est=(%.3f,%.3f,%.3f) vel=(%.3f,%.3f) "
 "sigma=(%.3f,%.3f) cmd_reason=%s constraint=%s goal=%d/%s\n",
 diag.estimate.x_m, diag.estimate.y_m, diag.estimate.yaw_rad,
 diag.estimate.forward_mps, diag.estimate.yaw_rate_radps,
 diag.estimate.position_sigma_m, diag.estimate.heading_sigma_rad,
 diag.evidence.reason_code.c_str(), diag.evidence.constraint_code.c_str(),
 static_cast<int>(diag.goal.state), diag.goal.detail.c_str());
 }
 assert(arrived);
 assert(near.current_goal().state == iface::GoalState::kDone);
 }

 std::printf("goal_auth ok\n");
  return 0;
}
