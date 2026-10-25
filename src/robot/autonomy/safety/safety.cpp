#include "robot/autonomy/safety/safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot/core/deskew.hpp"
#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {

namespace {

struct StaticClearance {
 double minimum_m = std::numeric_limits<double>::infinity();
 double terminal_m = std::numeric_limits<double>::infinity();
};

double point_set_clearance(const std::vector<core::Vec2>& points,
 double x_m, double y_m, double radius_m) {
 double minimum = std::numeric_limits<double>::infinity();
 for (const auto& point : points) {
 minimum = std::min(
 minimum, std::hypot(point.x - x_m, point.y - y_m) - radius_m);
 }
 return minimum;
}

StaticClearance static_clearance_for_command(
 const std::vector<core::Vec2>& points,
 const iface::MotionCommand& command, double reaction_time_s,
 double brake_decel_mps2, double radius_m, double check_dt_s) {
 StaticClearance out;
 if (points.empty()) return out;
 const double initial = point_set_clearance(points, 0.0, 0.0, radius_m);
 out.minimum_m = initial;
 out.terminal_m = initial;
 const double commanded_speed =
 std::hypot(command.forward_mps, command.lateral_mps);
 const double decel = std::max(1e-6, brake_decel_mps2);
 const double reaction = std::max(0.0, reaction_time_s);
 const double braking_time = commanded_speed / decel;
 const double horizon = reaction + braking_time;
 if (!(horizon > 0.0)) return out;

 const double requested_step = std::max(1e-3, check_dt_s);
 const int steps = std::max(
 1, static_cast<int>(std::ceil(horizon / requested_step)));
 double x = 0.0, y = 0.0, heading = 0.0, elapsed = 0.0;
 for (int step = 0; step < steps; ++step) {
 const double dt = std::min(requested_step, horizon - elapsed);
 const double mid_time = elapsed + 0.5 * dt;
 double scale = 1.0;
 if (mid_time > reaction && braking_time > 1e-9) {
 scale = std::clamp(
 1.0 - (mid_time - reaction) / braking_time, 0.0, 1.0);
 } else if (mid_time > reaction) {
 scale = 0.0;
 }
 const double forward = command.forward_mps * scale;
 const double lateral = command.lateral_mps * scale;
 const double yaw_rate = command.yaw_rate_radps * scale;
 const double middle_heading = heading + 0.5 * yaw_rate * dt;
 const double previous_x = x;
 const double previous_y = y;
 x += (forward * std::cos(middle_heading) -
 lateral * std::sin(middle_heading)) * dt;
 y += (forward * std::sin(middle_heading) +
 lateral * std::cos(middle_heading)) * dt;
 heading += yaw_rate * dt;

 const double vx = x - previous_x;
 const double vy = y - previous_y;
 const double segment2 = vx * vx + vy * vy;
 for (const auto& point : points) {
 const double u = segment2 > 1e-12
 ? std::clamp(((point.x - previous_x) * vx +
 (point.y - previous_y) * vy) / segment2,
 0.0, 1.0)
 : 0.0;
 const double qx = previous_x + u * vx;
 const double qy = previous_y + u * vy;
 out.minimum_m = std::min(
 out.minimum_m, std::hypot(point.x - qx, point.y - qy) - radius_m);
 }
 elapsed += dt;
 }
 out.terminal_m = point_set_clearance(points, x, y, radius_m);
 return out;
}

double dynamic_clearance_for_command(
 const EstimatedState& est, const std::vector<DynamicTrack>& tracks,
 const iface::MotionCommand& command, double horizon_s,
 double ego_contact_radius_m, double check_dt_s) {
 double minimum = std::numeric_limits<double>::infinity();
 const double step_s = std::max(1e-3, check_dt_s);
 const int steps = std::max(
 1, static_cast<int>(std::ceil(std::max(0.0, horizon_s) / step_s)));
 for (int step = 0; step <= steps; ++step) {
 const double t = std::min(horizon_s, step * step_s);
 double x = est.x_m;
 double y = est.y_m;
 const double w = command.yaw_rate_radps;
 if (std::abs(w) > 1e-9) {
 const double h1 = est.yaw_rad + w * t;
 x += (command.forward_mps *
 (std::sin(h1) - std::sin(est.yaw_rad)) +
 command.lateral_mps *
 (std::cos(h1) - std::cos(est.yaw_rad))) / w;
 y += (-command.forward_mps *
 (std::cos(h1) - std::cos(est.yaw_rad)) +
 command.lateral_mps *
 (std::sin(h1) - std::sin(est.yaw_rad))) / w;
 } else {
 x += (command.forward_mps * std::cos(est.yaw_rad) -
 command.lateral_mps * std::sin(est.yaw_rad)) * t;
 y += (command.forward_mps * std::sin(est.yaw_rad) +
 command.lateral_mps * std::cos(est.yaw_rad)) * t;
 }
 for (const auto& track : tracks) {
 const double tx = track.x_m + track.vx_mps * t;
 const double ty = track.y_m + track.vy_mps * t;
 minimum = std::min(minimum,
 std::hypot(x - tx, y - ty) - ego_contact_radius_m -
 std::max(0.0, track.occupied_radius_m()));
 }
 }
 return minimum;
}

} // namespace

SafetyMonitor::SafetyMonitor(SafetyConfig config) : config_(config) {}

void SafetyMonitor::reset() {
  latched_ = false;
 reverse_recovery_ = false;
 static_proximity_ = false;
 static_release_count_ = 0;
 dynamic_escape_required_ = false;
 dynamic_estimate_ = EstimatedState{};
 dynamic_tracks_.clear();
 raw_scan_points_.clear();
  latch_reason_.clear();
  last_drops_ = 0;
 rear_min_m_ = std::numeric_limits<float>::infinity();
 front_min_m_ = std::numeric_limits<float>::infinity();
 omni_min_m_ = std::numeric_limits<float>::infinity();
  last_ = {false, SafetyState::kStopped, "init"};
}

SafetyVerdict SafetyMonitor::evaluate(
 const EstimatedState& est, const PerceptionResult& perception,
    const iface::ObservationFrame& obs, const iface::ChannelHealth& channel,
    core::MonotonicNs now_ns) {
  if (latched_) {
    last_ = {false, SafetyState::kStopped, latch_reason_};
    return last_;
  }
 const core::MonotonicNs latest_scan_ns = iface::scan_stamp_ns(obs);
 if (latest_scan_ns == core::kInvalidTime || latest_scan_ns > now_ns ||
 now_ns - latest_scan_ns > config_.stale_timeout_ns) {
    latched_ = true;
    latch_reason_ = "obs-stale";
    last_ = {false, SafetyState::kStopped, latch_reason_};
    return last_;
  }
 dynamic_escape_required_ = false;
 dynamic_estimate_ = est;
 dynamic_tracks_.clear();
 raw_scan_points_.clear();
 const double ego_envelope_m =
 std::max(0.0, config_.footprint_radius_m) +
 std::max(0.0, config_.static_margin_m);
 // Dynamic collision avoidance is driven by anonymous, ROBOT-side tracks
 // extracted from the scan/static-reference difference. The raw scan gate
 // below remains the final contact authority; this forecast closes the gap
 // for a moving forklift or pedestrian crossing from the side between two
 // forward beams.
 if (est.valid && std::isfinite(config_.dynamic_lookahead_s) &&
 config_.dynamic_lookahead_s > 0.0) {
 const double c = std::cos(est.yaw_rad), s = std::sin(est.yaw_rad);
 const double rvx = est.forward_mps * c;
 const double rvy = est.forward_mps * s;
 for (const auto& track : perception.tracks) {
 if (!track.prediction_ready ||
 !std::isfinite(track.x_m) || !std::isfinite(track.y_m) ||
 !std::isfinite(track.vx_mps) || !std::isfinite(track.vy_mps)) {
 continue;
 }
 const double track_speed = std::hypot(track.vx_mps, track.vy_mps);
 if (track_speed < std::max(0.0, config_.dynamic_speed_gate_mps)) {
 continue;
 }
 const double dx = track.x_m - est.x_m;
 const double dy = track.y_m - est.y_m;
 const double relx = track.vx_mps - rvx;
 const double rely = track.vy_mps - rvy;
 const double rel2 = relx * relx + rely * rely;
 const double tau = rel2 > 1e-9
 ? std::clamp(-(dx * relx + dy * rely) / rel2, 0.0,
 config_.dynamic_lookahead_s)
 : 0.0;
 const double px = dx + relx * tau;
 const double py = dy + rely * tau;
 const double predicted = std::hypot(px, py);
 const double occupied = std::max(0.0, track.occupied_radius_m());
 const double contact = ego_envelope_m + occupied;
 if (predicted <= contact) {
 dynamic_escape_required_ = true;
 dynamic_tracks_.push_back(track);
 }
 // A crossing actor need not be in the instantaneous forward sector. Its
 // relative closest approach is still a braking event when the track lies
 // ahead of the body and within the occupied corridor.
 const double along = dx * c + dy * s;
 const double lateral = -dx * s + dy * c;
 const double need = ego_envelope_m + occupied +
 (std::abs(est.forward_mps) * std::abs(est.forward_mps)) /
 (2.0 * config_.brake_decel_mps2);
 if (along > 0.0 && std::abs(lateral) <= occupied + ego_envelope_m &&
 along < need) {
 dynamic_escape_required_ = true;
 const bool already_saved = std::any_of(
 dynamic_tracks_.begin(), dynamic_tracks_.end(),
 [&](const DynamicTrack& saved) { return saved.id == track.id; });
 if (!already_saved) dynamic_tracks_.push_back(track);
 }
 }
 }
  if (obs.scan_valid && !obs.ranges_m.empty()) {
    const std::size_t n = obs.ranges_m.size();
    float omni_min = std::numeric_limits<float>::infinity();
    float fwd_min = std::numeric_limits<float>::infinity();
 rear_min_m_ = std::numeric_limits<float>::infinity();
 const bool has_time_offsets =
 obs.time_offset_valid && obs.time_offsets_s.size() == n;
 const core::DeskewTwist scan_twist{
 est.forward_mps, est.yaw_rate_radps};
 raw_scan_points_.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
      const float r = obs.ranges_m[i];
      if (!(r > 0.0f) || !std::isfinite(r)) continue;
 const double angle = iface::scan_beam_angle_rad(obs, i);
 const double time_offset = has_time_offsets ? obs.time_offsets_s[i] : 0.0;
 const core::Vec2 sensor_point = core::deskew_beam(
 r, angle, time_offset, scan_twist);
 const core::Vec3 rotated = core::rotate_vector(
 base_from_lidar_.rotation, {sensor_point.x, sensor_point.y, 0.0});
 const core::Vec2 base_point{
 base_from_lidar_.translation_m.x + rotated.x,
 base_from_lidar_.translation_m.y + rotated.y};
 raw_scan_points_.push_back(base_point);
 const float base_range = static_cast<float>(
 std::hypot(base_point.x, base_point.y));
 omni_min = std::min(omni_min, base_range);
 const double base_angle = std::atan2(base_point.y, base_point.x);
 if (std::abs(core::wrap_angle(base_angle)) <=
          config_.forward_half_angle_rad) {
 fwd_min = std::min(fwd_min, base_range);
      }
 if (std::abs(core::wrap_angle(base_angle - core::kPi)) <=
 config_.forward_half_angle_rad) {
 rear_min_m_ = std::min(rear_min_m_, base_range);
 }
    }
 front_min_m_ = fwd_min;
 omni_min_m_ = omni_min;
 const double physical_clearance = point_set_clearance(
 raw_scan_points_, 0.0, 0.0,
 std::max(0.0, config_.footprint_radius_m));
 // Physical footprint contact is critical and latches. This calculation
 // includes the sensor mount; raw range alone is not a body clearance.
 if (physical_clearance < 0.0) {
      latched_ = true;
      latch_reason_ = "contact";
      last_ = {false, SafetyState::kStopped, latch_reason_};
      return last_;
    }
 const double padded_clearance =
 physical_clearance - std::max(0.0, config_.static_margin_m);
 if (padded_clearance < 0.0) {
 static_proximity_ = true;
 static_release_count_ = 0;
 } else if (static_proximity_) {
 const bool releasable = physical_clearance >= std::max(
 config_.static_margin_m, config_.static_release_margin_m);
 static_release_count_ = releasable ? static_release_count_ + 1 : 0;
 if (static_release_count_ >= std::max<std::size_t>(
 1, config_.static_release_cycles)) {
 static_proximity_ = false;
 static_release_count_ = 0;
 }
 }
 // Braking veto in the drive direction: margin + v^2/2a on the
 // measured forward speed. Once this trips, hold the transient recovery
 // state until a wider, configured release distance is observed. This
 // prevents scan jitter from alternating one tick of reverse with one tick
 // of forward route tracking at a wall.
    if (std::isfinite(fwd_min)) {
      const double speed = std::abs(est.forward_mps);
      const double need_m =
 ego_envelope_m +
          (speed * speed) / (2.0 * config_.brake_decel_mps2);
      if (fwd_min < need_m) {
   reverse_recovery_ = true;
        last_ = {false, SafetyState::kDegraded, "forward-proximity"};
        return last_;
      }
    }
 if (reverse_recovery_) {
 const double release_m = std::max(
 ego_envelope_m,
 static_cast<double>(config_.recovery_release_m));
 if (std::isfinite(fwd_min) && fwd_min >= release_m &&
 std::isfinite(omni_min) && omni_min >= release_m) {
   reverse_recovery_ = false;
  } else {
   last_ = {false, SafetyState::kDegraded, "forward-recovery"};
   return last_;
  }
 }
 if (static_proximity_) {
 last_ = {false, SafetyState::kDegraded, "static-proximity"};
 return last_;
 }
  } else {
    last_ = {false, SafetyState::kDegraded, "no-scan-evidence"};
    return last_;
  }
  if (channel.obs_dropped != last_drops_) {
    last_drops_ = channel.obs_dropped;
    last_ = {false, SafetyState::kDegraded, "channel-drops"};
    return last_;
  }
 if (dynamic_escape_required_) {
 last_ = {false, SafetyState::kDegraded, "dynamic-proximity"};
 return last_;
 }
  last_ = {true, SafetyState::kNominal, "nominal"};
  return last_;
}

iface::MotionCommand SafetyMonitor::gate(const iface::MotionCommand& cmd,
                                         const SafetyVerdict& verdict,
                                         core::MonotonicNs now_ns) {
 if (cmd.is_safe_stop()) return cmd;

 // Independent collision-monitor path: reproject the command through the
 // controller reaction interval and physical braking tail against the
 // current base-frame raw scan. This deliberately bypasses the costmap,
 // planner, validator, global pose, and simulator truth.
 bool static_command_admissible = true;
 if (!raw_scan_points_.empty()) {
 const double envelope =
 std::max(0.0, config_.footprint_radius_m) +
 std::max(0.0, config_.static_margin_m);
 const StaticClearance projected = static_clearance_for_command(
 raw_scan_points_, cmd, config_.command_reaction_time_s,
 config_.brake_decel_mps2, envelope, config_.static_check_dt_s);
 const double hold_clearance = point_set_clearance(
 raw_scan_points_, 0.0, 0.0, envelope);
 last_.static_hold_clearance_m = hold_clearance;
 last_.static_command_min_clearance_m = projected.minimum_m;
 last_.static_command_terminal_clearance_m = projected.terminal_m;
 const double speed = std::hypot(cmd.forward_mps, cmd.lateral_mps);
 const bool nominally_clear = projected.minimum_m >= 0.0;
 const double comparison_tolerance =
 std::max(0.0, config_.static_escape_clearance_tolerance_m);
 // When already inside only the padded envelope, permit a bounded escape
 // exactly when the independently projected stopping path never reduces
 // clearance and ends farther away. Requiring a fixed gain in one braking
 // interval deadlocks tangential corner escapes; the validator separately
 // requires substantial gain over its complete rollout.
 const bool minimum_risk =
 speed <= config_.static_escape_speed_cap_mps + 1e-9 &&
 projected.minimum_m + comparison_tolerance >= hold_clearance &&
 projected.terminal_m > hold_clearance + comparison_tolerance;
 static_command_admissible = nominally_clear || minimum_risk;
 if (!static_command_admissible) {
 last_.allow_motion = false;
 last_.state = SafetyState::kDegraded;
 last_.reason = "static-swept-proximity";
 return iface::MotionCommand::SafeStop(now_ns);
 }
 }

 if (verdict.allow_motion) return cmd;
 // A front proximity veto is transient. Permit a reverse command only when
 // the independent raw scan has a full rear braking margin. The trajectory
 // validator has already checked the swept reverse candidate; this gate is
 // the last conservative rear-sector check and never consults simulator
 // truth or planner internals.
 if (verdict.state == SafetyState::kDegraded &&
 (verdict.reason == "forward-proximity" ||
  verdict.reason == "forward-recovery") && cmd.forward_mps < -1e-6 &&
     std::isfinite(rear_min_m_)) {
 const double speed = std::abs(cmd.forward_mps);
 const double need_m = std::max(0.0, config_.footprint_radius_m) +
 std::max(0.0, config_.static_margin_m) +
     (speed * speed) / (2.0 * config_.brake_decel_mps2);
 if (rear_min_m_ >= need_m) return cmd;
 }
 // A slow command that increases raw footprint clearance is the only
 // automatic exit from transient all-around proximity. Physical contact is
 // separately latched above and never reaches this branch.
 if (verdict.state == SafetyState::kDegraded &&
 verdict.reason == "static-proximity" &&
 static_command_admissible) {
 return cmd;
 }
 // A forecasted impact is degraded, not a permanent contact latch. Recheck
 // the proposed command independently against ROBOT-side tracks and pass it
 // only when it avoids the contact envelope, or when a slow command improves
 // the unavoidable closest approach over stopping in place.
 if (verdict.state == SafetyState::kDegraded &&
 verdict.reason == "dynamic-proximity" &&
 dynamic_escape_required_ && dynamic_estimate_.valid &&
 !dynamic_tracks_.empty() && !cmd.is_safe_stop()) {
 const iface::MotionCommand hold = iface::MotionCommand::SafeStop(now_ns);
 const double dynamic_envelope =
 std::max(0.0, config_.footprint_radius_m) +
 std::max(0.0, config_.static_margin_m);
 const double hold_clearance = dynamic_clearance_for_command(
 dynamic_estimate_, dynamic_tracks_, hold,
 config_.dynamic_lookahead_s, dynamic_envelope,
 config_.dynamic_check_dt_s);
 const double command_clearance = dynamic_clearance_for_command(
 dynamic_estimate_, dynamic_tracks_, cmd,
 config_.dynamic_lookahead_s, dynamic_envelope,
 config_.dynamic_check_dt_s);
 last_.dynamic_hold_clearance_m = hold_clearance;
 last_.dynamic_command_clearance_m = command_clearance;
 const double speed = std::hypot(cmd.forward_mps, cmd.lateral_mps);
 const bool nominally_clear = command_clearance >= 0.0;
 const bool minimum_risk =
 speed <= config_.dynamic_escape_speed_cap_mps + 1e-9 &&
 command_clearance >= hold_clearance + config_.dynamic_escape_gain_m;
 if (nominally_clear || minimum_risk) return cmd;
 }
  return iface::MotionCommand::SafeStop(now_ns);
}

void SafetyMonitor::clear_safety() {
  latched_ = false;
 reverse_recovery_ = false;
 static_proximity_ = false;
 static_release_count_ = 0;
 dynamic_escape_required_ = false;
 dynamic_tracks_.clear();
 raw_scan_points_.clear();
  latch_reason_.clear();
}

}  // namespace lost_signal::robot::autonomy
