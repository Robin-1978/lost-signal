#include "robot/autonomy/safety/safety.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {

SafetyMonitor::SafetyMonitor(SafetyConfig config) : config_(config) {}

void SafetyMonitor::reset() {
  latched_ = false;
 reverse_recovery_ = false;
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
  if (now_ns - obs.stamp_ns > config_.stale_timeout_ns) {
    latched_ = true;
    latch_reason_ = "obs-stale";
    last_ = {false, SafetyState::kStopped, latch_reason_};
    return last_;
  }
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
 const double contact = static_cast<double>(config_.contact_radius_m) + occupied;
 if (predicted <= contact) {
 latched_ = true;
 latch_reason_ = "dynamic-contact";
 last_ = {false, SafetyState::kStopped, latch_reason_};
 return last_;
 }
 // A crossing actor need not be in the instantaneous forward sector. Its
 // relative closest approach is still a braking event when the track lies
 // ahead of the body and within the occupied corridor.
 const double along = dx * c + dy * s;
 const double lateral = -dx * s + dy * c;
 const double need = static_cast<double>(config_.near_field_m) + occupied +
 (std::abs(est.forward_mps) * std::abs(est.forward_mps)) /
 (2.0 * config_.brake_decel_mps2);
 if (along > 0.0 && std::abs(lateral) <= occupied +
 static_cast<double>(config_.near_field_m) && along < need) {
 reverse_recovery_ = true;
 last_ = {false, SafetyState::kDegraded, "dynamic-forward-proximity"};
 return last_;
 }
 }
 }
  if (obs.scan_valid && !obs.ranges_m.empty()) {
    const std::size_t n = obs.ranges_m.size();
    float omni_min = std::numeric_limits<float>::infinity();
    float fwd_min = std::numeric_limits<float>::infinity();
 rear_min_m_ = std::numeric_limits<float>::infinity();
    for (std::size_t i = 0; i < n; ++i) {
      const float r = obs.ranges_m[i];
      if (!(r > 0.0f) || !std::isfinite(r)) continue;
      omni_min = std::min(omni_min, r);
 const double angle = iface::scan_beam_angle_rad(obs, i);
 const core::Vec3 dir = core::rotate_vector(
 base_from_lidar_.rotation, {std::cos(angle), std::sin(angle), 0.0});
 const double base_angle = std::atan2(dir.y, dir.x);
 if (std::abs(core::wrap_angle(base_angle)) <=
          config_.forward_half_angle_rad) {
        fwd_min = std::min(fwd_min, r);
      }
 if (std::abs(core::wrap_angle(base_angle - core::kPi)) <=
 config_.forward_half_angle_rad) {
 rear_min_m_ = std::min(rear_min_m_, r);
 }
    }
 front_min_m_ = fwd_min;
 omni_min_m_ = omni_min;
    // Contact latch: omnidirectional, last resort only.
    if (omni_min < config_.contact_radius_m) {
      latched_ = true;
      latch_reason_ = "contact";
      last_ = {false, SafetyState::kStopped, latch_reason_};
      return last_;
    }
 // Braking veto in the drive direction: margin + v^2/2a on the
 // measured forward speed. Once this trips, hold the transient recovery
 // state until a wider, configured release distance is observed. This
 // prevents scan jitter from alternating one tick of reverse with one tick
 // of forward route tracking at a wall.
    if (std::isfinite(fwd_min)) {
      const double speed = std::abs(est.forward_mps);
      const double need_m =
          config_.near_field_m +
          (speed * speed) / (2.0 * config_.brake_decel_mps2);
      if (fwd_min < need_m) {
   reverse_recovery_ = true;
        last_ = {false, SafetyState::kDegraded, "forward-proximity"};
        return last_;
      }
    }
 if (reverse_recovery_) {
 const double release_m = std::max(
 static_cast<double>(config_.near_field_m),
 static_cast<double>(config_.recovery_release_m));
 if (std::isfinite(fwd_min) && fwd_min >= release_m &&
 std::isfinite(omni_min) && omni_min >= release_m) {
   reverse_recovery_ = false;
  } else {
   last_ = {false, SafetyState::kDegraded, "forward-recovery"};
   return last_;
  }
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
  last_ = {true, SafetyState::kNominal, "nominal"};
  return last_;
}

iface::MotionCommand SafetyMonitor::gate(const iface::MotionCommand& cmd,
                                         const SafetyVerdict& verdict,
                                         core::MonotonicNs now_ns) {
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
 const double need_m = config_.near_field_m +
     (speed * speed) / (2.0 * config_.brake_decel_mps2);
 if (rear_min_m_ >= need_m) return cmd;
 }
  return iface::MotionCommand::SafeStop(now_ns);
}

void SafetyMonitor::clear_safety() {
  latched_ = false;
 reverse_recovery_ = false;
  latch_reason_.clear();
}

}  // namespace lost_signal::robot::autonomy
