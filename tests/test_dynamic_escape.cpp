#include <cassert>
#include <cstdio>

#include "robot/autonomy/control/controller.hpp"
#include "robot/autonomy/safety/safety.hpp"
#include "robot/autonomy/validation/validator.hpp"

namespace {

lost_signal::robot::iface::ObservationFrame open_scan(
 lost_signal::robot::core::MonotonicNs stamp_ns) {
 lost_signal::robot::iface::ObservationFrame observation;
 observation.sequence = 1;
 observation.stamp_ns = stamp_ns;
 observation.source = "dynamic-escape-test";
 observation.frame = lost_signal::robot::core::Frame::kLidarLink;
 observation.odometry_valid = true;
 observation.scan_valid = true;
 observation.scan_angle_min_rad = -lost_signal::robot::core::kPi;
 observation.scan_angle_increment_rad =
 2.0 * lost_signal::robot::core::kPi / 32.0;
 observation.ranges_m.assign(32, 10.0f);
 observation.static_ranges_m = observation.ranges_m;
 observation.scan_static_valid = true;
 return observation;
}

} // namespace

int main() {
 using namespace lost_signal::robot;
 using namespace lost_signal::robot::autonomy;

 constexpr core::MonotonicNs stamp_ns = 1'000'000'000;
 EstimatedState estimate;
 estimate.valid = true;
 estimate.stamp_ns = stamp_ns;

 DynamicTrack approaching;
 approaching.id = 7;
 approaching.x_m = 0.6;
 approaching.vx_mps = -0.3;
 approaching.radius_m = 0.2;
 approaching.prediction_ready = true;
 approaching.last_observed_ns = stamp_ns;

 PerceptionResult perception;
 perception.scan_ok = true;
 perception.stamp_ns = stamp_ns;
 perception.tracks.push_back(approaching);
 const PredictionBatch prediction = predict_perception(
 perception, 1.0, {.horizon_s = 2.5, .frame_dt_s = 0.1});
 const auto observation = open_scan(stamp_ns);

 ClearanceValidator validator;
 LocalCandidate hold;
 hold.feasible = true;
 const ValidationResult stopped = validator.validate(
 hold, estimate, perception, prediction, observation);
 assert(!stopped.accepted);
 assert(stopped.verdict == TrajectoryVerdict::kDynamicCollision);
 assert(stopped.min_dynamic_clearance_m < 0.0);

 LocalCandidate retreat;
 retreat.feasible = true;
 retreat.forward_mps = -0.25;
 const ValidationResult escaped = validator.validate(
 retreat, estimate, perception, prediction, observation);
 assert(escaped.accepted);
 assert(escaped.min_dynamic_clearance_m >
 stopped.min_dynamic_clearance_m);

 // A confirmed moving return must not become permanent static geometry when
 // the scan has no mapper-produced static reference. The dynamic validator
 // remains the hard gate for the same track.
 auto unreferenced_observation = open_scan(stamp_ns);
 unreferenced_observation.scan_static_valid = false;
 unreferenced_observation.static_ranges_m.clear();
 unreferenced_observation.ranges_m[16] = 0.6f;
 const ValidationResult unreferenced_escape = validator.validate(
     retreat, estimate, perception, prediction, unreferenced_observation);
 assert(unreferenced_escape.accepted);

 LocalCandidate toward = retreat;
 toward.forward_mps = 0.25;
 const ValidationResult collision = validator.validate(
 toward, estimate, perception, prediction, observation);
 assert(!collision.accepted);
 assert(collision.verdict == TrajectoryVerdict::kDynamicCollision);

 // The independent final gate reaches the same ordering from ROBOT-side
 // tracks: future contact is degraded, a retreat that improves clearance is
 // allowed, and motion toward the threat is stopped. Raw contact remains a
 // separate latched condition covered by test_safety.
 SafetyMonitor safety;
 safety.reset();
 iface::ChannelHealth channel;
 const SafetyVerdict threat = safety.evaluate(
 estimate, perception, observation, channel, stamp_ns);
 assert(threat.state == SafetyState::kDegraded);
 assert(threat.reason == "dynamic-proximity");
 DirectController controller;
 const iface::MotionCommand reverse_command = controller.track(
 retreat, estimate, stamp_ns);
 assert(!safety.gate(reverse_command, threat, stamp_ns).is_safe_stop());
 const iface::MotionCommand toward_command = controller.track(
 toward, estimate, stamp_ns);
 assert(safety.gate(toward_command, threat, stamp_ns).is_safe_stop());

 // Dynamic clearance uses the same configured navigation margin as DWA,
 // behavior, and the final safety gate. Geometric non-contact alone is not
 // enough to admit a trajectory inside that declared margin.
 DynamicTrack margin_track;
 margin_track.id = 8;
 margin_track.x_m = 0.53;
 margin_track.radius_m = 0.05;
 margin_track.prediction_ready = true;
 margin_track.last_observed_ns = stamp_ns;
 PerceptionResult margin_perception;
 margin_perception.scan_ok = true;
 margin_perception.stamp_ns = stamp_ns;
 margin_perception.tracks = {margin_track};
 const PredictionBatch margin_prediction = predict_perception(
 margin_perception, 1.0, {.horizon_s = 2.5, .frame_dt_s = 0.1});
 const ValidationResult padded_hold = validator.validate(
 hold, estimate, margin_perception, margin_prediction, observation);
 assert(!padded_hold.accepted);
 assert(padded_hold.verdict == TrajectoryVerdict::kDynamicCollision);
 ClearanceValidatorConfig no_margin_config;
 no_margin_config.extra_margin_m = 0.0;
 ClearanceValidator no_margin_validator(no_margin_config);
 const ValidationResult geometric_hold = no_margin_validator.validate(
 hold, estimate, margin_perception, margin_prediction, observation);
 assert(geometric_hold.accepted);

 // A zero command starts from measured chassis speed. Its validator rollout
 // includes the configured response delay and braking tail, so an obstacle
 // that is clear at the current pose but intersects the physical stop is
 // rejected before the controller publishes the hold.
 EstimatedState moving_estimate = estimate;
 moving_estimate.forward_mps = 1.0;
 DynamicTrack braking_track;
 braking_track.id = 9;
 braking_track.x_m = 0.9;
 braking_track.radius_m = 0.1;
 braking_track.prediction_ready = true;
 braking_track.last_observed_ns = stamp_ns;
 PerceptionResult braking_perception;
 braking_perception.scan_ok = true;
 braking_perception.stamp_ns = stamp_ns;
 braking_perception.tracks = {braking_track};
 const PredictionBatch braking_prediction = predict_perception(
 braking_perception, 1.0, {.horizon_s = 2.5, .frame_dt_s = 0.1});
 const ValidationResult physical_hold = validator.validate(
 hold, moving_estimate, braking_perception, braking_prediction,
 observation);
 assert(!physical_hold.accepted);
 assert(physical_hold.verdict == TrajectoryVerdict::kDynamicCollision);

 std::printf("dynamic escape ok\n");
 return 0;
}
