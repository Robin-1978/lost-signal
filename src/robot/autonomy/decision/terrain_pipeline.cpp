#include "robot/autonomy/decision/terrain_pipeline.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lost_signal::robot::autonomy {

void TerrainAdaptationPipeline::update(
 core::MonotonicNs now_ns, const iface::ObservationFrame& observation,
 const iface::MapSnapshot& survey, const EstimatedState& estimate,
 const SlipObservation& slip,
 core::MonotonicNs& last_traverse_ns, GripAdapter& grip,
 TraversalModel& traversal) {
 double survey_pitch = std::numeric_limits<double>::quiet_NaN();
 if (estimate.valid) {
 survey_pitch = survey_pitch_along(survey, estimate.x_m, estimate.y_m,
 estimate.yaw_rad);
 }
 grip.update(now_ns, observation, estimate, slip, survey_pitch);
 if (estimate.valid) {
 traversal.observe_slip(estimate.x_m, estimate.y_m, grip.slip_severity(),
 observation.stamp_ns * 1e-9);
 }
 if (observation.imu_valid) {
 if (last_traverse_ns != core::kInvalidTime && now_ns > last_traverse_ns) {
 const double dt_s = std::min(
 1.0, static_cast<double>(now_ns - last_traverse_ns) * 1e-9);
 traversal.observe_motion(estimate, observation.imu_az_mps2,
 observation.stamp_ns * 1e-9, dt_s);
 }
 last_traverse_ns = now_ns;
 }
}

} // namespace lost_signal::robot::autonomy
