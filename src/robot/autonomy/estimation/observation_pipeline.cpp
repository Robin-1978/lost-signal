#include "robot/autonomy/estimation/observation_pipeline.hpp"

#include "robot/autonomy/decision/terrain_pipeline.hpp"

namespace lost_signal::robot::autonomy {

void ObservationPipeline::reset() {
 measurements_.reset();
 localization_.reset();
 perception_ = {};
 last_traverse_ns_ = core::kInvalidTime;
 last_localization_action_ = "init";
}

bool ObservationPipeline::ingest(
 const iface::ObservationFrame& observation,
 const core::FrameTransformTree& transforms, IEstimator& estimator,
 IPerception& perception, IMapper& mapper) {
 MeasurementBatch batch = measurements_.ingest(observation, transforms);
 if (!batch.accepted) return false;

 for (const iface::ObservationFrame& event : batch.estimator_events) {
 estimator.predict(event);
 estimator.correct(event);
 }
 if (batch.scan_event.has_value()) {
 perception_ = perception.update(*batch.scan_event, estimator.estimate());
 mapper.set_dynamic_tracks(perception_.tracks);
 mapper.update(*batch.scan_event, estimator.estimate());
 }
 return true;
}

ObservationPipeline::TickResult ObservationPipeline::update(
 const iface::MapSnapshot& map, core::MonotonicNs now_ns,
 IEstimator& estimator, IPerception& perception, IMapper& mapper,
 GripAdapter& grip, TraversalModel& traversal) {
 // Install the latest ROBOT map before the next scan arrives. Raycasting
 // consumes known occupancy only and never accesses simulator geometry.
 perception.set_static_map(map);

 TickResult result;
 iface::ObservationFrame localization_observation =
 measurements_.take_localization_observation();
 result.localization = localization_.update(
 map, localization_observation, now_ns, measurements_.has_observation(),
 estimator, mapper);
 last_localization_action_ = result.localization.action;

 if (const auto adaptation = measurements_.take_adaptation_observation()) {
 TerrainAdaptationPipeline::update(
 now_ns, *adaptation, map, estimator.estimate(), estimator.last_slip(),
 last_traverse_ns_, grip, traversal);
 }
 return result;
}

} // namespace lost_signal::robot::autonomy
