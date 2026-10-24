#pragma once

#include "robot/autonomy/control/grip_adapter.hpp"
#include "robot/autonomy/estimation/estimator.hpp"
#include "robot/autonomy/mapping/traversal.hpp"
#include "robot/iface/map.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::autonomy {

// Owns online traction and traversal-belief updates. The coordinator wires
// observations and estimates here; it does not decide adaptation policy.
class TerrainAdaptationPipeline {
 public:
 static void update(core::MonotonicNs now_ns,
 const iface::ObservationFrame& observation,
 const iface::MapSnapshot& survey, const EstimatedState& estimate,
 const SlipObservation& slip,
 core::MonotonicNs& last_traverse_ns, GripAdapter& grip,
 TraversalModel& traversal);
};

} // namespace lost_signal::robot::autonomy
