#pragma once

#include "autonomy/mapping/grid_map.hpp"
#include "core/dynamic_object_state.hpp"

#include <vector>

namespace mission {

enum class CollisionBackend { Occupancy, RigidBody };

struct CollisionBox {
 sim::Vec2 min;
 sim::Vec2 max;
};

struct CollisionActorObservation {
 sim::DynamicObjectState state;
 bool social{false};
 bool rigid_contact{false};
};

struct CollisionObservation {
 sim::Vec2 position;
 double time_s{0.0};
 double dt_s{0.0};
 CollisionBackend backend{CollisionBackend::Occupancy};
 bool rigid_static_contact{false};
 const sim::GridMap* occupancy{nullptr};
 std::vector<CollisionActorObservation> actors;
};

struct CollisionAssessment {
 bool static_collision{false};
 bool dynamic_collision{false};
 int social_contacts{0};
 double static_clearance_m{0.0};
 double dynamic_clearance_m{999.0};
 sim::Vec2 nearest_actor_position;
};

// Mission safety monitor. Physics and domain backends provide different
// contact evidence, but use one footprint, world-boundary, clearance, social
// contact and static-contact debounce contract.
class CollisionMonitor {
 public:
 CollisionMonitor(sim::Vec2 world_size, double robot_radius_m,
 std::vector<CollisionBox> static_obstacles);

 void reset();
 CollisionAssessment assess(const CollisionObservation& observation);
 // Continuous circular-footprint check used by local trajectory validation.
 // This is analytic world geometry, independent of the raster costmap used
 // for route preference and observed-obstacle layering.
 bool static_motion_free(sim::Vec2 from, sim::Vec2 to) const;

 private:
 double static_clearance(sim::Vec2 position) const;

 sim::Vec2 world_size_;
 double robot_radius_m_{0.0};
 std::vector<CollisionBox> static_obstacles_;
 double rigid_static_contact_s_{0.0};
};

} // namespace mission
