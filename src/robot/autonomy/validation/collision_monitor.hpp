#pragma once

#include "robot/autonomy/mapping/grid_map.hpp"
#include "robot/core/dynamic_object_state.hpp"

#include <vector>

namespace sim::autonomy {

enum class CollisionBackend { Occupancy, RigidBody };

struct CollisionBox {
 Vec2 min;
 Vec2 max;
 double yaw_rad{0.0};
};

struct CollisionActorObservation {
 DynamicObjectState state;
 bool social{false};
 bool rigid_contact{false};
};

struct CollisionObservation {
 Vec2 position;
 double time_s{0.0};
 double dt_s{0.0};
 CollisionBackend backend{CollisionBackend::Occupancy};
 bool rigid_static_contact{false};
 const GridMap* occupancy{nullptr};
 std::vector<CollisionActorObservation> actors;
};

struct CollisionAssessment {
 bool static_collision{false};
 bool dynamic_collision{false};
 int social_contacts{0};
 double static_clearance_m{0.0};
 double dynamic_clearance_m{999.0};
 Vec2 nearest_actor_position;
};

// Mission safety monitor. Physics and domain backends provide different
// contact evidence, but use one footprint, world-boundary, clearance, social
// contact and static-contact debounce contract.
class CollisionMonitor {
 public:
 CollisionMonitor(Vec2 world_size, double robot_radius_m,
 std::vector<CollisionBox> static_obstacles);

 void reset();
 CollisionAssessment assess(const CollisionObservation& observation);
 // Continuous circular-footprint check used by local trajectory validation.
 // This is analytic world geometry, independent of the raster costmap used
 // for route preference and observed-obstacle layering.
 bool static_motion_free(Vec2 from, Vec2 to,
 double execution_margin_m = 0.0) const;
 double static_clearance(Vec2 position) const;

 private:
 Vec2 world_size_;
 double robot_radius_m_{0.0};
 std::vector<CollisionBox> static_obstacles_;
 double rigid_static_contact_s_{0.0};
};

} // namespace sim::autonomy
