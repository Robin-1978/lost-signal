#pragma once

#include "sim/game/platform/simulated_platform.hpp"

#include <string>
#include <vector>

namespace mission {

// Physics truth is a SIM outcome, never a ROBOT observation. This judge is
// used for mission failure, scoring and replay diagnostics after the plant has
// stepped. It is intentionally not reachable from RobotApplication.
struct PhysicalCollisionBox {
 sim::Vec2 min;
 sim::Vec2 max;
 double yaw_rad{0.0};
};

struct PhysicalActorTruth {
 std::string id;
 sim::Vec2 position;
 double radius_m{0.0};
 bool social{false};
};

struct PhysicalCollisionOutcome {
 bool static_collision{false};
 bool dynamic_collision{false};
 int social_contacts{0};
 double static_clearance_m{0.0};
 double dynamic_clearance_m{999.0};
 sim::Vec2 nearest_actor_position;
};

class PhysicalCollisionJudge final {
 public:
 PhysicalCollisionJudge(sim::Vec2 world_size, double robot_radius_m,
 std::vector<PhysicalCollisionBox> obstacles);

 PhysicalCollisionOutcome assess(sim::Vec2 position,
 const sim::platform::RigidVehicleState& rigid_state,
 const std::vector<PhysicalActorTruth>& actors) const;

 private:
 sim::Vec2 world_size_;
 double robot_radius_m_{0.0};
 std::vector<PhysicalCollisionBox> obstacles_;
};

} // namespace mission
