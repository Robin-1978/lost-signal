#include "autonomy/planning/local/ackermann_local.hpp"
#include "autonomy/planning/global/hybrid_astar.hpp"
#include "hw/chassis/holonomic.hpp"

#include <algorithm>
#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 sim::GridMap map(40, 40, 0.25);
 // A wall with a broad opening forces a real turn rather than a straight
 // interpolation, while leaving enough room for the configured car radius.
 map.addBox(4.0, 0.0, 4.5, 5.0);
 sim::HybridAStarConfig hybrid;
 hybrid.wheel_base_m = 0.62;
 hybrid.max_steer_rad = 0.55;
 hybrid.primitive_length_m = 0.375;
 const auto plan = sim::hybrid_astar_plan(map, {1.0, 1.0}, 0.0, {7.5, 7.5}, sim::kPi * 0.5,
 hybrid);
 CHECK(plan.ok);
 CHECK(plan.path.size() > 8);
 CHECK(plan.path.front().arrival_direction == sim::TravelDirection::Unspecified);
 CHECK(std::hypot(plan.path.back().position.x - 7.5, plan.path.back().position.y - 7.5) < 1e-9);
 for (const auto& point : plan.path)
 CHECK(map.isFree(point.position.x, point.position.y));
 for (std::size_t point = 1; point < plan.path.size(); ++point) {
 CHECK(std::hypot(plan.path[point].position.x - plan.path[point - 1].position.x,
 plan.path[point].position.y - plan.path[point - 1].position.y) > 1e-9);
 }

 // A request already at its goal is a valid one-point route. It must not
 // append a duplicate zero-length terminal segment with an invented gear.
 const auto stationary_plan =
 sim::hybrid_astar_plan(map, {1.0, 1.0}, 0.0, {1.0, 1.0}, 0.0, hybrid);
 CHECK(stationary_plan.ok);
 CHECK(stationary_plan.path.size() == 1);
 CHECK(stationary_plan.path.front().arrival_direction == sim::TravelDirection::Unspecified);

 // A distinct goal inside the tolerance still has to search at least one
 // motion primitive, otherwise its terminal anchor would have no real gear.
 const auto nearby_plan =
 sim::hybrid_astar_plan(map, {1.0, 1.0}, 0.0, {1.1, 1.0}, 0.0, hybrid);
 CHECK(nearby_plan.ok);
 CHECK(nearby_plan.path.size() >= 2);
 CHECK(nearby_plan.path[1].arrival_direction != sim::TravelDirection::Unspecified);

 sim::AckermannLocalConfig local;
 local.wheel_base_m = 0.62;
 local.max_steer_rad = 0.55;
 local.max_steer_rate_radps = 2.0;
 local.max_speed_mps = 1.5;
 const auto trajectories = sim::ackermann_local_compute(
 {1.0, 1.0}, 0.0, 0.0, 0.0, {3.0, 2.0}, map, {}, {}, local);
 CHECK(trajectories.best.valid);
 CHECK(!trajectories.best_path.empty());
 CHECK(std::abs(trajectories.best.w) <= local.max_steer_rad + 1e-9);

 // A narrow backing task requires reverse primitives. Hybrid A* must retain
 // their gear in its typed path, and the local sampler must honor that gear.
 sim::GridMap backing_map(40, 12, 0.25);
 const auto backing_plan = sim::hybrid_astar_plan(
 backing_map, {7.5, 1.5}, 0.0, {2.0, 1.5}, 0.0, hybrid);
 CHECK(backing_plan.ok);
 CHECK(std::any_of(backing_plan.path.begin(), backing_plan.path.end(),
 [](const auto& point) { return point.arrival_direction == sim::TravelDirection::Reverse; }));
 sim::AckermannLocalConfig reverse_local = local;
 reverse_local.preferred_direction = sim::TravelDirection::Reverse;
 const auto reverse_trajectories = sim::ackermann_local_compute(
 {7.5, 1.5}, 0.0, 0.0, 0.0, {5.5, 1.8}, backing_map, {}, {}, reverse_local);
 CHECK(reverse_trajectories.best.valid);
 CHECK(reverse_trajectories.best.v <= 0.0);

 const auto mecanum = sim::mecanum_inverse(1.2, -0.4, 0.7, 0.6, 0.5);
 double recovered_forward = 0.0, recovered_left = 0.0, recovered_yaw = 0.0;
 sim::mecanum_forward(mecanum, 0.6, 0.5, recovered_forward, recovered_left, recovered_yaw);
 CHECK(std::abs(recovered_forward - 1.2) < 1e-9);
 CHECK(std::abs(recovered_left + 0.4) < 1e-9);
 CHECK(std::abs(recovered_yaw - 0.7) < 1e-9);

 const auto swerve = sim::swerve_inverse(0.0, 1.0, 0.0, 0.6, 0.5);
 for (const auto& module : swerve) {
 CHECK(std::abs(std::abs(module.speed_mps) - 1.0) < 1e-9);
 CHECK(std::abs(std::abs(module.steer_rad) - sim::kPi * 0.5) < 1e-9);
 }
 sim::HolonomicDrive drive({sim::HolonomicKind::Mecanum, 0.6, 0.5, 2.4, 4.0, 6.0, 26.0, 2.8});
 sim::DiffState state;
 sim::DiffCmd strafe;
 strafe.target_lateral_v = 1.0;
 for (int step = 0; step < 100; ++step)
 state = drive.step(state, strafe, 0.02, {});
 CHECK(state.y > 1.0);
 CHECK(std::abs(state.x) < 1e-9);
 return 0;
}
