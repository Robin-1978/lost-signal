#include "algo/ackermann_local.hpp"
#include "algo/hybrid_astar.hpp"
#include "hw/chassis/holonomic.hpp"

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
 CHECK(std::hypot(plan.path.back().x - 7.5, plan.path.back().y - 7.5) < 1e-9);
 for (const auto& point : plan.path)
 CHECK(map.isFree(point.x, point.y));

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
