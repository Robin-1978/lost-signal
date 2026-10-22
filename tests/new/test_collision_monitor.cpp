#include "sim/game/mission/systems/collision_monitor.hpp"

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
 sim::GridMap occupancy(100, 100, 0.1);
 occupancy.addBox(4.0, 4.0, 5.0, 5.0);
 mission::CollisionMonitor monitor({10.0, 10.0}, 0.5,
 {{{4.0, 4.0}, {5.0, 5.0}}});

 mission::CollisionObservation free;
 free.position = {2.0, 2.0};
 free.dt_s = 0.02;
 free.occupancy = &occupancy;
 const auto clear = monitor.assess(free);
 CHECK(!clear.static_collision);
 CHECK(std::abs(clear.static_clearance_m - 1.5) < 1e-12);

 // Continuous planner geometry expands authored boxes by the circular robot
 // footprint. A segment below the expanded box is free; touching or crossing
 // it is not. World boundaries obey the same footprint contract.
 CHECK(monitor.static_motion_free({2.0, 3.4}, {6.0, 3.4}));
 CHECK(!monitor.static_motion_free({2.0, 3.5}, {6.0, 3.5}));
 CHECK(!monitor.static_motion_free({3.0, 4.5}, {6.0, 4.5}));
 CHECK(!monitor.static_motion_free({0.49, 2.0}, {2.0, 2.0}));
 CHECK(monitor.static_motion_free({0.51, 2.0}, {2.0, 2.0}));

 // The execution reserve is a planner safety tube, not a larger physical
 // body. Outside it, crossing the reserve is refused. If estimation already
 // starts inside it, exact collision remains authoritative and only motion
 // that maintains or increases true clearance may recover.
 CHECK(!monitor.static_motion_free({2.0, 3.44}, {6.0, 3.44}, 0.10));
CHECK(monitor.static_motion_free({3.44, 4.5}, {3.43, 4.5}, 0.10));
CHECK(!monitor.static_motion_free({3.44, 4.5}, {3.45, 4.5}, 0.10));

 // A rotated rail keeps its real orientation in the continuous safety
 // oracle; points inside its world AABB corner are not false collisions.
 mission::CollisionMonitor rotated({10.0, 10.0}, 0.1,
 {{{4.0, 4.8}, {6.0, 5.2}, sim::kPi / 4.0}});
 mission::CollisionObservation rotated_corner;
 rotated_corner.position = {5.65, 4.65};
 CHECK(!rotated.assess(rotated_corner).static_collision);
 rotated_corner.position = {5.35, 5.35};
 CHECK(rotated.assess(rotated_corner).static_collision);

 auto domain_hit = free;
 domain_hit.position = {4.5, 4.5};
 CHECK(monitor.assess(domain_hit).static_collision);

// Raster inflation may occupy a diagonal cell even though the circular body
// remains clear of the authored box. Physical collision uses exact geometry;
// the planning grid stays conservative without becoming false ground truth.
auto clear_corner = free;
clear_corner.position = {5.4, 5.4};
CHECK(!monitor.assess(clear_corner).static_collision);

 auto actors = free;
 actors.actors = {
 {{{2.3, 2.0}, {}, 0.2}, true},
 {{{4.0, 2.0}, {}, 0.2}, false},
 };
 const auto social_only = monitor.assess(actors);
 CHECK(social_only.social_contacts == 1);
 CHECK(!social_only.dynamic_collision);
 CHECK(std::abs(social_only.dynamic_clearance_m + 0.4) < 1e-12);
 actors.actors[1].state.p = {2.4, 2.0};
 CHECK(monitor.assess(actors).dynamic_collision);

 // Raw LiDAR safety is a localization-independent stop field. Its input is
 // in base_link and never contains a map pose; invalid or stale data fails
 // closed instead of becoming free space.
 auto front_scan = [](double range_m) {
 sim::Scan scan;
 scan.n_rays = 1;
 scan.max_range = 6.0;
 scan.angle_min_rad = 0.0;
 scan.angle_increment_rad = sim::kPi * 0.5;
 scan.scan_period_s = 0.1;
 scan.ranges = {range_m};
 scan.hit = {1};
 return scan;
 };
 const auto front_safe = front_scan(2.0);
 sim::autonomy::LidarSafetyObservation lidar;
 lidar.scan = &front_safe;
 lidar.position_in_base = {0.20, 0.0};
 lidar.forward_speed_mps = 0.4;
 CHECK(!monitor.assess_lidar(lidar, {}).stop_required);
 CHECK(monitor.assess_lidar(lidar, {}).measurement_valid);

 const auto front_near = front_scan(0.5);
 lidar.scan = &front_near;
 lidar.forward_speed_mps = 1.0;
 const auto near = monitor.assess_lidar(lidar, {});
 CHECK(near.stop_required);
 CHECK(near.measurement_valid);
 CHECK(near.target_in_path);
 CHECK(near.forward_clearance_m < near.required_forward_clearance_m);

 lidar.measurement_fresh = false;
 CHECK(monitor.assess_lidar(lidar, {}).stop_required);
 lidar.measurement_fresh = true;
 lidar.scan = nullptr;
 CHECK(monitor.assess_lidar(lidar, {}).stop_required);

 // A return behind the forward sector is not allowed to inject a false
 // emergency stop merely because a spinning scanner also sees the rear.
 auto rear_scan = front_scan(0.30);
 rear_scan.angle_min_rad = sim::kPi;
 lidar.scan = &rear_scan;
 lidar.forward_speed_mps = 1.0;
 CHECK(!monitor.assess_lidar(lidar, {}).stop_required);

 return 0;
}
