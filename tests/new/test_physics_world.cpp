#include "physics/physics_world.hpp"
#include "physics/frame_transform.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace {

void settle(sim::physics::IPhysicsWorld& world, int steps = 100) {
 for (int step = 0; step < steps; ++step) {
  world.step(0.01);
 }
}

double climb_displacement(double angle_rad, double drive_force_N) {
 using namespace sim::physics;
 auto world = make_bullet_physics_world();
 constexpr double half_length = 6.0;
 constexpr double thickness = 0.10;
 BodySpec ramp;
 ramp.shape = ShapeKind::Box;
 ramp.half_extents_m = {half_length, 2.0, thickness};
 ramp.position_m = {0.0, 0.0, half_length * std::sin(angle_rad) - thickness * std::cos(angle_rad)};
 ramp.rotation = quaternion_from_euler(0.0, -angle_rad, 0.0);
 ramp.friction = 0.0;
 (void)world->spawn(ramp);

 const Vec3 uphill{std::cos(angle_rad), 0.0, std::sin(angle_rad)};
 const Vec3 normal{-std::sin(angle_rad), 0.0, std::cos(angle_rad)};
 constexpr double local_x = -4.0;
 BodySpec body;
 body.shape = ShapeKind::Box;
 body.half_extents_m = {0.35, 0.28, 0.20};
 body.mass_kg = 20.0;
 body.friction = 0.0;
 body.rotation = ramp.rotation;
 body.position_m = {
   ramp.position_m.x + uphill.x * local_x + normal.x * (thickness + body.half_extents_m.z + 0.01),
   0.0,
   ramp.position_m.z + uphill.z * local_x + normal.z * (thickness + body.half_extents_m.z + 0.01),
 };
 const BodyHandle robot = world->spawn(body);
 const BodyState start = world->state(robot);
 for (int step = 0; step < 300; ++step) {
  world->apply_drive(robot, {drive_force_N, 0.0, 0.0, 0.0});
  world->step(0.01);
 }
 const BodyState finish = world->state(robot);
 return (finish.position_m.x - start.position_m.x) * uphill.x +
        (finish.position_m.z - start.position_m.z) * uphill.z;
}

double braking_distance(double speed_mps, double brake_force_N) {
 using namespace sim::physics;
 auto world = make_bullet_physics_world();
 BodySpec ground;
 ground.shape = ShapeKind::Box;
 ground.half_extents_m = {20.0, 4.0, 0.1};
 ground.position_m = {0.0, 0.0, -0.1};
 ground.friction = 0.0;
 (void)world->spawn(ground);
 BodySpec body;
 body.shape = ShapeKind::Box;
 body.half_extents_m = {0.35, 0.28, 0.20};
 body.mass_kg = 20.0;
 body.position_m = {0.0, 0.0, 0.2};
 body.friction = 0.0;
 const BodyHandle robot = world->spawn(body);
 BodyState state = world->state(robot);
 state.linear_velocity_mps.x = speed_mps;
 world->set_state(robot, state);
 const double start_x = state.position_m.x;
 for (int step = 0; step < 1000; ++step) {
  world->apply_drive(robot, {0.0, 0.0, 0.0, brake_force_N});
  world->step(0.005);
  if (world->state(robot).linear_velocity_mps.x <= 0.0) {
   break;
  }
 }
 return world->state(robot).position_m.x - start_x;
}

} // namespace

int main() {
 using namespace sim::physics;

 FrameTransformTree frames;
 frames.set_transform("world", "base_link",
 {{1.0, 2.0, 0.0}, quaternion_from_euler(0.0, 0.0, 3.14159265358979323846 * 0.5)});
 frames.set_transform("base_link", "lidar_link", {{1.0, 0.0, 0.4}, {}});
 const auto world_from_lidar = frames.lookup("world", "lidar_link");
 CHECK(world_from_lidar.has_value());
 CHECK(std::abs(world_from_lidar->translation_m.x - 1.0) < 1e-12);
 CHECK(std::abs(world_from_lidar->translation_m.y - 3.0) < 1e-12);
 CHECK(std::abs(world_from_lidar->translation_m.z - 0.4) < 1e-12);
 const auto lidar_from_world = frames.lookup("lidar_link", "world");
 CHECK(lidar_from_world.has_value());
 const auto round_trip = transform_point(*lidar_from_world, world_from_lidar->translation_m);
 CHECK(std::abs(round_trip.x) < 1e-12);
 CHECK(std::abs(round_trip.y) < 1e-12);
 CHECK(std::abs(round_trip.z) < 1e-12);

 auto planar = make_planar_physics_world();
 CHECK(planar->backend_name() == "planar");
 BodySpec body;
 body.shape = ShapeKind::Box;
 body.half_extents_m = {0.4, 0.3, 0.2};
 body.mass_kg = 20.0;
 body.position_m = {0.0, 0.0, 1.0};
 const BodyHandle robot = planar->spawn(body);
 settle(*planar);
 CHECK(std::abs(planar->state(robot).position_m.z - 0.2) < 1e-9);
 planar->apply_drive(robot, {40.0, 0.0, 2.0, 0.0});
 planar->step(0.1);
 CHECK(planar->state(robot).linear_velocity_mps.x > 0.0);
 CHECK(planar->state(robot).angular_velocity_radps.z > 0.0);
 const auto floor_hit = planar->raycast({{0.0, 0.0, 2.0}, {0.0, 0.0, -1.0}, 3.0});
 CHECK(floor_hit.hit);
 CHECK(std::abs(floor_hit.distance_m - 2.0) < 1e-12);

 bool invalid_handle_rejected = false;
 try {
  (void)planar->state({999});
 } catch (const std::out_of_range&) {
  invalid_handle_rejected = true;
 }
 CHECK(invalid_handle_rejected);

 CHECK(bullet_physics_available());
  auto bullet = make_bullet_physics_world();
  CHECK(bullet->backend_name() == "bullet");
  BodySpec ground;
  ground.shape = ShapeKind::Box;
  ground.half_extents_m = {10.0, 10.0, 0.1};
  ground.position_m = {0.0, 0.0, -0.1};
 ground.collision_group = collision_layer::Terrain;
  (void)bullet->spawn(ground);
  const BodyHandle dynamic = bullet->spawn(body);
  settle(*bullet, 200);
  const auto state = bullet->state(dynamic);
  CHECK(state.position_m.z > 0.18);
  CHECK(state.position_m.z < 0.23);
  CHECK(!bullet->contacts().empty());
  const auto hit = bullet->raycast({{0.0, 0.0, 2.0}, {0.0, 0.0, -1.0}, 3.0});
  CHECK(hit.hit);
  CHECK(hit.distance_m > 1.5);
  CHECK(hit.distance_m < 1.7);

 BodySpec sensor_target;
 sensor_target.shape = ShapeKind::Box;
 sensor_target.half_extents_m = {0.25, 0.25, 0.25};
 sensor_target.position_m = {3.0, 0.0, 0.25};
 sensor_target.collision_group = collision_layer::StaticObstacle;
 const BodyHandle target = bullet->spawn(sensor_target);
 const auto self_filtered = bullet->raycast({{0.0, 0.0, 0.2}, {1.0, 0.0, 0.0}, 5.0,
 collision_layer::All, dynamic});
 CHECK(self_filtered.hit);
 CHECK(self_filtered.body == target);
 CHECK(self_filtered.distance_m > 2.7 && self_filtered.distance_m < 2.8);
 const auto terrain_only = bullet->raycast({{0.0, 0.0, 0.2}, {1.0, 0.0, 0.0}, 5.0,
 collision_layer::Terrain, dynamic});
 CHECK(!terrain_only.hit);

  const double standard_ten = climb_displacement(10.0 * 3.14159265358979323846 / 180.0, 40.0);
  const double standard_twenty = climb_displacement(20.0 * 3.14159265358979323846 / 180.0, 40.0);
  const double torque_twenty = climb_displacement(20.0 * 3.14159265358979323846 / 180.0, 90.0);
  CHECK(standard_ten > 0.0);
  CHECK(standard_twenty < standard_ten);
  CHECK(torque_twenty > standard_twenty + 0.5);
  CHECK(std::abs(climb_displacement(20.0 * 3.14159265358979323846 / 180.0, 90.0) - torque_twenty) < 1e-7);

  const double slow_braking = braking_distance(1.0, 40.0);
  const double fast_braking = braking_distance(2.0, 40.0);
  const double grippy_braking = braking_distance(2.0, 80.0);
  CHECK(fast_braking > slow_braking);
  CHECK(grippy_braking < fast_braking);

  auto crowd = make_bullet_physics_world();
  (void)crowd->spawn(ground);
  for (int index = 0; index < 100; ++index) {
   BodySpec item = body;
   item.position_m = {
     static_cast<double>(index % 10),
     static_cast<double>(index / 10),
     0.5 + static_cast<double>(index / 20),
   };
   (void)crowd->spawn(item);
  }
 settle(*crowd, 120);
 CHECK(crowd->contacts().size() > 50);
 return 0;
}
