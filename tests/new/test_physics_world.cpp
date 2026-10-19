#include "sim/hardware/physics/frame_transform.hpp"
#include "sim/hardware/physics/physics_world.hpp"
#include "sim/hardware/physics/raycast_geometry.hpp"
#include "sim/hardware/physics/tire_contact.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

void settle(sim::physics::MujocoSimulation& simulation, int steps = 200) {
 for (int step = 0; step < steps; ++step)
 simulation.step(0.005);
}

std::pair<double, double> mass_response() {
 using namespace sim::physics;
 MujocoSimulation simulation;
 BodyModel body;
 body.shape = ShapeKind::Box;
 body.half_extents_m = {0.35, 0.28, 0.20};
 body.mass_kg = 20.0;
 body.position_m = {0.0, 0.0, 100.0};
 body.friction = 0.0;
 const auto handle = simulation.add_body(body);
 simulation.compile();
 simulation.apply_drive(handle, {100.0, 0.0, 0.0, 0.0});
 simulation.step(0.1);
 const double light = simulation.state(handle).linear_velocity_mps.x;
 BodyState reset;
 reset.position_m = body.position_m;
 simulation.set_state(handle, reset);
 const double mass = 40.0;
 const double width = body.half_extents_m.x * 2.0;
 const double depth = body.half_extents_m.y * 2.0;
 const double height = body.half_extents_m.z * 2.0;
 simulation.set_mass_properties(handle, mass,
 {mass * (depth * depth + height * height) / 12.0,
 mass * (width * width + height * height) / 12.0,
 mass * (width * width + depth * depth) / 12.0});
 simulation.apply_drive(handle, {100.0, 0.0, 0.0, 0.0});
 simulation.step(0.1);
 return {light, simulation.state(handle).linear_velocity_mps.x};
}

double runtime_friction_response(double friction) {
 using namespace sim::physics;
 MujocoSimulation simulation;
 BodyModel floor;
 floor.shape = ShapeKind::Box;
 floor.half_extents_m = {5.0, 5.0, 0.1};
 floor.position_m = {0.0, 0.0, -0.1};
 floor.friction = 0.0;
 simulation.add_body(floor);
 BodyModel block;
 block.shape = ShapeKind::Box;
 block.half_extents_m = {0.2, 0.2, 0.2};
 block.position_m = {0.0, 0.0, 0.2};
 block.mass_kg = 2.0;
 block.friction = 0.0;
 // Tire/contact bodies have higher priority, so their runtime profile owns
 // the resulting pair instead of being max-mixed with the floor.
 block.contact_axle_axis_local = {0.0, 1.0, 0.0};
 const auto handle = simulation.add_body(block);
 simulation.compile();
 settle(simulation, 100);
 simulation.clear_velocities();
 simulation.set_contact_parameters(handle, friction, 0.0, 0.0, 0.006, 1.0);
 simulation.apply_drive(handle, {15.0, 0.0, 0.0, 0.0});
 simulation.step(0.05);
 return simulation.state(handle).linear_velocity_mps.x;
}

} // namespace

int main() {
 using namespace sim::physics;
 CHECK(mujoco_physics_available());

 // Closed finite-cylinder LiDAR proxy: side, cap, inside-origin, and miss
 // cases have independent analytic ranges. This helper is also used for
 // scan-time dynamic actor motion in planar and multi-line LiDAR.
 const BodyHandle analytic_cylinder{7};
 const auto side_hit = raycast_upright_finite_cylinder(
 {{-2.0, 0.0, 0.0}, {2.0, 0.0, 0.0}, 10.0}, analytic_cylinder,
 {0.0, 0.0, 0.0}, 1.0, 2.0);
 CHECK(side_hit.hit && side_hit.body == analytic_cylinder);
 CHECK(std::abs(side_hit.distance_m - 1.0) < 1e-12);
 CHECK(std::abs(side_hit.normal.x + 1.0) < 1e-12);
 const auto top_hit = raycast_upright_finite_cylinder(
 {{0.0, 0.0, 3.0}, {0.0, 0.0, -2.0}, 10.0}, analytic_cylinder,
 {0.0, 0.0, 0.0}, 1.0, 2.0);
 CHECK(top_hit.hit);
 CHECK(std::abs(top_hit.distance_m - 2.0) < 1e-12);
 CHECK(std::abs(top_hit.point_m.z - 1.0) < 1e-12);
 CHECK(std::abs(top_hit.normal.z - 1.0) < 1e-12);
 const auto inside_hit = raycast_upright_finite_cylinder(
 {{0.0, 0.0, 0.0}, {1.0, 0.0, 0.0}, 10.0}, analytic_cylinder,
 {0.0, 0.0, 0.0}, 1.0, 2.0);
 CHECK(inside_hit.hit && std::abs(inside_hit.distance_m - 1.0) < 1e-12);
 const auto cap_miss = raycast_upright_finite_cylinder(
 {{1.1, 0.0, 3.0}, {0.0, 0.0, -1.0}, 10.0}, analytic_cylinder,
 {0.0, 0.0, 0.0}, 1.0, 2.0);
 CHECK(!cap_miss.hit);

 // A runtime COM offset must affect moments without moving the body frame.
 // The same horizontal force through the body origin produces no pitch for
 // a centred inertia, but a non-zero pitch impulse when COM is 0.5 m above.
 {
 MujocoSimulation com_simulation;
 BodyModel centered_body;
 centered_body.shape = ShapeKind::Box;
 centered_body.half_extents_m = {0.2, 0.2, 0.2};
 centered_body.mass_kg = 2.0;
 centered_body.inertia_diagonal_kgm2 = {0.2, 0.2, 0.2};
 centered_body.position_m = {0.0, 0.0, 100.0};
 const auto centered_handle = com_simulation.add_body(centered_body);
 auto offset_body = centered_body;
 offset_body.position_m.x = 5.0;
 offset_body.movable_inertial_origin = true;
 const auto offset_handle = com_simulation.add_body(offset_body);
 com_simulation.compile();
 const auto offset_pose_before = com_simulation.state(offset_handle).position_m;
 com_simulation.set_mass_properties(
 offset_handle, 2.0, {0.2, 0.2, 0.2}, {0.0, 0.0, 0.5});
 const auto offset_pose_after = com_simulation.state(offset_handle).position_m;
 CHECK(std::abs(offset_pose_after.x - offset_pose_before.x) < 1e-12);
 CHECK(std::abs(offset_pose_after.y - offset_pose_before.y) < 1e-12);
 CHECK(std::abs(offset_pose_after.z - offset_pose_before.z) < 1e-12);
 com_simulation.apply_force_at_world_point(
 centered_handle, {10.0, 0.0, 0.0}, centered_body.position_m);
 com_simulation.apply_force_at_world_point(
 offset_handle, {10.0, 0.0, 0.0}, offset_body.position_m);
 com_simulation.step(0.01);
 CHECK(std::abs(com_simulation.state(centered_handle).angular_velocity_radps.y) < 1e-10);
 CHECK(std::abs(com_simulation.state(offset_handle).angular_velocity_radps.y) > 0.05);
 }

 TireContactSpec tire;
 tire.base_friction = 0.9;
 tire.spinning_resistance = 0.02;
 tire.radius_m = 0.1;
 tire.contact_time_constant_s = 0.006;
 tire.contact_damping_ratio = 1.0;
 tire.axle_axis_local = {0.0, 1.0, 0.0};
 const auto resolved = resolve_tire_contact(tire, {0.5, 0.03, 0.012, 1.2});
 CHECK(std::abs(resolved.friction - 0.45) < 1e-12);
CHECK(std::abs(resolved.rolling_friction_length_m - 0.003) < 1e-12);
 CHECK(std::abs(resolved.spinning_friction_length_m - 0.002) < 1e-12);
 CHECK(resolved.contact_time_constant_s == 0.012);
 CHECK(resolved.contact_damping_ratio == 1.2);

 FrameTransformTree frames;
 frames.set_transform("world", "base_link",
 {{1.0, 2.0, 0.0}, quaternion_from_euler(0.0, 0.0, 1.5707963267948966)});
 frames.set_transform("base_link", "lidar_link", {{1.0, 0.0, 0.4}, {}});
 const auto world_from_lidar = frames.lookup("world", "lidar_link");
 CHECK(world_from_lidar.has_value());
 CHECK(std::abs(world_from_lidar->translation_m.x - 1.0) < 1e-12);
 CHECK(std::abs(world_from_lidar->translation_m.y - 3.0) < 1e-12);

 MujocoSimulation simulation;
 CHECK(simulation.engine_name() == "mujoco");
 BodyModel ground;
 ground.shape = ShapeKind::Box;
 ground.half_extents_m = {10.0, 10.0, 0.1};
 ground.position_m = {0.0, 0.0, -0.1};
 ground.friction = 1.0;
 ground.collision_group = collision_layer::Terrain;
 ground.collision_mask = collision_layer::All;
 const auto ground_handle = simulation.add_body(ground);

 BodyModel body;
 body.shape = ShapeKind::Box;
 body.half_extents_m = {0.4, 0.3, 0.2};
 body.mass_kg = 20.0;
 body.position_m = {0.0, 0.0, 1.0};
 body.collision_group = collision_layer::Vehicle;
 body.collision_mask = collision_layer::All;
 const auto robot = simulation.add_body(body);
 simulation.compile();
 settle(simulation);
 const auto resting = simulation.state(robot);
 CHECK(resting.position_m.z > 0.18);
 CHECK(resting.position_m.z < 0.23);
 CHECK(!simulation.contacts().empty());
 bool checked_contact_normal = false;
 for (const auto& contact : simulation.contacts()) {
 if (contact.body_a == ground_handle && contact.body_b == robot) {
 CHECK(contact.normal_on_b.z > 0.9);
 checked_contact_normal = true;
 } else if (contact.body_a == robot && contact.body_b == ground_handle) {
 CHECK(contact.normal_on_b.z < -0.9);
 checked_contact_normal = true;
 }
 }
 CHECK(checked_contact_normal);
 const auto floor_hit = simulation.raycast(
 {{2.0, 2.0, 2.0}, {0.0, 0.0, -1.0}, 3.0, collision_layer::Terrain});
 CHECK(floor_hit.hit);
 CHECK(floor_hit.body == ground_handle);
 CHECK(floor_hit.distance_m > 1.9 && floor_hit.distance_m < 2.1);
 CHECK(floor_hit.normal.z > 0.9);

 BodyModel target;
 target.shape = ShapeKind::Box;
 target.half_extents_m = {0.25, 0.25, 0.25};
 target.position_m = {3.0, 0.0, 0.25};
 target.collision_group = collision_layer::StaticObstacle;
 target.collision_mask = collision_layer::All;
 const auto target_handle = simulation.add_body(target);
 simulation.compile();
 CHECK(std::abs(simulation.state(robot).position_m.z - resting.position_m.z) < 1e-6);
 const auto target_hit = simulation.raycast(
 {{0.0, 0.0, 0.25}, {1.0, 0.0, 0.0}, 5.0,
 collision_layer::StaticObstacle, robot});
 CHECK(target_hit.hit);
 CHECK(target_hit.body == target_handle);
 CHECK(target_hit.distance_m > 2.7 && target_hit.distance_m < 2.8);
 CHECK(target_hit.normal.x < -0.9);
 const auto terrain_only = simulation.raycast(
 {{0.0, 0.0, 0.25}, {1.0, 0.0, 0.0}, 5.0,
 collision_layer::Terrain, robot});
 CHECK(!terrain_only.hit);

 BodyModel actor;
 actor.shape = ShapeKind::Cylinder;
 actor.radius_m = 0.3;
 actor.height_m = 1.0;
 actor.position_m = {4.0, 2.0, 0.5};
 actor.kinematic = true;
 actor.collision_group = collision_layer::DynamicActor;
 actor.collision_mask = collision_layer::All;
 const auto actor_handle = simulation.add_body(actor);
 simulation.compile();
 auto actor_state = simulation.state(actor_handle);
 actor_state.position_m.x = 5.0;
 simulation.set_state(actor_handle, actor_state);
 CHECK(std::abs(simulation.state(actor_handle).position_m.x - 5.0) < 1e-12);

 // A prescribed actor is a dynamic free-joint proxy whose pose and velocity
 // are refreshed by the world provider. Its velocity must participate in the
 // contact Jacobian; a moving actor therefore transfers momentum instead of
 // behaving like a teleported zero-velocity mocap geom.
 MujocoSimulation moving_actor_simulation;
 BodyModel actor_floor = ground;
 actor_floor.half_extents_m = {5.0, 5.0, 0.1};
 actor_floor.friction = 0.0;
 moving_actor_simulation.add_body(actor_floor);
 BodyModel pusher;
 pusher.shape = ShapeKind::Cylinder;
 pusher.radius_m = 0.3;
 pusher.height_m = 0.4;
 pusher.position_m = {-0.8, 0.0, 0.2};
 pusher.kinematic = true;
 pusher.friction = 0.0;
 const auto pusher_handle = moving_actor_simulation.add_body(pusher);
 BodyModel pushed;
 pushed.shape = ShapeKind::Box;
 pushed.half_extents_m = {0.2, 0.2, 0.2};
 pushed.position_m = {0.0, 0.0, 0.2};
 pushed.mass_kg = 2.0;
 pushed.friction = 0.0;
 const auto pushed_handle = moving_actor_simulation.add_body(pushed);
 moving_actor_simulation.compile();
 BodyState pusher_state = moving_actor_simulation.state(pusher_handle);
 pusher_state.linear_velocity_mps = {2.0, 0.0, 0.0};
 moving_actor_simulation.set_state(pusher_handle, pusher_state);
 moving_actor_simulation.step(0.3);
 CHECK(moving_actor_simulation.state(pusher_handle).position_m.x > -0.35);
 CHECK(moving_actor_simulation.state(pushed_handle).linear_velocity_mps.x > 0.1);

 MujocoSimulation heightfield_simulation;
 BodyModel heightfield;
 heightfield.shape = ShapeKind::HeightField;
 heightfield.half_extents_m = {1.0, 1.0, 0.0};
 heightfield.heightfield_rows = 3;
 heightfield.heightfield_cols = 3;
 heightfield.heightfield_elevation_m = {0.0, 0.0, 0.0,
 0.0, 0.5, 0.0,
 0.0, 0.0, 0.0};
 heightfield.heightfield_base_depth_m = 0.2;
 heightfield.collision_group = collision_layer::Terrain;
 const auto heightfield_handle = heightfield_simulation.add_body(heightfield);
 heightfield_simulation.compile();
 const auto hill_hit = heightfield_simulation.raycast(
 {{0.0, 0.0, 2.0}, {0.0, 0.0, -1.0}, 3.0, collision_layer::Terrain});
 CHECK(hill_hit.hit);
 CHECK(hill_hit.body == heightfield_handle);
 CHECK(hill_hit.point_m.z > 0.45 && hill_hit.point_m.z < 0.55);

 MujocoSimulation slope_heightfield_simulation;
 BodyModel slope_heightfield = heightfield;
 slope_heightfield.heightfield_elevation_m = {
 0.0, 0.0, 0.0,
 0.25, 0.25, 0.25,
 0.5, 0.5, 0.5,
 };
 const auto slope_heightfield_handle = slope_heightfield_simulation.add_body(slope_heightfield);
 slope_heightfield_simulation.compile();
 const auto high_y_hit = slope_heightfield_simulation.raycast(
 {{0.0, 0.9, 2.0}, {0.0, 0.0, -1.0}, 3.0, collision_layer::Terrain});
 const auto low_y_hit = slope_heightfield_simulation.raycast(
 {{0.0, -0.9, 2.0}, {0.0, 0.0, -1.0}, 3.0, collision_layer::Terrain});
 CHECK(high_y_hit.hit && low_y_hit.hit);
 CHECK(high_y_hit.body == slope_heightfield_handle);
 CHECK(high_y_hit.point_m.z > low_y_hit.point_m.z + 0.4);

 MujocoSimulation articulation;
 BodyModel base;
 base.shape = ShapeKind::Box;
 base.half_extents_m = {0.5, 0.2, 0.2};
 base.mass_kg = 10.0;
 base.position_m = {0.0, 0.0, 2.0};
 const auto base_handle = articulation.add_body(base);
 BodyModel link = base;
 link.half_extents_m = {0.5, 0.1, 0.1};
 link.mass_kg = 1.0;
 link.position_m = {1.0, 0.0, 2.0};
 const auto link_handle = articulation.add_body(link);
 JointModel hinge;
 hinge.kind = JointKind::Revolute;
 hinge.parent = base_handle;
 hinge.child = link_handle;
 hinge.pivot_in_parent_m = {0.5, 0.0, 0.0};
 hinge.pivot_in_child_m = {-0.5, 0.0, 0.0};
 hinge.axis_in_parent = {0.0, 0.0, 1.0};
 hinge.axis_in_child = {0.0, 0.0, 1.0};
 hinge.armature_inertia_kgm2 = 0.02;
 const auto hinge_handle = articulation.add_joint(hinge);
 articulation.compile();
 for (int step = 0; step < 100; ++step) {
 articulation.apply_joint_effort(hinge_handle, 1.0);
 articulation.step(0.005);
 }
 CHECK(articulation.joint_state(hinge_handle).velocity_radps > 0.1);

 MujocoSimulation limited_articulation;
 const auto limited_base = limited_articulation.add_body(base);
 const auto limited_link = limited_articulation.add_body(link);
 JointModel limited_hinge = hinge;
 limited_hinge.parent = limited_base;
 limited_hinge.child = limited_link;
 limited_hinge.has_limits = true;
 limited_hinge.lower_limit_rad = -0.2;
 limited_hinge.upper_limit_rad = 0.2;
 const auto limited_hinge_handle = limited_articulation.add_joint(limited_hinge);
 limited_articulation.compile();
 for (int step = 0; step < 500; ++step) {
 limited_articulation.apply_joint_effort(limited_hinge_handle, 1.0);
 limited_articulation.step(0.002);
 }
 CHECK(limited_articulation.joint_state(limited_hinge_handle).position_rad > 0.15);
 CHECK(limited_articulation.joint_state(limited_hinge_handle).position_rad < 0.205);

 const auto [light, heavy] = mass_response();
 CHECK(light > 0.0);
 CHECK(heavy > light * 0.4);
 CHECK(heavy < light * 0.6);

 // Updating a compiled wheel/contact geom is a current-step operation. A
 // zero-friction block must accelerate under a force below the high-friction
 // Coulomb limit, while the high-friction block remains effectively stuck.
 const double low_friction_speed = runtime_friction_response(0.0);
 const double high_friction_speed = runtime_friction_response(2.0);
 CHECK(low_friction_speed > 0.25);
 CHECK(std::abs(high_friction_speed) < 0.03);

 // MuJoCo stores a free joint's rotational qvel in the body frame. The
 // project contract is world-frame BodyState, so a rotated body must round
 // trip an arbitrary angular velocity without rotating the value a second
 // time. Recompilation exercises the same conversion in state preservation.
 MujocoSimulation state_contract;
 BodyModel spinning_body;
 spinning_body.shape = ShapeKind::Box;
 spinning_body.half_extents_m = {0.2, 0.3, 0.4};
 spinning_body.mass_kg = 2.0;
 spinning_body.position_m = {0.0, 0.0, 100.0};
 spinning_body.rotation = quaternion_from_euler(0.4, -0.3, 1.1);
 const auto spinning_handle = state_contract.add_body(spinning_body);
 state_contract.compile();
 bool incomplete_inertia_rejected = false;
 try {
 state_contract.set_mass_properties(spinning_handle, 3.0, {});
 } catch (const std::invalid_argument&) {
 incomplete_inertia_rejected = true;
 }
 CHECK(incomplete_inertia_rejected);
 BodyState authored_state;
 authored_state.position_m = spinning_body.position_m;
 authored_state.rotation = spinning_body.rotation;
 authored_state.linear_velocity_mps = {1.0, -2.0, 0.5};
 authored_state.angular_velocity_radps = {0.7, -0.4, 1.2};
 state_contract.set_state(spinning_handle, authored_state);
 const auto immediate_state = state_contract.state(spinning_handle);
 CHECK(std::abs(immediate_state.angular_velocity_radps.x - 0.7) < 1e-10);
 CHECK(std::abs(immediate_state.angular_velocity_radps.y + 0.4) < 1e-10);
 CHECK(std::abs(immediate_state.angular_velocity_radps.z - 1.2) < 1e-10);
 // Re-identifying mass properties at runtime must preserve the live state.
 // MuJoCo's mj_setConst temporarily evaluates qpos0, so this guards against
 // payload changes teleporting a vehicle back to its spawn pose.
 state_contract.set_mass_properties(spinning_handle, 3.0, {0.4, 0.5, 0.6});
 const auto mass_updated_state = state_contract.state(spinning_handle);
 CHECK(std::abs(mass_updated_state.position_m.x - authored_state.position_m.x) < 1e-10);
 CHECK(std::abs(mass_updated_state.position_m.y - authored_state.position_m.y) < 1e-10);
 CHECK(std::abs(mass_updated_state.position_m.z - authored_state.position_m.z) < 1e-10);
 CHECK(std::abs(mass_updated_state.linear_velocity_mps.x - 1.0) < 1e-10);
 CHECK(std::abs(mass_updated_state.linear_velocity_mps.y + 2.0) < 1e-10);
 CHECK(std::abs(mass_updated_state.linear_velocity_mps.z - 0.5) < 1e-10);
 CHECK(std::abs(mass_updated_state.angular_velocity_radps.x - 0.7) < 1e-10);
 CHECK(std::abs(mass_updated_state.angular_velocity_radps.y + 0.4) < 1e-10);
 CHECK(std::abs(mass_updated_state.angular_velocity_radps.z - 1.2) < 1e-10);
 BodyModel remote_static;
 remote_static.shape = ShapeKind::Sphere;
 remote_static.radius_m = 0.1;
 remote_static.position_m = {100.0, 100.0, 100.0};
 state_contract.add_body(remote_static);
 state_contract.compile();
 const auto restored_state = state_contract.state(spinning_handle);
 CHECK(std::abs(restored_state.angular_velocity_radps.x - 0.7) < 1e-10);
 CHECK(std::abs(restored_state.angular_velocity_radps.y + 0.4) < 1e-10);
 CHECK(std::abs(restored_state.angular_velocity_radps.z - 1.2) < 1e-10);

 // Public state and ray queries must describe the end of the requested step,
 // not the beginning of MuJoCo's final internal substep.
 authored_state.angular_velocity_radps = {};
 authored_state.linear_velocity_mps = {1.0, 0.0, 0.0};
 state_contract.set_state(spinning_handle, authored_state);
 state_contract.step(0.001);
 CHECK(std::abs(state_contract.state(spinning_handle).position_m.x - 0.001) < 1e-8);

 // A revolute joint has one physical axis. Parent- and child-frame
 // declarations that do not map to the same world direction are invalid.
 bool mismatched_axis_rejected = false;
 try {
 MujocoSimulation bad_joint;
 BodyModel bad_parent = base;
 bad_parent.position_m = {0.0, 0.0, 10.0};
 const auto bad_parent_handle = bad_joint.add_body(bad_parent);
 BodyModel bad_child = link;
 bad_child.position_m = {1.0, 0.0, 10.0};
 const auto bad_child_handle = bad_joint.add_body(bad_child);
 JointModel mismatch = hinge;
 mismatch.parent = bad_parent_handle;
 mismatch.child = bad_child_handle;
 mismatch.axis_in_child = {0.0, 1.0, 0.0};
 bad_joint.add_joint(mismatch);
 bad_joint.compile();
 } catch (const std::invalid_argument&) {
 mismatched_axis_rejected = true;
 }
 CHECK(mismatched_axis_rejected);

 bool impossible_inertia_rejected = false;
 try {
 BodyModel impossible = spinning_body;
 impossible.inertia_diagonal_kgm2 = {1.0, 1.0, 3.0};
 MujocoSimulation bad_inertia;
 bad_inertia.add_body(impossible);
 } catch (const std::invalid_argument&) {
 impossible_inertia_rejected = true;
 }
 CHECK(impossible_inertia_rejected);

 bool invalid_rejected = false;
 try {
 (void)simulation.state({999});
 } catch (const std::out_of_range&) {
 invalid_rejected = true;
 }
 CHECK(invalid_rejected);
 return 0;
}
