#include "sim/game/platform/simulated_platform.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace sim::platform {

using mission::MountDefinition;
using mission::active_in_level;
namespace {

constexpr double kGravityMps2 = 9.81;

double patch_height(const TerrainPatchDefinition& patch, double x, double y) {
 const double plane = patch.base_height_m + patch.grade_x * (x - patch.min.x) +
 patch.grade_y * (y - patch.min.y);
 if (patch.roughness <= 0.0) {
 return plane;
 }
 const double width = std::max(0.1, patch.max.x - patch.min.x);
 const double depth = std::max(0.1, patch.max.y - patch.min.y);
 const double u = std::clamp((x - patch.min.x) / width, 0.0, 1.0);
 const double v = std::clamp((y - patch.min.y) / depth, 0.0, 1.0);
 const double edge_fade = std::sin(::sim::kPi * u) * std::sin(::sim::kPi * v);
 const double wave = std::sin(2.0 * ::sim::kPi * (x - patch.min.x) / 1.35) *
 std::sin(2.0 * ::sim::kPi * (y - patch.min.y) / 0.95);
 return plane + patch.roughness * edge_fade * wave;
}

bool same(::sim::physics::BodyHandle a, ::sim::physics::BodyHandle b) {
 return a && b && a == b;
}

::sim::physics::Vec3 subtract(::sim::physics::Vec3 lhs, ::sim::physics::Vec3 rhs) {
 return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

::sim::physics::Vec3 add(::sim::physics::Vec3 lhs, ::sim::physics::Vec3 rhs) {
 return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

::sim::physics::Vec3 scale(::sim::physics::Vec3 value, double factor) {
 return {value.x * factor, value.y * factor, value.z * factor};
}

::sim::physics::Vec3 cross(::sim::physics::Vec3 lhs, ::sim::physics::Vec3 rhs) {
 return {lhs.y * rhs.z - lhs.z * rhs.y,
 lhs.z * rhs.x - lhs.x * rhs.z,
 lhs.x * rhs.y - lhs.y * rhs.x};
}

double magnitude(::sim::physics::Vec3 value) {
 return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

} // namespace

SimulatedPlatform::SimulatedPlatform(const MapDefinition& map, const LevelDefinition& level,
 const RobotDefinition& robot, const ::sim::ChassisCaps& chassis_caps,
 const ::sim::TerrainModel& terrain, ::sim::Vec2 spawn,
 double spawn_heading_rad, double total_mass_kg, double center_height_m,
 double yaw_inertia_kgm2)
 : map_(map), level_(level), robot_(robot), terrain_(terrain),
 world_(::sim::physics::make_bullet_physics_world()),
 tire_contact_(::sim::physics::make_bullet_tire_contact()),
 total_mass_kg_(std::max(0.5, total_mass_kg)),
 body_center_height_m_(std::max(0.05, robot.base_link_height_m)),
 center_height_m_(std::max(0.05, center_height_m)),
 yaw_inertia_kgm2_(std::max(0.05, yaw_inertia_kgm2)) {
 support_width_m_ = std::max(0.05, chassis_caps.geometry.support_width_m);
 support_length_m_ = std::max(0.05, chassis_caps.geometry.support_length_m);
 drive_axle_x_m_ = chassis_caps.geometry.drive_axle_x_m;

 ::sim::physics::BodySpec floor;
 floor.shape = ::sim::physics::ShapeKind::Box;
 floor.half_extents_m = {map_.size_m.x * 0.5, map_.size_m.y * 0.5, 0.10};
 floor.position_m = {map_.size_m.x * 0.5, map_.size_m.y * 0.5, -0.10};
 floor.friction = 1.0;
 floor.collision_group = ::sim::physics::collision_layer::Terrain;
 floor.collision_mask = static_cast<::sim::physics::CollisionMask>(
 ::sim::physics::collision_layer::Default | ::sim::physics::collision_layer::Vehicle);
 terrain_bodies_.push_back(world_->spawn(floor));

 // Keep an authoritative rigid body inside the authored world. The domain
 // grid already rejects out-of-bounds poses; Bullet needs the same boundary
 // represented as contact geometry instead of allowing a long free fall.
 const double wall_height = 2.5;
 const double wall_thickness = 0.10;
 const ::sim::physics::Vec3 wall_half_x{map_.size_m.x * 0.5 + wall_thickness,
 wall_thickness, wall_height};
 const ::sim::physics::Vec3 wall_half_y{wall_thickness,
 map_.size_m.y * 0.5 + wall_thickness, wall_height};
 const std::pair<::sim::physics::Vec3, ::sim::physics::Vec3> walls[] = {
 {wall_half_x, {map_.size_m.x * 0.5, -wall_thickness, wall_height}},
 {wall_half_x, {map_.size_m.x * 0.5, map_.size_m.y + wall_thickness, wall_height}},
 {wall_half_y, {-wall_thickness, map_.size_m.y * 0.5, wall_height}},
 {wall_half_y, {map_.size_m.x + wall_thickness, map_.size_m.y * 0.5, wall_height}},
 };
 for (const auto& [half_extents, position] : walls) {
 ::sim::physics::BodySpec wall;
 wall.shape = ::sim::physics::ShapeKind::Box;
 wall.half_extents_m = half_extents;
 wall.position_m = position;
 wall.friction = 0.2;
 wall.collision_group = ::sim::physics::collision_layer::StaticObstacle;
 wall.collision_mask = static_cast<::sim::physics::CollisionMask>(
 ::sim::physics::collision_layer::Default | ::sim::physics::collision_layer::Vehicle);
 obstacle_bodies_.push_back(world_->spawn(wall));
 }

 for (const auto& patch : map_.terrain) {
 if (active_in_level(patch.active_in, level_.id)) {
 terrain_bodies_.push_back(world_->spawn(terrain_mesh(patch)));
 }
 }
 for (const auto& obstacle : map_.obstacles) {
 if (!active_in_level(obstacle.active_in, level_.id)) {
 continue;
 }
 ::sim::physics::BodySpec body;
 body.shape = ::sim::physics::ShapeKind::Box;
 body.half_extents_m = {(obstacle.max.x - obstacle.min.x) * 0.5,
 (obstacle.max.y - obstacle.min.y) * 0.5, 0.75};
 body.position_m = {(obstacle.min.x + obstacle.max.x) * 0.5,
 (obstacle.min.y + obstacle.max.y) * 0.5, 0.75};
 body.friction = 0.2;
 body.collision_group = ::sim::physics::collision_layer::StaticObstacle;
 body.collision_mask = static_cast<::sim::physics::CollisionMask>(
 ::sim::physics::collision_layer::Default | ::sim::physics::collision_layer::Vehicle);
 obstacle_bodies_.push_back(world_->spawn(body));
 }

 const auto ground = terrain_.sample(spawn);
 const double forward_grade = ::sim::longitudinal_grade(ground, spawn_heading_rad);
 const double cross_grade = ::sim::lateral_grade(ground, spawn_heading_rad);
 const double roll = std::atan(cross_grade);
 const double pitch = -std::atan(forward_grade);
 constexpr double kDegreesToRadians = ::sim::kPi / 180.0;
 const auto robot_rotation = ::sim::physics::quaternion_from_euler(roll, pitch, spawn_heading_rad);
 const ::sim::physics::RigidTransform world_from_robot{
 {spawn.x, spawn.y, ground.height_m + 0.01}, robot_rotation};
 const auto vehicle_mask = static_cast<::sim::physics::CollisionMask>(
 ::sim::physics::collision_layer::Default | ::sim::physics::collision_layer::Terrain |
 ::sim::physics::collision_layer::StaticObstacle | ::sim::physics::collision_layer::DynamicActor);
 if (!robot_.articulation.links.empty()) {
 articulated_ = true;
 for (const auto& link : robot_.articulation.links) {
 ::sim::physics::BodySpec body;
 if (link.shape == "box") {
 body.shape = ::sim::physics::ShapeKind::Box;
 body.half_extents_m = {link.size_m.x * 0.5, link.size_m.y * 0.5, link.size_m.z * 0.5};
 } else if (link.shape == "cylinder") {
 body.shape = ::sim::physics::ShapeKind::Cylinder;
 body.radius_m = link.radius_m;
 body.height_m = link.length_m;
 } else {
 throw std::runtime_error("unsupported articulated link collision shape '" + link.shape + "'");
 }
 body.mass_kg = link.mass_kg;
 body.inertia_diagonal_kgm2 = {link.inertia_diagonal_kgm2.x,
 link.inertia_diagonal_kgm2.y, link.inertia_diagonal_kgm2.z};
 for (const auto& actuator : robot_.articulation.actuators) {
 const auto joint = std::find_if(robot_.articulation.joints.begin(),
 robot_.articulation.joints.end(),
 [&](const auto& candidate) { return candidate.id == actuator.joint; });
 if (joint == robot_.articulation.joints.end() || joint->child != link.id ||
 actuator.armature_inertia_kgm2 <= 0.0)
 continue;
 const double axial = actuator.armature_inertia_kgm2;
 const double transverse = axial * 0.5;
 const double ax = std::abs(joint->axis_in_child.x);
 const double ay = std::abs(joint->axis_in_child.y);
 const double az = std::abs(joint->axis_in_child.z);
 body.inertia_diagonal_kgm2.x += ax > 0.5 ? axial : transverse;
 body.inertia_diagonal_kgm2.y += ay > 0.5 ? axial : transverse;
 body.inertia_diagonal_kgm2.z += az > 0.5 ? axial : transverse;
 }
 if (link.id == robot_.articulation.root_link) {
 articulated_root_mass_kg_ = link.mass_kg;
 articulated_root_inertia_kgm2_ = body.inertia_diagonal_kgm2;
 const double payload_mass_kg = std::max(0.0, total_mass_kg_ - robot_.mass_kg);
 const double payload_yaw_inertia = std::max(0.0, yaw_inertia_kgm2_ - robot_.yaw_inertia_kgm2);
 const double payload_height_m = payload_mass_kg > 1e-9
 ? (center_height_m_ * total_mass_kg_ -
 robot_.mass_kg * robot_.center_of_mass_height_m) / payload_mass_kg
 : robot_.base_link_height_m;
 const double vertical_offset_m = payload_height_m - robot_.base_link_height_m;
 const double payload_transverse_inertia =
 0.5 * payload_yaw_inertia + payload_mass_kg * vertical_offset_m * vertical_offset_m;
 body.mass_kg += payload_mass_kg;
 body.inertia_diagonal_kgm2.x += payload_transverse_inertia;
 body.inertia_diagonal_kgm2.y += payload_transverse_inertia;
 body.inertia_diagonal_kgm2.z += payload_yaw_inertia;
 } else {
 articulated_non_root_mass_kg_ += link.mass_kg;
 }
 body.friction = link.friction;
 body.rolling_friction = link.rolling_support_moment_arm_m;
 body.spinning_friction = link.spinning_resistance * body.radius_m;
 body.contact_stiffness_Npm = link.contact_stiffness_Npm;
 body.contact_damping_Nspm = link.contact_damping_Nspm;
 body.contact_axle_axis_local = {link.contact_axle_axis.x, link.contact_axle_axis.y,
 link.contact_axle_axis.z};
 const bool tire_body = magnitude(body.contact_axle_axis_local) > 0.5;
 ::sim::physics::TireContactSpec tire;
 if (tire_body) {
 tire = {
 link.friction,
 link.rolling_support_moment_arm_m,
 link.spinning_resistance,
 body.radius_m,
 link.contact_stiffness_Npm,
 link.contact_damping_Nspm,
 body.contact_axle_axis_local,
 true,
 };
 tire_contact_->configure_body(body, tire);
 }
 body.linear_damping = 0.02;
 body.angular_damping = 0.02;
 body.continuous_collision = true;
 body.collision_group = ::sim::physics::collision_layer::Vehicle;
 body.collision_mask = vehicle_mask;
 const ::sim::physics::RigidTransform robot_from_link{
 {link.position_m.x, link.position_m.y, link.position_m.z},
 ::sim::physics::quaternion_from_euler(link.rotation_deg.x * kDegreesToRadians,
 link.rotation_deg.y * kDegreesToRadians, link.rotation_deg.z * kDegreesToRadians)};
 const auto world_from_link = ::sim::physics::compose(world_from_robot, robot_from_link);
 body.position_m = world_from_link.translation_m;
 body.rotation = world_from_link.rotation;
 const auto handle = world_->spawn(body);
 robot_contact_materials_[handle.value] = {
 link.friction, link.rolling_support_moment_arm_m,
 link.spinning_resistance * body.radius_m};
 robot_links_[link.id] = handle;
 robot_link_names_[handle.value] = link.id;
 robot_bodies_.push_back(handle);
 if (tire_body)
 tire_contacts_[handle.value] = tire;
 if (link.id == robot_.articulation.root_link) {
 robot_body_ = handle;
 body_center_height_m_ = link.position_m.z;
 }
 }
 for (const auto& joint : robot_.articulation.joints) {
 ::sim::physics::JointSpec spec;
 spec.kind = joint.type == "continuous" ? ::sim::physics::JointKind::Revolute
 : ::sim::physics::JointKind::Fixed;
 spec.parent = robot_links_.at(joint.parent);
 spec.child = robot_links_.at(joint.child);
 spec.pivot_in_parent_m = {joint.pivot_in_parent_m.x, joint.pivot_in_parent_m.y,
 joint.pivot_in_parent_m.z};
 spec.pivot_in_child_m = {joint.pivot_in_child_m.x, joint.pivot_in_child_m.y,
 joint.pivot_in_child_m.z};
 spec.axis_in_parent = {joint.axis_in_parent.x, joint.axis_in_parent.y, joint.axis_in_parent.z};
 spec.axis_in_child = {joint.axis_in_child.x, joint.axis_in_child.y, joint.axis_in_child.z};
 robot_joints_[joint.id] = world_->create_joint(spec);
 child_link_joints_[joint.child] = robot_joints_.at(joint.id);
 }
 for (const auto& actuator : robot_.articulation.actuators) {
 actuator_bindings_[actuator.channel] = {
 robot_joints_.at(actuator.joint), actuator.effort_limit_Nm};
 const auto joint = std::find_if(robot_.articulation.joints.begin(),
 robot_.articulation.joints.end(),
 [&](const auto& value) { return value.id == actuator.joint; });
 if (joint != robot_.articulation.joints.end())
 driven_contact_bodies_.push_back(robot_links_.at(joint->child));
 }
 if (!robot_body_ || actuator_bindings_.empty())
 throw std::runtime_error("articulated platform requires a root and at least one actuator binding");
 // Resolve the authored assembly onto the terrain before simulation time
 // starts. Constraint stabilization from an exactly authored pose can
 // otherwise appear as a one-frame chassis kick. After the short deterministic
 // settle, preserve the solved poses and clear velocities.
 for (int index = 0; index < 100; ++index)
 world_->step(0.002);
 for (const auto body : robot_bodies_) {
 auto state = world_->state(body);
 state.linear_velocity_mps = {};
 state.angular_velocity_radps = {};
 world_->set_state(body, state);
 }
 } else {
 ::sim::physics::BodySpec vehicle;
 vehicle.shape = ::sim::physics::ShapeKind::Box;
 vehicle.half_extents_m = {robot_.rigid_body_size_m.x * 0.5,
 robot_.rigid_body_size_m.y * 0.5, robot_.rigid_body_size_m.z * 0.5};
 vehicle.mass_kg = total_mass_kg_;
 vehicle.friction = 0.0;
 vehicle.linear_damping = 0.03;
 vehicle.angular_damping = 0.10;
 vehicle.continuous_collision = true;
 vehicle.collision_group = ::sim::physics::collision_layer::Vehicle;
 vehicle.collision_mask = vehicle_mask;
 const double inertia_roll = std::max(0.05, total_mass_kg_ *
 (support_width_m_ * support_width_m_ + 4.0 * center_height_m_ * center_height_m_) / 12.0);
 const double inertia_pitch = std::max(0.05, total_mass_kg_ *
 (support_length_m_ * support_length_m_ + 4.0 * center_height_m_ * center_height_m_) / 12.0);
 vehicle.inertia_diagonal_kgm2 = {inertia_roll, inertia_pitch, yaw_inertia_kgm2_};
 const double vertical_half_extent = body_center_height_m_ /
 std::max(0.25, std::cos(roll) * std::cos(pitch));
 vehicle.position_m = {spawn.x, spawn.y, ground.height_m + vertical_half_extent + 0.01};
 vehicle.rotation = robot_rotation;
 robot_body_ = world_->spawn(vehicle);
 robot_bodies_.push_back(robot_body_);
 }

 for (const auto& mount : robot_.mounts) {
  const ::sim::physics::RigidTransform robot_from_mount{
 {mount.position_m.x, mount.position_m.y, mount.position_m.z},
 ::sim::physics::quaternion_from_euler(mount.rotation_deg.x * kDegreesToRadians,
 mount.rotation_deg.y * kDegreesToRadians,
 mount.rotation_deg.z * kDegreesToRadians),
   };
  frames_.set_transform("base_link", mount.frame_id, robot_from_mount);
  if (mount.frame_id == "imu_link") {
 // Mounts are calibrated in base_link. Bullet also reports the root body at
 // base_link, so this is the exact COM-to-sensor lever arm used by the IMU.
 body_from_imu_ = robot_from_mount;
  }
 }
 if (!frames_.lookup("base_link", "imu_link")) {
 frames_.set_transform("base_link", "imu_link", {});
 }
 frames_.set_transform("map", "odom", {});
 const auto initial_body_state = world_->state(robot_body_);
 update_inertial_truth(initial_body_state, initial_body_state, 1.0);
 update_dynamic_frames(initial_body_state);
 for (const auto& slot : robot_.slots) {
 if (slot.slot != "sensor" || slot.mounts.empty())
 continue;
 const auto mount = std::find_if(robot_.mounts.begin(), robot_.mounts.end(),
 [&](const MountDefinition& candidate) { return candidate.id == slot.mounts.front(); });
 if (mount != robot_.mounts.end()) {
 lidar_frame_id_ = mount->frame_id;
 has_lidar_mount_ = true;
 }
 break;
 }
}

void SimulatedPlatform::update_load(double total_mass_kg, double center_height_m,
 double yaw_inertia_kgm2) {
 total_mass_kg_ = std::max(0.5, total_mass_kg);
 center_height_m_ = std::max(0.05, center_height_m);
 yaw_inertia_kgm2_ = std::max(0.05, yaw_inertia_kgm2);
 if (articulated_) {
 const double payload_mass_kg = std::max(0.0, total_mass_kg_ - robot_.mass_kg);
 const double payload_yaw_inertia = std::max(0.0, yaw_inertia_kgm2_ - robot_.yaw_inertia_kgm2);
 const double payload_height_m = payload_mass_kg > 1e-9
 ? (center_height_m_ * total_mass_kg_ -
 robot_.mass_kg * robot_.center_of_mass_height_m) / payload_mass_kg
 : robot_.base_link_height_m;
 const double vertical_offset_m = payload_height_m - robot_.base_link_height_m;
 const double payload_transverse_inertia =
 0.5 * payload_yaw_inertia + payload_mass_kg * vertical_offset_m * vertical_offset_m;
 world_->set_mass_properties(robot_body_, articulated_root_mass_kg_ + payload_mass_kg,
 {articulated_root_inertia_kgm2_.x + payload_transverse_inertia,
 articulated_root_inertia_kgm2_.y + payload_transverse_inertia,
 articulated_root_inertia_kgm2_.z + payload_yaw_inertia});
 } else {
 const double inertia_roll = std::max(0.05, total_mass_kg_ *
 (support_width_m_ * support_width_m_ + 4.0 * center_height_m_ * center_height_m_) / 12.0);
 const double inertia_pitch = std::max(0.05, total_mass_kg_ *
 (support_length_m_ * support_length_m_ + 4.0 * center_height_m_ * center_height_m_) / 12.0);
 world_->set_mass_properties(robot_body_, total_mass_kg_,
 {inertia_roll, inertia_pitch, yaw_inertia_kgm2_});
 }
}

const char* SimulatedPlatform::backend_name() const {
 return world_ ? world_->backend_name().data() : "none";
}

::sim::physics::BodySpec SimulatedPlatform::terrain_mesh(const TerrainPatchDefinition& patch) const {
 ::sim::physics::BodySpec body;
 body.shape = ::sim::physics::ShapeKind::TriangleMesh;
 body.friction = 1.0;
 body.collision_group = ::sim::physics::collision_layer::Terrain;
 body.collision_mask = static_cast<::sim::physics::CollisionMask>(
 ::sim::physics::collision_layer::Default | ::sim::physics::collision_layer::Vehicle);
 const double resolution = std::min(0.25, map_.grid_resolution_m);
 const int nx = std::max(1, static_cast<int>(std::ceil((patch.max.x - patch.min.x) / resolution)));
 const int ny = std::max(1, static_cast<int>(std::ceil((patch.max.y - patch.min.y) / resolution)));
 body.mesh_vertices_m.reserve(static_cast<std::size_t>((nx + 1) * (ny + 1) + 4));
 for (int y = 0; y <= ny; ++y) {
 const double wy = patch.min.y + (patch.max.y - patch.min.y) * static_cast<double>(y) / ny;
 for (int x = 0; x <= nx; ++x) {
 const double wx = patch.min.x + (patch.max.x - patch.min.x) * static_cast<double>(x) / nx;
 body.mesh_vertices_m.push_back({wx, wy, patch_height(patch, wx, wy)});
 }
 }
 auto vertex = [nx](int x, int y) {
 return static_cast<std::uint32_t>(y * (nx + 1) + x);
 };
 for (int y = 0; y < ny; ++y) {
 for (int x = 0; x < nx; ++x) {
 const auto a = vertex(x, y);
 const auto b = vertex(x + 1, y);
 const auto c = vertex(x + 1, y + 1);
 const auto d = vertex(x, y + 1);
 body.mesh_indices.insert(body.mesh_indices.end(), {a, b, c, a, c, d});
 }
 }

 // Close the perimeter down to the base plane. This makes elevated patches
 // real platforms with retaining walls instead of infinitely thin carpets.
 const double bottom_z = std::min(-0.10, std::min({patch_height(patch, patch.min.x, patch.min.y),
 patch_height(patch, patch.max.x, patch.min.y), patch_height(patch, patch.max.x, patch.max.y),
 patch_height(patch, patch.min.x, patch.max.y)}) - 0.10);
 const std::uint32_t bottom_start = static_cast<std::uint32_t>(body.mesh_vertices_m.size());
 body.mesh_vertices_m.insert(body.mesh_vertices_m.end(), {
 {patch.min.x, patch.min.y, bottom_z},
 {patch.max.x, patch.min.y, bottom_z},
 {patch.max.x, patch.max.y, bottom_z},
 {patch.min.x, patch.max.y, bottom_z},
 });
 const std::uint32_t top_corners[] = {vertex(0, 0), vertex(nx, 0), vertex(nx, ny), vertex(0, ny)};
 const ::sim::Vec2 corners[] = {{patch.min.x, patch.min.y}, {patch.max.x, patch.min.y},
 {patch.max.x, patch.max.y}, {patch.min.x, patch.max.y}};
 const auto shared_edge = [&](int side) {
 const int next = (side + 1) % 4;
 for (const auto& other : map_.terrain) {
 if (&other == &patch || !active_in_level(other.active_in, level_.id))
 continue;
 bool adjacent = false;
 if (side == 0)
 adjacent = std::abs(other.max.y - patch.min.y) < 1e-6 &&
 other.min.x <= patch.min.x + 1e-6 && other.max.x >= patch.max.x - 1e-6;
 else if (side == 1)
 adjacent = std::abs(other.min.x - patch.max.x) < 1e-6 &&
 other.min.y <= patch.min.y + 1e-6 && other.max.y >= patch.max.y - 1e-6;
 else if (side == 2)
 adjacent = std::abs(other.min.y - patch.max.y) < 1e-6 &&
 other.min.x <= patch.min.x + 1e-6 && other.max.x >= patch.max.x - 1e-6;
 else
 adjacent = std::abs(other.max.x - patch.min.x) < 1e-6 &&
 other.min.y <= patch.min.y + 1e-6 && other.max.y >= patch.max.y - 1e-6;
 if (!adjacent)
 continue;
 const double a_delta = std::abs(patch_height(patch, corners[side].x, corners[side].y) -
 patch_height(other, corners[side].x, corners[side].y));
 const double b_delta = std::abs(patch_height(patch, corners[next].x, corners[next].y) -
 patch_height(other, corners[next].x, corners[next].y));
 if (a_delta < 0.05 && b_delta < 0.05)
 return true;
 }
 return false;
 };
 for (int side = 0; side < 4; ++side) {
 const int next = (side + 1) % 4;
 if (shared_edge(side))
 continue;
 const auto top_a = top_corners[side];
 const auto top_b = top_corners[next];
 const auto bottom_a = bottom_start + static_cast<std::uint32_t>(side);
 const auto bottom_b = bottom_start + static_cast<std::uint32_t>(next);
 body.mesh_indices.insert(body.mesh_indices.end(), {bottom_a, bottom_b, top_b, bottom_a, top_b, top_a});
 }
 body.mesh_indices.insert(body.mesh_indices.end(),
 {bottom_start, bottom_start + 2, bottom_start + 1, bottom_start, bottom_start + 3, bottom_start + 2});
 return body;
}

void SimulatedPlatform::sync_actors(const std::vector<PhysicsActorState>& actors) {
 std::set<std::string> active_ids;
 for (const auto& actor : actors) {
 active_ids.insert(actor.id);
 auto found = actor_bodies_.find(actor.id);
 if (found == actor_bodies_.end()) {
 ::sim::physics::BodySpec body;
 body.shape = ::sim::physics::ShapeKind::Cylinder;
 body.radius_m = actor.radius_m;
 body.height_m = std::max(0.5, actor.radius_m * 2.0);
 body.kinematic = true;
 body.friction = 0.2;
 body.collision_group = ::sim::physics::collision_layer::DynamicActor;
 body.collision_mask = static_cast<::sim::physics::CollisionMask>(
 ::sim::physics::collision_layer::Default | ::sim::physics::collision_layer::Vehicle);
 body.position_m = ::sim::physics::Vec3{actor.position.x, actor.position.y,
 terrain_.sample(actor.position).height_m + body.height_m * 0.5};
 found = actor_bodies_.emplace(actor.id, world_->spawn(body)).first;
 actor_radii_m_[actor.id] = actor.radius_m;
 actor_heights_m_[actor.id] = body.height_m;
 }
 ::sim::physics::BodyState state = world_->state(found->second);
 state.position_m = ::sim::physics::Vec3{actor.position.x, actor.position.y,
 terrain_.sample(actor.position).height_m + std::max(0.5, actor.radius_m * 2.0) * 0.5};
 state.linear_velocity_mps = {actor.velocity.x, actor.velocity.y, 0.0};
 if (std::hypot(actor.velocity.x, actor.velocity.y) > 1e-6) {
 state.rotation = ::sim::physics::quaternion_from_euler(0.0, 0.0,
 std::atan2(actor.velocity.y, actor.velocity.x));
 }
 world_->set_state(found->second, state);
 }
 for (auto found = actor_bodies_.begin(); found != actor_bodies_.end();) {
 if (active_ids.contains(found->first)) {
 ++found;
 continue;
 }
 world_->remove(found->second);
 actor_radii_m_.erase(found->first);
 actor_heights_m_.erase(found->first);
 found = actor_bodies_.erase(found);
 }
}

::sim::Scan SimulatedPlatform::raycast_lidar(const ::sim::physics::RigidTransform& world_from_lidar,
 int rays, double range_m, ::sim::physics::CollisionMask collision_mask,
 double timestamp_s, double scan_period_s,
 ::sim::physics::Vec3 linear_velocity_mps, double yaw_rate_radps,
 const ::sim::LidarReturnModel& return_model) const {
 if (rays <= 0 || !(range_m > 0.0)) {
 throw std::invalid_argument("physics lidar needs positive rays and range");
 }
 ::sim::Scan scan;
 scan.n_rays = rays;
 scan.max_range = range_m;
 scan.angle_increment_rad = 2.0 * ::sim::kPi / static_cast<double>(rays);
 scan.scan_period_s = scan_period_s;
 scan.time_increment_s = scan_period_s > 0.0 ? scan_period_s / static_cast<double>(rays) : 0.0;
 scan.header = {timestamp_s, 0, lidar_frame_id_, 0.0, kMeasurementSchemaVersion, "sim.physics.lidar"};
 scan.ranges.reserve(static_cast<std::size_t>(rays));
 scan.hit.reserve(static_cast<std::size_t>(rays));
 scan.intensities.reserve(static_cast<std::size_t>(rays));
 for (int ray = 0; ray < rays; ++ray) {
 const double time_offset_s = scan_period_s > 0.0
 ? -scan_period_s + scan.time_increment_s * static_cast<double>(ray)
 : 0.0;
 auto sample_from_lidar = world_from_lidar;
 sample_from_lidar.translation_m.x += linear_velocity_mps.x * time_offset_s;
 sample_from_lidar.translation_m.y += linear_velocity_mps.y * time_offset_s;
 sample_from_lidar.translation_m.z += linear_velocity_mps.z * time_offset_s;
 sample_from_lidar.rotation = ::sim::physics::quaternion_multiply(
 ::sim::physics::quaternion_from_euler(0.0, 0.0, yaw_rate_radps * time_offset_s),
 sample_from_lidar.rotation);
 const double angle = 2.0 * ::sim::kPi * static_cast<double>(ray) / static_cast<double>(rays);
 const auto direction = ::sim::physics::rotate_vector(sample_from_lidar.rotation,
 {std::cos(angle), std::sin(angle), 0.0});
 const bool include_dynamic =
 (collision_mask & ::sim::physics::collision_layer::DynamicActor) != 0;
 const auto static_only_mask = static_cast<::sim::physics::CollisionMask>(
 collision_mask & ~::sim::physics::collision_layer::DynamicActor);
 auto hit = world_->raycast({sample_from_lidar.translation_m, direction, range_m,
 static_only_mask, robot_body_});
 if (include_dynamic) {
 for (const auto& [actor_id, actor_body] : actor_bodies_) {
 const auto actor = world_->state(actor_body);
 const double center_x = actor.position_m.x + actor.linear_velocity_mps.x * time_offset_s;
 const double center_y = actor.position_m.y + actor.linear_velocity_mps.y * time_offset_s;
 const double center_z = actor.position_m.z + actor.linear_velocity_mps.z * time_offset_s;
 const double offset_x = sample_from_lidar.translation_m.x - center_x;
 const double offset_y = sample_from_lidar.translation_m.y - center_y;
 const double a = direction.x * direction.x + direction.y * direction.y;
 const double b = 2.0 * (offset_x * direction.x + offset_y * direction.y);
 const double radius = actor_radii_m_.at(actor_id);
 const double c = offset_x * offset_x + offset_y * offset_y - radius * radius;
 const double discriminant = b * b - 4.0 * a * c;
 if (a < 1e-12 || discriminant < 0.0)
 continue;
 const double actor_distance = (-b - std::sqrt(discriminant)) / (2.0 * a);
 if (actor_distance < 0.0 || actor_distance > range_m ||
 (hit.hit && actor_distance >= hit.distance_m))
 continue;
 const double hit_z = sample_from_lidar.translation_m.z + direction.z * actor_distance;
 if (std::abs(hit_z - center_z) > actor_heights_m_.at(actor_id) * 0.5)
 continue;
 const double hit_x = sample_from_lidar.translation_m.x + direction.x * actor_distance;
 const double hit_y = sample_from_lidar.translation_m.y + direction.y * actor_distance;
 const double normal_length = std::max(1e-12, std::hypot(hit_x - center_x, hit_y - center_y));
 hit = {
 true,
 actor_body,
 {hit_x, hit_y, hit_z},
 {(hit_x - center_x) / normal_length, (hit_y - center_y) / normal_length, 0.0},
 actor_distance,
 };
 }
 }
 scan.ranges.push_back(hit.hit ? hit.distance_m : range_m);
 scan.hit.push_back(hit.hit ? 1 : 0);
 double intensity = 0.0;
 if (hit.hit) {
 const double incidence = std::clamp(
 -(direction.x * hit.normal.x + direction.y * hit.normal.y + direction.z * hit.normal.z),
 0.0, 1.0);
 double reflectivity = return_model.default_reflectivity;
 if (is_terrain(hit.body)) {
 reflectivity = return_model.terrain_reflectivity;
 for (const auto& surface : map_.surfaces) {
 if (!active_in_level(surface.active_in, level_.id) ||
 hit.point_m.x < surface.min.x || hit.point_m.x >= surface.max.x ||
 hit.point_m.y < surface.min.y || hit.point_m.y >= surface.max.y)
 continue;
 reflectivity = surface.lidar_reflectivity;
 break;
 }
 }
 else if (is_static_obstacle(hit.body)) reflectivity = return_model.obstacle_reflectivity;
 else if (!actor_for(hit.body).empty()) reflectivity = return_model.actor_reflectivity;
 const double attenuation = 1.0 / (1.0 + return_model.distance_attenuation_per_m2 *
 hit.distance_m * hit.distance_m);
 const double angular_response = return_model.incidence_floor +
 (1.0 - return_model.incidence_floor) * incidence;
 intensity = std::clamp(reflectivity * angular_response * attenuation, 0.0, 1.0);
 }
 scan.intensities.push_back(intensity);
 }
 return scan;
}

::sim::LidarRaycastTruth SimulatedPlatform::scan_lidar(int rays, double range_m,
 ::sim::Vec2 estimated_position, double estimated_heading_rad,
 double timestamp_s, double scan_period_s,
 const ::sim::LidarReturnModel& return_model) const {
 if (!has_lidar_mount_) {
 throw std::runtime_error("robot has no sensor mount for physics lidar");
 }
 const auto body = world_->state(robot_body_);
 update_dynamic_frames(body);
 const auto world_from_lidar = frames_.lookup("map", lidar_frame_id_);
 if (!world_from_lidar) {
 throw std::runtime_error("physics frame tree cannot resolve world to lidar_link");
 }

 const auto estimated_ground = terrain_.sample(estimated_position);
 const double estimated_roll = std::atan(::sim::lateral_grade(estimated_ground, estimated_heading_rad));
 const double estimated_pitch = -std::atan(::sim::longitudinal_grade(estimated_ground,
 estimated_heading_rad));
 const ::sim::physics::RigidTransform world_from_estimated_base{
 {estimated_position.x, estimated_position.y,
 estimated_ground.height_m + body_center_height_m_},
 ::sim::physics::quaternion_from_euler(estimated_roll, estimated_pitch, estimated_heading_rad),
 };
 const auto base_from_lidar = frames_.lookup("base_link", lidar_frame_id_);
 if (!base_from_lidar) {
 throw std::runtime_error("physics frame tree cannot resolve base_link to lidar_link");
 }
 const auto world_from_estimated_lidar = ::sim::physics::compose(world_from_estimated_base,
 *base_from_lidar);

 constexpr auto static_mask = static_cast<::sim::physics::CollisionMask>(
 ::sim::physics::collision_layer::Terrain | ::sim::physics::collision_layer::StaticObstacle);
 constexpr auto combined_mask = static_cast<::sim::physics::CollisionMask>(
 static_mask | ::sim::physics::collision_layer::DynamicActor);
 return {
 raycast_lidar(*world_from_lidar, rays, range_m, combined_mask, timestamp_s,
 scan_period_s, body.linear_velocity_mps, body.angular_velocity_radps.z, return_model),
 raycast_lidar(world_from_estimated_lidar, rays, range_m, static_mask, timestamp_s,
 scan_period_s, body.linear_velocity_mps, body.angular_velocity_radps.z, return_model),
 };
}

::sim::LidarPointCloud SimulatedPlatform::raycast_lidar_point_cloud(
 const ::sim::physics::RigidTransform& world_from_lidar,
 int azimuth_samples, const std::vector<double>& vertical_angles_rad,
 double range_m, ::sim::physics::CollisionMask collision_mask,
 double timestamp_s, double scan_period_s,
 ::sim::physics::Vec3 linear_velocity_mps, double yaw_rate_radps,
 const ::sim::LidarReturnModel& return_model) const {
 if (azimuth_samples <= 0 || vertical_angles_rad.empty() || !(range_m > 0.0) ||
 !(scan_period_s > 0.0) || vertical_angles_rad.size() > 128) {
 throw std::invalid_argument("physics multibeam lidar configuration is invalid");
 }
 ::sim::LidarPointCloud cloud;
 cloud.pattern = ::sim::LidarPatternKind::MultiBeamSpinning;
 cloud.rings = static_cast<std::uint16_t>(vertical_angles_rad.size());
 cloud.azimuth_samples = static_cast<std::uint32_t>(azimuth_samples);
 cloud.max_range_m = range_m;
 cloud.scan_period_s = scan_period_s;
 cloud.header = {timestamp_s, 0, lidar_frame_id_, 0.0,
 kMeasurementSchemaVersion, "sim.physics.lidar-point-cloud"};
 cloud.points.reserve(static_cast<std::size_t>(azimuth_samples) *
 vertical_angles_rad.size());
 const bool include_dynamic =
 (collision_mask & ::sim::physics::collision_layer::DynamicActor) != 0;
 const auto static_only_mask = static_cast<::sim::physics::CollisionMask>(
 collision_mask & ~::sim::physics::collision_layer::DynamicActor);
 for (int azimuth_index = 0; azimuth_index < azimuth_samples; ++azimuth_index) {
 const double azimuth = 2.0 * ::sim::kPi * static_cast<double>(azimuth_index) /
 static_cast<double>(azimuth_samples);
 for (std::size_t ring = 0; ring < vertical_angles_rad.size(); ++ring) {
 const double firing_index = static_cast<double>(
 static_cast<std::size_t>(azimuth_index) * vertical_angles_rad.size() + ring);
 const double firing_count = static_cast<double>(
 static_cast<std::size_t>(azimuth_samples) * vertical_angles_rad.size());
 const double time_offset_s = -scan_period_s +
 scan_period_s * firing_index / firing_count;
 auto sample_from_lidar = world_from_lidar;
 sample_from_lidar.translation_m.x += linear_velocity_mps.x * time_offset_s;
 sample_from_lidar.translation_m.y += linear_velocity_mps.y * time_offset_s;
 sample_from_lidar.translation_m.z += linear_velocity_mps.z * time_offset_s;
 sample_from_lidar.rotation = ::sim::physics::quaternion_multiply(
 ::sim::physics::quaternion_from_euler(
 0.0, 0.0, yaw_rate_radps * time_offset_s),
 sample_from_lidar.rotation);
 const double vertical = vertical_angles_rad[ring];
 const ::sim::physics::Vec3 local_direction{
 std::cos(vertical) * std::cos(azimuth),
 std::cos(vertical) * std::sin(azimuth),
 std::sin(vertical),
 };
 const auto direction = ::sim::physics::rotate_vector(
 sample_from_lidar.rotation, local_direction);
 auto hit = world_->raycast({sample_from_lidar.translation_m, direction,
 range_m, static_only_mask, robot_body_});
 if (include_dynamic) {
 for (const auto& [actor_id, actor_body] : actor_bodies_) {
 const auto actor = world_->state(actor_body);
 const ::sim::physics::Vec3 center{
 actor.position_m.x + actor.linear_velocity_mps.x * time_offset_s,
 actor.position_m.y + actor.linear_velocity_mps.y * time_offset_s,
 actor.position_m.z + actor.linear_velocity_mps.z * time_offset_s,
 };
 const double offset_x = sample_from_lidar.translation_m.x - center.x;
 const double offset_y = sample_from_lidar.translation_m.y - center.y;
 const double a = direction.x * direction.x + direction.y * direction.y;
 const double b = 2.0 * (offset_x * direction.x + offset_y * direction.y);
 const double radius = actor_radii_m_.at(actor_id);
 const double c = offset_x * offset_x + offset_y * offset_y - radius * radius;
 const double discriminant = b * b - 4.0 * a * c;
 if (a < 1e-12 || discriminant < 0.0)
 continue;
 const double distance = (-b - std::sqrt(discriminant)) / (2.0 * a);
 if (distance < 0.0 || distance > range_m ||
 (hit.hit && distance >= hit.distance_m))
 continue;
 const double hit_z = sample_from_lidar.translation_m.z + direction.z * distance;
 if (std::abs(hit_z - center.z) > actor_heights_m_.at(actor_id) * 0.5)
 continue;
 const ::sim::physics::Vec3 point{
 sample_from_lidar.translation_m.x + direction.x * distance,
 sample_from_lidar.translation_m.y + direction.y * distance,
 hit_z,
 };
 const double normal_length = std::max(
 1e-12, std::hypot(point.x - center.x, point.y - center.y));
 hit = {true, actor_body, point,
 {(point.x - center.x) / normal_length,
 (point.y - center.y) / normal_length, 0.0}, distance};
 }
 }
 if (!hit.hit)
 continue;
 const double incidence = std::clamp(
 -(direction.x * hit.normal.x + direction.y * hit.normal.y +
 direction.z * hit.normal.z), 0.0, 1.0);
 double reflectivity = return_model.default_reflectivity;
 if (is_terrain(hit.body)) {
 reflectivity = return_model.terrain_reflectivity;
 for (const auto& surface : map_.surfaces) {
 if (!active_in_level(surface.active_in, level_.id) ||
 hit.point_m.x < surface.min.x || hit.point_m.x >= surface.max.x ||
 hit.point_m.y < surface.min.y || hit.point_m.y >= surface.max.y)
 continue;
 reflectivity = surface.lidar_reflectivity;
 break;
 }
 } else if (is_static_obstacle(hit.body)) {
 reflectivity = return_model.obstacle_reflectivity;
 } else if (!actor_for(hit.body).empty()) {
 reflectivity = return_model.actor_reflectivity;
 }
 const double attenuation = 1.0 /
 (1.0 + return_model.distance_attenuation_per_m2 *
 hit.distance_m * hit.distance_m);
 const double angular_response = return_model.incidence_floor +
 (1.0 - return_model.incidence_floor) * incidence;
 cloud.points.push_back({
 local_direction.x * hit.distance_m,
 local_direction.y * hit.distance_m,
 local_direction.z * hit.distance_m,
 std::clamp(reflectivity * angular_response * attenuation, 0.0, 1.0),
 time_offset_s,
 static_cast<std::uint16_t>(ring),
 0,
 });
 }
 }
 return cloud;
}

::sim::LidarPointCloudRaycastTruth SimulatedPlatform::scan_lidar_point_cloud(
 int azimuth_samples, const std::vector<double>& vertical_angles_rad,
 double range_m, ::sim::Vec2 estimated_position,
 double estimated_heading_rad, double timestamp_s, double scan_period_s,
 const ::sim::LidarReturnModel& return_model) const {
 if (!has_lidar_mount_)
 throw std::runtime_error("robot has no sensor mount for physics lidar");
 const auto body = world_->state(robot_body_);
 update_dynamic_frames(body);
 const auto world_from_lidar = frames_.lookup("map", lidar_frame_id_);
 if (!world_from_lidar)
 throw std::runtime_error("physics frame tree cannot resolve world to lidar_link");
 const auto estimated_ground = terrain_.sample(estimated_position);
 const double estimated_roll = std::atan(
 ::sim::lateral_grade(estimated_ground, estimated_heading_rad));
 const double estimated_pitch = -std::atan(
 ::sim::longitudinal_grade(estimated_ground, estimated_heading_rad));
 const ::sim::physics::RigidTransform world_from_estimated_base{
 {estimated_position.x, estimated_position.y,
 estimated_ground.height_m + body_center_height_m_},
 ::sim::physics::quaternion_from_euler(
 estimated_roll, estimated_pitch, estimated_heading_rad),
 };
 const auto base_from_lidar = frames_.lookup("base_link", lidar_frame_id_);
 if (!base_from_lidar)
 throw std::runtime_error("physics frame tree cannot resolve base_link to lidar_link");
 const auto world_from_estimated_lidar = ::sim::physics::compose(
 world_from_estimated_base, *base_from_lidar);
 constexpr auto static_mask = static_cast<::sim::physics::CollisionMask>(
 ::sim::physics::collision_layer::Terrain |
 ::sim::physics::collision_layer::StaticObstacle);
 constexpr auto combined_mask = static_cast<::sim::physics::CollisionMask>(
 static_mask | ::sim::physics::collision_layer::DynamicActor);
 return {
 raycast_lidar_point_cloud(*world_from_lidar, azimuth_samples,
 vertical_angles_rad, range_m, combined_mask, timestamp_s,
 scan_period_s, body.linear_velocity_mps, body.angular_velocity_radps.z,
 return_model),
 raycast_lidar_point_cloud(world_from_estimated_lidar, azimuth_samples,
 vertical_angles_rad, range_m, static_mask, timestamp_s,
 scan_period_s, body.linear_velocity_mps, body.angular_velocity_radps.z,
 return_model),
 };
}

void SimulatedPlatform::update_dynamic_frames(const ::sim::physics::BodyState& body) const {
 const auto angles = ::sim::physics::euler_from_quaternion(body.rotation);
 const auto ground = terrain_.sample({body.position_m.x, body.position_m.y});
 frames_.set_transform("map", "odom", {});
 frames_.set_transform("odom", "base_footprint", {
 {body.position_m.x, body.position_m.y, ground.height_m},
 ::sim::physics::quaternion_from_euler(0.0, 0.0, angles.z),
 });
 frames_.set_transform("base_footprint", "base_link", {
 {0.0, 0.0, body.position_m.z - ground.height_m},
 ::sim::physics::quaternion_from_euler(angles.x, angles.y, 0.0),
 });
}

RigidVehicleState SimulatedPlatform::step(const ::sim::ChassisTelemetry& actuation,
 const ::sim::GroundContact& ground, double fixed_dt_s) {
 const auto before = world_->state(robot_body_);
 const auto angles = ::sim::physics::euler_from_quaternion(before.rotation);
 const double yaw = angles.z;
 const double lateral_speed = -std::sin(yaw) * before.linear_velocity_mps.x +
 std::cos(yaw) * before.linear_velocity_mps.y;
 double forward_force_N = 0.0;
 double lateral_force_N = 0.0;
 double contact_yaw_torque_Nm = 0.0;
 if (articulated_) {
 const double mu_scale = std::clamp(ground.mu_eff, 0.0, 2.0);
 for (const auto body : robot_bodies_) {
 const auto material = robot_contact_materials_.find(body.value);
 if (material == robot_contact_materials_.end())
 continue;
 const auto tire = tire_contacts_.find(body.value);
 if (tire != tire_contacts_.end()) {
 tire_contact_->apply(*world_, body, tire->second, {
 mu_scale,
 ground.rolling_resist,
 ground.normal_stiffness_Npm,
 ground.normal_damping_Nspm,
 });
 } else {
 world_->set_contact_friction(body, material->second.friction * mu_scale,
 material->second.rolling_resistance, material->second.spinning_resistance);
 }
 }
 for (const auto& command : actuation.joint_efforts) {
 const auto binding = actuator_bindings_.find(command.channel);
 if (binding == actuator_bindings_.end())
 throw std::runtime_error("actuator command references unbound channel '" + command.channel + "'");
 world_->apply_joint_effort(binding->second.joint,
 std::clamp(command.effort_Nm, -binding->second.effort_limit_Nm,
 binding->second.effort_limit_Nm));
 }
 world_->step(fixed_dt_s);
  const auto after = world_->state(robot_body_);
  update_inertial_truth(before, after, fixed_dt_s);
  update_dynamic_frames(after);
 auto output = read_state();
 update_drive_slip(output);
 for (const auto& contact : world_->contacts()) {
 ::sim::physics::BodyHandle other;
 ::sim::physics::BodyHandle robot_contact_body;
 if (is_robot_body(contact.body_a)) {
 robot_contact_body = contact.body_a;
 other = contact.body_b;
 } else if (is_robot_body(contact.body_b)) {
 robot_contact_body = contact.body_b;
 other = contact.body_a;
 } else
 continue;
 if (is_robot_body(other))
 continue;
 ++output.contact_count;
 if (is_terrain(other)) {
 const auto name = robot_link_names_.find(robot_contact_body.value);
 if (name != robot_link_names_.end()) {
 if (std::find(output.ground_contact_links.begin(), output.ground_contact_links.end(), name->second) ==
 output.ground_contact_links.end())
 output.ground_contact_links.push_back(name->second);
 auto load = std::find_if(output.ground_contact_loads.begin(), output.ground_contact_loads.end(),
 [&](const LinkContactLoad& value) { return value.link == name->second; });
 if (load == output.ground_contact_loads.end()) {
 output.ground_contact_loads.push_back({name->second, 0, 0.0, 0.0, 0.0, {}, 0.0, {}});
 load = std::prev(output.ground_contact_loads.end());
 }
 ++load->contact_points;
 load->normal_impulse_Ns += std::abs(contact.impulse_Ns);
 load->normal_force_N += std::abs(contact.normal_force_N);
 const auto tangent = add(scale(contact.tangent_direction_1, contact.tangent_impulse_1_Ns),
 scale(contact.tangent_direction_2, contact.tangent_impulse_2_Ns));
 load->tangent_impulse_Ns += std::sqrt(tangent.x * tangent.x + tangent.y * tangent.y +
 tangent.z * tangent.z);
 load->tangent_impulse_world_Ns = add(load->tangent_impulse_world_Ns, tangent);
 const auto tangent_force = add(scale(contact.tangent_direction_1, contact.tangent_force_1_N),
 scale(contact.tangent_direction_2, contact.tangent_force_2_N));
 load->tangent_force_N += magnitude(tangent_force);
 load->tangent_force_world_N = add(load->tangent_force_world_N, tangent_force);
 }
 }
 if (is_static_obstacle(other)) {
 output.static_collision = true;
 output.max_contact_impulse_Ns = std::max(output.max_contact_impulse_Ns, contact.impulse_Ns);
 }
 const std::string actor = actor_for(other);
 if (!actor.empty()) {
 output.actor_collisions.push_back(actor);
 output.max_contact_impulse_Ns = std::max(output.max_contact_impulse_Ns, contact.impulse_Ns);
 }
 }
 return output;
 }
 if (actuation.holonomic) {
 forward_force_N = actuation.body_forward_force_N;
 lateral_force_N = actuation.body_lateral_force_N;
 } else if (actuation.differential) {
 forward_force_N = actuation.left.longitudinal_force_N + actuation.right.longitudinal_force_N;
 contact_yaw_torque_Nm =
 (actuation.right.longitudinal_force_N - actuation.left.longitudinal_force_N) *
 support_width_m_ * 0.5;
 } else {
 forward_force_N = actuation.drive.longitudinal_force_N;
 }

 // The current rigid-body proxy represents the sprung chassis hull, while
 // wheel/track contact and load transfer are owned by the chassis model. A
 // force applied below this proxy's centre creates an unopposed pitch moment
 // because the proxy has no suspension constraints; the hull then rocks on
 // its collision rim, sinks visually and jitters. Apply the already-resolved
 // contact resultant at the hull centre and preserve the exact differential
 // yaw moment explicitly. A future articulated wheel/suspension body can
 // replace this adapter without changing the actuator contract.
 world_->apply_force_at_local_point(robot_body_,
 {forward_force_N, lateral_force_N, 0.0}, {0.0, 0.0, 0.0});

 const double grip_force = std::max(0.0, ground.mu_eff) * total_mass_kg_ * kGravityMps2;
 // Static tire force balances the gravity component on a cross slope; the
 // damping term then removes actual lateral slip. Both share the same ground
 // friction circle, so low-grip surfaces still slide.
 const double static_slope_support = total_mass_kg_ * kGravityMps2 * std::sin(angles.x);
 const double lateral_damping =
 std::clamp(static_slope_support - lateral_speed * total_mass_kg_ / 0.12,
 -grip_force, grip_force);
 world_->apply_force_at_local_point(robot_body_, {0.0, lateral_damping, 0.0},
 {0.0, 0.0, 0.0});
 const double yaw_limit = grip_force * std::max(0.1, support_length_m_) * 0.5;
 double yaw_torque = contact_yaw_torque_Nm + actuation.extra_yaw_torque_Nm;
 if (actuation.yaw_rate_tracking_time_s > 0.0) {
 // A chassis may request a distributed-contact yaw response in addition to
 // its point resultants. SimulatedPlatform consumes the actuator contract without
 // knowing whether the source is a track, wheel module, or future mechanism.
 const double yaw_inertia = yaw_inertia_kgm2_;
 yaw_torque += yaw_inertia *
 (actuation.desired_yaw_rate_radps - before.angular_velocity_radps.z) /
 actuation.yaw_rate_tracking_time_s;
 }
 world_->apply_local_torque(robot_body_,
 {0.0, 0.0, std::clamp(yaw_torque, -yaw_limit, yaw_limit)});
 world_->step(fixed_dt_s);
 const auto after = world_->state(robot_body_);
 update_inertial_truth(before, after, fixed_dt_s);
 update_dynamic_frames(after);
 auto output = read_state();
 for (const auto& contact : world_->contacts()) {
 ::sim::physics::BodyHandle other;
 if (same(contact.body_a, robot_body_)) {
 other = contact.body_b;
 } else if (same(contact.body_b, robot_body_)) {
 other = contact.body_a;
 } else {
 continue;
 }
 ++output.contact_count;
 if (is_static_obstacle(other)) {
 output.static_collision = true;
 output.max_contact_impulse_Ns = std::max(output.max_contact_impulse_Ns, contact.impulse_Ns);
 }
 const std::string actor = actor_for(other);
 if (!actor.empty()) {
 output.actor_collisions.push_back(actor);
 output.max_contact_impulse_Ns = std::max(output.max_contact_impulse_Ns, contact.impulse_Ns);
 }
 }
 return output;
}

RigidVehicleState SimulatedPlatform::state() const {
 update_dynamic_frames(world_->state(robot_body_));
 return read_state();
}

std::optional<::sim::physics::JointState> SimulatedPlatform::joint_state_for_child(
 const std::string& child_link) const {
 const auto found = child_link_joints_.find(child_link);
 if (found == child_link_joints_.end())
 return std::nullopt;
 return world_->joint_state(found->second);
}

std::optional<::sim::physics::RigidTransform> SimulatedPlatform::link_transform_in_base(
 const std::string& child_link) const {
 const auto found = robot_links_.find(child_link);
 if (found == robot_links_.end())
 return std::nullopt;
 const auto base = world_->state(robot_body_);
 const auto child = world_->state(found->second);
 const ::sim::physics::RigidTransform world_from_base{base.position_m, base.rotation};
 const ::sim::physics::RigidTransform world_from_child{child.position_m, child.rotation};
 return ::sim::physics::compose(::sim::physics::inverse(world_from_base), world_from_child);
}

RigidVehicleState SimulatedPlatform::read_state() const {
 const auto body = world_->state(robot_body_);
 const auto angles = ::sim::physics::euler_from_quaternion(body.rotation);
 const double forward_speed = std::cos(angles.z) * body.linear_velocity_mps.x +
 std::sin(angles.z) * body.linear_velocity_mps.y;
 const double lateral_speed = -std::sin(angles.z) * body.linear_velocity_mps.x +
 std::cos(angles.z) * body.linear_velocity_mps.y;
 RigidVehicleState output;
 output.planar = {body.position_m.x, body.position_m.y, ::sim::wrap_angle(angles.z), forward_speed,
 body.angular_velocity_radps.z, lateral_speed};
 const auto imu_from_body = ::sim::physics::inverse(body_from_imu_);
 const auto angular_velocity_body = ::sim::physics::rotate_vector(
  ::sim::physics::inverse({{}, body.rotation}).rotation, body.angular_velocity_radps);
 output.imu_angular_velocity_radps =
  ::sim::physics::rotate_vector(imu_from_body.rotation, angular_velocity_body);
 output.imu_specific_force_mps2 = last_imu_specific_force_mps2_;
 output.height_m = body.position_m.z;
 output.roll_rad = angles.x;
 output.pitch_rad = angles.y;
 return output;
}

void SimulatedPlatform::update_drive_slip(RigidVehicleState& state) {
 const auto left_binding = actuator_bindings_.find("left");
 const auto right_binding = actuator_bindings_.find("right");
 if (left_binding == actuator_bindings_.end() || right_binding == actuator_bindings_.end()) {
 drive_slipping_ = false;
 drive_slip_confirm_steps_ = 0;
 drive_slip_release_steps_ = 0;
 return;
 }
 const auto body = world_->state(robot_body_);
 const auto body_from_world = ::sim::physics::inverse({{}, body.rotation}).rotation;
 const auto velocity_body = ::sim::physics::rotate_vector(body_from_world,
 body.linear_velocity_mps);
 const auto angular_body = ::sim::physics::rotate_vector(body_from_world,
 body.angular_velocity_radps);
 const double half_track = robot_.wheel_base_m * 0.5;
 const double expected_left_mps = velocity_body.x - angular_body.z * half_track;
 const double expected_right_mps = velocity_body.x + angular_body.z * half_track;
 const double tread_left_mps = world_->joint_state(left_binding->second.joint).velocity_radps *
 robot_.wheel_radius_m;
 const double tread_right_mps = world_->joint_state(right_binding->second.joint).velocity_radps *
 robot_.wheel_radius_m;
 const auto ratio = [](double tread_mps, double contact_mps) {
 return std::abs(tread_mps - contact_mps) /
 std::max({0.1, std::abs(tread_mps), std::abs(contact_mps)});
 };
 state.max_drive_slip_ratio = std::max(ratio(tread_left_mps, expected_left_mps),
 ratio(tread_right_mps, expected_right_mps));
 const double max_slip_speed = std::max(std::abs(tread_left_mps - expected_left_mps),
 std::abs(tread_right_mps - expected_right_mps));
 const bool instantaneous = max_slip_speed > 0.08 && state.max_drive_slip_ratio > 0.25;
 if (instantaneous) {
 drive_slip_confirm_steps_ = std::min(3, drive_slip_confirm_steps_ + 1);
 drive_slip_release_steps_ = 0;
 if (drive_slip_confirm_steps_ >= 3)
 drive_slipping_ = true;
 } else {
 drive_slip_confirm_steps_ = 0;
 drive_slip_release_steps_ = std::min(5, drive_slip_release_steps_ + 1);
 if (drive_slip_release_steps_ >= 5)
 drive_slipping_ = false;
 }
 state.drive_slipping = drive_slipping_;
}

void SimulatedPlatform::update_inertial_truth(const ::sim::physics::BodyState& before,
 const ::sim::physics::BodyState& after, double fixed_dt_s) {
 if (!(fixed_dt_s > 0.0))
  throw std::invalid_argument("physics inertial sample requires a positive timestep");
 const auto linear_acceleration_world = scale(
  subtract(after.linear_velocity_mps, before.linear_velocity_mps), 1.0 / fixed_dt_s);
 const auto angular_acceleration_world = scale(
  subtract(after.angular_velocity_radps, before.angular_velocity_radps), 1.0 / fixed_dt_s);
 const auto offset_world = ::sim::physics::rotate_vector(after.rotation,
  body_from_imu_.translation_m);
 const auto point_acceleration_world = subtract(
  subtract(linear_acceleration_world,
   scale(cross(angular_acceleration_world, offset_world), -1.0)),
  scale(cross(after.angular_velocity_radps,
   cross(after.angular_velocity_radps, offset_world)), -1.0));
 const ::sim::physics::Vec3 gravity_world{0.0, 0.0, -kGravityMps2};
 const auto specific_force_world = subtract(point_acceleration_world, gravity_world);
 const auto body_from_world_rotation =
  ::sim::physics::inverse({{}, after.rotation}).rotation;
 const auto specific_force_body = ::sim::physics::rotate_vector(body_from_world_rotation,
  specific_force_world);
 const auto imu_from_body = ::sim::physics::inverse(body_from_imu_);
 last_imu_specific_force_mps2_ = ::sim::physics::rotate_vector(imu_from_body.rotation,
  specific_force_body);
}

bool SimulatedPlatform::is_terrain(::sim::physics::BodyHandle body) const {
 return std::find(terrain_bodies_.begin(), terrain_bodies_.end(), body) != terrain_bodies_.end();
}

bool SimulatedPlatform::is_robot_body(::sim::physics::BodyHandle body) const {
 return std::find(robot_bodies_.begin(), robot_bodies_.end(), body) != robot_bodies_.end();
}

::sim::ChassisActuatorFeedback SimulatedPlatform::actuator_feedback() const {
 if (!articulated_)
 return {};
 ::sim::ChassisActuatorFeedback feedback;
 feedback.joints.reserve(actuator_bindings_.size());
 for (const auto& [channel, binding] : actuator_bindings_) {
 const auto state = world_->joint_state(binding.joint);
 feedback.joints.push_back({channel, state.position_rad, state.velocity_radps});
 }
 return feedback;
}

bool SimulatedPlatform::is_static_obstacle(::sim::physics::BodyHandle body) const {
 return std::find(obstacle_bodies_.begin(), obstacle_bodies_.end(), body) != obstacle_bodies_.end();
}

std::string SimulatedPlatform::actor_for(::sim::physics::BodyHandle body) const {
 for (const auto& [id, handle] : actor_bodies_) {
 if (same(body, handle)) {
 return id;
 }
 }
 return {};
}

} // namespace sim::platform
