#include "physics_scene.hpp"

#include <algorithm>
#include <cmath>
#include <set>
#include <stdexcept>

namespace game {
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
 const double edge_fade = std::sin(sim::kPi * u) * std::sin(sim::kPi * v);
 const double wave = std::sin(2.0 * sim::kPi * (x - patch.min.x) / 1.35) *
 std::sin(2.0 * sim::kPi * (y - patch.min.y) / 0.95);
 return plane + patch.roughness * edge_fade * wave;
}

bool same(sim::physics::BodyHandle a, sim::physics::BodyHandle b) {
 return a && b && a == b;
}

} // namespace

PhysicsScene::PhysicsScene(const MapDefinition& map, const LevelDefinition& level,
 const RobotDefinition& robot, const sim::ChassisCaps& chassis_caps,
 const sim::TerrainModel& terrain, sim::Vec2 spawn,
 double spawn_heading_rad, double total_mass_kg, double center_height_m)
 : map_(map), level_(level), robot_(robot), terrain_(terrain),
 world_(sim::physics::make_bullet_physics_world()), total_mass_kg_(std::max(0.5, total_mass_kg)),
 center_height_m_(std::max(0.05, center_height_m)) {
 support_width_m_ = std::max(0.05, chassis_caps.geometry.support_width_m);
 support_length_m_ = std::max(0.05, chassis_caps.geometry.support_length_m);

 sim::physics::BodySpec floor;
 floor.shape = sim::physics::ShapeKind::Box;
 floor.half_extents_m = {map_.size_m.x * 0.5, map_.size_m.y * 0.5, 0.10};
 floor.position_m = {map_.size_m.x * 0.5, map_.size_m.y * 0.5, -0.10};
 floor.friction = 0.0;
 floor.collision_group = sim::physics::collision_layer::Terrain;
 terrain_bodies_.push_back(world_->spawn(floor));

 // Keep an authoritative rigid body inside the authored world. The domain
 // grid already rejects out-of-bounds poses; Bullet needs the same boundary
 // represented as contact geometry instead of allowing a long free fall.
 const double wall_height = 2.5;
 const double wall_thickness = 0.10;
 const sim::physics::Vec3 wall_half_x{map_.size_m.x * 0.5 + wall_thickness,
 wall_thickness, wall_height};
 const sim::physics::Vec3 wall_half_y{wall_thickness,
 map_.size_m.y * 0.5 + wall_thickness, wall_height};
 const std::pair<sim::physics::Vec3, sim::physics::Vec3> walls[] = {
 {wall_half_x, {map_.size_m.x * 0.5, -wall_thickness, wall_height}},
 {wall_half_x, {map_.size_m.x * 0.5, map_.size_m.y + wall_thickness, wall_height}},
 {wall_half_y, {-wall_thickness, map_.size_m.y * 0.5, wall_height}},
 {wall_half_y, {map_.size_m.x + wall_thickness, map_.size_m.y * 0.5, wall_height}},
 };
 for (const auto& [half_extents, position] : walls) {
 sim::physics::BodySpec wall;
 wall.shape = sim::physics::ShapeKind::Box;
 wall.half_extents_m = half_extents;
 wall.position_m = position;
 wall.friction = 0.2;
 wall.collision_group = sim::physics::collision_layer::StaticObstacle;
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
 sim::physics::BodySpec body;
 body.shape = sim::physics::ShapeKind::Box;
 body.half_extents_m = {(obstacle.max.x - obstacle.min.x) * 0.5,
 (obstacle.max.y - obstacle.min.y) * 0.5, 0.75};
 body.position_m = {(obstacle.min.x + obstacle.max.x) * 0.5,
 (obstacle.min.y + obstacle.max.y) * 0.5, 0.75};
 body.friction = 0.2;
 body.collision_group = sim::physics::collision_layer::StaticObstacle;
 obstacle_bodies_.push_back(world_->spawn(body));
 }

 const auto ground = terrain_.sample(spawn);
 const double forward_grade = sim::longitudinal_grade(ground, spawn_heading_rad);
 const double cross_grade = sim::lateral_grade(ground, spawn_heading_rad);
 const double roll = std::atan(cross_grade);
 const double pitch = -std::atan(forward_grade);
 sim::physics::BodySpec vehicle;
 vehicle.shape = sim::physics::ShapeKind::Box;
 double collision_half_length = support_length_m_ * 0.5;
 double collision_half_width = support_width_m_ * 0.5;
 const double footprint_radius = std::hypot(collision_half_length, collision_half_width);
 if (footprint_radius > robot_.collision_radius_m && robot_.collision_radius_m > 0.0) {
 const double scale = robot_.collision_radius_m * 0.98 / footprint_radius;
 collision_half_length *= scale;
 collision_half_width *= scale;
 }
 vehicle.half_extents_m = {collision_half_length, collision_half_width, center_height_m_};
 vehicle.mass_kg = total_mass_kg_;
 vehicle.friction = 0.0;
 vehicle.linear_damping = 0.03;
 vehicle.angular_damping = 0.10;
 vehicle.continuous_collision = true;
 vehicle.collision_group = sim::physics::collision_layer::Vehicle;
 const double inertia_roll = std::max(0.05, total_mass_kg_ *
 (support_width_m_ * support_width_m_ + 4.0 * center_height_m_ * center_height_m_) / 12.0);
 const double inertia_pitch = std::max(0.05, total_mass_kg_ *
 (support_length_m_ * support_length_m_ + 4.0 * center_height_m_ * center_height_m_) / 12.0);
 vehicle.inertia_diagonal_kgm2 = {inertia_roll, inertia_pitch, std::max(0.05, robot_.yaw_inertia_kgm2)};
 const double vertical_half_extent = center_height_m_ /
 std::max(0.25, std::cos(roll) * std::cos(pitch));
 vehicle.position_m = sim::physics::Vec3{spawn.x, spawn.y, ground.height_m + vertical_half_extent + 0.01};
 vehicle.rotation = sim::physics::quaternion_from_euler(roll, pitch, spawn_heading_rad);
 robot_body_ = world_->spawn(vehicle);

 constexpr double kDegreesToRadians = sim::kPi / 180.0;
 for (const auto& mount : robot_.mounts) {
 frames_.set_transform("base_link", mount.frame_id, {
 {mount.position_m.x, mount.position_m.y, mount.position_m.z},
 sim::physics::quaternion_from_euler(mount.rotation_deg.x * kDegreesToRadians,
 mount.rotation_deg.y * kDegreesToRadians,
 mount.rotation_deg.z * kDegreesToRadians),
 });
 }
 if (!frames_.lookup("base_link", "imu_link")) {
 frames_.set_transform("base_link", "imu_link", {});
 }
 frames_.set_transform("map", "odom", {});
 update_dynamic_frames(world_->state(robot_body_));
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

const char* PhysicsScene::backend_name() const {
 return world_ ? world_->backend_name().data() : "none";
}

sim::physics::BodySpec PhysicsScene::terrain_mesh(const TerrainPatchDefinition& patch) const {
 sim::physics::BodySpec body;
 body.shape = sim::physics::ShapeKind::TriangleMesh;
 body.friction = 0.0;
 body.collision_group = sim::physics::collision_layer::Terrain;
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
 const sim::Vec2 corners[] = {{patch.min.x, patch.min.y}, {patch.max.x, patch.min.y},
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

void PhysicsScene::sync_actors(const std::vector<PhysicsActorState>& actors) {
 std::set<std::string> active_ids;
 for (const auto& actor : actors) {
 active_ids.insert(actor.id);
 auto found = actor_bodies_.find(actor.id);
 if (found == actor_bodies_.end()) {
 sim::physics::BodySpec body;
 body.shape = sim::physics::ShapeKind::Cylinder;
 body.radius_m = actor.radius_m;
 body.height_m = std::max(0.5, actor.radius_m * 2.0);
 body.kinematic = true;
 body.friction = 0.2;
 body.collision_group = sim::physics::collision_layer::DynamicActor;
 body.position_m = sim::physics::Vec3{actor.position.x, actor.position.y,
 terrain_.sample(actor.position).height_m + body.height_m * 0.5};
 found = actor_bodies_.emplace(actor.id, world_->spawn(body)).first;
 actor_radii_m_[actor.id] = actor.radius_m;
 actor_heights_m_[actor.id] = body.height_m;
 }
 sim::physics::BodyState state = world_->state(found->second);
 state.position_m = sim::physics::Vec3{actor.position.x, actor.position.y,
 terrain_.sample(actor.position).height_m + std::max(0.5, actor.radius_m * 2.0) * 0.5};
 state.linear_velocity_mps = {actor.velocity.x, actor.velocity.y, 0.0};
 if (std::hypot(actor.velocity.x, actor.velocity.y) > 1e-6) {
 state.rotation = sim::physics::quaternion_from_euler(0.0, 0.0,
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

sim::Scan PhysicsScene::raycast_lidar(const sim::physics::RigidTransform& world_from_lidar,
 int rays, double range_m, sim::physics::CollisionMask collision_mask,
 double timestamp_s, double scan_period_s,
 sim::physics::Vec3 linear_velocity_mps, double yaw_rate_radps) const {
 if (rays <= 0 || !(range_m > 0.0)) {
 throw std::invalid_argument("physics lidar needs positive rays and range");
 }
 sim::Scan scan;
 scan.n_rays = rays;
 scan.max_range = range_m;
 scan.angle_increment_rad = 2.0 * sim::kPi / static_cast<double>(rays);
 scan.scan_period_s = scan_period_s;
 scan.time_increment_s = scan_period_s > 0.0 ? scan_period_s / static_cast<double>(rays) : 0.0;
 scan.header = {timestamp_s, 0, lidar_frame_id_, 0.0};
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
 sample_from_lidar.rotation = sim::physics::quaternion_multiply(
 sim::physics::quaternion_from_euler(0.0, 0.0, yaw_rate_radps * time_offset_s),
 sample_from_lidar.rotation);
 const double angle = 2.0 * sim::kPi * static_cast<double>(ray) / static_cast<double>(rays);
 const auto direction = sim::physics::rotate_vector(sample_from_lidar.rotation,
 {std::cos(angle), std::sin(angle), 0.0});
 const bool include_dynamic =
 (collision_mask & sim::physics::collision_layer::DynamicActor) != 0;
 const auto static_only_mask = static_cast<sim::physics::CollisionMask>(
 collision_mask & ~sim::physics::collision_layer::DynamicActor);
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
 double reflectivity = 0.50;
 if (is_terrain(hit.body)) reflectivity = 0.30;
 else if (is_static_obstacle(hit.body)) reflectivity = 0.68;
 else if (!actor_for(hit.body).empty()) reflectivity = 0.52;
 const double attenuation = 1.0 / (1.0 + 0.018 * hit.distance_m * hit.distance_m);
 intensity = std::clamp(reflectivity * (0.15 + 0.85 * incidence) * attenuation, 0.0, 1.0);
 }
 scan.intensities.push_back(intensity);
 }
 return scan;
}

sim::LidarRaycastTruth PhysicsScene::scan_lidar(int rays, double range_m,
 sim::Vec2 estimated_position, double estimated_heading_rad,
 double timestamp_s, double scan_period_s) const {
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
 const double estimated_roll = std::atan(sim::lateral_grade(estimated_ground, estimated_heading_rad));
 const double estimated_pitch = -std::atan(sim::longitudinal_grade(estimated_ground,
 estimated_heading_rad));
 const sim::physics::RigidTransform world_from_estimated_base{
 {estimated_position.x, estimated_position.y, estimated_ground.height_m},
 sim::physics::quaternion_from_euler(estimated_roll, estimated_pitch, estimated_heading_rad),
 };
 const auto base_from_lidar = frames_.lookup("base_link", lidar_frame_id_);
 if (!base_from_lidar) {
 throw std::runtime_error("physics frame tree cannot resolve base_link to lidar_link");
 }
 const auto world_from_estimated_lidar = sim::physics::compose(world_from_estimated_base,
 *base_from_lidar);

 constexpr auto static_mask = static_cast<sim::physics::CollisionMask>(
 sim::physics::collision_layer::Terrain | sim::physics::collision_layer::StaticObstacle);
 constexpr auto combined_mask = static_cast<sim::physics::CollisionMask>(
 static_mask | sim::physics::collision_layer::DynamicActor);
 return {
 raycast_lidar(*world_from_lidar, rays, range_m, combined_mask, timestamp_s,
 scan_period_s, body.linear_velocity_mps, body.angular_velocity_radps.z),
 raycast_lidar(world_from_estimated_lidar, rays, range_m, static_mask, timestamp_s,
 scan_period_s, body.linear_velocity_mps, body.angular_velocity_radps.z),
 };
}

void PhysicsScene::update_dynamic_frames(const sim::physics::BodyState& body) const {
 const auto angles = sim::physics::euler_from_quaternion(body.rotation);
 const auto center_offset = sim::physics::rotate_vector(body.rotation, {0.0, 0.0, center_height_m_});
 const sim::physics::Vec3 base_origin{
 body.position_m.x - center_offset.x,
 body.position_m.y - center_offset.y,
 body.position_m.z - center_offset.z,
 };
 frames_.set_transform("map", "odom", {});
 frames_.set_transform("odom", "base_footprint", {
 {base_origin.x, base_origin.y, 0.0},
 sim::physics::quaternion_from_euler(0.0, 0.0, angles.z),
 });
 frames_.set_transform("base_footprint", "base_link", {
 {0.0, 0.0, base_origin.z},
 sim::physics::quaternion_from_euler(angles.x, angles.y, 0.0),
 });
}

void PhysicsScene::update_wheel_frames(const sim::ChassisTelemetry& actuation) {
 const auto tire_slot = std::find_if(robot_.slots.begin(), robot_.slots.end(),
 [](const auto& slot) { return slot.slot == "tires"; });
 constexpr double kDegreesToRadians = sim::kPi / 180.0;
 const auto update_mount = [&](const std::string& mount_id, double steer) {
 const auto mount = std::find_if(robot_.mounts.begin(), robot_.mounts.end(),
 [&](const auto& candidate) { return candidate.id == mount_id; });
 if (mount == robot_.mounts.end())
 return;
 frames_.set_transform("base_link", mount->frame_id, {
 {mount->position_m.x, mount->position_m.y, mount->position_m.z},
 sim::physics::quaternion_from_euler(mount->rotation_deg.x * kDegreesToRadians,
 mount->rotation_deg.y * kDegreesToRadians,
 mount->rotation_deg.z * kDegreesToRadians + steer),
 });
 };
 if (tire_slot != robot_.slots.end()) {
 for (std::size_t index = 0; index < tire_slot->mounts.size(); ++index) {
 update_mount(tire_slot->mounts[index],
 index < actuation.wheel_steer_count ? actuation.wheel_steer_rad[index] : 0.0);
 }
 }
 const std::size_t passive_count = std::min(robot_.passive_support_mounts.size(),
 actuation.passive_caster_steer_count);
 for (std::size_t index = 0; index < passive_count; ++index) {
 update_mount(robot_.passive_support_mounts[index],
 actuation.passive_caster_steer_rad[index]);
 }
}

RigidVehicleState PhysicsScene::step(const sim::ChassisTelemetry& actuation,
 const sim::GroundContact& ground, double fixed_dt_s) {
 update_wheel_frames(actuation);
 const auto before = world_->state(robot_body_);
 const auto angles = sim::physics::euler_from_quaternion(before.rotation);
 const double yaw = angles.z;
 const double lateral_speed = -std::sin(yaw) * before.linear_velocity_mps.x +
 std::cos(yaw) * before.linear_velocity_mps.y;
 const double contact_z = -center_height_m_ + 0.02;

 if (actuation.holonomic) {
 world_->apply_force_at_local_point(robot_body_,
 {actuation.body_forward_force_N, actuation.body_lateral_force_N, 0.0},
 {0.0, 0.0, contact_z});
 } else if (actuation.differential) {
 world_->apply_force_at_local_point(robot_body_, {actuation.left.longitudinal_force_N, 0.0, 0.0},
 {0.0, support_width_m_ * 0.5, contact_z});
 world_->apply_force_at_local_point(robot_body_, {actuation.right.longitudinal_force_N, 0.0, 0.0},
 {0.0, -support_width_m_ * 0.5, contact_z});
 } else {
 world_->apply_force_at_local_point(robot_body_, {actuation.drive.longitudinal_force_N, 0.0, 0.0},
 {-support_length_m_ * 0.5, 0.0, contact_z});
 }

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
 double yaw_torque = actuation.extra_yaw_torque_Nm;
 if (actuation.yaw_rate_tracking_time_s > 0.0) {
 // A chassis may request a distributed-contact yaw response in addition to
 // its point resultants. PhysicsScene consumes the actuator contract without
 // knowing whether the source is a track, wheel module, or future mechanism.
 const double yaw_inertia = std::max(0.05, robot_.yaw_inertia_kgm2 +
 level_.payload_mass_kg * robot_.collision_radius_m * robot_.collision_radius_m);
 yaw_torque += yaw_inertia *
 (actuation.desired_yaw_rate_radps - before.angular_velocity_radps.z) /
 actuation.yaw_rate_tracking_time_s;
 }
 world_->apply_local_torque(robot_body_,
 {0.0, 0.0, std::clamp(yaw_torque, -yaw_limit, yaw_limit)});
 world_->step(fixed_dt_s);

 update_dynamic_frames(world_->state(robot_body_));
 auto output = read_state();
 for (const auto& contact : world_->contacts()) {
 sim::physics::BodyHandle other;
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
 }
 const std::string actor = actor_for(other);
 if (!actor.empty()) {
 output.actor_collisions.push_back(actor);
 }
 }
 return output;
}

RigidVehicleState PhysicsScene::state() const {
 update_dynamic_frames(world_->state(robot_body_));
 return read_state();
}

RigidVehicleState PhysicsScene::read_state() const {
 const auto body = world_->state(robot_body_);
 const auto angles = sim::physics::euler_from_quaternion(body.rotation);
 const double forward_speed = std::cos(angles.z) * body.linear_velocity_mps.x +
 std::sin(angles.z) * body.linear_velocity_mps.y;
 const double lateral_speed = -std::sin(angles.z) * body.linear_velocity_mps.x +
 std::cos(angles.z) * body.linear_velocity_mps.y;
 RigidVehicleState output;
 output.planar = {body.position_m.x, body.position_m.y, sim::wrap_angle(angles.z), forward_speed,
 body.angular_velocity_radps.z, lateral_speed};
 output.height_m = body.position_m.z;
 output.roll_rad = angles.x;
 output.pitch_rad = angles.y;
 return output;
}

bool PhysicsScene::is_terrain(sim::physics::BodyHandle body) const {
 return std::find(terrain_bodies_.begin(), terrain_bodies_.end(), body) != terrain_bodies_.end();
}

bool PhysicsScene::is_static_obstacle(sim::physics::BodyHandle body) const {
 return std::find(obstacle_bodies_.begin(), obstacle_bodies_.end(), body) != obstacle_bodies_.end();
}

std::string PhysicsScene::actor_for(sim::physics::BodyHandle body) const {
 for (const auto& [id, handle] : actor_bodies_) {
 if (same(body, handle)) {
 return id;
 }
 }
 return {};
}

} // namespace game
