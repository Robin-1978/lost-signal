#pragma once

#include "game/content.hpp"
#include "algo/terrain.hpp"
#include "hw/chassis/motor_drive.hpp"
#include "hw/sensors/sensor.hpp"
#include "physics/frame_transform.hpp"
#include "physics/physics_world.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace game {

struct PhysicsActorState {
 std::string id;
 sim::Vec2 position;
 sim::Vec2 velocity;
 double radius_m{0.0};
};

struct RigidVehicleState {
 sim::DiffState planar;
 double height_m{0.0};
 double roll_rad{0.0};
 double pitch_rad{0.0};
 bool static_collision{false};
 std::vector<std::string> actor_collisions;
 int contact_count{0};
};

// Bullet owns the vehicle pose in this adapter. Chassis implementations still
// own motors, encoders and steering actuators; their per-wheel forces enter
// here at physical contact points exactly once.
class PhysicsScene {
 public:
 PhysicsScene(const MapDefinition& map, const LevelDefinition& level, const RobotDefinition& robot,
 const sim::ChassisCaps& chassis_caps, const sim::TerrainModel& terrain,
 sim::Vec2 spawn, double spawn_heading_rad,
 double total_mass_kg, double center_height_m);

 const char* backend_name() const;
 void sync_actors(const std::vector<PhysicsActorState>& actors);
 sim::LidarRaycastTruth scan_lidar(int rays, double range_m,
 sim::Vec2 estimated_position, double estimated_heading_rad,
 double timestamp_s = 0.0, double scan_period_s = 0.0) const;
 std::optional<sim::physics::RigidTransform> lookup_transform(
 const std::string& target_frame, const std::string& source_frame) const {
 return frames_.lookup(target_frame, source_frame);
 }
 std::vector<std::string> frame_ids() const { return frames_.frame_ids(); }
 RigidVehicleState step(const sim::ChassisTelemetry& actuation, const sim::GroundContact& ground,
 double fixed_dt_s);
 RigidVehicleState state() const;

 private:
 sim::physics::BodySpec terrain_mesh(const TerrainPatchDefinition& patch) const;
 bool is_terrain(sim::physics::BodyHandle body) const;
 bool is_static_obstacle(sim::physics::BodyHandle body) const;
 std::string actor_for(sim::physics::BodyHandle body) const;
 RigidVehicleState read_state() const;
 sim::Scan raycast_lidar(const sim::physics::RigidTransform& world_from_lidar,
 int rays, double range_m, sim::physics::CollisionMask collision_mask,
 double timestamp_s, double scan_period_s,
 sim::physics::Vec3 linear_velocity_mps, double yaw_rate_radps) const;
 void update_dynamic_frames(const sim::physics::BodyState& body) const;
 void update_wheel_frames(const sim::ChassisTelemetry& actuation);

 const MapDefinition& map_;
 const LevelDefinition& level_;
 const RobotDefinition& robot_;
 const sim::TerrainModel& terrain_;
 std::unique_ptr<sim::physics::IPhysicsWorld> world_;
 sim::physics::BodyHandle robot_body_;
 std::vector<sim::physics::BodyHandle> terrain_bodies_;
 std::vector<sim::physics::BodyHandle> obstacle_bodies_;
 std::map<std::string, sim::physics::BodyHandle> actor_bodies_;
 std::map<std::string, double> actor_radii_m_;
 std::map<std::string, double> actor_heights_m_;
 mutable sim::physics::FrameTransformTree frames_;
 bool has_lidar_mount_{false};
 std::string lidar_frame_id_{"lidar_link"};
 double total_mass_kg_{1.0};
 double center_height_m_{0.2};
 double support_width_m_{0.5};
 double support_length_m_{0.7};
};

} // namespace game
