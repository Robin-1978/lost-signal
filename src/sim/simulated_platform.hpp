#pragma once

#include "mission/content.hpp"
#include "autonomy/mapping/terrain.hpp"
#include "hw/chassis/motor_drive.hpp"
#include "hw/sensors/sensor.hpp"
#include "physics/frame_transform.hpp"
#include "physics/physics_world.hpp"
#include "physics/tire_contact.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace sim::platform {

using mission::LevelDefinition;
using mission::MapDefinition;
using mission::RobotDefinition;
using mission::TerrainPatchDefinition;

struct PhysicsActorState {
 std::string id;
 ::sim::Vec2 position;
 ::sim::Vec2 velocity;
 double radius_m{0.0};
};

struct LinkContactLoad {
 std::string link;
 int contact_points{0};
 double normal_impulse_Ns{0.0};
 double normal_force_N{0.0};
 // Magnitude of the resultant tangent impulse after resolving Bullet's two
 // contact-plane basis components into world coordinates.
 double tangent_impulse_Ns{0.0};
 ::sim::physics::Vec3 tangent_impulse_world_Ns;
 double tangent_force_N{0.0};
 ::sim::physics::Vec3 tangent_force_world_N;
};

struct RigidVehicleState {
 ::sim::DiffState planar;
 // Ideal inertial truth at imu_link, expressed in imu_link. Accelerometers
 // measure specific force (point acceleration minus gravity), not a finite
 // difference of commanded planar speed. The value includes rigid-body
 // lever-arm terms when the IMU is mounted away from the body COM.
 ::sim::physics::Vec3 imu_angular_velocity_radps;
 ::sim::physics::Vec3 imu_specific_force_mps2;
 double height_m{0.0};
 double roll_rad{0.0};
 double pitch_rad{0.0};
 bool static_collision{false};
 std::vector<std::string> actor_collisions;
 int contact_count{0};
 double max_contact_impulse_Ns{0.0};
 std::vector<std::string> ground_contact_links;
 std::vector<LinkContactLoad> ground_contact_loads;
 // Model-side longitudinal tire slip truth. This is platform telemetry for
 // validation and presentation, not an encoder measurement. The ratio uses
 // |R*omega - V_contact| / max(|R*omega|, |V_contact|, 0.1 m/s).
 double max_drive_slip_ratio{0.0};
 bool drive_slipping{false};
};

// Bullet owns the vehicle pose and, for articulated platforms, all joint
// positions and velocities. Chassis controllers own targets and electronics;
// actuator feedback crosses this boundary before the next control update.
class SimulatedPlatform {
 public:
 SimulatedPlatform(const MapDefinition& map, const LevelDefinition& level, const RobotDefinition& robot,
 const ::sim::ChassisCaps& chassis_caps, const ::sim::TerrainModel& terrain,
 ::sim::Vec2 spawn, double spawn_heading_rad,
 double total_mass_kg, double center_height_m, double yaw_inertia_kgm2);

 const char* backend_name() const;
 void sync_actors(const std::vector<PhysicsActorState>& actors);
 ::sim::LidarRaycastTruth scan_lidar(int rays, double range_m,
 ::sim::Vec2 estimated_position, double estimated_heading_rad,
 double timestamp_s = 0.0, double scan_period_s = 0.0,
 const ::sim::LidarReturnModel& return_model = {}) const;
 std::optional<::sim::physics::RigidTransform> lookup_transform(
 const std::string& target_frame, const std::string& source_frame) const {
 return frames_.lookup(target_frame, source_frame);
 }
 std::vector<std::string> frame_ids() const { return frames_.frame_ids(); }
 RigidVehicleState step(const ::sim::ChassisTelemetry& actuation, const ::sim::GroundContact& ground,
 double fixed_dt_s);
 void update_load(double total_mass_kg, double center_height_m, double yaw_inertia_kgm2);
 RigidVehicleState state() const;
 ::sim::ChassisActuatorFeedback actuator_feedback() const;
 std::optional<::sim::physics::JointState> joint_state_for_child(
 const std::string& child_link) const;
 std::optional<::sim::physics::RigidTransform> link_transform_in_base(
 const std::string& child_link) const;
 bool articulated() const { return articulated_; }

 private:
 ::sim::physics::BodySpec terrain_mesh(const TerrainPatchDefinition& patch) const;
 bool is_terrain(::sim::physics::BodyHandle body) const;
 bool is_robot_body(::sim::physics::BodyHandle body) const;
 bool is_static_obstacle(::sim::physics::BodyHandle body) const;
 std::string actor_for(::sim::physics::BodyHandle body) const;
 RigidVehicleState read_state() const;
 void update_inertial_truth(const ::sim::physics::BodyState& before,
 const ::sim::physics::BodyState& after, double fixed_dt_s);
 ::sim::Scan raycast_lidar(const ::sim::physics::RigidTransform& world_from_lidar,
 int rays, double range_m, ::sim::physics::CollisionMask collision_mask,
 double timestamp_s, double scan_period_s,
 ::sim::physics::Vec3 linear_velocity_mps, double yaw_rate_radps,
 const ::sim::LidarReturnModel& return_model) const;
 void update_dynamic_frames(const ::sim::physics::BodyState& body) const;
 void update_drive_slip(RigidVehicleState& state);

 const MapDefinition& map_;
 const LevelDefinition& level_;
 const RobotDefinition& robot_;
 const ::sim::TerrainModel& terrain_;
 std::unique_ptr<::sim::physics::IPhysicsWorld> world_;
 std::unique_ptr<::sim::physics::ITireContact> tire_contact_;
 ::sim::physics::BodyHandle robot_body_;
 std::map<std::string, ::sim::physics::BodyHandle> robot_links_;
 std::map<std::uint32_t, std::string> robot_link_names_;
 std::map<std::string, ::sim::physics::JointHandle> robot_joints_;
 std::map<std::string, ::sim::physics::JointHandle> child_link_joints_;
 std::vector<::sim::physics::BodyHandle> robot_bodies_;
 std::vector<::sim::physics::BodyHandle> driven_contact_bodies_;
 struct LinkContactMaterial {
 double friction{0.0};
 double rolling_resistance{0.0};
 double spinning_resistance{0.0};
 };
 std::map<std::uint32_t, LinkContactMaterial> robot_contact_materials_;
 std::map<std::uint32_t, ::sim::physics::TireContactSpec> tire_contacts_;
 struct ActuatorBinding {
 ::sim::physics::JointHandle joint;
 double effort_limit_Nm{0.0};
 };
 std::map<std::string, ActuatorBinding> actuator_bindings_;
 double articulated_non_root_mass_kg_{0.0};
 double articulated_root_mass_kg_{0.0};
 ::sim::physics::Vec3 articulated_root_inertia_kgm2_;
 bool articulated_{false};
 std::vector<::sim::physics::BodyHandle> terrain_bodies_;
 std::vector<::sim::physics::BodyHandle> obstacle_bodies_;
 std::map<std::string, ::sim::physics::BodyHandle> actor_bodies_;
 std::map<std::string, double> actor_radii_m_;
 std::map<std::string, double> actor_heights_m_;
 mutable ::sim::physics::FrameTransformTree frames_;
 ::sim::physics::RigidTransform body_from_imu_;
 ::sim::physics::Vec3 last_imu_specific_force_mps2_{0.0, 0.0, 9.81};
 bool has_lidar_mount_{false};
 std::string lidar_frame_id_{"lidar_link"};
 double total_mass_kg_{1.0};
 double body_center_height_m_{0.2};
 double center_height_m_{0.2};
 double yaw_inertia_kgm2_{0.05};
 double support_width_m_{0.5};
 double support_length_m_{0.7};
 double drive_axle_x_m_{0.0};
 int drive_slip_confirm_steps_{0};
 int drive_slip_release_steps_{0};
 bool drive_slipping_{false};
};

} // namespace sim::platform
