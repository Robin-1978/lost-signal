#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

namespace sim::physics {

struct Vec3 {
 double x{0.0};
 double y{0.0};
 double z{0.0};
};

struct Quaternion {
 double x{0.0};
 double y{0.0};
 double z{0.0};
 double w{1.0};
};

struct BodyHandle {
 std::uint32_t value{0};
 explicit operator bool() const { return value != 0; }
 bool operator==(BodyHandle other) const { return value == other.value; }
};

struct JointHandle {
 std::uint32_t value{0};
 explicit operator bool() const { return value != 0; }
 bool operator==(JointHandle other) const { return value == other.value; }
};

using CollisionMask = std::uint16_t;

namespace collision_layer {
inline constexpr CollisionMask Default = CollisionMask{1} << 0;
inline constexpr CollisionMask Terrain = CollisionMask{1} << 1;
inline constexpr CollisionMask StaticObstacle = CollisionMask{1} << 2;
inline constexpr CollisionMask DynamicActor = CollisionMask{1} << 3;
inline constexpr CollisionMask Vehicle = CollisionMask{1} << 4;
inline constexpr CollisionMask All = static_cast<CollisionMask>(~CollisionMask{0});
} // namespace collision_layer

enum class ShapeKind { Box, Sphere, Cylinder, Ellipsoid, HeightField };

// A complete collection of these model values is compiled before stepping.
// This describes physical semantics; it is not a runtime rigid-body command.
struct BodyModel {
 std::string name;
 ShapeKind shape{ShapeKind::Box};
 Vec3 half_extents_m{0.5, 0.5, 0.5};
 double radius_m{0.5};
 double height_m{1.0};
 double mass_kg{0.0};
 double friction{0.8};
 double rolling_friction{0.0};
 double spinning_friction{0.0};
// MuJoCo positive-format solref semantics. These are an effective contact
// response time and damping ratio, not SI spring/damper coefficients. Zero
// selects MuJoCo's documented 0.02 s, critically damped default.
double contact_time_constant_s{0.0};
double contact_damping_ratio{0.0};
 Vec3 contact_axle_axis_local;
 // Per-body drag coefficients applied at the body's inertial origin. Units
 // are N/(m/s) and N*m/(rad/s); articulated child links are included.
 double linear_damping{0.0};
 double angular_damping{0.0};
 // Inertial origin expressed in the body frame. Geometry and TF remain at
 // the body origin while mass, gravity, and moments act about this COM.
 Vec3 inertial_position_m;
 Vec3 inertia_diagonal_kgm2;
 // Disable MuJoCo's simple-body allocation only when this body's inertial
 // origin may move after compilation (for example a payload-bearing root).
 bool movable_inertial_origin{false};
 // Native regular height field. Samples are row-major from minimum Y to
 // maximum Y, each row from minimum X to maximum X. They are local heights
 // in metres above position_m.z. half_extents_m.x/y are horizontal radii.
 std::size_t heightfield_rows{0};
 std::size_t heightfield_cols{0};
 std::vector<double> heightfield_elevation_m;
 double heightfield_base_depth_m{0.1};
 Vec3 position_m;
 Quaternion rotation;
 // Externally prescribed dynamic proxy. It has a free joint and an
 // auto-derived mass/inertia so MuJoCo sees its authored velocity in contact
 // Jacobians; the provider rewrites pose/velocity at each outer update.
 // This deliberately does not use a mocap body, whose motion has no physical
 // velocity in MuJoCo contact dynamics.
 bool kinematic{false};
 CollisionMask collision_group{collision_layer::Default};
 CollisionMask collision_mask{collision_layer::All};
};

enum class JointKind { Fixed, Revolute };

struct JointModel {
 std::string name;
 JointKind kind{JointKind::Fixed};
 BodyHandle parent;
 BodyHandle child;
 Vec3 pivot_in_parent_m;
 Vec3 pivot_in_child_m;
 Vec3 axis_in_parent{0.0, 0.0, 1.0};
 Vec3 axis_in_child{0.0, 0.0, 1.0};
 bool has_limits{false};
 double lower_limit_rad{0.0};
 double upper_limit_rad{0.0};
 double armature_inertia_kgm2{0.0};
 // Passive joint properties. For a revolute joint these have units of
 // N*m*s/rad and N*m respectively. They are authored hardware properties,
 // independent of actuator control and contact compliance.
 double damping_Nms_per_rad{0.0};
 double friction_loss_Nm{0.0};
};

struct JointState {
 double position_rad{0.0};
 double velocity_radps{0.0};
};

struct BodyState {
 Vec3 position_m;
 Quaternion rotation;
 Vec3 linear_velocity_mps;
 Vec3 angular_velocity_radps;
};

struct DriveForces {
 double forward_force_N{0.0};
 double lateral_force_N{0.0};
 double yaw_torque_Nm{0.0};
 double brake_force_N{0.0};
};

struct ContactEvent {
 BodyHandle body_a;
 BodyHandle body_b;
 Vec3 point_m;
 Vec3 normal_on_b;
 double impulse_Ns{0.0};
 double normal_force_N{0.0};
 Vec3 tangent_direction_1;
 Vec3 tangent_direction_2;
 double tangent_impulse_1_Ns{0.0};
 double tangent_impulse_2_Ns{0.0};
 double tangent_force_1_N{0.0};
 double tangent_force_2_N{0.0};
};

struct Ray {
 Vec3 origin_m;
 Vec3 direction;
 double max_distance_m{0.0};
 CollisionMask collision_mask{collision_layer::All};
 BodyHandle ignore_body;

 Ray(Vec3 origin = {}, Vec3 direction_value = {}, double max_distance = 0.0,
 CollisionMask mask = collision_layer::All, BodyHandle ignored = {})
 : origin_m(origin), direction(direction_value), max_distance_m(max_distance),
 collision_mask(mask), ignore_body(ignored) {}
};

struct RaycastResult {
 bool hit{false};
 BodyHandle body;
 Vec3 point_m;
 Vec3 normal;
 double distance_m{0.0};
};

// Owns the complete authored model and one authoritative mjData instance.
// add_* and remove_body change topology and require an explicit compile()
// transaction. Recompilation preserves state for surviving stable handles.
class MujocoSimulation {
 public:
 MujocoSimulation();
 ~MujocoSimulation();
 MujocoSimulation(MujocoSimulation&&) noexcept;
 MujocoSimulation& operator=(MujocoSimulation&&) noexcept;
 MujocoSimulation(const MujocoSimulation&) = delete;
 MujocoSimulation& operator=(const MujocoSimulation&) = delete;

 std::string_view engine_name() const { return "mujoco"; }
 void clear_model();
 BodyHandle add_body(const BodyModel& body);
 JointHandle add_joint(const JointModel& joint);
 void remove_body(BodyHandle body);
 void compile();
 bool compiled() const;
 // Preserve the settled configuration while clearing every generalized
 // velocity, including passive articulation joints.
 void clear_velocities();

 void set_state(BodyHandle body, const BodyState& state);
 void set_mass_properties(BodyHandle body, double mass_kg, Vec3 inertia_diagonal_kgm2,
 Vec3 inertial_position_m = {});
 void set_contact_parameters(BodyHandle body, double friction,
 double rolling_friction, double spinning_friction,
double time_constant_s, double damping_ratio);
 void apply_drive(BodyHandle body, const DriveForces& forces);
 void apply_force_at_local_point(BodyHandle body, Vec3 force_body_N, Vec3 point_body_m);
 void apply_force_at_world_point(BodyHandle body, Vec3 force_world_N, Vec3 point_world_m);
 void apply_local_torque(BodyHandle body, Vec3 torque_body_Nm);
 void apply_world_torque(BodyHandle body, Vec3 torque_world_Nm);
 void apply_joint_effort(JointHandle joint, double effort_Nm);
 // Register a passive brake capacity for the next public step. The provider
 // reevaluates direction from joint velocity at every internal substep, so a
 // brake cannot propel a joint through zero as a sampled external torque can.
 void apply_joint_brake(JointHandle joint, double maximum_torque_Nm,
 double low_speed_smoothing_radps);
 void step(double fixed_dt_s);
 BodyState state(BodyHandle body) const;
 JointState joint_state(JointHandle joint) const;
 const std::vector<ContactEvent>& contacts() const;
 RaycastResult raycast(const Ray& ray) const;

 private:
 class Impl;
 std::unique_ptr<Impl> impl_;
};

bool mujoco_physics_available();

Quaternion quaternion_from_euler(double roll_rad, double pitch_rad, double yaw_rad);
Vec3 euler_from_quaternion(const Quaternion& rotation);

} // namespace sim::physics
