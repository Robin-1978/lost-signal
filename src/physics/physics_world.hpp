#pragma once

#include <cstdint>
#include <memory>
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

// Semantic collision layers keep sensor visibility separate from contact
// response. Bodies still collide according to collision_mask; ray queries can
// select terrain, authored obstacles, actors, or vehicles independently.
namespace collision_layer {
inline constexpr CollisionMask Default = CollisionMask{1} << 0;
inline constexpr CollisionMask Terrain = CollisionMask{1} << 1;
inline constexpr CollisionMask StaticObstacle = CollisionMask{1} << 2;
inline constexpr CollisionMask DynamicActor = CollisionMask{1} << 3;
inline constexpr CollisionMask Vehicle = CollisionMask{1} << 4;
inline constexpr CollisionMask All = static_cast<CollisionMask>(~CollisionMask{0});
} // namespace collision_layer

enum class ShapeKind { Box, Sphere, Cylinder, TriangleMesh };

struct BodySpec {
 ShapeKind shape{ShapeKind::Box};
 Vec3 half_extents_m{0.5, 0.5, 0.5};
 double radius_m{0.5};
 double height_m{1.0};
 double mass_kg{0.0};
 double friction{0.8};
 double rolling_friction{0.0};
 double spinning_friction{0.0};
 // Local-axis scaling for Bullet-style rolling moment constraints. A tire
 // can retain finite contact-patch support about its non-rolling axes while
 // using Crr * radius about the wheel axle.
 Vec3 rolling_friction_anisotropy{1.0, 1.0, 1.0};
 // Preserve a positional static-friction anchor while a manifold contact
 // persists. This is useful for slowly loaded wheel contacts and must not be
 // confused with rolling or spinning resistance.
 bool friction_anchor{false};
 // Optional compliant contact parameters. A zero stiffness keeps the physics
 // engine's rigid-contact default. Units match a linear spring/damper at the
 // contact patch.
 double contact_stiffness_Npm{0.0};
 double contact_damping_Nspm{0.0};
 // A non-zero local axle enables a wheel-aligned contact frame. The physics
 // backend resolves longitudinal and lateral friction from this axis and the
 // contact normal instead of choosing directions from instantaneous slip.
 Vec3 contact_axle_axis_local;
 double restitution{0.0};
 double linear_damping{0.0};
 double angular_damping{0.0};
 bool continuous_collision{false};
 // Optional body-space principal inertia. Zero means derive it from shape and
 // mass. Vehicle content supplies yaw inertia so controller and rigid body use
 // the same hardware value.
 Vec3 inertia_diagonal_kgm2;
 // Static triangle mesh data. Indices are triples into mesh_vertices_m.
 std::vector<Vec3> mesh_vertices_m;
 std::vector<std::uint32_t> mesh_indices;
 Vec3 position_m;
 Quaternion rotation;
 bool kinematic{false};
 CollisionMask collision_group{collision_layer::Default};
 CollisionMask collision_mask{collision_layer::All};
};

enum class JointKind { Fixed, Revolute };

// Generic articulated-body boundary. Pivots and axes are expressed in the
// respective parent / child body frames. A revolute joint reports child
// motion about axis_in_parent; effort is positive about that same axis.
struct JointSpec {
 JointKind kind{JointKind::Fixed};
 BodyHandle parent;
 BodyHandle child;
 Vec3 pivot_in_parent_m;
 Vec3 pivot_in_child_m;
 Vec3 axis_in_parent{0.0, 0.0, 1.0};
 Vec3 axis_in_child{0.0, 0.0, 1.0};
 bool disable_parent_child_collision{true};
 bool has_limits{false};
 double lower_limit_rad{0.0};
 double upper_limit_rad{0.0};
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
 // Friction impulse is resolved in two orthogonal directions tangent to the
 // contact plane. Together these components can represent any in-plane
 // direction; consumers must not interpret them as two separate contacts.
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

class IPhysicsWorld {
 public:
 virtual ~IPhysicsWorld() = default;

 virtual std::string_view backend_name() const = 0;
 virtual void reset() = 0;
 virtual BodyHandle spawn(const BodySpec& spec) = 0;
 virtual JointHandle create_joint(const JointSpec& spec) = 0;
 virtual void remove(BodyHandle body) = 0;
 virtual void set_state(BodyHandle body, const BodyState& state) = 0;
 virtual void set_mass_properties(BodyHandle body, double mass_kg,
 Vec3 inertia_diagonal_kgm2) = 0;
 virtual void set_contact_friction(BodyHandle body, double friction,
 double rolling_friction, double spinning_friction) = 0;
 virtual void set_contact_rolling_anisotropy(BodyHandle body, Vec3 local_scale) = 0;
 virtual void set_contact_compliance(BodyHandle body, double stiffness_Npm,
 double damping_Nspm) = 0;
 virtual void apply_drive(BodyHandle body, const DriveForces& forces) = 0;
 virtual void apply_force_at_local_point(BodyHandle body, Vec3 force_body_N, Vec3 point_body_m) = 0;
 virtual void apply_force_at_world_point(BodyHandle body, Vec3 force_world_N,
 Vec3 point_world_m) = 0;
 virtual void apply_local_torque(BodyHandle body, Vec3 torque_body_Nm) = 0;
 virtual void apply_world_torque(BodyHandle body, Vec3 torque_world_Nm) = 0;
 virtual void apply_joint_effort(JointHandle joint, double effort_Nm) = 0;
 virtual void step(double fixed_dt_s) = 0;
 virtual BodyState state(BodyHandle body) const = 0;
 virtual JointState joint_state(JointHandle joint) const = 0;
 virtual const std::vector<ContactEvent>& contacts() const = 0;
 virtual RaycastResult raycast(const Ray& ray) const = 0;
};

std::unique_ptr<IPhysicsWorld> make_planar_physics_world();
std::unique_ptr<IPhysicsWorld> make_bullet_physics_world();
bool bullet_physics_available();

Quaternion quaternion_from_euler(double roll_rad, double pitch_rad, double yaw_rad);
Vec3 euler_from_quaternion(const Quaternion& rotation);

} // namespace sim::physics
