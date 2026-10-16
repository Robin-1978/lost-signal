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
 virtual void remove(BodyHandle body) = 0;
 virtual void set_state(BodyHandle body, const BodyState& state) = 0;
 virtual void apply_drive(BodyHandle body, const DriveForces& forces) = 0;
 virtual void apply_force_at_local_point(BodyHandle body, Vec3 force_body_N, Vec3 point_body_m) = 0;
 virtual void apply_local_torque(BodyHandle body, Vec3 torque_body_Nm) = 0;
 virtual void step(double fixed_dt_s) = 0;
 virtual BodyState state(BodyHandle body) const = 0;
 virtual const std::vector<ContactEvent>& contacts() const = 0;
 virtual RaycastResult raycast(const Ray& ray) const = 0;
};

std::unique_ptr<IPhysicsWorld> make_planar_physics_world();
std::unique_ptr<IPhysicsWorld> make_bullet_physics_world();
bool bullet_physics_available();

Quaternion quaternion_from_euler(double roll_rad, double pitch_rad, double yaw_rad);
Vec3 euler_from_quaternion(const Quaternion& rotation);

} // namespace sim::physics
