#include "physics_world.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <vector>

namespace sim::physics {
namespace {

constexpr double kGravityMps2 = 9.81;

struct StoredBody {
 BodySpec spec;
 BodyState state;
 Vec3 force_N;
 Vec3 torque_Nm;
 bool active{true};
};

double length(Vec3 value) {
 return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 normalized(Vec3 value) {
 const double magnitude = length(value);
 if (magnitude < 1e-12) {
  return {};
 }
 return {value.x / magnitude, value.y / magnitude, value.z / magnitude};
}

double yaw_of(const Quaternion& rotation) {
 return euler_from_quaternion(rotation).z;
}

double support_height(const BodySpec& spec) {
 switch (spec.shape) {
 case ShapeKind::Box:
  return spec.half_extents_m.z;
 case ShapeKind::Sphere:
  return spec.radius_m;
 case ShapeKind::Cylinder:
  return spec.height_m * 0.5;
 case ShapeKind::TriangleMesh:
 return 0.0;
 }
 return 0.0;
}

class PlanarPhysicsWorld final : public IPhysicsWorld {
 public:
 std::string_view backend_name() const override { return "planar"; }

 void reset() override {
  bodies_.clear();
  contacts_.clear();
 }

 BodyHandle spawn(const BodySpec& spec) override {
 if (spec.mass_kg < 0.0 || spec.friction < 0.0 || spec.restitution < 0.0 ||
 spec.collision_group == 0) {
   throw std::invalid_argument("invalid physics body properties");
  }
  StoredBody body;
  body.spec = spec;
  body.state.position_m = spec.position_m;
  body.state.rotation = spec.rotation;
  bodies_.push_back(body);
  return {static_cast<std::uint32_t>(bodies_.size())};
 }

 void remove(BodyHandle body) override { checked(body).active = false; }

 void set_state(BodyHandle body, const BodyState& state) override { checked(body).state = state; }

 void apply_drive(BodyHandle body, const DriveForces& drive) override {
  auto& stored = checked(body);
  if (stored.spec.mass_kg <= 0.0 || stored.spec.kinematic) {
   return;
  }
  const double yaw = yaw_of(stored.state.rotation);
  const double forward_x = std::cos(yaw);
  const double forward_y = std::sin(yaw);
  const double lateral_x = -forward_y;
  const double lateral_y = forward_x;
  stored.force_N.x += forward_x * drive.forward_force_N + lateral_x * drive.lateral_force_N;
  stored.force_N.y += forward_y * drive.forward_force_N + lateral_y * drive.lateral_force_N;
  const double forward_speed =
    stored.state.linear_velocity_mps.x * forward_x + stored.state.linear_velocity_mps.y * forward_y;
  if (std::abs(forward_speed) > 1e-9 && drive.brake_force_N > 0.0) {
   const double braking = std::copysign(drive.brake_force_N, forward_speed);
   stored.force_N.x -= forward_x * braking;
   stored.force_N.y -= forward_y * braking;
  }
  stored.torque_Nm.z += drive.yaw_torque_Nm;
 }

 void apply_force_at_local_point(BodyHandle body, Vec3 force_body_N, Vec3 point_body_m) override {
 auto& stored = checked(body);
 if (stored.spec.mass_kg <= 0.0 || stored.spec.kinematic) {
 return;
 }
 const double yaw = yaw_of(stored.state.rotation);
 const double cosine = std::cos(yaw);
 const double sine = std::sin(yaw);
 const Vec3 force_world{cosine * force_body_N.x - sine * force_body_N.y,
 sine * force_body_N.x + cosine * force_body_N.y,
 force_body_N.z};
 const Vec3 point_world{cosine * point_body_m.x - sine * point_body_m.y,
 sine * point_body_m.x + cosine * point_body_m.y,
 point_body_m.z};
 stored.force_N.x += force_world.x;
 stored.force_N.y += force_world.y;
 stored.force_N.z += force_world.z;
 stored.torque_Nm.z += point_world.x * force_world.y - point_world.y * force_world.x;
 }

 void apply_local_torque(BodyHandle body, Vec3 torque_body_Nm) override {
 auto& stored = checked(body);
 if (stored.spec.mass_kg <= 0.0 || stored.spec.kinematic) {
 return;
 }
 stored.torque_Nm.z += torque_body_Nm.z;
 }

 void step(double fixed_dt_s) override {
  if (!(fixed_dt_s > 0.0) || !std::isfinite(fixed_dt_s)) {
   throw std::invalid_argument("physics step must be finite and positive");
  }
  contacts_.clear();
  for (std::size_t index = 0; index < bodies_.size(); ++index) {
   auto& body = bodies_[index];
   if (!body.active || body.spec.mass_kg <= 0.0 || body.spec.kinematic) {
    body.force_N = {};
    body.torque_Nm = {};
    continue;
   }
   const double inverse_mass = 1.0 / body.spec.mass_kg;
   body.state.linear_velocity_mps.x += body.force_N.x * inverse_mass * fixed_dt_s;
   body.state.linear_velocity_mps.y += body.force_N.y * inverse_mass * fixed_dt_s;
   body.state.linear_velocity_mps.z += (body.force_N.z * inverse_mass - kGravityMps2) * fixed_dt_s;
 const double inertia_z = body.spec.inertia_diagonal_kgm2.z > 0.0
 ? body.spec.inertia_diagonal_kgm2.z
 : std::max(0.05, body.spec.mass_kg *
                                             (body.spec.half_extents_m.x * body.spec.half_extents_m.x +
                                              body.spec.half_extents_m.y * body.spec.half_extents_m.y) /
 3.0);
   body.state.angular_velocity_radps.z += body.torque_Nm.z / inertia_z * fixed_dt_s;
   body.state.position_m.x += body.state.linear_velocity_mps.x * fixed_dt_s;
   body.state.position_m.y += body.state.linear_velocity_mps.y * fixed_dt_s;
   body.state.position_m.z += body.state.linear_velocity_mps.z * fixed_dt_s;
   const Vec3 angles = euler_from_quaternion(body.state.rotation);
   body.state.rotation =
     quaternion_from_euler(angles.x, angles.y, angles.z + body.state.angular_velocity_radps.z * fixed_dt_s);

   const double floor_z = support_height(body.spec);
   if (body.state.position_m.z < floor_z) {
    body.state.position_m.z = floor_z;
    const double impact_speed = std::max(0.0, -body.state.linear_velocity_mps.z);
    body.state.linear_velocity_mps.z = impact_speed * body.spec.restitution;
    contacts_.push_back({
      {static_cast<std::uint32_t>(index + 1)},
      {},
      {body.state.position_m.x, body.state.position_m.y, 0.0},
      {0.0, 0.0, 1.0},
      impact_speed * body.spec.mass_kg,
    });
   }
   body.force_N = {};
   body.torque_Nm = {};
  }
 }

 BodyState state(BodyHandle body) const override { return checked(body).state; }

 const std::vector<ContactEvent>& contacts() const override { return contacts_; }

 RaycastResult raycast(const Ray& ray) const override {
  RaycastResult result;
  const Vec3 direction = normalized(ray.direction);
 if (length(direction) < 1e-12 || ray.max_distance_m <= 0.0 || ray.collision_mask == 0 ||
 !(ray.collision_mask & collision_layer::Default)) {
   return result;
  }
  if (direction.z < -1e-12 && ray.origin_m.z >= 0.0) {
   const double distance = -ray.origin_m.z / direction.z;
   if (distance <= ray.max_distance_m) {
    result.hit = true;
    result.point_m = {
      ray.origin_m.x + direction.x * distance,
      ray.origin_m.y + direction.y * distance,
      0.0,
    };
    result.normal = {0.0, 0.0, 1.0};
    result.distance_m = distance;
   }
  }
  return result;
 }

 private:
 StoredBody& checked(BodyHandle body) {
  if (!body || body.value > bodies_.size() || !bodies_[body.value - 1].active) {
   throw std::out_of_range("invalid physics body handle");
  }
  return bodies_[body.value - 1];
 }

 const StoredBody& checked(BodyHandle body) const {
  if (!body || body.value > bodies_.size() || !bodies_[body.value - 1].active) {
   throw std::out_of_range("invalid physics body handle");
  }
  return bodies_[body.value - 1];
 }

 std::vector<StoredBody> bodies_;
 std::vector<ContactEvent> contacts_;
};

} // namespace

std::unique_ptr<IPhysicsWorld> make_planar_physics_world() {
 return std::make_unique<PlanarPhysicsWorld>();
}

Quaternion quaternion_from_euler(double roll_rad, double pitch_rad, double yaw_rad) {
 const double cr = std::cos(roll_rad * 0.5);
 const double sr = std::sin(roll_rad * 0.5);
 const double cp = std::cos(pitch_rad * 0.5);
 const double sp = std::sin(pitch_rad * 0.5);
 const double cy = std::cos(yaw_rad * 0.5);
 const double sy = std::sin(yaw_rad * 0.5);
 return {
   sr * cp * cy - cr * sp * sy,
   cr * sp * cy + sr * cp * sy,
   cr * cp * sy - sr * sp * cy,
   cr * cp * cy + sr * sp * sy,
 };
}

Vec3 euler_from_quaternion(const Quaternion& rotation) {
 const double sin_roll = 2.0 * (rotation.w * rotation.x + rotation.y * rotation.z);
 const double cos_roll = 1.0 - 2.0 * (rotation.x * rotation.x + rotation.y * rotation.y);
 const double roll = std::atan2(sin_roll, cos_roll);
 const double sin_pitch = 2.0 * (rotation.w * rotation.y - rotation.z * rotation.x);
 const double pitch = std::asin(std::clamp(sin_pitch, -1.0, 1.0));
 const double sin_yaw = 2.0 * (rotation.w * rotation.z + rotation.x * rotation.y);
 const double cos_yaw = 1.0 - 2.0 * (rotation.y * rotation.y + rotation.z * rotation.z);
 return {roll, pitch, std::atan2(sin_yaw, cos_yaw)};
}

} // namespace sim::physics
