#include "physics_world.hpp"

#include <btBulletDynamicsCommon.h>

#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace sim::physics {
namespace {

btVector3 to_bullet(Vec3 value) {
 return {btScalar(value.x), btScalar(value.y), btScalar(value.z)};
}

Vec3 from_bullet(const btVector3& value) {
 return {value.x(), value.y(), value.z()};
}

btQuaternion to_bullet(Quaternion value) {
 return {btScalar(value.x), btScalar(value.y), btScalar(value.z), btScalar(value.w)};
}

Quaternion from_bullet(const btQuaternion& value) {
 return {value.x(), value.y(), value.z(), value.w()};
}

struct BulletBody {
 std::unique_ptr<btTriangleMesh> triangle_mesh;
 std::unique_ptr<btCollisionShape> shape;
 std::unique_ptr<btDefaultMotionState> motion_state;
 std::unique_ptr<btRigidBody> body;
 bool active{true};
};

class ClosestRayIgnoringBodyCallback final : public btCollisionWorld::ClosestRayResultCallback {
 public:
 ClosestRayIgnoringBodyCallback(const btVector3& from, const btVector3& to, BodyHandle ignored)
 : btCollisionWorld::ClosestRayResultCallback(from, to), ignored_(ignored) {}

 bool needsCollision(btBroadphaseProxy* proxy) const override {
 if (!btCollisionWorld::ClosestRayResultCallback::needsCollision(proxy)) {
 return false;
 }
 const auto* object = static_cast<const btCollisionObject*>(proxy->m_clientObject);
 return !ignored_ || object->getUserIndex() != static_cast<int>(ignored_.value);
 }

 private:
 BodyHandle ignored_;
};

class BulletPhysicsWorld final : public IPhysicsWorld {
 public:
 BulletPhysicsWorld()
     : solver_(std::make_unique<btSequentialImpulseConstraintSolver>()),
       broadphase_(std::make_unique<btDbvtBroadphase>()),
       dispatcher_(std::make_unique<btCollisionDispatcher>(&collision_configuration_)),
       world_(std::make_unique<btDiscreteDynamicsWorld>(dispatcher_.get(), broadphase_.get(), solver_.get(),
                                                        &collision_configuration_)) {
  world_->setGravity({0.0, 0.0, -9.81});
 }

 ~BulletPhysicsWorld() override { reset(); }

 std::string_view backend_name() const override { return "bullet"; }

 void reset() override {
  for (auto& stored : bodies_) {
   if (stored.active) {
    world_->removeRigidBody(stored.body.get());
    stored.active = false;
   }
  }
  bodies_.clear();
  contacts_.clear();
 }

 BodyHandle spawn(const BodySpec& spec) override {
 if (spec.mass_kg < 0.0 || spec.friction < 0.0 || spec.restitution < 0.0 ||
 spec.collision_group == 0) {
   throw std::invalid_argument("invalid physics body properties");
  }
  BulletBody stored;
  switch (spec.shape) {
  case ShapeKind::Box:
   stored.shape = std::make_unique<btBoxShape>(to_bullet(spec.half_extents_m));
   break;
  case ShapeKind::Sphere:
   stored.shape = std::make_unique<btSphereShape>(btScalar(spec.radius_m));
   break;
  case ShapeKind::Cylinder:
   stored.shape = std::make_unique<btCylinderShapeZ>(
     btVector3(btScalar(spec.radius_m), btScalar(spec.radius_m), btScalar(spec.height_m * 0.5)));
   break;
 case ShapeKind::TriangleMesh:
 if (spec.mass_kg != 0.0 || spec.mesh_indices.size() % 3 != 0 || spec.mesh_vertices_m.empty()) {
 throw std::invalid_argument("triangle meshes must be non-empty static bodies with triangle indices");
 }
 stored.triangle_mesh = std::make_unique<btTriangleMesh>();
 for (std::size_t index = 0; index < spec.mesh_indices.size(); index += 3) {
 const auto a = spec.mesh_indices[index];
 const auto b = spec.mesh_indices[index + 1];
 const auto c = spec.mesh_indices[index + 2];
 if (a >= spec.mesh_vertices_m.size() || b >= spec.mesh_vertices_m.size() ||
 c >= spec.mesh_vertices_m.size()) {
 throw std::invalid_argument("triangle mesh index is out of range");
 }
 stored.triangle_mesh->addTriangle(to_bullet(spec.mesh_vertices_m[a]),
 to_bullet(spec.mesh_vertices_m[b]), to_bullet(spec.mesh_vertices_m[c]), true);
 }
 stored.shape = std::make_unique<btBvhTriangleMeshShape>(stored.triangle_mesh.get(), true);
 break;
  }
  btTransform transform(to_bullet(spec.rotation), to_bullet(spec.position_m));
  stored.motion_state = std::make_unique<btDefaultMotionState>(transform);
  btVector3 inertia(0.0, 0.0, 0.0);
  if (spec.mass_kg > 0.0) {
 if (spec.inertia_diagonal_kgm2.x > 0.0 && spec.inertia_diagonal_kgm2.y > 0.0 &&
 spec.inertia_diagonal_kgm2.z > 0.0) {
 inertia = to_bullet(spec.inertia_diagonal_kgm2);
 } else {
 stored.shape->calculateLocalInertia(btScalar(spec.mass_kg), inertia);
 }
  }
  btRigidBody::btRigidBodyConstructionInfo info(btScalar(spec.mass_kg), stored.motion_state.get(),
                                                stored.shape.get(), inertia);
  info.m_friction = btScalar(spec.friction);
  info.m_restitution = btScalar(spec.restitution);
  stored.body = std::make_unique<btRigidBody>(info);
 stored.body->setDamping(btScalar(spec.linear_damping), btScalar(spec.angular_damping));
 if (spec.continuous_collision && spec.mass_kg > 0.0) {
 stored.body->setCcdMotionThreshold(btScalar(0.05));
 const double radius = spec.shape == ShapeKind::Box
 ? std::min({spec.half_extents_m.x, spec.half_extents_m.y, spec.half_extents_m.z})
 : spec.radius_m;
 stored.body->setCcdSweptSphereRadius(btScalar(std::max(0.01, radius * 0.8)));
 }
  if (spec.kinematic) {
   stored.body->setCollisionFlags(stored.body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
   stored.body->setActivationState(DISABLE_DEACTIVATION);
  }
  stored.body->setUserIndex(static_cast<int>(bodies_.size() + 1));
 world_->addRigidBody(stored.body.get(), static_cast<short>(spec.collision_group),
 static_cast<short>(spec.collision_mask));
  bodies_.push_back(std::move(stored));
  return {static_cast<std::uint32_t>(bodies_.size())};
 }

 void remove(BodyHandle body) override {
  auto& stored = checked(body);
  world_->removeRigidBody(stored.body.get());
  stored.active = false;
 }

 void set_state(BodyHandle body, const BodyState& state) override {
  auto& rigid = *checked(body).body;
  const btTransform transform(to_bullet(state.rotation), to_bullet(state.position_m));
  rigid.setWorldTransform(transform);
  rigid.setLinearVelocity(to_bullet(state.linear_velocity_mps));
  rigid.setAngularVelocity(to_bullet(state.angular_velocity_radps));
  if (rigid.getMotionState()) {
   rigid.getMotionState()->setWorldTransform(transform);
  }
  rigid.activate(true);
 }

 void apply_drive(BodyHandle body, const DriveForces& forces) override {
  auto& rigid = *checked(body).body;
  const btMatrix3x3 basis = rigid.getWorldTransform().getBasis();
  const btVector3 forward = basis.getColumn(0);
  const btVector3 lateral = basis.getColumn(1);
  btVector3 force = forward * btScalar(forces.forward_force_N) + lateral * btScalar(forces.lateral_force_N);
  const btScalar forward_speed = rigid.getLinearVelocity().dot(forward);
  if (btFabs(forward_speed) > SIMD_EPSILON && forces.brake_force_N > 0.0) {
   force -= forward * btScalar(std::copysign(forces.brake_force_N, double(forward_speed)));
  }
  rigid.applyCentralForce(force);
  rigid.applyTorque({0.0, 0.0, btScalar(forces.yaw_torque_Nm)});
  rigid.activate(true);
 }

 void apply_force_at_local_point(BodyHandle body, Vec3 force_body_N, Vec3 point_body_m) override {
 auto& rigid = *checked(body).body;
 const btMatrix3x3 basis = rigid.getWorldTransform().getBasis();
 rigid.applyForce(basis * to_bullet(force_body_N), basis * to_bullet(point_body_m));
 rigid.activate(true);
 }

 void apply_local_torque(BodyHandle body, Vec3 torque_body_Nm) override {
 auto& rigid = *checked(body).body;
 rigid.applyTorque(rigid.getWorldTransform().getBasis() * to_bullet(torque_body_Nm));
 rigid.activate(true);
 }

 void step(double fixed_dt_s) override {
  if (!(fixed_dt_s > 0.0) || !std::isfinite(fixed_dt_s)) {
   throw std::invalid_argument("physics step must be finite and positive");
  }
  world_->stepSimulation(btScalar(fixed_dt_s), 0, btScalar(fixed_dt_s));
  contacts_.clear();
  const int manifolds = dispatcher_->getNumManifolds();
  for (int manifold_index = 0; manifold_index < manifolds; ++manifold_index) {
   const btPersistentManifold* manifold = dispatcher_->getManifoldByIndexInternal(manifold_index);
   const auto* body_a = static_cast<const btCollisionObject*>(manifold->getBody0());
   const auto* body_b = static_cast<const btCollisionObject*>(manifold->getBody1());
   for (int point_index = 0; point_index < manifold->getNumContacts(); ++point_index) {
    const btManifoldPoint& point = manifold->getContactPoint(point_index);
    if (point.getDistance() > 0.0) {
     continue;
    }
    contacts_.push_back({
      {static_cast<std::uint32_t>(body_a->getUserIndex())},
      {static_cast<std::uint32_t>(body_b->getUserIndex())},
      from_bullet(point.getPositionWorldOnB()),
      from_bullet(point.m_normalWorldOnB),
      point.getAppliedImpulse(),
    });
   }
  }
 }

 BodyState state(BodyHandle body) const override {
  const auto& rigid = *checked(body).body;
  const btTransform& transform = rigid.getWorldTransform();
  return {
    from_bullet(transform.getOrigin()),
    from_bullet(transform.getRotation()),
    from_bullet(rigid.getLinearVelocity()),
    from_bullet(rigid.getAngularVelocity()),
  };
 }

 const std::vector<ContactEvent>& contacts() const override { return contacts_; }

 RaycastResult raycast(const Ray& ray) const override {
 const double direction_length = std::sqrt(ray.direction.x * ray.direction.x +
 ray.direction.y * ray.direction.y + ray.direction.z * ray.direction.z);
 if (!(ray.max_distance_m > 0.0) || !std::isfinite(ray.max_distance_m) ||
 direction_length < 1e-12 || !std::isfinite(direction_length) || ray.collision_mask == 0) {
 return {};
 }
  const btVector3 from = to_bullet(ray.origin_m);
  const btVector3 direction = to_bullet(ray.direction).normalized();
  const btVector3 to = from + direction * btScalar(ray.max_distance_m);
 ClosestRayIgnoringBodyCallback callback(from, to, ray.ignore_body);
 callback.m_collisionFilterMask = static_cast<short>(ray.collision_mask);
  world_->rayTest(from, to, callback);
  if (!callback.hasHit()) {
   return {};
  }
  return {
    true,
    {static_cast<std::uint32_t>(callback.m_collisionObject->getUserIndex())},
    from_bullet(callback.m_hitPointWorld),
    from_bullet(callback.m_hitNormalWorld),
    ray.max_distance_m * callback.m_closestHitFraction,
  };
 }

 private:
 BulletBody& checked(BodyHandle body) {
  if (!body || body.value > bodies_.size() || !bodies_[body.value - 1].active) {
   throw std::out_of_range("invalid physics body handle");
  }
  return bodies_[body.value - 1];
 }

 const BulletBody& checked(BodyHandle body) const {
  if (!body || body.value > bodies_.size() || !bodies_[body.value - 1].active) {
   throw std::out_of_range("invalid physics body handle");
  }
  return bodies_[body.value - 1];
 }

 btDefaultCollisionConfiguration collision_configuration_;
 std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;
 std::unique_ptr<btBroadphaseInterface> broadphase_;
 std::unique_ptr<btCollisionDispatcher> dispatcher_;
 std::unique_ptr<btDiscreteDynamicsWorld> world_;
 std::vector<BulletBody> bodies_;
 std::vector<ContactEvent> contacts_;
};

} // namespace

std::unique_ptr<IPhysicsWorld> make_bullet_physics_world() {
 return std::make_unique<BulletPhysicsWorld>();
}

bool bullet_physics_available() {
 return true;
}

} // namespace sim::physics
