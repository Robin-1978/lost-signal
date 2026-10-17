#include "physics_world.hpp"

#include <btBulletDynamicsCommon.h>
#include <BulletCollision/CollisionDispatch/btManifoldResult.h>

#include <algorithm>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <vector>

namespace sim::physics {
namespace {

struct BulletContactFrame {
 btVector3 axle_axis_local{0.0, 0.0, 0.0};
};

const BulletContactFrame* contact_frame(const btCollisionObject* object) {
 if (!object || !(object->getCollisionFlags() & btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK))
 return nullptr;
 return static_cast<const BulletContactFrame*>(object->getUserPointer());
}

bool align_wheel_contact(btManifoldPoint& point,
 const btCollisionObjectWrapper* object_a, int, int,
 const btCollisionObjectWrapper* object_b, int, int) {
 const auto* wheel_object = object_a ? object_a->getCollisionObject() : nullptr;
 const auto* frame = contact_frame(wheel_object);
 if (!frame) {
 wheel_object = object_b ? object_b->getCollisionObject() : nullptr;
 frame = contact_frame(wheel_object);
 }
 if (!frame || !wheel_object)
 return false;

 btVector3 normal = point.m_normalWorldOnB;
 if (normal.length2() < SIMD_EPSILON)
 return false;
 normal.normalize();
 btVector3 axle = wheel_object->getWorldTransform().getBasis() * frame->axle_axis_local;
 axle -= normal * axle.dot(normal);
 if (axle.length2() < SIMD_EPSILON)
 return false;
 axle.normalize();
 btVector3 rolling = axle.cross(normal);
 if (rolling.length2() < SIMD_EPSILON)
 return false;
 rolling.normalize();

 // Both directions lie in the contact plane. Bullet still computes their
 // impulses and normal-force coupling; this only removes the ambiguity in
 // the tangent basis at zero or very low slip.
 point.m_lateralFrictionDir1 = axle;
 point.m_lateralFrictionDir2 = rolling;
 point.m_contactPointFlags |= BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED;
 return true;
}

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
 std::unique_ptr<BulletContactFrame> contact_frame;
 std::unique_ptr<btTriangleMesh> triangle_mesh;
 std::unique_ptr<btCollisionShape> shape;
 std::unique_ptr<btDefaultMotionState> motion_state;
 std::unique_ptr<btRigidBody> body;
 bool active{true};
};

struct BulletJoint {
 std::unique_ptr<btTypedConstraint> constraint;
 btRigidBody* parent{nullptr};
 btRigidBody* child{nullptr};
 btVector3 axis_in_parent{0.0, 0.0, 1.0};
 bool revolute{false};
 double accumulated_position_rad{0.0};
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
 gContactAddedCallback = align_wheel_contact;
  world_->setGravity({0.0, 0.0, -9.81});
 // Small wheels and stacked revolute constraints need a tighter contact
 // solve than Bullet's demo defaults. These settings remain deterministic;
 // the public clock still advances by one caller-owned fixed step.
 world_->getSolverInfo().m_numIterations = 30;
 world_->getSolverInfo().m_splitImpulse = true;
 // A wheel contact needs two independent tangent constraints: longitudinal
 // traction and lateral cornering / hill-hold. Bullet's default solves one
 // velocity-selected direction, which let a stationary three-wheel chassis
 // slide several metres sideways on a 20-degree slope despite mu > tan(a).
 // Solve both tangent directions. This is still Coulomb contact; wheel
 // rotation remains free through the revolute joint.
 world_->getSolverInfo().m_solverMode |= SOLVER_USE_2_FRICTION_DIRECTIONS |
 SOLVER_ENABLE_FRICTION_DIRECTION_CACHING |
 SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION;
 }

 ~BulletPhysicsWorld() override { reset(); }

 std::string_view backend_name() const override { return "bullet"; }

 void reset() override {
 for (auto& joint : joints_) {
 world_->removeConstraint(joint.constraint.get());
 }
 joints_.clear();
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
 info.m_rollingFriction = btScalar(spec.rolling_friction);
 info.m_spinningFriction = btScalar(spec.spinning_friction);
  info.m_restitution = btScalar(spec.restitution);
  stored.body = std::make_unique<btRigidBody>(info);
 stored.body->setAnisotropicFriction(to_bullet(spec.rolling_friction_anisotropy),
 btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
 if (to_bullet(spec.contact_axle_axis_local).length2() > SIMD_EPSILON) {
 stored.contact_frame = std::make_unique<BulletContactFrame>();
 stored.contact_frame->axle_axis_local = to_bullet(spec.contact_axle_axis_local).normalized();
 stored.body->setUserPointer(stored.contact_frame.get());
 stored.body->setCollisionFlags(stored.body->getCollisionFlags() |
 btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
 }
 if (spec.friction_anchor) {
 stored.body->setCollisionFlags(stored.body->getCollisionFlags() |
 btCollisionObject::CF_HAS_FRICTION_ANCHOR);
 }
 if (spec.contact_stiffness_Npm > 0.0) {
 if (spec.contact_damping_Nspm < 0.0)
 throw std::invalid_argument("contact damping must be non-negative");
 stored.body->setContactStiffnessAndDamping(btScalar(spec.contact_stiffness_Npm),
 btScalar(spec.contact_damping_Nspm));
 }
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

 JointHandle create_joint(const JointSpec& spec) override {
 if (!spec.parent || !spec.child || spec.parent == spec.child) {
 throw std::invalid_argument("joint needs distinct parent and child bodies");
 }
 auto& parent = *checked(spec.parent).body;
 auto& child = *checked(spec.child).body;
 BulletJoint stored;
 stored.parent = &parent;
 stored.child = &child;
 stored.axis_in_parent = to_bullet(spec.axis_in_parent).normalized();
 if (stored.axis_in_parent.length2() < SIMD_EPSILON) {
 throw std::invalid_argument("joint axis must be non-zero");
 }
 if (spec.kind == JointKind::Revolute) {
 auto hinge = std::make_unique<btHingeConstraint>(
 parent, child, to_bullet(spec.pivot_in_parent_m), to_bullet(spec.pivot_in_child_m),
 stored.axis_in_parent, to_bullet(spec.axis_in_child).normalized(), true);
 if (spec.has_limits) {
 hinge->setLimit(btScalar(spec.lower_limit_rad), btScalar(spec.upper_limit_rad));
 } else {
 hinge->setLimit(btScalar(1.0), btScalar(-1.0));
 }
 stored.revolute = true;
 stored.constraint = std::move(hinge);
 } else {
 btTransform parent_frame;
 parent_frame.setIdentity();
 parent_frame.setOrigin(to_bullet(spec.pivot_in_parent_m));
 btTransform child_frame;
 child_frame.setIdentity();
 child_frame.setOrigin(to_bullet(spec.pivot_in_child_m));
 stored.constraint = std::make_unique<btFixedConstraint>(parent, child, parent_frame, child_frame);
 }
 world_->addConstraint(stored.constraint.get(), spec.disable_parent_child_collision);
 joints_.push_back(std::move(stored));
 return {static_cast<std::uint32_t>(joints_.size())};
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

 void set_mass_properties(BodyHandle body, double mass_kg,
 Vec3 inertia_diagonal_kgm2) override {
 if (!(mass_kg > 0.0) || inertia_diagonal_kgm2.x < 0.0 ||
 inertia_diagonal_kgm2.y < 0.0 || inertia_diagonal_kgm2.z < 0.0) {
 throw std::invalid_argument("invalid dynamic mass properties");
 }
 auto& stored = checked(body);
 auto& rigid = *stored.body;
 if (rigid.isKinematicObject()) {
 throw std::invalid_argument("cannot change mass properties of a kinematic body");
 }
 btVector3 inertia(0.0, 0.0, 0.0);
 if (inertia_diagonal_kgm2.x > 0.0 && inertia_diagonal_kgm2.y > 0.0 &&
 inertia_diagonal_kgm2.z > 0.0) {
 inertia = to_bullet(inertia_diagonal_kgm2);
 } else {
 stored.shape->calculateLocalInertia(btScalar(mass_kg), inertia);
 }
 rigid.setMassProps(btScalar(mass_kg), inertia);
 rigid.updateInertiaTensor();
 rigid.activate(true);
 }

 void set_contact_friction(BodyHandle body, double friction,
 double rolling_friction, double spinning_friction) override {
 if (friction < 0.0 || rolling_friction < 0.0 || spinning_friction < 0.0) {
 throw std::invalid_argument("contact friction must be non-negative");
 }
 auto& rigid = *checked(body).body;
 rigid.setFriction(btScalar(friction));
 rigid.setRollingFriction(btScalar(rolling_friction));
 rigid.setSpinningFriction(btScalar(spinning_friction));
 refresh_contact_material(rigid);
 rigid.activate(true);
 }

 void set_contact_rolling_anisotropy(BodyHandle body, Vec3 local_scale) override {
 if (local_scale.x < 0.0 || local_scale.y < 0.0 || local_scale.z < 0.0 ||
 !std::isfinite(local_scale.x) || !std::isfinite(local_scale.y) ||
 !std::isfinite(local_scale.z)) {
 throw std::invalid_argument("rolling friction anisotropy must be finite and non-negative");
 }
 auto& rigid = *checked(body).body;
 rigid.setAnisotropicFriction(to_bullet(local_scale),
 btCollisionObject::CF_ANISOTROPIC_ROLLING_FRICTION);
 rigid.activate(true);
 }

 void set_contact_compliance(BodyHandle body, double stiffness_Npm,
 double damping_Nspm) override {
 if (!(stiffness_Npm > 0.0) || damping_Nspm < 0.0 ||
 !std::isfinite(stiffness_Npm) || !std::isfinite(damping_Nspm))
 throw std::invalid_argument("invalid contact compliance");
 auto& rigid = *checked(body).body;
 rigid.setContactStiffnessAndDamping(btScalar(stiffness_Npm), btScalar(damping_Nspm));
 refresh_contact_material(rigid);
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

 void apply_force_at_world_point(BodyHandle body, Vec3 force_world_N,
 Vec3 point_world_m) override {
 auto& rigid = *checked(body).body;
 const btVector3 relative_point =
 to_bullet(point_world_m) - rigid.getCenterOfMassPosition();
 rigid.applyForce(to_bullet(force_world_N), relative_point);
 rigid.activate(true);
 }

 void apply_local_torque(BodyHandle body, Vec3 torque_body_Nm) override {
 auto& rigid = *checked(body).body;
 rigid.applyTorque(rigid.getWorldTransform().getBasis() * to_bullet(torque_body_Nm));
 rigid.activate(true);
 }

 void apply_world_torque(BodyHandle body, Vec3 torque_world_Nm) override {
 auto& rigid = *checked(body).body;
 rigid.applyTorque(to_bullet(torque_world_Nm));
 rigid.activate(true);
 }

 void apply_joint_effort(JointHandle joint, double effort_Nm) override {
 auto& stored = checked(joint);
 if (!stored.revolute || !std::isfinite(effort_Nm)) {
 if (!std::isfinite(effort_Nm))
 throw std::invalid_argument("joint effort must be finite");
 return;
 }
 const btVector3 axis_world = stored.parent->getWorldTransform().getBasis() * stored.axis_in_parent;
 const btVector3 torque = axis_world * btScalar(effort_Nm);
 stored.child->applyTorque(torque);
 stored.parent->applyTorque(-torque);
 stored.child->activate(true);
 stored.parent->activate(true);
 }

 void step(double fixed_dt_s) override {
  if (!(fixed_dt_s > 0.0) || !std::isfinite(fixed_dt_s)) {
   throw std::invalid_argument("physics step must be finite and positive");
  }
 constexpr int kInternalSubsteps = 4;
 const double internal_dt_s = fixed_dt_s / static_cast<double>(kInternalSubsteps);
 world_->stepSimulation(btScalar(fixed_dt_s), kInternalSubsteps,
 btScalar(internal_dt_s));
 for (auto& joint : joints_) {
 if (!joint.revolute)
 continue;
 const btVector3 axis_world = joint.parent->getWorldTransform().getBasis() * joint.axis_in_parent;
 joint.accumulated_position_rad +=
 (joint.child->getAngularVelocity() - joint.parent->getAngularVelocity()).dot(axis_world) * fixed_dt_s;
 }
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
 point.getAppliedImpulse() / internal_dt_s,
 from_bullet(point.m_lateralFrictionDir1),
 from_bullet(point.m_lateralFrictionDir2),
 point.m_appliedImpulseLateral1,
 point.m_appliedImpulseLateral2,
 point.m_appliedImpulseLateral1 / internal_dt_s,
 point.m_appliedImpulseLateral2 / internal_dt_s,
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

 JointState joint_state(JointHandle joint) const override {
 const auto& stored = checked(joint);
 if (!stored.revolute) {
 return {};
 }
 const auto& hinge = static_cast<const btHingeConstraint&>(*stored.constraint);
 const btVector3 axis_world = stored.parent->getWorldTransform().getBasis() * stored.axis_in_parent;
 const double relative_velocity =
 (stored.child->getAngularVelocity() - stored.parent->getAngularVelocity()).dot(axis_world);
 (void)hinge;
 return {stored.accumulated_position_rad, relative_velocity};
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
 void refresh_contact_material(btRigidBody& changed) {
 const int manifolds = dispatcher_->getNumManifolds();
 for (int manifold_index = 0; manifold_index < manifolds; ++manifold_index) {
 auto* manifold = dispatcher_->getManifoldByIndexInternal(manifold_index);
 auto* body_a = const_cast<btCollisionObject*>(
 static_cast<const btCollisionObject*>(manifold->getBody0()));
 auto* body_b = const_cast<btCollisionObject*>(
 static_cast<const btCollisionObject*>(manifold->getBody1()));
 if (body_a != &changed && body_b != &changed)
 continue;
 for (int point_index = 0; point_index < manifold->getNumContacts(); ++point_index) {
 auto& point = manifold->getContactPoint(point_index);
 point.m_combinedFriction = btManifoldResult::calculateCombinedFriction(body_a, body_b);
 point.m_combinedRollingFriction =
 btManifoldResult::calculateCombinedRollingFriction(body_a, body_b);
 point.m_combinedSpinningFriction =
 btManifoldResult::calculateCombinedSpinningFriction(body_a, body_b);
 if ((body_a->getCollisionFlags() & btCollisionObject::CF_HAS_CONTACT_STIFFNESS_DAMPING) ||
 (body_b->getCollisionFlags() & btCollisionObject::CF_HAS_CONTACT_STIFFNESS_DAMPING)) {
 point.m_combinedContactDamping1 =
 btManifoldResult::calculateCombinedContactDamping(body_a, body_b);
 point.m_combinedContactStiffness1 =
 btManifoldResult::calculateCombinedContactStiffness(body_a, body_b);
 point.m_contactPointFlags |= BT_CONTACT_FLAG_CONTACT_STIFFNESS_DAMPING;
 }
 }
 }
 }

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

 BulletJoint& checked(JointHandle joint) {
 if (!joint || joint.value > joints_.size()) {
 throw std::out_of_range("invalid physics joint handle");
 }
 return joints_[joint.value - 1];
 }

 const BulletJoint& checked(JointHandle joint) const {
 if (!joint || joint.value > joints_.size()) {
 throw std::out_of_range("invalid physics joint handle");
 }
 return joints_[joint.value - 1];
 }

 btDefaultCollisionConfiguration collision_configuration_;
 std::unique_ptr<btSequentialImpulseConstraintSolver> solver_;
 std::unique_ptr<btBroadphaseInterface> broadphase_;
 std::unique_ptr<btCollisionDispatcher> dispatcher_;
 std::unique_ptr<btDiscreteDynamicsWorld> world_;
 std::vector<BulletBody> bodies_;
 std::vector<BulletJoint> joints_;
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
