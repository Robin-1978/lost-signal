#include "physics_world.hpp"

#include "frame_transform.hpp"

#include <mujoco/mujoco.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <iomanip>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace sim::physics {
namespace {

// Small wheels and high contact stiffness require a millisecond integration
// step. The public simulation clock remains caller-owned and is substepped.
constexpr double kInternalTimestepS = 0.001;
constexpr double kEpsilon = 1e-10;
// Freeze MuJoCo's documented soft-contact impedance instead of inheriting a
// library default implicitly. Positive-format solref provides the effective
// response time and damping ratio; solimp controls how strongly the
// regularized constraint is enforced as penetration grows.
constexpr std::array<mjtNum, mjNIMP> kContactSolimp{0.9, 0.95, 0.001, 0.5, 2.0};
constexpr double kDefaultContactTimeConstantS = 0.02;
constexpr double kDefaultContactDampingRatio = 1.0;

bool finite(double value) { return std::isfinite(value); }

bool finite(Vec3 value) {
 return finite(value.x) && finite(value.y) && finite(value.z);
}

double norm(Vec3 value) {
 return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

Vec3 normalized(Vec3 value) {
 const double length = norm(value);
 if (length < kEpsilon)
 throw std::invalid_argument("physics vector must be non-zero");
 return {value.x / length, value.y / length, value.z / length};
}

Quaternion normalized(Quaternion value) {
 const double length = std::sqrt(value.x * value.x + value.y * value.y +
 value.z * value.z + value.w * value.w);
 if (length < kEpsilon || !finite(length))
 throw std::invalid_argument("physics quaternion must be finite and non-zero");
 return {value.x / length, value.y / length, value.z / length, value.w / length};
}

Vec3 add(Vec3 lhs, Vec3 rhs) {
 return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

Vec3 subtract(Vec3 lhs, Vec3 rhs) {
 return {lhs.x - rhs.x, lhs.y - rhs.y, lhs.z - rhs.z};
}

Vec3 scale(Vec3 value, double factor) {
 return {value.x * factor, value.y * factor, value.z * factor};
}

double dot(Vec3 lhs, Vec3 rhs) {
 return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z;
}

bool valid_diagonal_inertia(Vec3 inertia, bool allow_unspecified) {
 if (!finite(inertia))
 return false;
 const bool unspecified = inertia.x == 0.0 && inertia.y == 0.0 && inertia.z == 0.0;
 if (allow_unspecified && unspecified)
 return true;
 return inertia.x > 0.0 && inertia.y > 0.0 && inertia.z > 0.0 &&
 inertia.x + inertia.y >= inertia.z &&
 inertia.x + inertia.z >= inertia.y &&
 inertia.y + inertia.z >= inertia.x;
}

std::string number(double value) {
 if (!finite(value))
 throw std::invalid_argument("physics model contains a non-finite value");
 std::ostringstream output;
 output << std::setprecision(17) << value;
 return output.str();
}

std::string vector(Vec3 value) {
 return number(value.x) + " " + number(value.y) + " " + number(value.z);
}

std::string quaternion(Quaternion value) {
 value = normalized(value);
 return number(value.w) + " " + number(value.x) + " " +
 number(value.y) + " " + number(value.z);
}

std::string body_name(BodyHandle handle) {
 return "ls_body_" + std::to_string(handle.value);
}

std::string geom_name(BodyHandle handle) {
 return "ls_geom_" + std::to_string(handle.value);
}

std::string joint_name(JointHandle handle) {
 return "ls_joint_" + std::to_string(handle.value);
}

std::string free_joint_name(BodyHandle handle) {
 return "ls_free_" + std::to_string(handle.value);
}

std::string heightfield_name(BodyHandle handle) {
 return "ls_hfield_" + std::to_string(handle.value);
}

int collision_group_index(CollisionMask group) {
 for (int index = 0; index < 5; ++index) {
 if (group & (CollisionMask{1} << index))
 return index;
 }
 return 0;
}

struct ModelDeleter {
 void operator()(mjModel* value) const { mj_deleteModel(value); }
};

struct DataDeleter {
 void operator()(mjData* value) const { mj_deleteData(value); }
};

struct SpecDeleter {
 void operator()(mjSpec* value) const { mj_deleteSpec(value); }
};

using ModelPtr = std::unique_ptr<mjModel, ModelDeleter>;
using DataPtr = std::unique_ptr<mjData, DataDeleter>;
using SpecPtr = std::unique_ptr<mjSpec, SpecDeleter>;

} // namespace

class MujocoSimulation::Impl {
 public:
 struct StoredBody {
 BodyModel model;
 bool active{true};
 int body_id{-1};
 int geom_id{-1};
 int free_joint_id{-1};
 };

 struct StoredJoint {
 JointModel model;
 bool active{true};
 int joint_id{-1};
 double brake_capacity_Nm{0.0};
 double brake_smoothing_radps{1.0};
 };

 struct ContactAccumulator {
 BodyHandle body_a;
 BodyHandle body_b;
 Vec3 weighted_point;
 Vec3 weighted_normal_on_b;
 Vec3 tangent_impulse_world;
 double normal_impulse_Ns{0.0};
 double geometry_weight{0.0};
 };

 BodyHandle add_body(const BodyModel& model) {
 validate_body(model);
bodies_.push_back({model, true, -1, -1, -1});
 dirty_ = true;
 return {static_cast<std::uint32_t>(bodies_.size())};
 }

 JointHandle add_joint(const JointModel& model) {
 if (!model.parent || !model.child || model.parent == model.child)
 throw std::invalid_argument("joint needs distinct parent and child bodies");
 checked_body(model.parent);
 checked_body(model.child);
 if (!finite(model.pivot_in_parent_m) || !finite(model.pivot_in_child_m) ||
 !finite(model.axis_in_parent) || !finite(model.axis_in_child) ||
 !finite(model.armature_inertia_kgm2) || model.armature_inertia_kgm2 < 0.0 ||
 !finite(model.damping_Nms_per_rad) || model.damping_Nms_per_rad < 0.0 ||
 !finite(model.friction_loss_Nm) || model.friction_loss_Nm < 0.0)
 throw std::invalid_argument("joint contains invalid values");
 if (model.kind == JointKind::Revolute) {
 normalized(model.axis_in_parent);
 normalized(model.axis_in_child);
 if (model.has_limits && (!(model.lower_limit_rad < model.upper_limit_rad) ||
 !finite(model.lower_limit_rad) || !finite(model.upper_limit_rad)))
 throw std::invalid_argument("revolute joint limits are invalid");
 }
 for (const auto& joint : joints_) {
 if (joint.active && joint.model.child == model.child)
 throw std::invalid_argument("a physics body cannot have multiple parent joints");
 }
 joints_.push_back({model});
 dirty_ = true;
 return {static_cast<std::uint32_t>(joints_.size())};
 }

 void remove_body(BodyHandle handle) {
 auto& body = checked_body(handle);
 body.active = false;
 for (auto& joint : joints_) {
 if (joint.active && (joint.model.parent == handle || joint.model.child == handle))
 joint.active = false;
 }
 dirty_ = true;
 }

 void clear_model() {
 bodies_.clear();
 joints_.clear();
 body_by_mj_id_.clear();
 body_by_geom_id_.clear();
 contacts_.clear();
 data_.reset();
 model_.reset();
 spec_.reset();
 dirty_ = true;
 }

 bool compiled() const { return model_ && data_ && !dirty_; }

 void clear_velocities() {
 require_compiled();
 std::fill(data_->qvel, data_->qvel + model_->nv, 0.0);
 std::fill(data_->qacc_warmstart, data_->qacc_warmstart + model_->nv, 0.0);
 std::fill(data_->qfrc_applied, data_->qfrc_applied + model_->nv, 0.0);
 std::fill(data_->xfrc_applied, data_->xfrc_applied + 6 * model_->nbody, 0.0);
 mj_forward(model_.get(), data_.get());
 }

 void compile() {
 struct PreservedState {
 BodyHandle handle;
 BodyState state;
 };
 struct PreservedJointState {
 JointHandle handle;
 JointState state;
 };
 std::vector<PreservedState> body_states;
 std::vector<PreservedJointState> joint_states;
 double preserved_time = 0.0;
 if (model_ && data_) {
 preserved_time = data_->time;
 for (std::size_t index = 0; index < bodies_.size(); ++index) {
 if (!bodies_[index].active || bodies_[index].body_id < 0)
 continue;
 if (bodies_[index].free_joint_id >= 0)
 body_states.push_back({{static_cast<std::uint32_t>(index + 1)},
 read_body_state(bodies_[index])});
 }
 for (std::size_t index = 0; index < joints_.size(); ++index) {
 if (joints_[index].active && joints_[index].joint_id >= 0 &&
 joints_[index].model.kind == JointKind::Revolute)
 joint_states.push_back({{static_cast<std::uint32_t>(index + 1)},
 read_joint_state(joints_[index])});
 }
 }

 validate_tree();
 const std::string xml = build_xml();
 std::array<char, 2048> error{};
 SpecPtr new_spec(mj_parseXMLString(xml.c_str(), nullptr, error.data(), error.size()));
 if (!new_spec)
 throw std::runtime_error("MuJoCo model parse failed: " + std::string(error.data()));
 ModelPtr new_model(mj_compile(new_spec.get(), nullptr));
 if (!new_model)
 throw std::runtime_error("MuJoCo model compilation failed");
 DataPtr new_data(mj_makeData(new_model.get()));
 if (!new_data)
 throw std::runtime_error("MuJoCo data allocation failed");

 spec_ = std::move(new_spec);
 model_ = std::move(new_model);
 data_ = std::move(new_data);
 map_ids();
 dirty_ = false;
 for (const auto& saved : body_states) {
 if (saved.handle.value <= bodies_.size() && bodies_[saved.handle.value - 1].active)
 write_body_state(bodies_[saved.handle.value - 1], saved.state);
 }
 for (const auto& saved : joint_states) {
 if (saved.handle.value > joints_.size())
 continue;
 auto& joint = joints_[saved.handle.value - 1];
 if (!joint.active || joint.joint_id < 0 || joint.model.kind != JointKind::Revolute)
 continue;
 const int qpos = model_->jnt_qposadr[joint.joint_id];
 const int dof = model_->jnt_dofadr[joint.joint_id];
 data_->qpos[qpos] = saved.state.position_rad;
 data_->qvel[dof] = saved.state.velocity_radps;
 }
 data_->time = preserved_time;
 mj_forward(model_.get(), data_.get());
 contacts_.clear();
 }

 void set_state(BodyHandle handle, const BodyState& state) {
 require_compiled();
 if (!finite(state.position_m) || !finite(state.linear_velocity_mps) ||
 !finite(state.angular_velocity_radps))
 throw std::invalid_argument("body state must be finite");
 write_body_state(checked_body(handle), state);
 mj_forward(model_.get(), data_.get());
 }

 void set_mass_properties(BodyHandle handle, double mass, Vec3 inertia, Vec3 inertial_position) {
 require_compiled();
 auto& body = checked_body(handle);
 if (!(mass > 0.0) || !finite(mass) || !valid_diagonal_inertia(inertia, false) ||
 !finite(inertial_position) || body.model.kinematic)
 throw std::invalid_argument("invalid dynamic mass properties");
 model_->body_mass[body.body_id] = mass;
 model_->body_inertia[3 * body.body_id] = inertia.x;
 model_->body_inertia[3 * body.body_id + 1] = inertia.y;
 model_->body_inertia[3 * body.body_id + 2] = inertia.z;
 model_->body_ipos[3 * body.body_id] = inertial_position.x;
 model_->body_ipos[3 * body.body_id + 1] = inertial_position.y;
 model_->body_ipos[3 * body.body_id + 2] = inertial_position.z;
 body.model.mass_kg = mass;
 body.model.inertia_diagonal_kgm2 = inertia;
 body.model.inertial_position_m = inertial_position;
 // mj_setConst evaluates model constants in qpos0 and leaves mjData there.
 // Calling it directly during a payload update therefore teleports every
 // free body back to its authored pose. Runtime identification and payload
 // changes are model updates, not state resets: preserve the complete
 // generalized pose/velocity and pending inputs, recompute constants, then
 // refresh derived data at the original state. Warm-start accelerations are
 // deliberately cleared because they were solved for the old mass matrix.
 const std::vector<mjtNum> qpos(data_->qpos, data_->qpos + model_->nq);
 const std::vector<mjtNum> qvel(data_->qvel, data_->qvel + model_->nv);
 const std::vector<mjtNum> qfrc_applied(
 data_->qfrc_applied, data_->qfrc_applied + model_->nv);
 const std::vector<mjtNum> xfrc_applied(
 data_->xfrc_applied, data_->xfrc_applied + 6 * model_->nbody);
 const double time = data_->time;
 mj_setConst(model_.get(), data_.get());
 std::copy(qpos.begin(), qpos.end(), data_->qpos);
 std::copy(qvel.begin(), qvel.end(), data_->qvel);
 std::copy(qfrc_applied.begin(), qfrc_applied.end(), data_->qfrc_applied);
 std::copy(xfrc_applied.begin(), xfrc_applied.end(), data_->xfrc_applied);
 std::fill(data_->qacc_warmstart, data_->qacc_warmstart + model_->nv, 0.0);
 data_->time = time;
 mj_forward(model_.get(), data_.get());
 }

 void set_contact_parameters(BodyHandle handle, double friction, double rolling,
 double spinning, double time_constant, double damping_ratio) {
 require_compiled();
 auto& body = checked_body(handle);
 if (friction < 0.0 || rolling < 0.0 || spinning < 0.0 || time_constant < 0.0 ||
 damping_ratio < 0.0 || !finite(friction) || !finite(rolling) || !finite(spinning) ||
 !finite(time_constant) || !finite(damping_ratio) ||
 (time_constant > 0.0 && (time_constant < 2.0 * kInternalTimestepS ||
 !(damping_ratio > 0.0))))
 throw std::invalid_argument("contact parameters must be finite and non-negative");
 const int geom = body.geom_id;
 model_->geom_friction[3 * geom] = friction;
 model_->geom_friction[3 * geom + 1] = spinning;
 model_->geom_friction[3 * geom + 2] = rolling;
 model_->geom_solref[mjNREF * geom] =
 time_constant > 0.0 ? time_constant : kDefaultContactTimeConstantS;
 model_->geom_solref[mjNREF * geom + 1] =
 time_constant > 0.0 ? damping_ratio : kDefaultContactDampingRatio;
 std::copy(kContactSolimp.begin(), kContactSolimp.end(),
 model_->geom_solimp + mjNIMP * geom);
 body.model.friction = friction;
 body.model.rolling_friction = rolling;
 body.model.spinning_friction = spinning;
body.model.contact_time_constant_s = time_constant;
body.model.contact_damping_ratio = damping_ratio;
 }

 void apply_force_world(BodyHandle handle, Vec3 force, Vec3 point) {
 require_compiled();
 auto& body = checked_body(handle);
 if (!finite(force) || !finite(point))
 throw std::invalid_argument("applied force must be finite");
 const mjtNum f[3]{force.x, force.y, force.z};
 const mjtNum p[3]{point.x, point.y, point.z};
 mj_applyFT(model_.get(), data_.get(), f, nullptr, p, body.body_id,
 data_->qfrc_applied);
 }

 void apply_torque_world(BodyHandle handle, Vec3 torque) {
 require_compiled();
 auto& body = checked_body(handle);
 if (!finite(torque))
 throw std::invalid_argument("applied torque must be finite");
 const mjtNum t[3]{torque.x, torque.y, torque.z};
 const mjtNum p[3]{data_->xipos[3 * body.body_id], data_->xipos[3 * body.body_id + 1],
 data_->xipos[3 * body.body_id + 2]};
 mj_applyFT(model_.get(), data_.get(), nullptr, t, p, body.body_id,
 data_->qfrc_applied);
 }

 void apply_joint_effort(JointHandle handle, double effort) {
 require_compiled();
 auto& joint = checked_joint(handle);
 if (!finite(effort))
 throw std::invalid_argument("joint effort must be finite");
 if (joint.model.kind != JointKind::Revolute || joint.joint_id < 0)
 return;
 data_->qfrc_applied[model_->jnt_dofadr[joint.joint_id]] += effort;
 }

 void apply_joint_brake(JointHandle handle, double maximum_torque,
 double low_speed_smoothing) {
 require_compiled();
 auto& joint = checked_joint(handle);
 if (!finite(maximum_torque) || !finite(low_speed_smoothing) ||
 maximum_torque < 0.0 || !(low_speed_smoothing > 0.0))
 throw std::invalid_argument("joint brake parameters must be finite and non-negative");
 if (joint.model.kind != JointKind::Revolute || joint.joint_id < 0)
 return;
 joint.brake_capacity_Nm += maximum_torque;
 joint.brake_smoothing_radps = low_speed_smoothing;
 }

 void step(double dt) {
 require_compiled();
 if (!(dt > 0.0) || !finite(dt))
 throw std::invalid_argument("physics step must be finite and positive");
 std::vector<mjtNum> applied(data_->qfrc_applied, data_->qfrc_applied + model_->nv);
 double remaining = dt;
 contact_accumulators_.clear();
 while (remaining > 1e-12) {
 const double step_dt = std::min(kInternalTimestepS, remaining);
 model_->opt.timestep = step_dt;

 // MuJoCo's split-step contract computes position/velocity-dependent
 // quantities first, then accepts controls and applied forces before the
 // acceleration/constraint solve. In particular, xvel/xpos and the contact
 // Jacobians used below must correspond to the current substep state rather
 // than the preceding integration point.
 mj_step1(model_.get(), data_.get());
 std::copy(applied.begin(), applied.end(), data_->qfrc_applied);
 apply_damping();
 for (auto& joint : joints_) {
 if (!joint.active || joint.joint_id < 0 || joint.brake_capacity_Nm <= 0.0)
 continue;
 const int dof = model_->jnt_dofadr[joint.joint_id];
 const double direction = std::clamp(
 data_->qvel[dof] / joint.brake_smoothing_radps, -1.0, 1.0);
 data_->qfrc_applied[dof] -= direction * joint.brake_capacity_Nm;
 }
 mj_step2(model_.get(), data_.get());
 accumulate_contacts(step_dt);
 remaining -= step_dt;
 }
 model_->opt.timestep = kInternalTimestepS;
 finalize_contacts(dt);
 std::fill(data_->qfrc_applied, data_->qfrc_applied + model_->nv, 0.0);
 std::fill(data_->xfrc_applied, data_->xfrc_applied + 6 * model_->nbody, 0.0);
 for (auto& joint : joints_)
 joint.brake_capacity_Nm = 0.0;
 // Integration advances qpos/qvel, while xpos/xquat/cvel still describe the
 // state at the start of the last split step. Refresh derived quantities so
 // state(), raycast(), and the next public update observe the same timestamp.
 mj_forward(model_.get(), data_.get());
 }

 BodyState state(BodyHandle handle) const {
 require_compiled();
 return read_body_state(checked_body(handle));
 }

 JointState joint_state(JointHandle handle) const {
 require_compiled();
 return read_joint_state(checked_joint(handle));
 }

 RaycastResult raycast(const Ray& ray) const {
 require_compiled();
 const double length = norm(ray.direction);
 if (!(ray.max_distance_m > 0.0) || !finite(ray.max_distance_m) ||
 length < kEpsilon || ray.collision_mask == 0)
 return {};
 const Vec3 direction = scale(ray.direction, 1.0 / length);
 const mjtNum origin[3]{ray.origin_m.x, ray.origin_m.y, ray.origin_m.z};
 const mjtNum direction_mj[3]{direction.x, direction.y, direction.z};
 std::array<mjtByte, mjNGROUP> groups{};
 for (int index = 0; index < std::min(5, mjNGROUP); ++index)
 groups[index] = (ray.collision_mask & (CollisionMask{1} << index)) ? 1 : 0;
 int geom = -1;
 mjtNum normal_mj[3]{};
 int excluded_body = -1;
 if (ray.ignore_body) {
 const auto& ignored = checked_body(ray.ignore_body);
 excluded_body = ignored.body_id;
 }
 const double distance = mj_ray(model_.get(), data_.get(), origin, direction_mj,
 groups.data(), 1, excluded_body, &geom, normal_mj);
 if (distance < 0.0 || distance > ray.max_distance_m || geom < 0)
 return {};
 const auto found = body_by_geom_id_.find(geom);
 if (found == body_by_geom_id_.end())
 return {};
 return {true, found->second,
 add(ray.origin_m, scale(direction, distance)),
 {normal_mj[0], normal_mj[1], normal_mj[2]}, distance};
 }

 const std::vector<ContactEvent>& contacts() const { return contacts_; }

 private:
 void require_compiled() const {
 if (!compiled())
 throw std::logic_error("MuJoCo model must be compiled before runtime access");
 }

 void validate_body(const BodyModel& body) const {
 if (body.mass_kg < 0.0 || body.friction < 0.0 || body.rolling_friction < 0.0 ||
 body.spinning_friction < 0.0 || body.linear_damping < 0.0 ||
 body.angular_damping < 0.0 || body.contact_time_constant_s < 0.0 ||
 body.contact_damping_ratio < 0.0 ||
 (body.contact_time_constant_s > 0.0 &&
 (body.contact_time_constant_s < 2.0 * kInternalTimestepS ||
 !(body.contact_damping_ratio > 0.0))) ||
 body.collision_group == 0 || !finite(body.mass_kg) || !finite(body.friction) ||
 !finite(body.rolling_friction) || !finite(body.spinning_friction) ||
 !finite(body.linear_damping) || !finite(body.angular_damping) ||
 !finite(body.contact_time_constant_s) || !finite(body.contact_damping_ratio) ||
 !finite(body.contact_axle_axis_local) || !finite(body.position_m) ||
 !finite(body.inertial_position_m) ||
 !valid_diagonal_inertia(body.inertia_diagonal_kgm2, true))
 throw std::invalid_argument("invalid physics body properties");
 if (body.mass_kg == 0.0 && !body.kinematic &&
 norm(body.inertia_diagonal_kgm2) > 0.0)
 throw std::invalid_argument("static bodies cannot declare inertia");
 normalized(body.rotation);
 if (body.kinematic && body.mass_kg > 0.0)
 throw std::invalid_argument("kinematic bodies must not declare dynamic mass");
 switch (body.shape) {
 case ShapeKind::Box:
 if (!(body.half_extents_m.x > 0.0 && body.half_extents_m.y > 0.0 &&
 body.half_extents_m.z > 0.0))
 throw std::invalid_argument("box half extents must be positive");
 break;
 case ShapeKind::Sphere:
 if (!(body.radius_m > 0.0))
 throw std::invalid_argument("sphere radius must be positive");
 break;
 case ShapeKind::Cylinder:
 if (!(body.radius_m > 0.0 && body.height_m > 0.0))
 throw std::invalid_argument("cylinder dimensions must be positive");
 break;
 case ShapeKind::Ellipsoid:
 if (!(body.half_extents_m.x > 0.0 && body.half_extents_m.y > 0.0 &&
 body.half_extents_m.z > 0.0))
 throw std::invalid_argument("ellipsoid radii must be positive");
 break;
 case ShapeKind::HeightField:
 if (body.mass_kg != 0.0 || body.kinematic || body.heightfield_rows < 2 ||
 body.heightfield_cols < 2 ||
 body.heightfield_elevation_m.size() != body.heightfield_rows * body.heightfield_cols ||
 !(body.half_extents_m.x > 0.0 && body.half_extents_m.y > 0.0) ||
 !(body.heightfield_base_depth_m > 0.0))
 throw std::invalid_argument("invalid static height field");
 for (double height : body.heightfield_elevation_m) {
 if (!finite(height))
 throw std::invalid_argument("height field contains a non-finite sample");
 }
 break;
 }
 }

 void validate_tree() const {
 std::map<std::uint32_t, std::uint32_t> parent;
 for (const auto& joint : joints_) {
 if (!joint.active)
 continue;
 checked_body(joint.model.parent);
 checked_body(joint.model.child);
 if (!parent.emplace(joint.model.child.value, joint.model.parent.value).second)
 throw std::invalid_argument("physics model contains multiple parents");
 const auto& parent_body = checked_body(joint.model.parent).model;
 const auto& child_body = checked_body(joint.model.child).model;
 const auto parent_pivot_world = add(parent_body.position_m,
 rotate_vector(parent_body.rotation, joint.model.pivot_in_parent_m));
 const auto child_pivot_world = add(child_body.position_m,
 rotate_vector(child_body.rotation, joint.model.pivot_in_child_m));
 if (norm(subtract(parent_pivot_world, child_pivot_world)) > 1e-4)
 throw std::invalid_argument("joint pivots do not coincide in the authored pose");
 if (joint.model.kind == JointKind::Revolute) {
 const Vec3 parent_axis_world = normalized(
 rotate_vector(parent_body.rotation, joint.model.axis_in_parent));
 const Vec3 child_axis_world = normalized(
 rotate_vector(child_body.rotation, joint.model.axis_in_child));
 if (dot(parent_axis_world, child_axis_world) < 1.0 - 1e-6)
 throw std::invalid_argument("joint axes do not coincide in the authored pose");
 }
 }
 for (const auto& [child, unused] : parent) {
 (void)unused;
 std::set<std::uint32_t> visited;
 std::uint32_t cursor = child;
 while (parent.contains(cursor)) {
 if (!visited.insert(cursor).second)
 throw std::invalid_argument("physics model contains an articulation cycle");
 cursor = parent.at(cursor);
 }
 }
 }

 std::string build_xml() const {
 std::ostringstream xml;
 xml << std::setprecision(17);
 xml << "<mujoco model=\"lost_signal\">\n"
 << " <compiler angle=\"radian\" coordinate=\"local\" fusestatic=\"false\" inertiafromgeom=\"auto\"/>\n"
 << " <option timestep=\"" << kInternalTimestepS
 << "\" gravity=\"0 0 -9.81\" cone=\"elliptic\" solver=\"Newton\" iterations=\"50\" tolerance=\"1e-10\" impratio=\"5\" integrator=\"implicitfast\"/>\n";
 bool have_assets = false;
 for (std::size_t index = 0; index < bodies_.size(); ++index)
 have_assets = have_assets || (bodies_[index].active && bodies_[index].model.shape == ShapeKind::HeightField);
 if (have_assets) {
 xml << " <asset>\n";
 for (std::size_t index = 0; index < bodies_.size(); ++index) {
 if (!bodies_[index].active || bodies_[index].model.shape != ShapeKind::HeightField)
 continue;
 const BodyHandle handle{static_cast<std::uint32_t>(index + 1)};
 const auto& body = bodies_[index].model;
 const auto [minimum, maximum] = std::minmax_element(
 body.heightfield_elevation_m.begin(), body.heightfield_elevation_m.end());
 const double range = std::max(1e-6, *maximum - *minimum);
 xml << "  <hfield name=\"" << heightfield_name(handle) << "\" nrow=\""
 << body.heightfield_rows << "\" ncol=\"" << body.heightfield_cols
 << "\" size=\"" << body.half_extents_m.x << " " << body.half_extents_m.y
 << " " << range << " " << body.heightfield_base_depth_m << "\" elevation=\"";
 // BodyModel rows run from minimum Y to maximum Y. MJCF's inline elevation
 // matrix is image-ordered and the compiler flips its rows, so emit rows in
 // reverse order to preserve the project/world coordinate contract.
 for (std::size_t row = body.heightfield_rows; row-- > 0;) {
 for (std::size_t col = 0; col < body.heightfield_cols; ++col) {
 const double height = body.heightfield_elevation_m[row * body.heightfield_cols + col];
 xml << (height - *minimum) / range << " ";
 }
 }
 xml << "\"/>\n";
 }
 xml << " </asset>\n";
 }
 xml << " <worldbody>\n";
 std::map<std::uint32_t, JointHandle> parent_joint;
 std::map<std::uint32_t, std::vector<BodyHandle>> children;
 for (std::size_t index = 0; index < joints_.size(); ++index) {
 if (!joints_[index].active)
 continue;
 const JointHandle handle{static_cast<std::uint32_t>(index + 1)};
 parent_joint[joints_[index].model.child.value] = handle;
 children[joints_[index].model.parent.value].push_back(joints_[index].model.child);
 }
 for (std::size_t index = 0; index < bodies_.size(); ++index) {
 const BodyHandle handle{static_cast<std::uint32_t>(index + 1)};
 if (bodies_[index].active && !parent_joint.contains(handle.value))
 emit_body(xml, handle, std::nullopt, children, parent_joint, 2);
 }
 xml << " </worldbody>\n</mujoco>\n";
 return xml.str();
 }

 void emit_body(std::ostringstream& xml, BodyHandle handle,
 std::optional<BodyHandle> parent,
 const std::map<std::uint32_t, std::vector<BodyHandle>>& children,
 const std::map<std::uint32_t, JointHandle>& parent_joint, int indent) const {
 const auto& body = checked_body(handle).model;
 RigidTransform relative{{body.position_m.x, body.position_m.y, body.position_m.z}, body.rotation};
 if (parent) {
 const auto& parent_body = checked_body(*parent).model;
 const RigidTransform world_from_parent{parent_body.position_m, parent_body.rotation};
 relative = compose(inverse(world_from_parent), relative);
 }
 const std::string padding(static_cast<std::size_t>(indent), ' ');
 xml << padding << "<body name=\"" << body_name(handle) << "\" pos=\""
 << vector(relative.translation_m) << "\" quat=\"" << quaternion(relative.rotation) << "\"";
 // Runtime payload and identification updates may move the inertial frame.
 // MuJoCo's simple-body optimization allocates a diagonal-only mass block
 // and cannot accept that change in mj_setConst; compile a general block for
 // dynamic bodies so the public mass-properties contract remains valid.
 if (body.mass_kg > 0.0 && !body.kinematic && body.movable_inertial_origin)
 xml << " simple=\"false\"";
 xml << ">\n";
 if (!parent && (body.mass_kg > 0.0 || body.kinematic))
 xml << padding << " <freejoint name=\"" << free_joint_name(handle) << "\"/>\n";
 if (parent) {
 const auto joint_handle = parent_joint.at(handle.value);
 const auto& joint = checked_joint(joint_handle).model;
 if (joint.kind == JointKind::Revolute) {
 xml << padding << " <joint name=\"" << joint_name(joint_handle)
 << "\" type=\"hinge\" pos=\"" << vector(joint.pivot_in_child_m)
 << "\" axis=\"" << vector(normalized(joint.axis_in_child)) << "\"";
 if (joint.has_limits)
 xml << " limited=\"true\" range=\"" << joint.lower_limit_rad << " "
 << joint.upper_limit_rad << "\"";
 if (joint.armature_inertia_kgm2 > 0.0)
 xml << " armature=\"" << joint.armature_inertia_kgm2 << "\"";
 if (joint.damping_Nms_per_rad > 0.0)
 xml << " damping=\"" << joint.damping_Nms_per_rad << "\"";
 if (joint.friction_loss_Nm > 0.0)
 xml << " frictionloss=\"" << joint.friction_loss_Nm << "\"";
 xml << "/>\n";
 }
 }
 if (body.mass_kg > 0.0 && body.inertia_diagonal_kgm2.x > 0.0 &&
 body.inertia_diagonal_kgm2.y > 0.0 && body.inertia_diagonal_kgm2.z > 0.0) {
 xml << padding << " <inertial pos=\"" << vector(body.inertial_position_m)
 << "\" mass=\"" << body.mass_kg
 << "\" diaginertia=\"" << vector(body.inertia_diagonal_kgm2) << "\"/>\n";
 }
 const int group = collision_group_index(body.collision_group);
 xml << padding << " <geom name=\"" << geom_name(handle) << "\"";
 double height_offset = 0.0;
 switch (body.shape) {
 case ShapeKind::Box:
 xml << " type=\"box\" size=\"" << vector(body.half_extents_m) << "\"";
 break;
 case ShapeKind::Sphere:
 xml << " type=\"sphere\" size=\"" << body.radius_m << "\"";
 break;
 case ShapeKind::Cylinder:
 xml << " type=\"cylinder\" size=\"" << body.radius_m << " "
 << body.height_m * 0.5 << "\"";
 break;
 case ShapeKind::Ellipsoid:
 xml << " type=\"ellipsoid\" size=\"" << vector(body.half_extents_m) << "\"";
 break;
 case ShapeKind::HeightField: {
 const double minimum = *std::min_element(body.heightfield_elevation_m.begin(),
 body.heightfield_elevation_m.end());
 height_offset = minimum;
 xml << " type=\"hfield\" hfield=\"" << heightfield_name(handle) << "\"";
 break;
 }
 }
 if (height_offset != 0.0)
 xml << " pos=\"0 0 " << height_offset << "\"";
 xml << " contype=\"" << body.collision_group << "\" conaffinity=\""
 << body.collision_mask << "\" group=\"" << group << "\" friction=\""
 << body.friction << " " << body.spinning_friction << " " << body.rolling_friction
 << "\"";
 if (norm(body.contact_axle_axis_local) > 0.5)
 xml << " condim=\"6\" priority=\"1\"";
 else
 xml << " condim=\"3\"";
if (body.contact_time_constant_s > 0.0) {
xml << " solref=\"" << body.contact_time_constant_s << " "
<< body.contact_damping_ratio
 << "\" solimp=\"0.9 0.95 0.001 0.5 2\"";
 }
 if (body.mass_kg > 0.0 && !(body.inertia_diagonal_kgm2.x > 0.0 &&
 body.inertia_diagonal_kgm2.y > 0.0 && body.inertia_diagonal_kgm2.z > 0.0))
 xml << " mass=\"" << body.mass_kg << "\"";
 else if (body.mass_kg == 0.0 && !body.kinematic)
 xml << " mass=\"0\"";
 xml << "/>\n";
 const auto child_it = children.find(handle.value);
 if (child_it != children.end()) {
 for (BodyHandle child : child_it->second)
 emit_body(xml, child, handle, children, parent_joint, indent + 1);
 }
 xml << padding << "</body>\n";
 }

 void map_ids() {
 body_by_mj_id_.clear();
 body_by_geom_id_.clear();
 for (std::size_t index = 0; index < bodies_.size(); ++index) {
 auto& body = bodies_[index];
body.body_id = body.geom_id = body.free_joint_id = -1;
 if (!body.active)
 continue;
 const BodyHandle handle{static_cast<std::uint32_t>(index + 1)};
 body.body_id = mj_name2id(model_.get(), mjOBJ_BODY, body_name(handle).c_str());
 body.geom_id = mj_name2id(model_.get(), mjOBJ_GEOM, geom_name(handle).c_str());
 body.free_joint_id = mj_name2id(model_.get(), mjOBJ_JOINT, free_joint_name(handle).c_str());
 if (body.body_id < 0 || body.geom_id < 0)
 throw std::runtime_error("MuJoCo compiled model lost a body or geom mapping");
 body_by_mj_id_[body.body_id] = handle;
 body_by_geom_id_[body.geom_id] = handle;
 }
 for (std::size_t index = 0; index < joints_.size(); ++index) {
 auto& joint = joints_[index];
 joint.joint_id = -1;
 if (!joint.active || joint.model.kind != JointKind::Revolute)
 continue;
 const JointHandle handle{static_cast<std::uint32_t>(index + 1)};
 joint.joint_id = mj_name2id(model_.get(), mjOBJ_JOINT, joint_name(handle).c_str());
 if (joint.joint_id < 0)
 throw std::runtime_error("MuJoCo compiled model lost a joint mapping");
 }
 }

 BodyState read_body_state(const StoredBody& body) const {
 const int id = body.body_id;
 mjtNum velocity[6]{};
 mj_objectVelocity(model_.get(), data_.get(), mjOBJ_BODY, id, velocity, 0);
 BodyState state{{data_->xpos[3 * id], data_->xpos[3 * id + 1], data_->xpos[3 * id + 2]},
 {data_->xquat[4 * id + 1], data_->xquat[4 * id + 2], data_->xquat[4 * id + 3],
 data_->xquat[4 * id]},
 {velocity[3], velocity[4], velocity[5]},
 {velocity[0], velocity[1], velocity[2]}};
 return state;
 }

 JointState read_joint_state(const StoredJoint& joint) const {
 if (joint.model.kind != JointKind::Revolute || joint.joint_id < 0)
 return {};
 return {data_->qpos[model_->jnt_qposadr[joint.joint_id]],
 data_->qvel[model_->jnt_dofadr[joint.joint_id]]};
 }

 void write_body_state(StoredBody& body, const BodyState& state) {
 const Quaternion rotation = normalized(state.rotation);
 if (body.free_joint_id < 0)
 throw std::invalid_argument("articulated or static body cannot be teleported independently");
 const int qpos = model_->jnt_qposadr[body.free_joint_id];
 const int dof = model_->jnt_dofadr[body.free_joint_id];
 data_->qpos[qpos] = state.position_m.x;
 data_->qpos[qpos + 1] = state.position_m.y;
 data_->qpos[qpos + 2] = state.position_m.z;
 data_->qpos[qpos + 3] = rotation.w;
 data_->qpos[qpos + 4] = rotation.x;
 data_->qpos[qpos + 5] = rotation.y;
 data_->qpos[qpos + 6] = rotation.z;
 data_->qvel[dof] = state.linear_velocity_mps.x;
 data_->qvel[dof + 1] = state.linear_velocity_mps.y;
 data_->qvel[dof + 2] = state.linear_velocity_mps.z;
 // Free-joint translational velocity is world-aligned, while its rotational
 // three-vector is expressed in the body frame (MuJoCo maps it to world with
 // the body rotation). BodyState deliberately exposes both in world.
 const Vec3 angular_local = rotate_vector(
 {-rotation.x, -rotation.y, -rotation.z, rotation.w},
 state.angular_velocity_radps);
 data_->qvel[dof + 3] = angular_local.x;
 data_->qvel[dof + 4] = angular_local.y;
 data_->qvel[dof + 5] = angular_local.z;
 }

 void apply_damping() {
 for (const auto& body : bodies_) {
 if (!body.active || body.body_id < 0 ||
 (body.model.mass_kg <= 0.0 && !body.model.kinematic))
 continue;
 const auto state = read_body_state(body);
 const Vec3 force = scale(state.linear_velocity_mps, -body.model.linear_damping);
 const Vec3 torque = scale(state.angular_velocity_radps, -body.model.angular_damping);
 const mjtNum force_mj[3]{force.x, force.y, force.z};
 const mjtNum torque_mj[3]{torque.x, torque.y, torque.z};
 const mjtNum point[3]{data_->xipos[3 * body.body_id], data_->xipos[3 * body.body_id + 1],
 data_->xipos[3 * body.body_id + 2]};
 mj_applyFT(model_.get(), data_.get(), force_mj, torque_mj, point,
 body.body_id, data_->qfrc_applied);
 }
 }

 void accumulate_contacts(double step_dt) {
 for (int index = 0; index < data_->ncon; ++index) {
 const auto& contact = data_->contact[index];
 if (contact.geom[0] < 0 || contact.geom[1] < 0)
 continue;
 const auto first = body_by_geom_id_.find(contact.geom[0]);
 const auto second = body_by_geom_id_.find(contact.geom[1]);
 if (first == body_by_geom_id_.end() || second == body_by_geom_id_.end())
 continue;
 mjtNum wrench[6]{};
 mj_contactForce(model_.get(), data_.get(), index, wrench);
 const double normal_impulse = std::max(0.0, wrench[0]) * step_dt;
 // MuJoCo's contact frame normal and force convention point from geom 1
 // toward geom 2. body_a/body_b preserve that same order, so this is the
 // outward reaction direction acting on body_b.
 const Vec3 normal_on_b{contact.frame[0], contact.frame[1], contact.frame[2]};
 const Vec3 tangent1{contact.frame[3], contact.frame[4], contact.frame[5]};
 const Vec3 tangent2{contact.frame[6], contact.frame[7], contact.frame[8]};
 auto accumulated = std::find_if(contact_accumulators_.begin(), contact_accumulators_.end(),
 [&](const ContactAccumulator& value) {
 return value.body_a == first->second && value.body_b == second->second;
 });
 if (accumulated == contact_accumulators_.end()) {
 contact_accumulators_.push_back({first->second, second->second, {}, {}, {}, 0.0, 0.0});
 accumulated = std::prev(contact_accumulators_.end());
 }
 const double weight = std::max(normal_impulse, step_dt * 1e-9);
 accumulated->weighted_point = add(accumulated->weighted_point,
 scale({contact.pos[0], contact.pos[1], contact.pos[2]}, weight));
 accumulated->weighted_normal_on_b = add(accumulated->weighted_normal_on_b,
 scale(normal_on_b, weight));
 accumulated->tangent_impulse_world = add(accumulated->tangent_impulse_world,
 add(scale(tangent1, wrench[1] * step_dt), scale(tangent2, wrench[2] * step_dt)));
 accumulated->normal_impulse_Ns += normal_impulse;
 accumulated->geometry_weight += weight;
 }
 }

 void finalize_contacts(double public_dt) {
 contacts_.clear();
 contacts_.reserve(contact_accumulators_.size());
 for (const auto& accumulated : contact_accumulators_) {
 const double tangent_impulse = norm(accumulated.tangent_impulse_world);
 const Vec3 tangent = tangent_impulse > kEpsilon
 ? scale(accumulated.tangent_impulse_world, 1.0 / tangent_impulse)
 : Vec3{};
 const double normal_length = norm(accumulated.weighted_normal_on_b);
 const Vec3 normal = normal_length > kEpsilon
 ? scale(accumulated.weighted_normal_on_b, 1.0 / normal_length)
 : Vec3{0.0, 0.0, 1.0};
 contacts_.push_back({accumulated.body_a, accumulated.body_b,
 scale(accumulated.weighted_point, 1.0 / accumulated.geometry_weight), normal,
 accumulated.normal_impulse_Ns, accumulated.normal_impulse_Ns / public_dt,
 tangent, {}, tangent_impulse, 0.0, tangent_impulse / public_dt, 0.0});
 }
 }

 StoredBody& checked_body(BodyHandle handle) {
 if (!handle || handle.value > bodies_.size() || !bodies_[handle.value - 1].active)
 throw std::out_of_range("invalid physics body handle");
 return bodies_[handle.value - 1];
 }

 const StoredBody& checked_body(BodyHandle handle) const {
 if (!handle || handle.value > bodies_.size() || !bodies_[handle.value - 1].active)
 throw std::out_of_range("invalid physics body handle");
 return bodies_[handle.value - 1];
 }

 StoredJoint& checked_joint(JointHandle handle) {
 if (!handle || handle.value > joints_.size() || !joints_[handle.value - 1].active)
 throw std::out_of_range("invalid physics joint handle");
 return joints_[handle.value - 1];
 }

 const StoredJoint& checked_joint(JointHandle handle) const {
 if (!handle || handle.value > joints_.size() || !joints_[handle.value - 1].active)
 throw std::out_of_range("invalid physics joint handle");
 return joints_[handle.value - 1];
 }

 std::vector<StoredBody> bodies_;
 std::vector<StoredJoint> joints_;
 SpecPtr spec_;
 ModelPtr model_;
 DataPtr data_;
 bool dirty_{true};
 std::map<int, BodyHandle> body_by_mj_id_;
 std::map<int, BodyHandle> body_by_geom_id_;
 std::vector<ContactAccumulator> contact_accumulators_;
 std::vector<ContactEvent> contacts_;
};

MujocoSimulation::MujocoSimulation() : impl_(std::make_unique<Impl>()) {}
MujocoSimulation::~MujocoSimulation() = default;
MujocoSimulation::MujocoSimulation(MujocoSimulation&&) noexcept = default;
MujocoSimulation& MujocoSimulation::operator=(MujocoSimulation&&) noexcept = default;

void MujocoSimulation::clear_model() { impl_->clear_model(); }
BodyHandle MujocoSimulation::add_body(const BodyModel& body) { return impl_->add_body(body); }
JointHandle MujocoSimulation::add_joint(const JointModel& joint) { return impl_->add_joint(joint); }
void MujocoSimulation::remove_body(BodyHandle body) { impl_->remove_body(body); }
void MujocoSimulation::compile() { impl_->compile(); }
bool MujocoSimulation::compiled() const { return impl_->compiled(); }
void MujocoSimulation::clear_velocities() { impl_->clear_velocities(); }
void MujocoSimulation::set_state(BodyHandle body, const BodyState& state) { impl_->set_state(body, state); }
void MujocoSimulation::set_mass_properties(
 BodyHandle body, double mass, Vec3 inertia, Vec3 inertial_position) {
 impl_->set_mass_properties(body, mass, inertia, inertial_position);
}

void MujocoSimulation::set_contact_parameters(BodyHandle body, double friction,
 double rolling, double spinning, double time_constant, double damping_ratio) {
impl_->set_contact_parameters(body, friction, rolling, spinning, time_constant, damping_ratio);
}
void MujocoSimulation::apply_drive(BodyHandle body, const DriveForces& forces) {
 const auto state = impl_->state(body);
 const auto forward = rotate_vector(state.rotation, {1.0, 0.0, 0.0});
 const auto lateral = rotate_vector(state.rotation, {0.0, 1.0, 0.0});
 Vec3 force = add(scale(forward, forces.forward_force_N), scale(lateral, forces.lateral_force_N));
 const double forward_speed = dot(state.linear_velocity_mps, forward);
 if (std::abs(forward_speed) > kEpsilon && forces.brake_force_N > 0.0)
 force = add(force, scale(forward, -std::copysign(forces.brake_force_N, forward_speed)));
 impl_->apply_force_world(body, force, state.position_m);
 impl_->apply_torque_world(body, {0.0, 0.0, forces.yaw_torque_Nm});
}
void MujocoSimulation::apply_force_at_local_point(BodyHandle body, Vec3 force, Vec3 point) {
 const auto state = impl_->state(body);
 impl_->apply_force_world(body, rotate_vector(state.rotation, force),
 add(state.position_m, rotate_vector(state.rotation, point)));
}
void MujocoSimulation::apply_force_at_world_point(BodyHandle body, Vec3 force, Vec3 point) {
 impl_->apply_force_world(body, force, point);
}
void MujocoSimulation::apply_local_torque(BodyHandle body, Vec3 torque) {
 const auto state = impl_->state(body);
 impl_->apply_torque_world(body, rotate_vector(state.rotation, torque));
}
void MujocoSimulation::apply_world_torque(BodyHandle body, Vec3 torque) {
 impl_->apply_torque_world(body, torque);
}
void MujocoSimulation::apply_joint_effort(JointHandle joint, double effort) {
 impl_->apply_joint_effort(joint, effort);
}
void MujocoSimulation::apply_joint_brake(JointHandle joint, double maximum_torque_Nm,
 double low_speed_smoothing_radps) {
 impl_->apply_joint_brake(joint, maximum_torque_Nm, low_speed_smoothing_radps);
}
void MujocoSimulation::step(double dt) { impl_->step(dt); }
BodyState MujocoSimulation::state(BodyHandle body) const { return impl_->state(body); }
JointState MujocoSimulation::joint_state(JointHandle joint) const { return impl_->joint_state(joint); }
const std::vector<ContactEvent>& MujocoSimulation::contacts() const { return impl_->contacts(); }
RaycastResult MujocoSimulation::raycast(const Ray& ray) const { return impl_->raycast(ray); }

bool mujoco_physics_available() { return true; }

Quaternion quaternion_from_euler(double roll, double pitch, double yaw) {
 const double cr = std::cos(roll * 0.5);
 const double sr = std::sin(roll * 0.5);
 const double cp = std::cos(pitch * 0.5);
 const double sp = std::sin(pitch * 0.5);
 const double cy = std::cos(yaw * 0.5);
 const double sy = std::sin(yaw * 0.5);
 return normalized({sr * cp * cy - cr * sp * sy,
 cr * sp * cy + sr * cp * sy,
 cr * cp * sy - sr * sp * cy,
 cr * cp * cy + sr * sp * sy});
}

Vec3 euler_from_quaternion(const Quaternion& input) {
 const Quaternion q = normalized(input);
 const double sin_roll = 2.0 * (q.w * q.x + q.y * q.z);
 const double cos_roll = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
 const double sin_pitch = 2.0 * (q.w * q.y - q.z * q.x);
 const double sin_yaw = 2.0 * (q.w * q.z + q.x * q.y);
 const double cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
 return {std::atan2(sin_roll, cos_roll),
 std::asin(std::clamp(sin_pitch, -1.0, 1.0)),
 std::atan2(sin_yaw, cos_yaw)};
}

} // namespace sim::physics
