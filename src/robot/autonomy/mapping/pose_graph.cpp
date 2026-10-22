#include "robot/autonomy/mapping/pose_graph.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Sparse>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <utility>

namespace lost_signal::robot::autonomy {
namespace {

core::Vec2 rotate(core::Vec2 point, double heading) {
 const double c = std::cos(heading);
 const double s = std::sin(heading);
 return {c * point.x - s * point.y, s * point.x + c * point.y};
}

bool finite_pose(const Pose2& pose) {
 return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) &&
 std::isfinite(pose.heading_rad);
}

bool information_is_valid(const std::array<double, 9>& values) {
 for (double value : values) {
 if (!std::isfinite(value))
 return false;
 }
 Eigen::Matrix3d information;
 for (Eigen::Index row = 0; row < 3; ++row)
 for (Eigen::Index column = 0; column < 3; ++column)
 information(row, column) = values[static_cast<std::size_t>(row * 3 + column)];
 if ((information - information.transpose()).cwiseAbs().maxCoeff() > 1e-9)
 return false;
 Eigen::LLT<Eigen::Matrix3d> factor(information);
 return factor.info() == Eigen::Success;
}

struct Residual {
 Eigen::Vector3d error;
 Eigen::Matrix3d jacobian_from;
 Eigen::Matrix3d jacobian_to;
};

Eigen::Vector3d residual_error(const Pose2& from, const Pose2& to,
 const PoseGraphConstraint& constraint) {
 const auto& z = constraint.measured_relative_pose;
 const double dx = to.position.x - from.position.x;
 const double dy = to.position.y - from.position.y;
 const double ci = std::cos(from.heading_rad);
 const double si = std::sin(from.heading_rad);
 const double predicted_x = ci * dx + si * dy;
 const double predicted_y = -si * dx + ci * dy;

 Eigen::Vector3d output;
 output[0] = predicted_x - z.position.x;
 output[1] = predicted_y - z.position.y;
 output[2] = core::wrap_angle(to.heading_rad - from.heading_rad -
 z.heading_rad);

 return output;
}

Residual residual(const Pose2& from, const Pose2& to,
 const PoseGraphConstraint& constraint) {
 Residual output;
 output.error = residual_error(from, to, constraint);
 // Numerical derivatives keep the SE(2) convention in one place and avoid
 // a subtle sign error at heading wrap boundaries. The graph is sparse and
 // keyframes are deliberately bounded, so this small fixed stencil is cheap.
 constexpr double epsilon = 1e-6;
 for (int column = 0; column < 3; ++column) {
 Pose2 from_plus = from;
 Pose2 from_minus = from;
 Pose2 to_plus = to;
 Pose2 to_minus = to;
 if (column == 0) {
 from_plus.position.x += epsilon;
 from_minus.position.x -= epsilon;
 to_plus.position.x += epsilon;
 to_minus.position.x -= epsilon;
 } else if (column == 1) {
 from_plus.position.y += epsilon;
 from_minus.position.y -= epsilon;
 to_plus.position.y += epsilon;
 to_minus.position.y -= epsilon;
 } else {
 from_plus.heading_rad += epsilon;
 from_minus.heading_rad -= epsilon;
 to_plus.heading_rad += epsilon;
 to_minus.heading_rad -= epsilon;
 }
 output.jacobian_from.col(column) =
 (residual_error(from_plus, to, constraint) -
 residual_error(from_minus, to, constraint)) / (2.0 * epsilon);
 output.jacobian_to.col(column) =
 (residual_error(from, to_plus, constraint) -
 residual_error(from, to_minus, constraint)) / (2.0 * epsilon);
 }
 // The optimizer increment is a right/local SE(2) perturbation. The finite
 // difference above is easiest to express in world coordinates, so rotate
 // each pose's translational columns into its local tangent basis before
 // assembling the normal equations.
 const auto localize_translation_columns = [](Eigen::Matrix3d& jacobian,
 double heading) {
 const Eigen::Vector3d world_x = jacobian.col(0);
 const Eigen::Vector3d world_y = jacobian.col(1);
 const double cosine = std::cos(heading);
 const double sine = std::sin(heading);
 jacobian.col(0) = cosine * world_x + sine * world_y;
 jacobian.col(1) = -sine * world_x + cosine * world_y;
 };
 localize_translation_columns(output.jacobian_from, from.heading_rad);
 localize_translation_columns(output.jacobian_to, to.heading_rad);
 return output;
}

double edge_cost(const Eigen::Vector3d& error, const Eigen::Matrix3d& information,
 bool robust, double huber_threshold, double* robust_weight) {
 const double squared = error.dot(information * error);
 const double scale = std::sqrt(std::max(0.0, squared));
 if (robust_weight)
 *robust_weight = robust && scale > huber_threshold && scale > 1e-12
 ? huber_threshold / scale
 : 1.0;
 if (robust && scale > huber_threshold)
 return 2.0 * huber_threshold * scale - huber_threshold * huber_threshold;
 return squared;
}

bool validate_config(const PoseGraphOptimizationConfig& config, std::string* reason) {
 if (config.maximum_iterations == 0) {
 *reason = "pose graph requires at least one optimization iteration";
 return false;
 }
 if (!(config.convergence_translation_m > 0.0) ||
 !(config.convergence_rotation_rad > 0.0) ||
 !(config.loop_closure_huber_threshold > 0.0)) {
 *reason = "invalid pose graph optimization configuration";
 return false;
 }
 return true;
}

} // namespace

Pose2 compose_pose(const Pose2& first, const Pose2& second) {
 const auto translated = rotate(second.position, first.heading_rad);
 return {{first.position.x + translated.x, first.position.y + translated.y},
 core::wrap_angle(first.heading_rad + second.heading_rad)};
}

Pose2 inverse_pose(const Pose2& pose) {
 const auto translated = rotate({-pose.position.x, -pose.position.y}, -pose.heading_rad);
 return {translated, core::wrap_angle(-pose.heading_rad)};
}

Pose2 relative_pose(const Pose2& from, const Pose2& to) {
 return compose_pose(inverse_pose(from), to);
}

double pose_translation_error(const Pose2& first, const Pose2& second) {
 return std::hypot(first.position.x - second.position.x,
 first.position.y - second.position.y);
}

std::array<double, 9> pose_graph_information(double translation_sigma_m,
 double heading_sigma_rad) {
 if (!(translation_sigma_m > 0.0) || !(heading_sigma_rad > 0.0))
 return {};
 std::array<double, 9> result{};
 result[0] = 1.0 / (translation_sigma_m * translation_sigma_m);
 result[4] = result[0];
 result[8] = 1.0 / (heading_sigma_rad * heading_sigma_rad);
 return result;
}

bool PoseGraphBackend::add_keyframe(PoseGraphKeyframe keyframe, std::string* reason) {
 const auto fail = [&](std::string message) {
 if (reason)
 *reason = std::move(message);
 return false;
 };
 if (keyframe.id == 0)
 return fail("keyframe id must be nonzero");
 if (!std::isfinite(keyframe.stamp_s) || keyframe.stamp_s < 0.0 ||
 !finite_pose(keyframe.initial_pose))
 return fail("keyframe timestamp or pose is not finite");
 if (keyframe_indices_.contains(keyframe.id))
 return fail("duplicate keyframe id");
 if (!keyframes_.empty() && keyframe.stamp_s <= keyframes_.back().stamp_s)
 return fail("keyframe timestamps must be strictly increasing");
 keyframe_indices_.emplace(keyframe.id, keyframes_.size());
 keyframes_.push_back(keyframe);
 return true;
}

bool PoseGraphBackend::add_constraint(PoseGraphConstraint constraint, std::string* reason) {
 const auto fail = [&](std::string message) {
 if (reason)
 *reason = std::move(message);
 return false;
 };
 if (constraint.id == 0)
 return fail("constraint id must be nonzero");
 if (constraint.from == constraint.to)
 return fail("self edges are not a pose-graph constraint");
 if (!keyframe_indices_.contains(constraint.from) ||
 !keyframe_indices_.contains(constraint.to))
 return fail("constraint references an unknown keyframe");
 if (!finite_pose(constraint.measured_relative_pose))
 return fail("constraint measurement is not finite");
 if (!information_is_valid(constraint.information))
 return fail("constraint information matrix must be finite and positive definite");
 if (std::any_of(constraints_.begin(), constraints_.end(),
 [&](const auto& existing) { return existing.id == constraint.id; }))
 return fail("duplicate constraint id");
 constraints_.push_back(constraint);
 return true;
}

void PoseGraphBackend::clear() {
 keyframes_.clear();
 constraints_.clear();
 keyframe_indices_.clear();
}

const PoseGraphKeyframe* PoseGraphBackend::find_keyframe(std::uint64_t id) const {
 const auto found = keyframe_indices_.find(id);
 return found == keyframe_indices_.end() ? nullptr : &keyframes_[found->second];
}

double PoseGraphBackend::total_error() const {
 double cost = 0.0;
 for (const auto& constraint : constraints_) {
 const auto from = find_keyframe(constraint.from);
 const auto to = find_keyframe(constraint.to);
 if (!from || !to)
 return std::numeric_limits<double>::infinity();
 Eigen::Matrix3d information;
 for (Eigen::Index row = 0; row < 3; ++row)
 for (Eigen::Index column = 0; column < 3; ++column)
 information(row, column) = constraint.information[
 static_cast<std::size_t>(row * 3 + column)];
 cost += edge_cost(residual(from->initial_pose, to->initial_pose, constraint).error,
 information, false, 0.0, nullptr);
 }
 return cost;
}

bool PoseGraphBackend::validate(std::string* reason) const {
 if (keyframes_.empty()) {
 *reason = "pose graph has no keyframes";
 return false;
 }
 if (constraints_.empty()) {
 *reason = "pose graph has no constraints";
 return false;
 }
 std::size_t fixed_count = static_cast<std::size_t>(std::count_if(
 keyframes_.begin(), keyframes_.end(), [](const auto& node) { return node.fixed; }));
 if (fixed_count == 0) {
 *reason = "pose graph has no fixed anchor";
 return false;
 }

 // Every optimized pose must have an algebraic path to a fixed node. A
 // silently ignored disconnected component would be a false success.
 std::vector<std::vector<std::size_t>> adjacency(keyframes_.size());
 for (const auto& constraint : constraints_) {
 const auto from = keyframe_indices_.find(constraint.from);
 const auto to = keyframe_indices_.find(constraint.to);
 if (from == keyframe_indices_.end() || to == keyframe_indices_.end()) {
 *reason = "constraint references an unknown keyframe";
 return false;
 }
 adjacency[from->second].push_back(to->second);
 adjacency[to->second].push_back(from->second);
 }
 std::vector<bool> reached(keyframes_.size(), false);
 std::queue<std::size_t> pending;
 for (std::size_t index = 0; index < keyframes_.size(); ++index) {
 if (keyframes_[index].fixed) {
 reached[index] = true;
 pending.push(index);
 }
 }
 while (!pending.empty()) {
 const auto node = pending.front();
 pending.pop();
 for (const auto neighbour : adjacency[node]) {
 if (!reached[neighbour]) {
 reached[neighbour] = true;
 pending.push(neighbour);
 }
 }
 }
 if (std::find(reached.begin(), reached.end(), false) != reached.end()) {
 *reason = "pose graph is disconnected from its fixed anchor";
 return false;
 }
 return true;
}

PoseGraphOptimizationResult PoseGraphBackend::optimize(
 PoseGraphOptimizationConfig config) {
 PoseGraphOptimizationResult result;
 std::string reason;
 if (!validate_config(config, &reason) || !validate(&reason)) {
 result.diagnostic = std::move(reason);
 return result;
 }

 std::vector<Pose2> poses;
 poses.reserve(keyframes_.size());
 const auto initial_poses = keyframes_;
 for (const auto& keyframe : keyframes_)
 poses.push_back(keyframe.initial_pose);

 std::vector<int> variable_index(keyframes_.size(), -1);
 std::size_t variables = 0;
 for (std::size_t index = 0; index < keyframes_.size(); ++index) {
 if (!keyframes_[index].fixed) {
 variable_index[index] = static_cast<int>(variables);
 variables += 3;
 }
 }

 const auto evaluate = [&](const std::vector<Pose2>& candidate, bool build_system,
 Eigen::SparseMatrix<double>* hessian,
 Eigen::VectorXd* gradient) {
 double cost = 0.0;
 if (build_system) {
 hessian->resize(static_cast<Eigen::Index>(variables),
 static_cast<Eigen::Index>(variables));
 gradient->resize(static_cast<Eigen::Index>(variables));
 gradient->setZero();
 }
 std::vector<Eigen::Triplet<double>> coefficients;
 for (const auto& constraint : constraints_) {
 const auto from = keyframe_indices_.at(constraint.from);
 const auto to = keyframe_indices_.at(constraint.to);
 Eigen::Matrix3d information;
 for (Eigen::Index row = 0; row < 3; ++row)
 for (Eigen::Index column = 0; column < 3; ++column)
 information(row, column) = constraint.information[
 static_cast<std::size_t>(row * 3 + column)];
 const auto factor = residual(candidate[from], candidate[to], constraint);
 double robust_weight = 1.0;
 cost += edge_cost(factor.error, information,
 constraint.kind == PoseGraphConstraintKind::LoopClosure,
 config.loop_closure_huber_threshold, &robust_weight);
 if (!build_system)
 continue;
 const Eigen::Matrix3d weighted_information = information * robust_weight;
 const Eigen::Matrix3d h_from_from =
 factor.jacobian_from.transpose() * weighted_information * factor.jacobian_from;
 const Eigen::Matrix3d h_from_to =
 factor.jacobian_from.transpose() * weighted_information * factor.jacobian_to;
 const Eigen::Matrix3d h_to_to =
 factor.jacobian_to.transpose() * weighted_information * factor.jacobian_to;
 const Eigen::Vector3d b_from =
 factor.jacobian_from.transpose() * weighted_information * factor.error;
 const Eigen::Vector3d b_to =
 factor.jacobian_to.transpose() * weighted_information * factor.error;
 const int from_offset = variable_index[from];
 const int to_offset = variable_index[to];
 const auto add_block = [&](int row_offset, int column_offset,
 const Eigen::Matrix3d& block) {
 if (row_offset < 0 || column_offset < 0)
 return;
 for (Eigen::Index row = 0; row < 3; ++row)
 for (Eigen::Index column = 0; column < 3; ++column)
 coefficients.emplace_back(row_offset + row, column_offset + column,
 block(row, column));
 };
 add_block(from_offset, from_offset, h_from_from);
 add_block(from_offset, to_offset, h_from_to);
 add_block(to_offset, from_offset, h_from_to.transpose());
 add_block(to_offset, to_offset, h_to_to);
 if (from_offset >= 0)
 gradient->segment<3>(from_offset) += b_from;
 if (to_offset >= 0)
 gradient->segment<3>(to_offset) += b_to;
 }
 if (build_system)
 hessian->setFromTriplets(coefficients.begin(), coefficients.end());
 return cost;
 };

 const double initial_cost = evaluate(poses, false, nullptr, nullptr);
 double current_cost = initial_cost;
 double damping = 1e-6;
 for (std::size_t iteration = 0; iteration < config.maximum_iterations; ++iteration) {
 result.iterations = iteration + 1;
 Eigen::SparseMatrix<double> hessian;
 Eigen::VectorXd gradient;
 evaluate(poses, true, &hessian, &gradient);
 bool accepted = false;
 double accepted_correction = std::numeric_limits<double>::infinity();
 double accepted_rotation = std::numeric_limits<double>::infinity();
 std::vector<Pose2> accepted_poses;
 for (int attempt = 0; attempt < 8 && !accepted; ++attempt) {
 Eigen::SparseMatrix<double> damped = hessian;
 std::vector<Eigen::Triplet<double>> damping_triplets;
 for (std::size_t index = 0; index < variables; ++index)
 damping_triplets.emplace_back(static_cast<Eigen::Index>(index),
 static_cast<Eigen::Index>(index), damping);
 Eigen::SparseMatrix<double> damping_matrix(
 static_cast<Eigen::Index>(variables), static_cast<Eigen::Index>(variables));
 damping_matrix.setFromTriplets(damping_triplets.begin(), damping_triplets.end());
 damped += damping_matrix;
 Eigen::SimplicialLDLT<Eigen::SparseMatrix<double>> solver;
 solver.compute(damped);
 if (solver.info() != Eigen::Success) {
 damping *= 10.0;
 continue;
 }
 const Eigen::VectorXd delta = solver.solve(-gradient);
 if (solver.info() != Eigen::Success || !delta.allFinite()) {
 damping *= 10.0;
 continue;
 }
 auto trial = poses;
 double maximum_translation = 0.0;
 double maximum_rotation = 0.0;
 for (std::size_t index = 0; index < keyframes_.size(); ++index) {
 const int offset = variable_index[index];
 if (offset < 0)
 continue;
 const double cosine = std::cos(poses[index].heading_rad);
 const double sine = std::sin(poses[index].heading_rad);
 const core::Vec2 correction{
 cosine * delta[offset] - sine * delta[offset + 1],
 sine * delta[offset] + cosine * delta[offset + 1]};
 const double correction_heading = delta[offset + 2];
 trial[index].position.x += correction.x;
 trial[index].position.y += correction.y;
 trial[index].heading_rad = core::wrap_angle(trial[index].heading_rad + correction_heading);
 maximum_translation = std::max(maximum_translation, std::hypot(correction.x, correction.y));
 maximum_rotation = std::max(maximum_rotation, std::abs(correction_heading));
 }
 const double trial_cost = evaluate(trial, false, nullptr, nullptr);
 if (std::isfinite(trial_cost) && trial_cost < current_cost) {
 poses = std::move(trial);
 current_cost = trial_cost;
 accepted = true;
 accepted_correction = maximum_translation;
 accepted_rotation = maximum_rotation;
 damping = std::max(1e-12, damping * 0.3);
 } else {
 damping *= 10.0;
 }
 }
 if (!accepted) {
 // Reaching a local stationary point is success; inability to obtain any
 // finite descent at all is not.
 if (result.iterations == 1 && !(current_cost < initial_cost)) {
 result.diagnostic = "pose graph did not produce a finite descent step";
 return result;
 }
 break;
 }
 if (accepted_correction <= config.convergence_translation_m &&
 accepted_rotation <= config.convergence_rotation_rad)
 break;
 }

 for (std::size_t index = 0; index < keyframes_.size(); ++index) {
 if (!finite_pose(poses[index])) {
 result.diagnostic = "pose graph optimization produced a non-finite pose";
 return result;
 }
 }
 for (std::size_t index = 0; index < keyframes_.size(); ++index) {
 const auto& before = initial_poses[index].initial_pose;
 result.maximum_translation_correction_m = std::max(
 result.maximum_translation_correction_m, pose_translation_error(before, poses[index]));
 result.maximum_heading_correction_rad = std::max(
 result.maximum_heading_correction_rad,
 std::abs(core::wrap_angle(before.heading_rad - poses[index].heading_rad)));
 keyframes_[index].initial_pose = poses[index];
 }
 result.success = true;
 result.initial_cost = initial_cost;
 result.final_cost = current_cost;
 result.diagnostic = "pose graph optimized";
 return result;
}

bool PoseGraphBackend::apply_keyframe_poses(
 const std::vector<PoseGraphKeyframe>& poses, std::string* reason) {
 const auto fail = [&](std::string message) {
 if (reason)
 *reason = std::move(message);
 return false;
 };
 if (poses.size() != keyframes_.size())
 return fail("optimized pose set does not match the keyframe count");
 for (std::size_t index = 0; index < poses.size(); ++index) {
 if (poses[index].id != keyframes_[index].id)
 return fail("optimized pose ids do not match this graph");
 if (poses[index].stamp_s != keyframes_[index].stamp_s)
 return fail("optimized pose timestamps do not match this graph");
 if (!finite_pose(poses[index].initial_pose))
 return fail("optimized pose set contains a non-finite pose");
 }
 for (std::size_t index = 0; index < poses.size(); ++index)
 keyframes_[index].initial_pose = poses[index].initial_pose;
 if (reason)
 reason->clear();
 return true;
}

}  // namespace lost_signal::robot::autonomy
