#include "robot/autonomy/mapping/loop_closure.hpp"

#include <Eigen/Cholesky>

#include <cmath>

namespace sim::autonomy {
namespace {

bool valid_information(const std::array<double, 9>& values) {
 for (const double value : values) {
 if (!std::isfinite(value))
 return false;
 }
 Eigen::Matrix3d matrix;
 for (Eigen::Index row = 0; row < 3; ++row)
 for (Eigen::Index column = 0; column < 3; ++column)
 matrix(row, column) = values[static_cast<std::size_t>(row * 3 + column)];
 if ((matrix - matrix.transpose()).cwiseAbs().maxCoeff() > 1e-9)
 return false;
 Eigen::LLT<Eigen::Matrix3d> factor(matrix);
 return factor.info() == Eigen::Success;
}

} // namespace

LoopClosureDecision LoopClosureGate::validate(
 const PoseGraphKeyframe& from, const PoseGraphKeyframe& to,
 const LoopClosureEvidence& evidence) const {
 LoopClosureDecision output;
 const auto reject = [&](const char* reason) {
 output.diagnostic = reason;
 return output;
 };
 if (from.id == 0 || to.id == 0 || from.id == to.id)
 return reject("loop closure requires two distinct keyframes");
 if (!std::isfinite(from.stamp_s) || !std::isfinite(to.stamp_s) ||
 std::abs(to.stamp_s - from.stamp_s) < config_.minimum_temporal_separation_s)
 return reject("loop closure temporal separation is too small");
 if (!(config_.maximum_candidate_distance_m > 0.0) ||
 !(config_.minimum_overlap >= 0.0 && config_.minimum_overlap <= 1.0) ||
 !(config_.minimum_score_margin >= 0.0))
 return reject("invalid loop closure gate configuration");
 if (!std::isfinite(evidence.match_score) ||
 !std::isfinite(evidence.overlap) || !std::isfinite(evidence.score_margin) ||
 !std::isfinite(evidence.measured_relative_pose.position.x) ||
 !std::isfinite(evidence.measured_relative_pose.position.y) ||
 !std::isfinite(evidence.measured_relative_pose.heading_rad))
 return reject("loop closure evidence is not finite");
 if (pose_translation_error(from.initial_pose, to.initial_pose) >
 config_.maximum_candidate_distance_m)
 return reject("loop closure candidate is outside spatial gate");
 if (evidence.overlap < config_.minimum_overlap)
 return reject("loop closure scan overlap below gate");
 if (evidence.score_margin < config_.minimum_score_margin)
 return reject("loop closure match is ambiguous");
 if (!valid_information(evidence.information))
 return reject("loop closure information matrix is invalid");
 output.accepted = true;
 output.diagnostic = "loop closure candidate accepted";
 return output;
}

} // namespace sim::autonomy
