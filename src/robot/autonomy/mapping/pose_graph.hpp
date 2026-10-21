#pragma once

#include "robot/core/math.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>
#include <vector>

namespace sim::autonomy {

// Planar pose used by the mapping backend. This is deliberately a local
// representation: consumers receive the ordinary localization/map interfaces
// and never need a graph or solver type.
struct Pose2 {
 Vec2 position;
 double heading_rad{0.0};
};

Pose2 compose_pose(const Pose2& first, const Pose2& second);
Pose2 inverse_pose(const Pose2& pose);
Pose2 relative_pose(const Pose2& from, const Pose2& to);
double pose_translation_error(const Pose2& first, const Pose2& second);

enum class PoseGraphConstraintKind {
 Odometry,
 ScanMatch,
 LoopClosure,
};

struct PoseGraphKeyframe {
 std::uint64_t id{0};
 double stamp_s{0.0};
 Pose2 initial_pose;
 // A graph has at least one fixed node to remove the global gauge freedom.
 bool fixed{false};
};

struct PoseGraphConstraint {
 std::uint64_t id{0};
 std::uint64_t from{0};
 std::uint64_t to{0};
 Pose2 measured_relative_pose;
 // Row-major 3x3 information matrix in [x, y, heading] order. It must be
 // finite and positive definite. Public callers can construct it from
 // standard deviations with pose_graph_information().
 std::array<double, 9> information{1.0, 0.0, 0.0,
 0.0, 1.0, 0.0,
 0.0, 0.0, 1.0};
 PoseGraphConstraintKind kind{PoseGraphConstraintKind::Odometry};
};

std::array<double, 9> pose_graph_information(double translation_sigma_m,
 double heading_sigma_rad);

struct PoseGraphOptimizationConfig {
 std::size_t maximum_iterations{20};
 double convergence_translation_m{1e-5};
 double convergence_rotation_rad{1e-5};
 // Huber threshold in whitened residual units for loop-closure factors.
 // Candidate generation must still reject poor or ambiguous loops; this is
 // a second line of defence against an inconsistent association.
 double loop_closure_huber_threshold{3.0};
};

struct PoseGraphOptimizationResult {
 bool success{false};
 std::string diagnostic;
 std::size_t iterations{0};
 double initial_cost{0.0};
 double final_cost{0.0};
 double maximum_translation_correction_m{0.0};
 double maximum_heading_correction_rad{0.0};
};

// Sparse SE(2) pose-graph factor graph for a mapping session. Odometry and
// scan-match constraints form the local chain; verified loop closures add
// global constraints. The solver is a small Gauss-Newton implementation over
// the sparse normal equations, so it does not make GTSAM/Ceres a public or
// deployment dependency. The graph is session data and is not embedded in a
// published navigation-map snapshot.
class PoseGraphBackend {
 public:
 bool add_keyframe(PoseGraphKeyframe keyframe, std::string* reason = nullptr);
 bool add_constraint(PoseGraphConstraint constraint, std::string* reason = nullptr);

 void clear();
 bool empty() const { return keyframes_.empty(); }
 std::size_t keyframe_count() const { return keyframes_.size(); }
 std::size_t constraint_count() const { return constraints_.size(); }
 const std::vector<PoseGraphKeyframe>& keyframes() const { return keyframes_; }
 const std::vector<PoseGraphConstraint>& constraints() const { return constraints_; }
 const PoseGraphKeyframe* find_keyframe(std::uint64_t id) const;

 // Optimizes in place. Invalid input leaves every pose untouched. The first
 // fixed node (or the lowest-id node when none is marked) anchors the graph.
 PoseGraphOptimizationResult optimize(
 PoseGraphOptimizationConfig config = {});
 double total_error() const;

 // Publishes poses produced by optimize() back into this graph. The caller
 // passes the optimized keyframe list from the same session; identity,
 // ordering and timestamps must match exactly so a correction can never be
 // attached to the wrong node. Constraint measurements are untouched.
 bool apply_keyframe_poses(const std::vector<PoseGraphKeyframe>& poses,
 std::string* reason = nullptr);

 private:
 bool validate(std::string* reason) const;
 std::vector<PoseGraphKeyframe> keyframes_;
 std::vector<PoseGraphConstraint> constraints_;
 std::unordered_map<std::uint64_t, std::size_t> keyframe_indices_;
};

} // namespace sim::autonomy
