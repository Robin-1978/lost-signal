#include "robot/autonomy/mapping/pose_graph.hpp"

#include <cmath>
#include <iostream>
#include <string>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

sim::autonomy::PoseGraphConstraint edge(std::uint64_t id,
 std::uint64_t from, std::uint64_t to, sim::autonomy::Pose2 measurement,
 sim::autonomy::PoseGraphConstraintKind kind) {
 sim::autonomy::PoseGraphConstraint output;
 output.id = id;
 output.from = from;
 output.to = to;
 output.measured_relative_pose = measurement;
 output.information = sim::autonomy::pose_graph_information(0.03, 0.03);
 output.kind = kind;
 return output;
}

} // namespace

int main() {
 using namespace sim::autonomy;
 PoseGraphBackend graph;
 CHECK(graph.add_keyframe({1, 0.0, {{0.0, 0.0}, 0.0}, true}));
 // The chain has accumulated translation and heading drift.
 CHECK(graph.add_keyframe({2, 1.0, {{1.20, 0.08}, 0.04}, false}));
 CHECK(graph.add_keyframe({3, 2.0, {{1.15, 1.18}, 1.61}, false}));
 CHECK(graph.add_keyframe({4, 3.0, {{-0.08, 1.05}, 3.19}, false}));
 CHECK(graph.add_constraint(edge(1, 1, 2, {{1.0, 0.0}, 0.0},
 PoseGraphConstraintKind::Odometry)));
 CHECK(graph.add_constraint(edge(2, 2, 3, {{0.0, 1.0}, sim::kPi / 2.0},
 PoseGraphConstraintKind::Odometry)));
 CHECK(graph.add_constraint(edge(3, 3, 4, {{0.0, 1.0}, sim::kPi / 2.0},
 PoseGraphConstraintKind::Odometry)));
 // The final edge closes the square. It is deliberately weaker and robust so
 // a bad loop association cannot arbitrarily destroy the local chain.
 auto closure = edge(4, 4, 1, {{0.0, 1.0}, sim::kPi},
 PoseGraphConstraintKind::LoopClosure);
 closure.information = pose_graph_information(0.10, 0.10);
 CHECK(graph.add_constraint(closure));
 const auto before = graph.total_error();
 CHECK(std::isfinite(before) && before > 1.0);
 const auto result = graph.optimize();
 CHECK(result.success);
 CHECK(result.final_cost < result.initial_cost);
 CHECK(result.final_cost < before);
 CHECK(result.maximum_translation_correction_m > 0.01);
 CHECK(graph.find_keyframe(1)->initial_pose.position.x == 0.0);
 CHECK(graph.find_keyframe(1)->initial_pose.position.y == 0.0);
 CHECK(graph.find_keyframe(1)->initial_pose.heading_rad == 0.0);
 const auto optimized_loop = relative_pose(graph.find_keyframe(4)->initial_pose,
 graph.find_keyframe(1)->initial_pose);
 CHECK(std::hypot(optimized_loop.position.x - 0.0,
 optimized_loop.position.y - 1.0) < 0.20);

 PoseGraphBackend invalid;
 std::string reason;
 CHECK(invalid.add_keyframe({1, 0.0, {{0.0, 0.0}, 0.0}, false}));
 CHECK(invalid.add_keyframe({2, 1.0, {{1.0, 0.0}, 0.0}, false}));
 CHECK(!invalid.add_constraint(edge(1, 1, 3, {{1.0, 0.0}, 0.0},
 PoseGraphConstraintKind::Odometry), &reason));
 CHECK(reason == "constraint references an unknown keyframe");
 CHECK(!invalid.optimize().success);
 CHECK(!invalid.add_keyframe({2, 2.0, {{0.0, 0.0}, 0.0}, false}, &reason));
 CHECK(reason == "duplicate keyframe id");
 return 0;
}
