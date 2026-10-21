#include "robot/autonomy/mapping/mapping_session.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

sim::Scan scan(double stamp_s, std::uint64_t sequence) {
 sim::Scan value;
 value.n_rays = 4;
 value.max_range = 5.0;
 value.ranges = {1.0, 2.0, 3.0, 5.0};
 value.hit = {1, 1, 1, 0};
 value.intensities = {1.0, 1.0, 1.0, 0.0};
 value.header = {stamp_s, sequence, "lidar_link", 0.0,
 sim::kMeasurementSchemaVersion, "test.mapping-session"};
 return value;
}

} // namespace

int main() {
 sim::autonomy::MappingSessionConfig config;
 config.keyframes.translation_threshold_m = 0.5;
 config.keyframes.heading_threshold_rad = 0.3;
 config.keyframes.maximum_interval_s = 2.0;
 config.keyframes.maximum_keyframes = 4;
 config.loop_closure.minimum_temporal_separation_s = 2.0;
 sim::autonomy::MappingSession session(config);

 auto first = session.ingest({{0.0, 0.0}, 0.0}, scan(1.0, 1), 1.0);
 CHECK(first.accepted && first.created_keyframe && first.keyframe_id == 1);
 auto ordinary = session.ingest({{0.1, 0.0}, 0.0}, scan(1.1, 2), 1.1);
 CHECK(ordinary.accepted && !ordinary.created_keyframe);
 auto moved = session.ingest({{0.6, 0.0}, 0.0}, scan(1.2, 3), 1.2);
 CHECK(moved.accepted && moved.created_keyframe && moved.keyframe_id == 2);
 auto turned = session.ingest({{0.6, 0.0}, 0.4}, scan(1.3, 4), 1.3);
 CHECK(turned.accepted && turned.created_keyframe && turned.keyframe_id == 3);
 auto timed = session.ingest({{0.61, 0.0}, 0.4}, scan(3.4, 5), 3.4);
 CHECK(timed.accepted && timed.created_keyframe && timed.keyframe_id == 4);
 CHECK(session.keyframes().size() == 4);
 CHECK(session.graph().keyframe_count() == 4);
 CHECK(session.graph().constraint_count() == 3);
 CHECK(session.graph().find_keyframe(1)->fixed);
 CHECK(!session.graph().find_keyframe(2)->fixed);

 sim::autonomy::LoopClosureEvidence closure;
 closure.match_score = 0.92;
 closure.overlap = 0.88;
 closure.score_margin = 0.20;
 closure.measured_relative_pose =
 sim::autonomy::relative_pose(session.graph().find_keyframe(4)->initial_pose,
 session.graph().find_keyframe(1)->initial_pose);
 closure.information = sim::autonomy::pose_graph_information(0.05, 0.05);
 auto accepted_closure = session.add_loop_closure(4, 1, closure);
 CHECK(accepted_closure.accepted);
 CHECK(session.graph().constraint_count() == 4);
 closure.score_margin = 0.01;
 auto ambiguous_closure = session.add_loop_closure(4, 1, closure);
 CHECK(!ambiguous_closure.accepted);
 CHECK(ambiguous_closure.diagnostic == "loop closure match is ambiguous");

 auto over_budget = session.ingest({{1.2, 0.0}, 0.4}, scan(4.0, 6), 4.0);
 CHECK(!over_budget.accepted);
 CHECK(over_budget.diagnostic == "mapping session keyframe budget exhausted");
 auto stale = session.ingest({{1.3, 0.0}, 0.4}, scan(3.3, 7), 3.3);
 CHECK(!stale.accepted);
 CHECK(stale.diagnostic == "keyframe timestamp is not increasing");

 session.reset();
 auto malformed = session.ingest({{0.0, 0.0}, 0.0}, scan(5.0, 8), 6.0);
 CHECK(!malformed.accepted);
 CHECK(session.keyframes().empty());
 return 0;
}
