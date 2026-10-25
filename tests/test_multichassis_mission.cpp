#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/scenario/scenario_session.hpp"

using namespace lost_signal;

namespace {

struct RunResult {
 bool done = false;
 bool lateral_seen = false;
 bool ackermann_pivot = false;
};

RunResult run_kind(const std::string& kind) {
 const std::string dir = std::string(TEST_CONFIG_DIR) + "/robot/";
 robot::platform::RobotConfig cfg;
 std::string err;
 assert(robot::platform::load_robot_config(dir + kind + ".json", cfg, &err));
 sim::scenario::ScenarioSession session(
 robot::platform::description_from_config(cfg),
 sim::eval::motor_from_config(cfg));
 const bool holonomic = kind == "mecanum" || kind == "swerve";
 // A pose-covariance-aware robot cannot manufacture absolute confidence in
 // an empty plane. Off-lane landmarks make the navigation experiment
 // observable without constraining any chassis-specific path.
 session.setup_world().set_static_boxes({
 {-1.5, 2.5, -0.5, 3.5, 1.5},
 {2.5, -3.5, 3.5, -2.5, 1.5},
 {6.5, 2.5, 7.5, 3.5, 1.5},
 });
 session.set_initial_pose(0.0, 0.0, 0.0, 0.05, 0.02);
 session.setup_world().add_place({"goal", 6.0, 0.0, 0.0});
 session.set_objectives({"goal"}, 1.0);
 RunResult out;
 for (int i = 0; i < 1200 && !session.objectives_done(); ++i) {
 session.step_once();
 if (session.episode().steps().empty()) continue;
 const auto& cmd = session.episode().steps().back().cmd;
 out.lateral_seen |= std::abs(cmd.lateral_mps) > 0.05;
 if (kind == "ackermann" && std::abs(cmd.forward_mps) < 1e-6 &&
 std::abs(cmd.yaw_rate_radps) > 0.05) {
 out.ackermann_pivot = true;
 }
 }
 out.done = session.objectives_done();
 assert(!session.episode().steps().empty());
 if (!out.done) {
 const auto& step = session.episode().steps().back();
 std::fprintf(stderr,
 "%s final truth=(%.3f,%.3f,%.3f) est=(%.3f,%.3f,%.3f) "
 "vel=(%.3f,%.3f) sigma=(%.3f,%.3f) loc=%s reason=%s\n",
 kind.c_str(), step.truth_after.body_x_m, step.truth_after.body_y_m,
 step.truth_after.body_yaw_rad, step.diag.estimate.x_m,
 step.diag.estimate.y_m, step.diag.estimate.yaw_rad,
 step.diag.estimate.forward_mps, step.diag.estimate.yaw_rate_radps,
 step.diag.estimate.position_sigma_m, step.diag.estimate.heading_sigma_rad,
 step.diag.localization_action.c_str(),
 step.diag.evidence.reason_code.c_str());
 }
 return out;
}

} // namespace

int main() {
 const auto diff = run_kind("differential");
 const auto ack = run_kind("ackermann");
 const auto mec = run_kind("mecanum");
 const auto swerve = run_kind("swerve");
 std::printf("runs: diff=%d ack=%d mecanum=%d swerve=%d lateral=%d/%d pivot=%d\n",
 diff.done, ack.done, mec.done, swerve.done, mec.lateral_seen,
 swerve.lateral_seen, ack.ackermann_pivot);
 assert(diff.done && ack.done && mec.done && swerve.done);
 assert(!ack.ackermann_pivot);
 assert(mec.lateral_seen || swerve.lateral_seen);
 std::printf("multichassis_mission ok: diff=%d ack=%d mecanum=%d swerve=%d\n",
 diff.done, ack.done, mec.done, swerve.done);
 return 0;
}
