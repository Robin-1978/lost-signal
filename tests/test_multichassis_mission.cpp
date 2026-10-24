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
