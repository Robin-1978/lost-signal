#include "robot/autonomy/control/controlled_stop_latch.hpp"
#include "robot/config/motion_policy.hpp"

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

namespace {

int failures = 0;

void check(bool condition, const std::string& message) {
 if (!condition) {
 std::fprintf(stderr, "FAIL: %s\n", message.c_str());
 ++failures;
 }
}

std::filesystem::path repo_root() {
 // The test binary runs from the build directory during CTest and from the
 // repository root in local sweeps; resolve both against the policy file.
 std::filesystem::path current = std::filesystem::current_path();
 for (int depth = 0; depth < 4; ++depth) {
 const auto candidate = current / "robot_data" / "policy" / "motion_policy.json";
 if (std::filesystem::exists(candidate)) {
 return current;
 }
 if (!current.has_parent_path()) {
 break;
 }
 current = current.parent_path();
 }
 return {};
}

void test_deployed_policy_matches_defaults(const std::filesystem::path& root) {
 if (root.empty()) {
 check(false, "robot_data/policy/motion_policy.json not found from the test cwd");
 return;
 }
 const sim::MotionPolicy deployed = sim::MotionPolicy::load(
 root / "robot_data" / "policy" / "motion_policy.json");
 const sim::MotionPolicy defaults = sim::MotionPolicy::defaults();
 const auto& a = deployed.execution;
 const auto& b = defaults.execution;
 check(a.gravity_mps2 == b.gravity_mps2, "gravity default drifted from the deployed file");
 check(a.physical_standoff_reserve_m == b.physical_standoff_reserve_m,
 "physical stand-off default drifted from the deployed file");
 check(a.prediction_step_s == b.prediction_step_s,
 "prediction step default drifted from the deployed file");
 check(a.dynamic_rollover_risk_limit == b.dynamic_rollover_risk_limit,
 "rollover risk limit drifted from the deployed file");
 check(a.planning_rest_deadband_mps == b.planning_rest_deadband_mps,
 "rest deadband drifted from the deployed file");
 check(a.planning_pose_velocity_deadband_mps == b.planning_pose_velocity_deadband_mps,
 "pose velocity deadband drifted from the deployed file");
 check(a.committed_progress_floor_mps == b.committed_progress_floor_mps,
 "committed progress floor drifted from the deployed file");
 check(a.controlled_stop_release_mps == b.controlled_stop_release_mps,
 "controlled stop release speed drifted from the deployed file");
 check(a.creep_speed_mps == b.creep_speed_mps,
 "creep speed drifted from the deployed file");
 check(a.controlled_stop_settle_cycles == b.controlled_stop_settle_cycles,
 "controlled stop settle cycles drifted from the deployed file");
 check(a.visible_dwa_candidates == b.visible_dwa_candidates,
 "visible DWA candidate count drifted from the deployed file");
}

void test_unknown_execution_key_is_rejected(const std::filesystem::path& root) {
 if (root.empty()) {
 return;
 }
 std::ifstream input(root / "robot_data" / "policy" / "motion_policy.json");
 const std::string text((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
 const auto marker = std::string("\"creep_speed_mps\": 0.15");
 const auto position = text.find(marker);
 check(position != std::string::npos, "creep_speed_mps missing from the deployed file");
 if (position == std::string::npos) {
 return;
 }
 const std::string mutated = text.substr(0, position + marker.size()) +
 ",\n \"creepSpeedMps\": 0.15" + text.substr(position + marker.size());
 const auto path = std::filesystem::temp_directory_path() / "lost_signal_motion_policy_typo.json";
 {
 std::ofstream output(path);
 output << mutated;
 }
 bool rejected = false;
 try {
 (void)sim::MotionPolicy::load(path);
 } catch (const std::runtime_error&) {
 rejected = true;
 }
 std::filesystem::remove(path);
 check(rejected, "a misspelled execution key must fail bring-up");
}

void test_latch_requires_sustained_settling() {
 sim::ControlledStopLatch latch;
 const sim::ControlledStopLatch::Config config{0.04, 3};
 // A stop that is not carrying authority never holds motion.
 check(!latch.update(false, 0.0, config), "idle latch must not withhold motion");
 check(latch.settle_cycles() == 0, "idle latch must not accumulate settle evidence");
 // First settled sample after a stop is not enough to release authority.
 check(latch.update(true, 0.01, config), "latch released on the first settled sample");
 check(latch.update(true, 0.01, config), "latch released on the second settled sample");
 check(!latch.update(true, 0.01, config), "latch stayed closed after the settle window");
 // Motion during the window re-arms the full settle requirement.
 latch.reset();
 check(latch.update(true, 0.01, config), "latch released too early after reset");
 check(latch.update(true, 0.20, config), "latch released while wheel motion continued");
 check(latch.settle_cycles() == 0, "moving wheels must clear settle evidence");
 check(latch.update(true, 0.01, config), "latch released on the first sample after motion");
 // Feedback loss (non-finite) must never release the latch.
 check(latch.update(true, -1.0, config), "invalid feedback released the latch");
}

} // namespace

int main() {
 const auto root = repo_root();
 test_deployed_policy_matches_defaults(root);
 test_unknown_execution_key_is_rejected(root);
 test_latch_requires_sustained_settling();
 if (failures != 0) {
 std::fprintf(stderr, "%d motion policy checks failed\n", failures);
 return 1;
 }
 return 0;
}
