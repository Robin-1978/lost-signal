#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>

#include "sim/experiment/ab_experiment.hpp"

int main(int argc, char** argv) {
 std::string experience_path;
 std::string output_path;
 double scan_period_s = 0.0;
 long step_budget = 4000;
 for (int i = 1; i < argc; ++i) {
 const std::string argument = argv[i];
 if (argument == "--experience" && i + 1 < argc) {
 experience_path = argv[++i];
 } else if (argument == "--scan-period" && i + 1 < argc) {
 char* end = nullptr;
 scan_period_s = std::strtod(argv[++i], &end);
 if (end == argv[i] || *end != '\0') scan_period_s = 0.0;
 } else if (argument == "--steps" && i + 1 < argc) {
 char* end = nullptr;
 step_budget = std::strtol(argv[++i], &end, 10);
 if (end == argv[i] || *end != '\0') step_budget = 0;
 } else if (argument == "--out" && i + 1 < argc) {
 output_path = argv[++i];
 } else {
 std::fprintf(stderr,
 "usage: sim_ab_experiment --experience FILE --scan-period S "
 "[--steps N] [--out FILE]\n");
 return 2;
 }
 }
 if (experience_path.empty() || output_path.empty() ||
 !std::isfinite(scan_period_s) || !(scan_period_s > 0.0) ||
 step_budget <= 0) {
 std::fprintf(stderr,
 "usage: sim_ab_experiment --experience FILE --scan-period S "
 "[--steps N] --out FILE\n");
 return 2;
 }
 lost_signal::sim::experiment::ScanPeriodComparison comparison;
 std::string error;
 if (!lost_signal::sim::experiment::run_scan_period_comparison(
 experience_path, scan_period_s, step_budget, comparison, &error) ||
 !lost_signal::sim::experiment::write_scan_period_comparison(
 output_path, comparison, &error)) {
 std::fprintf(stderr, "A/B experiment failed: %s\n", error.c_str());
 return 1;
 }
 std::printf(
 "A/B %s scan %.3fs -> %.3fs baseline=%s variant=%s collisions=%zu/%zu\n",
 comparison.experience_id.c_str(), comparison.baseline.scan_period_s,
 comparison.variant.scan_period_s,
 comparison.baseline.objectives_passed ? "passed" : "failed",
 comparison.variant.objectives_passed ? "passed" : "failed",
 comparison.baseline.score.collision_steps,
 comparison.variant.score.collision_steps);
 return 0;
}
