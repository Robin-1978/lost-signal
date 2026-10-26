#include <cassert>
#include <string>

#include "sim/experiment/ab_experiment.hpp"

int main() {
 lost_signal::sim::experiment::ScanPeriodComparison comparison;
 std::string error;
 assert(lost_signal::sim::experiment::run_scan_period_comparison(
 std::string(TEST_SOURCE_DIR) + "/configs/experiences/first_contact.json",
 0.10, 1200, comparison, &error));
 assert(comparison.experience_id == "first_contact");
 assert(comparison.baseline.scan_period_s == 0.02);
 assert(comparison.variant.scan_period_s == 0.10);
 assert(comparison.baseline.delivery_seed == comparison.variant.delivery_seed);
 assert(comparison.baseline.imu_seed == comparison.variant.imu_seed);
 assert(comparison.baseline.actor_seed_digest ==
 comparison.variant.actor_seed_digest);
 assert(comparison.baseline.score.steps > 0);
 assert(comparison.variant.score.steps > 0);
 assert(comparison.baseline.command_digest != 0);
 assert(comparison.variant.command_digest != 0);
 assert(comparison.baseline.config_digest != 0);
 assert(comparison.variant.config_digest != 0);
 assert(comparison.baseline.config_digest != comparison.variant.config_digest);
 assert(comparison.baseline.first_scan.observed);
 assert(comparison.variant.first_scan.observed);
 assert(comparison.baseline.first_brake.observed);
 assert(comparison.variant.first_brake.observed);
 assert(comparison.baseline.stable_stop.observed);
 assert(comparison.variant.stable_stop.observed);
 assert(comparison.baseline.first_scan.step <
 comparison.baseline.first_brake.step);
 assert(comparison.baseline.first_brake.step <=
 comparison.baseline.stable_stop.step);
 assert(!lost_signal::sim::experiment::run_scan_period_comparison(
 std::string(TEST_SOURCE_DIR) + "/configs/experiences/first_contact.json",
 0.0, 1200, comparison, &error));
 return 0;
}
