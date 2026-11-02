#include <cassert>
#include <string>

#include "sim/experiment/ab_experiment.hpp"
#include "sim/viewer/experience_flow.hpp"
#include "sim/viewer/experience_manifest.hpp"

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
 lost_signal::sim::viewer::ExperienceManifest experience;
 assert(lost_signal::sim::viewer::load_experience_manifest(
 std::string(TEST_SOURCE_DIR) +
 "/configs/experiences/first_contact.json", experience, &error));
 lost_signal::sim::viewer::ExperienceEvidence evidence;
 evidence.comparison_complete = true;
 evidence.comparison_valid = true;
 evidence.baseline_safe = comparison.baseline.objectives_passed &&
 comparison.baseline.score.collision_steps == 0;
 evidence.variant_safe = comparison.variant.objectives_passed &&
 comparison.variant.score.collision_steps == 0;
 evidence.elapsed_delta_s = comparison.variant.score.elapsed_s -
 comparison.baseline.score.elapsed_s;
 evidence.clearance_delta_m = comparison.variant.score.min_clearance_m -
 comparison.baseline.score.min_clearance_m;
 const auto& spec = experience.experiments.front();
 evidence.minimum_elapsed_increase_s = spec.minimum_elapsed_increase_s;
 evidence.minimum_clearance_loss_m = spec.minimum_clearance_loss_m;
 evidence.equivalent_elapsed_tolerance_s =
 spec.equivalent_elapsed_tolerance_s;
 evidence.equivalent_clearance_tolerance_m =
 spec.equivalent_clearance_tolerance_m;
 std::size_t supported = 0;
 for (const auto& choice : experience.choices) {
 supported += lost_signal::sim::viewer::choice_supported(choice, evidence);
 }
 assert(supported == 1);
 assert(!lost_signal::sim::experiment::run_scan_period_comparison(
 std::string(TEST_SOURCE_DIR) + "/configs/experiences/first_contact.json",
 0.0, 1200, comparison, &error));
 return 0;
}
