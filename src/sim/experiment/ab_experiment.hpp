#pragma once

#include <cstdint>
#include <string>

#include "sim/eval/scoring.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::experiment {

struct VariantResult {
 double scan_period_s = 0.0;
 std::uint64_t delivery_seed = 0;
 std::uint64_t imu_seed = 0;
 std::uint64_t actor_seed_digest = 0;
 std::uint64_t command_digest = 0;
 bool objectives_done = false;
 bool objectives_passed = false;
 eval::EpisodeScore score;
 world::WorldTruth final_truth;
};

struct ScanPeriodComparison {
 std::string experience_id;
 VariantResult baseline;
 VariantResult variant;
};

// Runs the same experience twice. Every authored file and seed is reloaded
// unchanged; only the variant's physical scan cadence is replaced. Both runs
// use the ordinary RobotApplication sensor/actuator boundary.
bool run_scan_period_comparison(const std::string& experience_path,
 double variant_scan_period_s, long step_budget,
 ScanPeriodComparison& comparison, std::string* error = nullptr);

bool write_scan_period_comparison(const std::string& path,
 const ScanPeriodComparison& comparison, std::string* error = nullptr);

} // namespace lost_signal::sim::experiment
