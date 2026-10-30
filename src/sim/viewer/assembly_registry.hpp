#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace lost_signal::sim::viewer {

// Presentation manifests select these IDs; the typed spec below is the only
// place that maps a selectable part to physical/profile consequences.  It is
// a SIM-side registry and never exposes simulator truth to ROBOT.
struct AssemblySelection {
  std::string wheel = "robots.ro1.wheel_standard";
  std::string lidar = "robots.ro1.lidar_standard";
  std::string compute = "robots.ro1.compute_light";
};

struct AssemblySpec {
  const char* id;
  const char* slot;
  const char* asset_id;
  double tire_friction_mu;
  double rolling_resist;
  double lidar_max_range_m;
  int lidar_beams;
  double scan_period_s;
  double decision_period_s;
};

const std::vector<AssemblySpec>& assembly_registry();
const AssemblySpec* find_assembly(const std::string& slot,
                                  const std::string& asset_id);
bool validate_assembly_selection(const AssemblySelection& selection,
                                 std::string* error = nullptr);
std::uint64_t assembly_digest(const AssemblySelection& selection);

}  // namespace lost_signal::sim::viewer
