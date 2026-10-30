#include "sim/viewer/assembly_registry.hpp"

#include <algorithm>

namespace lost_signal::sim::viewer {
namespace {

const std::vector<AssemblySpec> kRegistry{
    {"wheel.standard", "wheel", "robots.ro1.wheel_standard", 0.70,
     0.010, 10.0, 72, 0.10, 0.20},
    {"wheel.grip", "wheel", "robots.ro1.wheel_grip", 1.20, 0.014,
     10.0, 72, 0.10, 0.20},
    {"wheel.eco", "wheel", "robots.ro1.wheel_eco", 0.50, 0.007,
     10.0, 72, 0.10, 0.20},
    {"lidar.standard", "lidar", "robots.ro1.lidar_standard", 0.70,
     0.010, 10.0, 72, 0.10, 0.20},
    {"lidar.fast", "lidar", "robots.ro1.lidar_fast", 0.70, 0.010,
     5.0, 48, 0.05, 0.20},
    {"lidar.long", "lidar", "robots.ro1.lidar_long", 0.70, 0.010,
     12.0, 72, 0.20, 0.20},
    {"compute.light", "compute", "robots.ro1.compute_light", 0.70,
     0.010, 10.0, 72, 0.10, 0.50},
    {"compute.performance", "compute", "robots.ro1.compute_performance",
     0.70, 0.010, 10.0, 72, 0.10, 0.10},
};

std::uint64_t fnv1a(const std::string& text, std::uint64_t hash) {
  for (const unsigned char byte : text) {
    hash ^= byte;
    hash *= 1099511628211ULL;
  }
  return hash;
}

}  // namespace

const std::vector<AssemblySpec>& assembly_registry() { return kRegistry; }

const AssemblySpec* find_assembly(const std::string& slot,
                                  const std::string& asset_id) {
  const auto it = std::find_if(
      kRegistry.begin(), kRegistry.end(), [&](const AssemblySpec& spec) {
        return spec.slot == slot && spec.asset_id == asset_id;
      });
  return it == kRegistry.end() ? nullptr : &*it;
}

bool validate_assembly_selection(const AssemblySelection& selection,
                                 std::string* error) {
  const auto fail = [&](const char* slot, const std::string& id) {
    if (error != nullptr) *error = std::string("unknown ") + slot +
        " assembly: " + id;
    return false;
  };
  if (find_assembly("wheel", selection.wheel) == nullptr) {
    return fail("wheel", selection.wheel);
  }
  if (find_assembly("lidar", selection.lidar) == nullptr) {
    return fail("lidar", selection.lidar);
  }
  if (find_assembly("compute", selection.compute) == nullptr) {
    return fail("compute", selection.compute);
  }
  if (error != nullptr) error->clear();
  return true;
}

std::uint64_t assembly_digest(const AssemblySelection& selection) {
  std::uint64_t hash = 1469598103934665603ULL;
  hash = fnv1a("lost-signal-assembly/1", hash);
  hash = fnv1a(selection.wheel, hash);
  hash = fnv1a(selection.lidar, hash);
  hash = fnv1a(selection.compute, hash);
  return hash;
}

}  // namespace lost_signal::sim::viewer
