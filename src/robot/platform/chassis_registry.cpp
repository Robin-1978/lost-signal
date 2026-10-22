#include "robot/platform/chassis_registry.hpp"

#include <algorithm>

namespace lost_signal::robot::platform {

ChassisRegistry& ChassisRegistry::global() {
  static ChassisRegistry instance;
  return instance;
}

bool ChassisRegistry::register_factory(std::string kind, Factory factory) {
  if (kind.empty() || !factory) return false;
  return factories_.emplace(std::move(kind), std::move(factory)).second;
}

bool ChassisRegistry::contains(const std::string& kind) const {
  return factories_.find(kind) != factories_.end();
}

std::vector<std::string> ChassisRegistry::kinds() const {
  std::vector<std::string> out;
  out.reserve(factories_.size());
  for (const auto& [kind, _] : factories_) out.push_back(kind);
  std::sort(out.begin(), out.end());
  return out;
}

std::unique_ptr<IChassis> ChassisRegistry::create(
    const RobotDescription& desc) const {
  const auto it = factories_.find(desc.chassis_kind);
  if (it == factories_.end()) return nullptr;
  return it->second(desc);
}

RegisterChassis::RegisterChassis(std::string kind,
                                 ChassisRegistry::Factory factory) {
  ChassisRegistry::global().register_factory(std::move(kind),
                                             std::move(factory));
}

}  // namespace lost_signal::robot::platform
