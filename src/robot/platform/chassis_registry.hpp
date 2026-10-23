#pragma once

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot/platform/chassis.hpp"
#include "robot/platform/description.hpp"

namespace lost_signal::robot::platform {

// Plugin registry for chassis factories, keyed by RobotDescription
// chassis_kind ("differential", "ackermann", "swerve", "mecanum",
// "tracked").
// Native bring-ups register new bases without touching the factory call
// site; unknown kinds fail closed to nullptr (no silent default chassis).
class ChassisRegistry {
 public:
  using Factory =
      std::function<std::unique_ptr<IChassis>(const RobotDescription&)>;

  static ChassisRegistry& global();

  // False when kind is empty or already registered (first wins).
  bool register_factory(std::string kind, Factory factory);
  bool contains(const std::string& kind) const;
  std::vector<std::string> kinds() const;

  // nullptr when kind is unknown: fail closed, never a default chassis.
  std::unique_ptr<IChassis> create(const RobotDescription& desc) const;

 private:
  std::unordered_map<std::string, Factory> factories_;
};

// Static plugin helper: constructs at startup to register one chassis kind.
struct RegisterChassis {
  RegisterChassis(std::string kind, ChassisRegistry::Factory factory);
};

}  // namespace lost_signal::robot::platform
