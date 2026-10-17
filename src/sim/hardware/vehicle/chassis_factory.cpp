#include "sim/hardware/vehicle/chassis_factory.hpp"

#include <stdexcept>

namespace sim {

std::map<std::string, ChassisCreator>& ChassisFactory::registry() {
 static std::map<std::string, ChassisCreator> instance;
 return instance;
}

void ChassisFactory::register_creator(const std::string& drive_type, ChassisCreator creator) {
 registry()[drive_type] = std::move(creator);
}

BuiltChassis ChassisFactory::create(const std::string& drive_type, const ChassisBuildParams& params,
                                    const WheelPidGains& gains) {
 const auto found = registry().find(drive_type);
 if (found == registry().end()) {
  throw std::runtime_error("unsupported drive type '" + drive_type + "'");
 }
 return found->second(params, gains);
}

void ChassisFactory::register_builtin_chassis() {
 register_creator("differential", create_differential_chassis);
 register_creator("ackermann", create_ackermann_chassis);
 register_creator("tracked", create_tracked_chassis);
 register_creator("swerve", create_swerve_chassis);
 register_creator("mecanum", create_mecanum_chassis);
}

} // namespace sim
