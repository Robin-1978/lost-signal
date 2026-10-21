#include "robot/runtime/robot_application.hpp"

#include <iostream>
#include <utility>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 sim::robot::RobotApplicationConfig config;
 config.channel.instance_id = "test-robot-process";
 config.channel.capabilities.chassis.command_space = sim::ChassisCommandSpace::YawRate;
 config.channel.capabilities.chassis.max_yaw_rate_radps = 2.0;
 config.channel.capabilities.max_forward_speed_mps = 1.0;
 sim::robot::RobotApplication application(std::move(config));

 CHECK(application.state() == sim::runtime::LifecycleState::Unconfigured);
 application.configure();
 CHECK(application.state() == sim::runtime::LifecycleState::Inactive);
 application.activate();
 CHECK(application.state() == sim::runtime::LifecycleState::Active);
 CHECK(application.health().level == sim::runtime::HealthLevel::Healthy);
 application.deactivate();
 CHECK(application.state() == sim::runtime::LifecycleState::Inactive);
 return 0;
}
