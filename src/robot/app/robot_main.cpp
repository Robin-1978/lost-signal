#include "robot/runtime/robot_application.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

sim::runtime::RuntimeChannelConfig default_channel_config() {
 sim::runtime::RuntimeChannelConfig config;
 config.instance_id = "robot-process";
 config.observation_queue_capacity = 64;
 config.maximum_command_ttl_s = 0.25;
 config.capabilities.chassis.command_space = sim::ChassisCommandSpace::YawRate;
 config.capabilities.chassis.max_yaw_rate_radps = 2.0;
 config.capabilities.max_forward_speed_mps = 1.0;
 return config;
}

void print_usage(const char* executable) {
 std::cout << "Usage: " << executable << " [--check] [--list-providers]\n";
}

} // namespace

int main(int argc, char** argv) {
 bool check = false;
 bool list_providers = false;
 for (int index = 1; index < argc; ++index) {
 const std::string argument = argv[index];
 if (argument == "--check")
 check = true;
 else if (argument == "--list-providers")
 list_providers = true;
 else if (argument == "--help" || argument == "-h") {
 print_usage(argv[0]);
 return EXIT_SUCCESS;
 } else {
 std::cerr << "unknown argument: " << argument << '\n';
 print_usage(argv[0]);
 return EXIT_FAILURE;
 }
 }

 const auto registry = sim::runtime::RuntimeRegistry::with_builtins();
 if (list_providers) {
 for (const auto& provider : registry.provider_ids())
 std::cout << provider << '\n';
 if (!check)
 return EXIT_SUCCESS;
 }

 try {
 sim::robot::RobotApplicationConfig application_config;
 application_config.runtime_registry = registry;
 application_config.channel = default_channel_config();
 sim::robot::RobotApplication application(std::move(application_config));
 application.configure();
 application.activate();

 // The native process currently exposes the lifecycle and provider gate.
 // Motion execution is intentionally not fabricated: until a native provider
 // and the remaining ROBOT cycle are configured, the safe startup check is the
 // only command this executable is allowed to publish.
 if (check || list_providers) {
 std::cout << "robot process ready: provider="
 << application.channel().descriptor().id
 << " state=" << sim::runtime::lifecycle_state_name(application.state())
 << " health=" << sim::runtime::health_level_name(application.health().level)
 << '\n';
 }
 application.deactivate();
 return EXIT_SUCCESS;
 } catch (const std::exception& error) {
 std::cerr << "robot process startup failed: " << error.what() << '\n';
 return EXIT_FAILURE;
 }
}
