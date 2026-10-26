#pragma once

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "robot/platform/robot_config.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/scenario/scenario_session.hpp"

namespace lost_signal::sim::experiment {

struct ExperienceRuntimeConfig {
 std::string robot_path;
 std::string sim_path;
 std::string map_path;
 std::vector<std::string> objectives;
 // Optional controlled experiment input. It is applied before the fresh
 // session publishes its first observation; every other authored value and
 // random seed remains unchanged.
 std::optional<double> scan_period_s_override;
};

struct ExperienceRuntime {
 std::unique_ptr<scenario::ScenarioSession> session;
 std::optional<robot::platform::RobotConfig> robot_config;
 std::optional<eval::SimConfig> sim_config;
 std::string used_robot_path;
 std::string used_sim_path;
};

// Creates one complete, fresh run from authored inputs. Retry calls this
// factory again instead of clearing a recorder: physics, sensors, ROBOT
// lifecycle, estimators, planners, actors, clocks, and random streams all
// start from the same configuration and seeds.
bool build_experience_runtime(const ExperienceRuntimeConfig& config,
 ExperienceRuntime& runtime,
 std::string* error = nullptr);

} // namespace lost_signal::sim::experiment
