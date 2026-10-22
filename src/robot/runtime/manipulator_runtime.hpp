#pragma once

#include "robot/control/manipulator_joint_controller.hpp"
#include "robot/platform/serial_manipulator.hpp"
#include "robot/runtime/runtime_channel.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace sim::robot {

// ROBOT-side manipulator composition. The provider only publishes joint
// feedback and consumes the typed actuator command; IK, reference limiting
// and joint effort control remain on this side for both SIM and native use.
struct ManipulatorRuntimeConfig {
 ManipulatorGeometry geometry;
 ManipulatorJointPosition initial_joints;
 ManipulatorJointControllerConfig controller;
 std::vector<std::string> channels;
 std::string frame_id{"base_link"};
 std::string source_id{"robot.manipulator"};
};

struct ManipulatorCycleInput {
 EndEffectorPose target_pose;
 double dt_s{0.0};
 runtime::TimePoint now;
 runtime::TimePoint valid_until;
 std::uint64_t sequence{0};
};

struct ManipulatorCycleOutput {
 ManipulatorJointControl control;
 runtime::ActuatorCommand command;
 EndEffectorPose target_pose;
 bool target_reachable{false};
};

class ManipulatorRuntime {
 public:
 ManipulatorRuntime(runtime::IRuntimeChannel& channel, ManipulatorRuntimeConfig config);

 ManipulatorCycleOutput tick(const ManipulatorCycleInput& input);
 void reset();

 const ManipulatorJointPosition& target_joints() const { return target_joints_; }
 const ManipulatorRuntimeConfig& config() const { return config_; }

 private:
 runtime::IRuntimeChannel& channel_;
 ManipulatorRuntimeConfig config_;
 ManipulatorJointController controller_;
 ManipulatorJointPosition target_joints_;
 bool initialized_{false};
};

} // namespace sim::robot
