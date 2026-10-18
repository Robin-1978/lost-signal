#pragma once

#include <array>

namespace sim {

struct ManipulatorJointControllerConfig {
 std::array<double, 4> kp{};
 std::array<double, 4> kd{};
 std::array<double, 4> maximum_effort_Nm{};
 std::array<double, 4> maximum_velocity_radps{};
 // Gravity moments for the upper-arm, forearm and tool mass groups. For the
 // yaw + planar 3R chain they multiply sin(q1), sin(q1+q2) and
 // sin(q1+q2+q3), respectively. They are identified/model parameters in Nm,
 // not simulator state, so the same effort controller can run on real I/O.
 std::array<double, 3> gravity_moment_Nm{};
};

struct ManipulatorJointFeedback {
 std::array<double, 4> position_rad{};
 std::array<double, 4> velocity_radps{};
 bool valid{false};
};

struct ManipulatorJointControl {
 std::array<double, 4> target_rad{};
 std::array<double, 4> effort_Nm{};
 std::array<bool, 4> saturated{};
};

// A bounded joint-space position loop. It rate-limits the internal reference
// before applying PD effort, so a distant pose cannot request an impulse from
// either the simulator or a real actuator adapter.
class ManipulatorJointController {
 public:
 explicit ManipulatorJointController(ManipulatorJointControllerConfig config);
 void reset(const std::array<double, 4>& measured_position_rad);
 ManipulatorJointControl update(const std::array<double, 4>& requested_position_rad,
 const ManipulatorJointFeedback& feedback, double dt_s);

 private:
 ManipulatorJointControllerConfig config_;
 std::array<double, 4> reference_rad_{};
 bool initialized_{false};
};

} // namespace sim
