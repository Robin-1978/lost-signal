#pragma once

#include <cstdint>

namespace lost_signal::stm32_reference {

enum class SetpointKind : std::uint8_t {
 kDuty = 0,
 kJointVelocity = 1,
 kBodyTwist = 2,
 kStop = 3,
};

enum class ControlOwner : std::uint8_t {
 kHostMotionLoop = 0,
 kMcuJointLoop = 1,
 kMcuBodyLoop = 2,
};

// Fixed-width wire DTO. The host transport serializes ChassisSetpoint into
// this shape; firmware never receives a C++ object with ownership or strings.
struct WireSetpoint {
 SetpointKind kind = SetpointKind::kStop;
 std::uint8_t reserved[3] = {};
 std::uint64_t sequence = 0;
 std::int64_t stamp_ns = -1;
 std::int64_t expiry_ns = -1;
 float drive_duty[4] = {};
 float steer_duty[4] = {};
 float drive_radps[4] = {};
 float steer_rad[4] = {};
 float forward_mps = 0.0f;
 float lateral_mps = 0.0f;
 float yaw_rate_radps = 0.0f;
 float service_brake = 1.0f;
};

struct EncoderFeedback {
 bool valid = false;
 float drive_radps[4] = {};
 float steer_rad[4] = {};
};

struct MotorOutput {
 float drive_duty[4] = {};
 float steer_duty[4] = {};
 float service_brake = 1.0f;
 bool watchdog_stop = true;
};

struct PidConfig {
 float kp = 0.6f;
 float ki = 2.0f;
 float kd = 0.0f;
 float output_limit = 1.0f;
 float integral_limit = 0.5f;
};

struct ChassisConfig {
 ControlOwner owner = ControlOwner::kMcuBodyLoop;
 std::uint8_t drive_count = 2;
 std::uint8_t steer_count = 0;
 float wheel_radius_m = 0.1f;
 float track_width_m = 0.5f;
 float wheel_base_m = 0.56f;
 float max_wheel_radps = 30.0f;
 std::int64_t control_period_ns = 1'000'000;
 std::int64_t max_command_age_ns = 200'000'000;
 PidConfig drive_pid;
 PidConfig steer_pid;
};

enum class Status : std::uint8_t {
 kIdle = 0,
 kApplied = 1,
 kRejected = 2,
 kSequence = 3,
 kWatchdogStop = 4,
};

class ChassisController {
 public:
 explicit ChassisController(ChassisConfig config = {});
 Status accept(const WireSetpoint& setpoint, std::int64_t now_ns);
 Status tick(std::int64_t now_ns, const EncoderFeedback& feedback,
 MotorOutput& output);
 void reset();

 private:
 struct Pid {
 explicit Pid(PidConfig config = {}) : config(config) {}
 void reset();
 float update(float error, float dt_s);
 PidConfig config;
 float integral = 0.0f;
 float previous_error = 0.0f;
 bool first = true;
 };

 static float clamp(float value, float lower, float upper);
 void stop(MotorOutput& output) const;
 bool finite_setpoint(const WireSetpoint& setpoint) const;

 ChassisConfig config_;
 Pid drive_pid_[4];
 Pid steer_pid_[4];
 WireSetpoint setpoint_{};
 std::uint64_t last_sequence_ = 0;
 bool have_setpoint_ = false;
};

} // namespace lost_signal::stm32_reference
