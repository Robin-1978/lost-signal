#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/platform/chassis.hpp"
#include "robot/platform/chassis_registry.hpp"
#include "robot/platform/drive_profile.hpp"

int main() {
  using namespace lost_signal::robot;

  platform::ChassisLimits limits{
      .max_forward_mps = 2.0,
      .max_yaw_rate_radps = 1.5,
      .footprint_radius_m = 0.4,
  };
  platform::DifferentialChassis diff(0.1, 0.5, limits);
  assert(diff.command_space() == iface::CommandSpace::kYawRate);

  // Inverse then forward round-trips.
  platform::WheelSpeeds w = diff.inverse(1.0, 0.5);
  double v = 0.0, yaw = 0.0;
  diff.forward(w, v, yaw);
  assert(std::abs(v - 1.0) < 1e-9);
  assert(std::abs(yaw - 0.5) < 1e-9);

  // Straight motion: equal wheel speeds.
  platform::WheelSpeeds s = diff.inverse(1.0, 0.0);
  assert(std::abs(s.left_radps - s.right_radps) < 1e-9);
  assert(std::abs(s.left_radps - 10.0) < 1e-9);  // v / r

  // Clamp enforces the envelope; wrong space fails closed to safe stop.
  iface::MotionCommand fast;
  fast.space = iface::CommandSpace::kYawRate;
  fast.stamp_ns = 100;
  fast.expiry_ns = 500;
  fast.forward_mps = 99.0;
  fast.yaw_rate_radps = 99.0;
  iface::MotionCommand clamped = diff.clamp_command(fast, 100);
  assert(clamped.forward_mps == 2.0);
  assert(clamped.yaw_rate_radps == 1.5);

  iface::MotionCommand stop = iface::MotionCommand::SafeStop(100);
  iface::MotionCommand kept = diff.clamp_command(stop, 100);
  assert(kept.is_safe_stop());

  // Stop with measured joints is a closed-loop hold at zero (active
  // electrical braking), not open-loop zero duty: the force plant has no
  // passive drag, so SafeZero would coast. Blind stops stay SafeZero.
  {
    iface::MotionFeedback moving;
    moving.source = "test.enc";
    moving.frame = core::Frame::kBaseLink;
    moving.stamp_ns = 100;
    moving.sequence = 1;
    moving.odometry_valid = true;
    moving.drive_count = 2;
    moving.drive_radps[0] = moving.drive_radps[1] = 5.0;  // rolling forward
    iface::MotionCommand stop_cmd = iface::MotionCommand::SafeStop(100);
    iface::ActuatorCommand brake = diff.to_actuator(stop_cmd, moving, 100);
    assert(brake.mode == iface::ActuatorMode::kDiffDuty);
    assert(!brake.is_safe_zero());  // braking duties, not coast
    assert(brake.drive[0] < 0.0 && brake.drive[1] < 0.0);  // oppose motion
    assert(iface::valid_actuator_command(brake, 100));
    // No joints: fail closed, loops never close on stale joints.
    iface::MotionFeedback blind;
    iface::ActuatorCommand zero = diff.to_actuator(stop_cmd, blind, 100);
    assert(zero.is_safe_zero());
  }

  // Drive profile: factory defaults per kind, unknown fails closed.
  {
    platform::DriveProfile prof;
    assert(platform::DriveProfile::for_chassis("differential", prof));
    assert(prof.wheel_radius_m == 0.1 && prof.track_width_m == 0.5);
    assert(prof.encoder_counts_per_revolution == 2048);
    assert(platform::DriveProfile::for_chassis("ackermann", prof));
    assert(prof.wheel_base_m == 0.62 && prof.track_width_m == 0.58);
    assert(platform::DriveProfile::for_chassis("swerve", prof));
    assert(prof.wheel_base_m == 0.56 && prof.track_width_m == 0.56);
    assert(platform::DriveProfile::for_chassis("mecanum", prof));
    assert(!platform::DriveProfile::for_chassis("nope", prof));
  }

  // Plugin registry: builtins present, unknown fails closed, first wins,
  // and a native bring-up can register a new base without touching the
  // factory call site.
  platform::ChassisRegistry& registry = platform::ChassisRegistry::global();
  assert(registry.contains("differential"));
  assert(registry.contains("ackermann"));
  assert(registry.contains("swerve"));
  assert(registry.contains("mecanum"));
  assert(!registry.contains("nope"));
  assert(!registry.register_factory("", [](const platform::RobotDescription&) {
    return nullptr;
  }));
  assert(!registry.register_factory(
      "differential", [](const platform::RobotDescription&) { return nullptr; }));
  platform::RobotDescription custom;
  custom.chassis_kind = "test-trike";
  custom.limits = limits;
  assert(registry.create(custom) == nullptr);
  assert(registry.register_factory(
      "test-trike", [](const platform::RobotDescription& desc) {
        return std::make_unique<platform::DifferentialChassis>(0.2, 0.6,
                                                              desc.limits);
      }));
  auto trike = registry.create(custom);
  assert(trike && std::string(trike->chassis_id()) == "differential");
  assert(trike->command_space() == iface::CommandSpace::kYawRate);

  std::printf("chassis ok\n");
  return 0;
}
