#include <cassert>
#include <cstdio>

#include "sim/scenario/virtual_mcu.hpp"

using namespace lost_signal::robot::iface;
using lost_signal::sim::scenario::SimTimeNs;
using lost_signal::sim::scenario::VirtualMcu;
using lost_signal::sim::scenario::VirtualMcuStatus;

namespace {
MotionCapabilities caps() {
 MotionCapabilities c;
 c.control_owner = MotionControlOwner::kHostMotionLoop;
 c.chassis_mode = ActuatorMode::kDiffDuty;
 c.drive_count = 2;
 c.control_period_ns = 1'000'000;
 c.feedback_period_ns = 2'000'000;
 c.maximum_command_age_ns = 10'000'000;
 c.profile_id = "sim.diff";
 c.profile_revision = 1;
 return c;
}

MotionSetpoint duty(std::uint64_t sequence, SimTimeNs stamp,
 SimTimeNs expiry) {
 MotionSetpoint s;
 s.kind = MotionSetpointKind::kActuatorDuty;
 s.chassis_mode = ActuatorMode::kDiffDuty;
 s.sequence = sequence;
 s.stamp_ns = stamp;
 s.expiry_ns = expiry;
 s.drive_duty = {0.4, 0.4, 0.0, 0.0};
 return s;
}
} // namespace

int main() {
 constexpr SimTimeNs kMs = 1'000'000;
 VirtualMcu mcu(caps(), 7);
 assert(mcu.enqueue(7, duty(1, 1 * kMs, 9 * kMs), 4 * kMs));
 assert(mcu.tick(3 * kMs).status == VirtualMcuStatus::kIdle);
 assert(mcu.tick(4 * kMs).status == VirtualMcuStatus::kApplied);
 assert(mcu.last_accepted_sequence() == 1);
 assert(mcu.applied().drive_duty[0] == 0.4);

 assert(mcu.enqueue(7, duty(2, 2 * kMs, 3 * kMs), 5 * kMs));
 assert(mcu.tick(5 * kMs).status == VirtualMcuStatus::kRejectedExpired);
 assert(mcu.last_accepted_sequence() == 1);

 assert(mcu.enqueue(7, duty(1, 4 * kMs, 8 * kMs), 6 * kMs));
 assert(mcu.tick(6 * kMs).status == VirtualMcuStatus::kRejectedSequence);
 assert(!mcu.enqueue(8, duty(3, 6 * kMs, 9 * kMs), 6 * kMs));

 assert(mcu.tick(11 * kMs).status == VirtualMcuStatus::kWatchdogStop);
 assert(mcu.applied().is_safe_stop());
 std::puts("virtual_mcu ok");
 return 0;
}
