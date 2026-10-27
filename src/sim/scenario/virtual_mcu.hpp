#pragma once

#include <cstdint>
#include <vector>

#include "robot/iface/motion.hpp"
#include "sim/scenario/scheduler.hpp"

namespace lost_signal::sim::scenario {

// SIM-only board transport. It models a fixed-period MCU acceptance boundary
// with sim-time delivery; it is deliberately not an IProvider and never sees
// WorldTruth, actor identity, score, or rendered state.
enum class VirtualMcuStatus {
 kIdle = 0,
 kApplied = 1,
 kRejectedSession = 2,
 kRejectedInvalid = 3,
 kRejectedExpired = 4,
 kRejectedSequence = 5,
 kWatchdogStop = 6,
};

struct VirtualMcuEvent {
 VirtualMcuStatus status = VirtualMcuStatus::kIdle;
 std::uint64_t sequence = 0;
 SimTimeNs stamp_ns = 0;
};

class VirtualMcu {
 public:
 VirtualMcu(robot::iface::MotionCapabilities capabilities,
 std::uint64_t session_id);

 bool enqueue(std::uint64_t session_id,
 robot::iface::MotionSetpoint setpoint, SimTimeNs delivery_ns);
 VirtualMcuEvent tick(SimTimeNs now_ns);
 bool control_due_ns(SimTimeNs now_ns) {
 return control_deadline_.due_ns(now_ns);
 }
 SimTimeNs next_control_ns() const { return control_deadline_.next_ns(); }

 const robot::iface::MotionCapabilities& capabilities() const {
 return capabilities_;
 }
 std::uint64_t session_id() const { return session_id_; }
 const robot::iface::MotionSetpoint& applied() const { return applied_; }
 std::uint64_t last_accepted_sequence() const { return last_sequence_; }

 private:
 struct Pending {
 std::uint64_t insertion_order = 0;
 SimTimeNs delivery_ns = 0;
 std::uint64_t session_id = 0;
 robot::iface::MotionSetpoint setpoint;
 };

 robot::iface::MotionSetpoint stop_setpoint(SimTimeNs now_ns) const;
 robot::iface::MotionCapabilities capabilities_;
 std::uint64_t session_id_ = 0;
 std::uint64_t insertion_order_ = 0;
 std::uint64_t last_sequence_ = 0;
 robot::iface::MotionSetpoint applied_;
 PeriodicDeadline control_deadline_;
 std::vector<Pending> pending_;
};

} // namespace lost_signal::sim::scenario
