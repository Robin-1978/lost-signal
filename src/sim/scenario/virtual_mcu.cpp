#include "sim/scenario/virtual_mcu.hpp"

#include <algorithm>
#include <stdexcept>
#include <utility>

namespace lost_signal::sim::scenario {

VirtualMcu::VirtualMcu(robot::iface::MotionCapabilities capabilities,
 std::uint64_t session_id)
 : capabilities_(std::move(capabilities)), session_id_(session_id),
 applied_(stop_setpoint(0)) {
 if (!robot::iface::valid_motion_capabilities(capabilities_) ||
 session_id_ == 0) {
 throw std::invalid_argument("invalid virtual MCU capability contract");
 }
 control_deadline_.reset_ns(capabilities_.control_period_ns,
 capabilities_.control_period_ns);
}

robot::iface::MotionSetpoint VirtualMcu::stop_setpoint(SimTimeNs now_ns) const {
 robot::iface::MotionSetpoint stop;
 stop.kind = robot::iface::MotionSetpointKind::kStop;
 stop.chassis_mode = capabilities_.chassis_mode;
 stop.sequence = last_sequence_;
 stop.stamp_ns = now_ns;
 stop.expiry_ns = now_ns + capabilities_.maximum_command_age_ns;
 stop.frame = robot::core::Frame::kBaseLink;
 stop.service_brake = 1.0;
 return stop;
}

bool VirtualMcu::enqueue(std::uint64_t session_id,
 robot::iface::MotionSetpoint setpoint, SimTimeNs delivery_ns) {
 if (session_id != session_id_ || delivery_ns < 0 ||
 delivery_ns < setpoint.stamp_ns ||
 !robot::iface::valid_motion_capabilities(capabilities_)) {
 return false;
 }
 pending_.push_back({insertion_order_++, delivery_ns, session_id,
 std::move(setpoint)});
 return true;
}

VirtualMcuEvent VirtualMcu::tick(SimTimeNs now_ns) {
 VirtualMcuEvent event;
 event.stamp_ns = now_ns;
 if (now_ns < 0 || !control_due_ns(now_ns)) return event;
 std::stable_sort(pending_.begin(), pending_.end(),
 [](const Pending& lhs, const Pending& rhs) {
 if (lhs.delivery_ns != rhs.delivery_ns) {
 return lhs.delivery_ns < rhs.delivery_ns;
 }
 return lhs.insertion_order < rhs.insertion_order;
 });
 while (!pending_.empty() && pending_.front().delivery_ns <= now_ns) {
 Pending item = std::move(pending_.front());
 pending_.erase(pending_.begin());
 event.sequence = item.setpoint.sequence;
 if (item.session_id != session_id_) {
 event.status = VirtualMcuStatus::kRejectedSession;
 continue;
 }
 if (!robot::iface::valid_motion_setpoint(item.setpoint, capabilities_,
 now_ns)) {
 event.status = item.setpoint.expiry_ns <= item.delivery_ns
 ? VirtualMcuStatus::kRejectedExpired
 : VirtualMcuStatus::kRejectedInvalid;
 continue;
 }
 if (item.setpoint.sequence <= last_sequence_) {
 event.status = VirtualMcuStatus::kRejectedSequence;
 continue;
 }
 last_sequence_ = item.setpoint.sequence;
 applied_ = std::move(item.setpoint);
 event.status = VirtualMcuStatus::kApplied;
 }
 if (applied_.expiry_ns <= now_ns) {
 applied_ = stop_setpoint(now_ns);
 event.status = VirtualMcuStatus::kWatchdogStop;
 event.sequence = last_sequence_;
 }
 return event;
}

} // namespace lost_signal::sim::scenario
