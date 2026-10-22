#include "robot/robots/differential_robot.hpp"

#include <utility>

namespace lost_signal::robot::robots {

DifferentialRobot::DifferentialRobot(std::unique_ptr<app::IRobot> inner,
                                     platform::DifferentialChassis chassis)
    : inner_(std::move(inner)), chassis_(chassis) {}

core::Status DifferentialRobot::configure() { return inner_->configure(); }

core::Status DifferentialRobot::activate() { return inner_->activate(); }

void DifferentialRobot::shutdown() { inner_->shutdown(); }

lifecycle::ModuleState DifferentialRobot::bringup_state() const {
  return inner_->bringup_state();
}

core::Status DifferentialRobot::submit_task(const iface::TaskRequest& req) {
  return inner_->submit_task(req);
}

core::Status DifferentialRobot::request_map(const std::string& map_id) {
  return inner_->request_map(map_id);
}

bool DifferentialRobot::register_provider(
    const iface::EndpointDescriptor& desc) {
  return inner_->register_provider(desc);
}

bool DifferentialRobot::publish_observation(
    const iface::ObservationFrame& obs) {
  return inner_->publish_observation(obs);
}

iface::MotionCommand DifferentialRobot::take_command(
    core::MonotonicNs now_ns) {
  return inner_->take_command(now_ns);
}

iface::MotionCommand DifferentialRobot::tick(core::MonotonicNs now_ns) {
  iface::MotionCommand cmd = inner_->tick(now_ns);
  return chassis_.clamp_command(cmd, now_ns);
}

iface::TaskStatus DifferentialRobot::task_status() const {
  return inner_->task_status();
}

iface::MapSnapshot DifferentialRobot::active_map() const {
  return inner_->active_map();
}

iface::ChannelHealth DifferentialRobot::channel_health() const {
  return inner_->channel_health();
}

}  // namespace lost_signal::robot::robots
