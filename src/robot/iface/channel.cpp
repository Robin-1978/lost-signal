#include "robot/iface/channel.hpp"

namespace lost_signal::robot::iface {

RuntimeChannel::RuntimeChannel(ChannelConfig cfg)
    : cfg_(cfg), obs_ring_(cfg_.obs_capacity), cmd_ring_(cfg_.cmd_capacity) {}

void RuntimeChannel::deactivate() {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  obs_ring_.clear();
  cmd_ring_.clear();
}

bool RuntimeChannel::publish_observation(const ObservationFrame& obs) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) {
    health_.obs_rejected += 1;
    return false;
  }
  if (!valid_observation_frame(obs)) {
    health_.obs_rejected += 1;
    return false;
  }
  auto seq_it = last_seq_.find(obs.source);
  if (seq_it != last_seq_.end() && obs.sequence <= seq_it->second) {
    health_.obs_rejected += 1;  // stale or repeated
    return false;
  }
  auto stamp_it = last_stamp_.find(obs.source);
  if (stamp_it != last_stamp_.end() && obs.stamp_ns <= stamp_it->second) {
    health_.obs_rejected += 1;
    return false;
  }
  if (obs_ring_.push(obs)) health_.obs_dropped += 1;  // ring overwrite
  last_seq_[obs.source] = obs.sequence;
  last_stamp_[obs.source] = obs.stamp_ns;
  health_.obs_received += 1;
  return true;
}

std::vector<ObservationFrame> RuntimeChannel::drain_observations() {
  std::lock_guard<std::mutex> lock(mutex_);
  return obs_ring_.drain();
}

bool RuntimeChannel::publish_command(const MotionCommand& cmd) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) return false;
  cmd_ring_.push(cmd);  // newest wins; oldest overwritten (no counter)
  health_.cmd_published += 1;
  return true;
}

MotionCommand RuntimeChannel::take_command(core::MonotonicNs now_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  auto cmds = cmd_ring_.drain();
  if (cmds.empty()) return MotionCommand::SafeStop(now_ns);
  MotionCommand cmd = cmds.back();
  if (!valid_motion_command(cmd, now_ns)) {
    health_.cmd_expired += 1;
    return MotionCommand::SafeStop(now_ns);
  }
  return cmd;
}

ChannelHealth RuntimeChannel::health() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return health_;
}

}  // namespace lost_signal::robot::iface
