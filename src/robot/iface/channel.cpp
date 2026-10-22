#include "robot/iface/channel.hpp"

namespace lost_signal::robot::iface {

RuntimeChannel::RuntimeChannel(ChannelConfig cfg) : cfg_(cfg) {
  obs_queue_.reserve(cfg_.obs_capacity);
  cmd_queue_.reserve(cfg_.cmd_capacity);
}

void RuntimeChannel::deactivate() {
  std::lock_guard<std::mutex> lock(mutex_);
  active_ = false;
  obs_queue_.clear();
  cmd_queue_.clear();
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
  if (obs_queue_.size() >= cfg_.obs_capacity) {
    obs_queue_.erase(obs_queue_.begin());  // drop oldest, keep newest
    health_.obs_dropped += 1;
  }
  obs_queue_.push_back(obs);
  last_seq_[obs.source] = obs.sequence;
  last_stamp_[obs.source] = obs.stamp_ns;
  health_.obs_received += 1;
  return true;
}

std::vector<ObservationFrame> RuntimeChannel::drain_observations() {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<ObservationFrame> out;
  out.swap(obs_queue_);
  return out;
}

bool RuntimeChannel::publish_command(const MotionCommand& cmd) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (!active_) return false;
  if (cmd_queue_.size() >= cfg_.cmd_capacity) {
    cmd_queue_.erase(cmd_queue_.begin());
  }
  cmd_queue_.push_back(cmd);
  health_.cmd_published += 1;
  return true;
}

MotionCommand RuntimeChannel::take_command(core::MonotonicNs now_ns) {
  std::lock_guard<std::mutex> lock(mutex_);
  if (cmd_queue_.empty()) return MotionCommand::SafeStop(now_ns);
  MotionCommand cmd = cmd_queue_.back();
  cmd_queue_.clear();
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
