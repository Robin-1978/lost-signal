#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot/iface/command.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::iface {

struct ChannelConfig {
  std::size_t obs_capacity = 64;
  std::size_t cmd_capacity = 16;
};

struct ChannelHealth {
  std::uint64_t obs_received = 0;
  std::uint64_t obs_dropped = 0;    // queue overflow: oldest dropped
  std::uint64_t obs_rejected = 0;   // invalid / stale / out-of-order
  std::uint64_t cmd_published = 0;
  std::uint64_t cmd_expired = 0;  // consumed after expiry
};

// Bounded in-process transport below the ROBOT algorithm graph.
// Thread-safe for concurrent sensor producers; the tick side drains from one
// thread. Deterministic policy: overflow drops the oldest observation and
// marks health degraded. Commands carry expiry; expired commands never reach
// a provider (consumer sees safe stop instead).
class RuntimeChannel {
 public:
  explicit RuntimeChannel(ChannelConfig cfg = {});

  bool active() const { return active_; }
  void activate() { active_ = true; }
  void deactivate();

  // Provider side: publish one validated frame. Returns false when rejected.
  bool publish_observation(const ObservationFrame& obs);
  // Robot side: drain all queued observations in FIFO order.
  std::vector<ObservationFrame> drain_observations();

  // Robot side: publish the validated command for this tick.
  bool publish_command(const MotionCommand& cmd);
  // Provider side: take the newest non-expired command, or safe stop.
  MotionCommand take_command(core::MonotonicNs now_ns);

  ChannelHealth health() const;

 private:
  ChannelConfig cfg_;
  bool active_ = false;
  mutable std::mutex mutex_;
  std::vector<ObservationFrame> obs_queue_;
  std::vector<MotionCommand> cmd_queue_;
  ChannelHealth health_{};
  // Per-source sequence / timestamp gate (strictly increasing).
  std::unordered_map<std::string, std::uint64_t> last_seq_;
  std::unordered_map<std::string, core::MonotonicNs> last_stamp_;
};

}  // namespace lost_signal::robot::iface
