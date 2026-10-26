#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include "robot/iface/command.hpp"
#include "robot/iface/motion.hpp"
#include "robot/iface/observation.hpp"

namespace lost_signal::robot::iface {

struct ChannelConfig {
  std::size_t obs_capacity = 64;
  std::size_t cmd_capacity = 16;
  std::size_t act_capacity = 16;
};

struct ChannelHealth {
  std::uint64_t obs_received = 0;
  std::uint64_t obs_dropped = 0;    // ring overwrite: oldest evicted
  std::uint64_t obs_rejected = 0;   // invalid / stale / out-of-order
  std::uint64_t cmd_published = 0;
  std::uint64_t cmd_expired = 0;  // consumed after expiry
 std::uint64_t act_published = 0;
 std::uint64_t act_rejected = 0;
  std::uint64_t act_expired = 0;
};

// Fixed-capacity ring queue: O(1) push/pop, full push overwrites the oldest.
// Not lock-free (callers hold the channel mutex); single allocation at resize.
template <typename T>
class RingQueue {
 public:
  RingQueue() = default;
  explicit RingQueue(std::size_t capacity) { resize(capacity); }

  void resize(std::size_t capacity) {
    slots_.assign(capacity, T{});
    head_ = 0;
    size_ = 0;
  }
  std::size_t capacity() const { return slots_.size(); }
  std::size_t size() const { return size_; }
  bool empty() const { return size_ == 0; }

  // Returns true when an oldest element was evicted.
  bool push(const T& v) {
    if (slots_.empty()) return true;  // zero capacity: drop everything
    if (size_ < slots_.size()) {
      slots_[(head_ + size_) % slots_.size()] = v;
      size_ += 1;
      return false;
    }
    slots_[head_] = v;
    head_ = (head_ + 1) % slots_.size();
    return true;
  }

  std::vector<T> drain() {
    std::vector<T> out;
    out.reserve(size_);
    for (std::size_t i = 0; i < size_; ++i) {
      out.push_back(slots_[(head_ + i) % slots_.size()]);
    }
    head_ = 0;
    size_ = 0;
    return out;
  }

  void clear() {
    head_ = 0;
    size_ = 0;
  }

 private:
  std::vector<T> slots_;
  std::size_t head_ = 0;  // index of oldest element
  std::size_t size_ = 0;
};

// Bounded in-process transport below the ROBOT algorithm graph.
// Thread-safe for concurrent sensor producers; the tick side drains from one
// thread. Ring policy: a full queue overwrites the oldest observation and
// marks health degraded. Commands carry expiry; expired commands never reach
// a provider (consumer sees safe stop instead).
//
// This is the transport mechanism, not a bus: no topics, no discovery, no
// cross-process routing, no serialization. A future DDS/Zenoh bridge keeps
// the same envelope + policies at the App edge.
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

  // Robot side: publish the validated actuator setpoints for this tick.
  bool publish_actuator(const ActuatorCommand& cmd);
 // Bind the watchdog fallback to the configured chassis before the first
 // command. Subsequent valid publications keep this mode current.
 void set_actuator_mode(ActuatorMode mode);
 // Plant side: take the newest non-expired setpoints, or mechanical stop.
  ActuatorCommand take_actuator(core::MonotonicNs now_ns);

  ChannelHealth health() const;

 private:
  ChannelConfig cfg_;
  bool active_ = false;
  mutable std::mutex mutex_;
  RingQueue<ObservationFrame> obs_ring_;
  RingQueue<MotionCommand> cmd_ring_;
  RingQueue<ActuatorCommand> act_ring_;
 ActuatorMode last_actuator_mode_ = ActuatorMode::kDiffDuty;
  ChannelHealth health_{};
  // Per-source sequence / timestamp gate (strictly increasing).
  std::unordered_map<std::string, std::uint64_t> last_seq_;
  std::unordered_map<std::string, core::MonotonicNs> last_stamp_;
};

}  // namespace lost_signal::robot::iface
