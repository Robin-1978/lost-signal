#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace lost_signal::robot::iface {

// Byte ring used at native serial and socket boundaries. It stores bytes only;
// framing and validation belong to the device protocol. The owner must keep
// one producer and one consumer. push/pop are index updates around a fixed
// allocation, while overwrite() is the opt-in policy for diagnostic streams
// that prefer the newest bytes over backpressure.
class RingBuffer {
 public:
 explicit RingBuffer(std::size_t capacity);

 std::size_t capacity() const { return bytes_.size(); }
 std::size_t size() const;
 bool empty() const { return size() == 0; }

 // Copy without consuming. This lets a parser inspect a complete frame and
 // discard it atomically after CRC/length validation, even if another thread
 // is concurrently appending bytes.
 bool peek(std::size_t offset, std::uint8_t& value) const;

 std::size_t peek(std::uint8_t* data, std::size_t count,
  std::size_t offset = 0) const;

 std::size_t discard(std::size_t count);

 private:
 std::size_t size_unlocked() const { return count_; }

 public:
  std::size_t push(const std::uint8_t* data, std::size_t count);

  std::size_t pop(std::uint8_t* data, std::size_t count);

  std::size_t overwrite(const std::uint8_t* data, std::size_t count);

  void clear();

 private:
 mutable std::mutex mutex_;
 std::vector<std::uint8_t> bytes_;
 std::size_t read_ = 0;
 std::size_t write_ = 0;
 std::size_t count_ = 0;
};

} // namespace lost_signal::robot::iface
