#pragma once

#include <algorithm>
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
 explicit RingBuffer(std::size_t capacity) : bytes_(capacity) {}

 std::size_t capacity() const { return bytes_.size(); }
 std::size_t size() const {
 std::lock_guard<std::mutex> lock(mutex_);
 return size_unlocked();
 }
 bool empty() const { return size() == 0; }

 // Copy without consuming. This lets a parser inspect a complete frame and
 // discard it atomically after CRC/length validation, even if another thread
 // is concurrently appending bytes.
 bool peek(std::size_t offset, std::uint8_t& value) const {
 std::lock_guard<std::mutex> lock(mutex_);
 if (offset >= size_unlocked()) return false;
 value = bytes_[(read_ + offset) % capacity()];
 return true;
 }

 std::size_t peek(std::uint8_t* data, std::size_t count,
 std::size_t offset = 0) const {
 if (data == nullptr || count == 0) return 0;
 std::lock_guard<std::mutex> lock(mutex_);
 const std::size_t available = size_unlocked();
 if (offset >= available) return 0;
 const std::size_t copied = std::min(count, available - offset);
 for (std::size_t i = 0; i < copied; ++i) {
 data[i] = bytes_[(read_ + offset + i) % capacity()];
 }
 return copied;
 }

 std::size_t discard(std::size_t count) {
 if (bytes_.empty()) return 0;
 std::lock_guard<std::mutex> lock(mutex_);
 const std::size_t discarded = std::min(count, size_unlocked());
 read_ = (read_ + discarded) % capacity();
 count_ -= discarded;
 return discarded;
 }

 private:
 std::size_t size_unlocked() const {
 return count_;
 }

 public:
 std::size_t push(const std::uint8_t* data, std::size_t count) {
 if (data == nullptr || count == 0 || bytes_.empty()) return 0;
 std::lock_guard<std::mutex> lock(mutex_);
 const std::size_t free = capacity() - size_unlocked();
 const std::size_t copied = std::min(count, free);
 for (std::size_t i = 0; i < copied; ++i) {
 bytes_[write_] = data[i];
 write_ = (write_ + 1) % capacity();
  ++count_;
 }
 return copied;
 }

 std::size_t pop(std::uint8_t* data, std::size_t count) {
 if (data == nullptr || count == 0) return 0;
 std::lock_guard<std::mutex> lock(mutex_);
 const std::size_t copied = std::min(count, size_unlocked());
 for (std::size_t i = 0; i < copied; ++i) {
 data[i] = bytes_[read_];
 read_ = (read_ + 1) % capacity();
  --count_;
 }
 return copied;
 }

 std::size_t overwrite(const std::uint8_t* data, std::size_t count) {
 if (data == nullptr || count == 0 || bytes_.empty()) return 0;
 std::lock_guard<std::mutex> lock(mutex_);
 for (std::size_t i = 0; i < count; ++i) {
 const bool full = size_unlocked() == capacity();
 bytes_[write_] = data[i];
 write_ = (write_ + 1) % capacity();
 if (full) {
 read_ = (read_ + 1) % capacity();
 } else {
 ++count_;
 }
 }
 return count;
 }

 void clear() {
 std::lock_guard<std::mutex> lock(mutex_);
 read_ = 0;
 write_ = 0;
 count_ = 0;
 }

 private:
 mutable std::mutex mutex_;
 std::vector<std::uint8_t> bytes_;
 std::size_t read_ = 0;
 std::size_t write_ = 0;
 std::size_t count_ = 0;
};

} // namespace lost_signal::robot::iface
