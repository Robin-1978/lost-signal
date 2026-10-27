#include "robot/iface/ring_buffer.hpp"

#include <algorithm>

namespace lost_signal::robot::iface {

RingBuffer::RingBuffer(std::size_t capacity) : bytes_(capacity) {}

std::size_t RingBuffer::size() const {
 std::lock_guard<std::mutex> lock(mutex_);
 return size_unlocked();
}

bool RingBuffer::peek(std::size_t offset, std::uint8_t& value) const {
 std::lock_guard<std::mutex> lock(mutex_);
 if (offset >= size_unlocked()) return false;
 value = bytes_[(read_ + offset) % capacity()];
 return true;
}

std::size_t RingBuffer::peek(std::uint8_t* data, std::size_t count,
 std::size_t offset) const {
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

std::size_t RingBuffer::discard(std::size_t count) {
 if (bytes_.empty()) return 0;
 std::lock_guard<std::mutex> lock(mutex_);
 const std::size_t discarded = std::min(count, size_unlocked());
 read_ = (read_ + discarded) % capacity();
 count_ -= discarded;
 return discarded;
}

std::size_t RingBuffer::push(const std::uint8_t* data, std::size_t count) {
 if (data == nullptr || count == 0 || bytes_.empty()) return 0;
 std::lock_guard<std::mutex> lock(mutex_);
 const std::size_t copied = std::min(count, capacity() - size_unlocked());
 for (std::size_t i = 0; i < copied; ++i) {
 bytes_[write_] = data[i];
 write_ = (write_ + 1) % capacity();
 ++count_;
 }
 return copied;
}

std::size_t RingBuffer::pop(std::uint8_t* data, std::size_t count) {
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

std::size_t RingBuffer::overwrite(const std::uint8_t* data, std::size_t count) {
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

void RingBuffer::clear() {
 std::lock_guard<std::mutex> lock(mutex_);
 read_ = 0;
 write_ = 0;
 count_ = 0;
}

} // namespace lost_signal::robot::iface
