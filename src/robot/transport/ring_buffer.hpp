#pragma once

#include <cstddef>
#include <cstdint>
#include <mutex>
#include <vector>

namespace lost_signal::robot::transport {

// Thread-safe byte handoff for native serial, USB, socket and SPI drivers.
// The parser peeks a complete frame and discards it only after validation.
class RingBuffer {
 public:
 explicit RingBuffer(std::size_t capacity);
 std::size_t capacity() const { return bytes_.size(); }
 std::size_t size() const;
 bool empty() const { return size() == 0; }
 bool peek(std::size_t offset, std::uint8_t& value) const;
 std::size_t peek(std::uint8_t* data, std::size_t count,
 std::size_t offset = 0) const;
 std::size_t discard(std::size_t count);
 std::size_t push(const std::uint8_t* data, std::size_t count);
 std::size_t pop(std::uint8_t* data, std::size_t count);
 std::size_t overwrite(const std::uint8_t* data, std::size_t count);
 void clear();

 private:
 std::size_t size_unlocked() const { return count_; }
 mutable std::mutex mutex_;
 std::vector<std::uint8_t> bytes_;
 std::size_t read_ = 0;
 std::size_t write_ = 0;
 std::size_t count_ = 0;
};

} // namespace lost_signal::robot::transport
