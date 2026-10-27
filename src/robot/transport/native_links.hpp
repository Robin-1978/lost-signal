#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "robot/transport/posix_serial_port.hpp"

namespace lost_signal::robot::transport {

class UdpLink {
 public:
 UdpLink() = default;
 ~UdpLink();
 UdpLink(const UdpLink&) = delete;
 UdpLink& operator=(const UdpLink&) = delete;
 bool bind(const std::string& address, std::uint16_t port);
 void close();
 bool send_to(const std::string& address, std::uint16_t port,
 const std::uint8_t* data, std::size_t size);
 int receive(std::uint8_t* data, std::size_t capacity,
 int timeout_ms, std::string* sender = nullptr);
 std::uint16_t local_port() const { return local_port_; }

 private:
 int fd_ = -1;
 std::uint16_t local_port_ = 0;
};

struct CanChunk {
 std::uint32_t id = 0;
 std::uint8_t length = 0;
 std::array<std::uint8_t, 8> data{};
};

class CanSegmenter {
 public:
 static bool split(std::uint32_t can_id, std::uint16_t transfer_id,
 const std::uint8_t* bytes, std::size_t size,
 std::vector<CanChunk>& chunks);
 bool accept(const CanChunk& chunk, std::vector<std::uint8_t>& complete);
 void reset();

 private:
 std::uint32_t can_id_ = 0;
 std::uint16_t transfer_id_ = 0;
 std::uint8_t expected_index_ = 0;
 std::uint8_t total_ = 0;
 std::vector<std::uint8_t> pending_;
};

class SocketCanLink {
 public:
 SocketCanLink() = default;
 ~SocketCanLink();
 bool open(const std::string& interface_name, std::uint32_t can_id);
 void close();
 bool send(const CanChunk& chunk);
 bool receive(CanChunk& chunk, int timeout_ms);

 private:
 int fd_ = -1;
 std::uint32_t can_id_ = 0;
};

class SpiLink {
 public:
 SpiLink() = default;
 ~SpiLink();
 bool open(const std::string& device, std::uint32_t speed_hz,
 std::uint8_t mode = 0, std::uint8_t bits = 8);
 void close();
 bool transfer(const std::uint8_t* tx, std::uint8_t* rx, std::size_t size);

 private:
 int fd_ = -1;
};

// USB CDC-ACM uses the same byte-stream framing as UART. Keeping the wrapper
// separate makes `/dev/ttyACM*` obvious in a device profile without creating
// a second protocol or a libusb dependency.
using UsbCdcLink = serial_sensor::PosixSerialPort;

} // namespace lost_signal::robot::transport
