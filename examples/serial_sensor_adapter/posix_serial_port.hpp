#pragma once

#include <cstddef>
#include <cstdint>
#include <string>

namespace lost_signal::examples::serial_sensor {

// Minimal host-side serial boundary. It never parses a sensor message; the
// frame decoder owns that contract. Reads are non-blocking so NativeRuntime's
// decision loop cannot be held by a disconnected or slow sensor.
class PosixSerialPort {
 public:
 PosixSerialPort() = default;
 ~PosixSerialPort();

 PosixSerialPort(const PosixSerialPort&) = delete;
 PosixSerialPort& operator=(const PosixSerialPort&) = delete;

 bool open(const std::string& path, int baud);
 void close();
 bool is_open() const { return fd_ >= 0; }
 // Returns bytes read, 0 for no data, or -1 for a terminal I/O error.
 int read(std::uint8_t* destination, std::size_t capacity);

 private:
 int fd_ = -1;
};

} // namespace lost_signal::examples::serial_sensor
