#include "posix_serial_port.hpp"

#include <cerrno>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

namespace lost_signal::examples::serial_sensor {
namespace {

speed_t to_speed(int baud) {
 switch (baud) {
 case 9'600: return B9600;
 case 19'200: return B19200;
 case 38'400: return B38400;
 case 57'600: return B57600;
 case 115'200: return B115200;
 case 230'400: return B230400;
 case 460'800: return B460800;
 case 921'600: return B921600;
 default: return 0;
 }
}

} // namespace

PosixSerialPort::~PosixSerialPort() { close(); }

bool PosixSerialPort::open(const std::string& path, int baud) {
 close();
 const speed_t speed = to_speed(baud);
 if (speed == 0) return false;
 const int fd = ::open(path.c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK);
 if (fd < 0) return false;
 termios settings{};
 if (tcgetattr(fd, &settings) != 0) {
 ::close(fd);
 return false;
 }
 cfmakeraw(&settings);
 cfsetispeed(&settings, speed);
 cfsetospeed(&settings, speed);
 settings.c_cflag |= CLOCAL | CREAD;
 settings.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
 settings.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
 settings.c_cc[VMIN] = 0;
 settings.c_cc[VTIME] = 0;
 if (tcsetattr(fd, TCSANOW, &settings) != 0) {
 ::close(fd);
 return false;
 }
 fd_ = fd;
 return true;
}

void PosixSerialPort::close() {
 if (fd_ >= 0) {
 ::close(fd_);
 fd_ = -1;
 }
}

int PosixSerialPort::read(std::uint8_t* destination, std::size_t capacity) {
 if (fd_ < 0 || destination == nullptr || capacity == 0) return -1;
 const ssize_t count = ::read(fd_, destination, capacity);
 if (count > 0) return static_cast<int>(count);
 if (count == 0 || errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) {
 return 0;
 }
 return -1;
}

} // namespace lost_signal::examples::serial_sensor
