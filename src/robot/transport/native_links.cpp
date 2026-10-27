#include "robot/transport/native_links.hpp"

#include <algorithm>
#include <cerrno>
#include <cstring>

#if defined(__linux__)
#include <arpa/inet.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/spi/spidev.h>
#include <unistd.h>
#endif

namespace lost_signal::robot::transport {

#if defined(__linux__)
namespace {
bool address_of(const std::string& text, std::uint16_t port, sockaddr_in& out) {
 out = {};
 out.sin_family = AF_INET;
 out.sin_port = htons(port);
 return inet_pton(AF_INET, text.c_str(), &out.sin_addr) == 1;
}
}
#endif

UdpLink::~UdpLink() { close(); }

bool UdpLink::bind(const std::string& address, std::uint16_t port) {
 close();
#if defined(__linux__)
 sockaddr_in endpoint{};
 if (!address_of(address, port, endpoint)) return false;
 fd_ = ::socket(AF_INET, SOCK_DGRAM | SOCK_CLOEXEC, 0);
 if (fd_ < 0 || ::bind(fd_, reinterpret_cast<sockaddr*>(&endpoint),
 sizeof(endpoint)) != 0) {
 close();
 return false;
 }
 sockaddr_in actual{};
 socklen_t length = sizeof(actual);
 if (::getsockname(fd_, reinterpret_cast<sockaddr*>(&actual), &length) != 0) {
 close();
 return false;
 }
 local_port_ = ntohs(actual.sin_port);
 return true;
#else
 (void)address;
 (void)port;
 return false;
#endif
}

void UdpLink::close() {
#if defined(__linux__)
 if (fd_ >= 0) ::close(fd_);
#endif
 fd_ = -1;
 local_port_ = 0;
}

bool UdpLink::send_to(const std::string& address, std::uint16_t port,
 const std::uint8_t* data, std::size_t size) {
#if defined(__linux__)
 if (fd_ < 0 || data == nullptr || size == 0 || size > 65507) return false;
 sockaddr_in endpoint{};
 if (!address_of(address, port, endpoint)) return false;
 return ::sendto(fd_, data, size, MSG_NOSIGNAL,
 reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) ==
 static_cast<ssize_t>(size);
#else
 (void)address; (void)port; (void)data; (void)size;
 return false;
#endif
}

int UdpLink::receive(std::uint8_t* data, std::size_t capacity,
 int timeout_ms, std::string* sender) {
#if defined(__linux__)
 if (fd_ < 0 || data == nullptr || capacity == 0) return -1;
 pollfd pfd{fd_, POLLIN, 0};
 const int ready = ::poll(&pfd, 1, timeout_ms);
 if (ready <= 0) return ready;
 sockaddr_in source{};
 socklen_t length = sizeof(source);
 const ssize_t count = ::recvfrom(fd_, data, capacity, 0,
 reinterpret_cast<sockaddr*>(&source), &length);
 if (count < 0) return errno == EAGAIN || errno == EINTR ? 0 : -1;
 if (sender != nullptr) {
 char text[INET_ADDRSTRLEN]{};
 *sender = inet_ntop(AF_INET, &source.sin_addr, text, sizeof(text));
 }
 return static_cast<int>(count);
#else
 (void)data; (void)capacity; (void)timeout_ms; (void)sender;
 return -1;
#endif
}

bool CanSegmenter::split(std::uint32_t can_id, std::uint16_t transfer_id,
 const std::uint8_t* bytes, std::size_t size,
 std::vector<CanChunk>& chunks) {
 chunks.clear();
 if (bytes == nullptr || size == 0 || size > 1020) return false;
 const std::size_t total = (size + 3) / 4;
 if (total == 0 || total > 255) return false;
 for (std::size_t index = 0; index < total; ++index) {
 CanChunk chunk;
 chunk.id = can_id;
 chunk.data[0] = static_cast<std::uint8_t>(transfer_id >> 8);
 chunk.data[1] = static_cast<std::uint8_t>(transfer_id);
 chunk.data[2] = static_cast<std::uint8_t>(index);
 chunk.data[3] = static_cast<std::uint8_t>(total);
 const std::size_t offset = index * 4;
 chunk.length = static_cast<std::uint8_t>(
 std::min<std::size_t>(4, size - offset) + 4);
 std::copy(bytes + offset, bytes + offset + chunk.length - 4,
 chunk.data.begin() + 4);
 chunks.push_back(chunk);
 }
 return true;
}

bool CanSegmenter::accept(const CanChunk& chunk,
 std::vector<std::uint8_t>& complete) {
 complete.clear();
 if (chunk.length < 4 || chunk.length > 8 || chunk.data[3] == 0) return false;
 const std::uint16_t transfer = static_cast<std::uint16_t>(chunk.data[0]) << 8 |
 chunk.data[1];
 const std::uint8_t total = chunk.data[3];
 if (chunk.data[2] >= total) return false;
 if (chunk.data[2] == 0) {
 pending_.clear();
 can_id_ = chunk.id;
 transfer_id_ = transfer;
 total_ = total;
 expected_index_ = 0;
 } else if (chunk.id != can_id_ || transfer != transfer_id_ ||
 total != total_ || chunk.data[2] != expected_index_) {
 reset();
 return false;
 }
 pending_.insert(pending_.end(), chunk.data.begin() + 4,
 chunk.data.begin() + chunk.length);
 if (chunk.data[2] == total - 1) {
 complete = pending_;
 reset();
 return true;
 }
 ++expected_index_;
 return false;
}

void CanSegmenter::reset() {
 can_id_ = 0;
 transfer_id_ = 0;
 expected_index_ = 0;
 total_ = 0;
 pending_.clear();
}

SocketCanLink::~SocketCanLink() { close(); }

bool SocketCanLink::open(const std::string& interface_name,
 std::uint32_t can_id) {
 close();
#if defined(__linux__)
 fd_ = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
 if (fd_ < 0) return false;
 ifreq request{};
 std::strncpy(request.ifr_name, interface_name.c_str(), IFNAMSIZ - 1);
 if (::ioctl(fd_, SIOCGIFINDEX, &request) != 0) {
 close();
 return false;
 }
 sockaddr_can address{};
 address.can_family = AF_CAN;
 address.can_ifindex = request.ifr_ifindex;
 if (::bind(fd_, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0) {
 close();
 return false;
 }
 can_id_ = can_id;
 return true;
#else
 (void)interface_name; (void)can_id;
 return false;
#endif
}

void SocketCanLink::close() {
#if defined(__linux__)
 if (fd_ >= 0) ::close(fd_);
#endif
 fd_ = -1;
}

bool SocketCanLink::send(const CanChunk& chunk) {
#if defined(__linux__)
 if (fd_ < 0 || chunk.length > 8 || chunk.id != can_id_) return false;
 can_frame frame{};
 frame.can_id = chunk.id;
 frame.can_dlc = chunk.length;
 std::copy(chunk.data.begin(), chunk.data.end(), frame.data);
 return ::write(fd_, &frame, sizeof(frame)) == static_cast<ssize_t>(sizeof(frame));
#else
 (void)chunk;
 return false;
#endif
}

bool SocketCanLink::receive(CanChunk& chunk, int timeout_ms) {
#if defined(__linux__)
 if (fd_ < 0) return false;
 pollfd pfd{fd_, POLLIN, 0};
 if (::poll(&pfd, 1, timeout_ms) <= 0) return false;
 can_frame frame{};
 if (::read(fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame)) ||
 frame.can_id != can_id_ || frame.can_dlc > 8) return false;
 chunk.id = frame.can_id;
 chunk.length = frame.can_dlc;
 std::copy(frame.data, frame.data + 8, chunk.data.begin());
 return true;
#else
 (void)chunk; (void)timeout_ms;
 return false;
#endif
}

SpiLink::~SpiLink() { close(); }

bool SpiLink::open(const std::string& device, std::uint32_t speed_hz,
 std::uint8_t mode, std::uint8_t bits) {
 close();
#if defined(__linux__)
 fd_ = ::open(device.c_str(), O_RDWR | O_CLOEXEC);
 if (fd_ < 0) return false;
 if (::ioctl(fd_, SPI_IOC_WR_MODE, &mode) != 0 ||
 ::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) != 0 ||
 ::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) != 0) {
 close();
 return false;
 }
 return true;
#else
 (void)device; (void)speed_hz; (void)mode; (void)bits;
 return false;
#endif
}

void SpiLink::close() {
#if defined(__linux__)
 if (fd_ >= 0) ::close(fd_);
#endif
 fd_ = -1;
}

bool SpiLink::transfer(const std::uint8_t* tx, std::uint8_t* rx,
 std::size_t size) {
#if defined(__linux__)
 if (fd_ < 0 || tx == nullptr || rx == nullptr || size == 0 || size > 4096) {
 return false;
 }
 spi_ioc_transfer message{};
 message.tx_buf = reinterpret_cast<std::uintptr_t>(tx);
 message.rx_buf = reinterpret_cast<std::uintptr_t>(rx);
 message.len = static_cast<std::uint32_t>(size);
 return ::ioctl(fd_, SPI_IOC_MESSAGE(1), &message) >= 0;
#else
 (void)tx; (void)rx; (void)size;
 return false;
#endif
}

} // namespace lost_signal::robot::transport
