#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <vector>

#include "observation_adapter.hpp"
#include "serial_protocol.hpp"

using namespace lost_signal::examples::serial_sensor;

namespace {

void put_u16(std::vector<std::uint8_t>& out, std::uint16_t value) {
 out.push_back(static_cast<std::uint8_t>(value));
 out.push_back(static_cast<std::uint8_t>(value >> 8));
}

void put_u32(std::vector<std::uint8_t>& out, std::uint32_t value) {
 for (int i = 0; i < 4; ++i) out.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
}

void put_u64(std::vector<std::uint8_t>& out, std::uint64_t value) {
 for (int i = 0; i < 8; ++i) out.push_back(static_cast<std::uint8_t>(value >> (8 * i)));
}

void put_i64(std::vector<std::uint8_t>& out, std::int64_t value) {
 put_u64(out, static_cast<std::uint64_t>(value));
}

void put_f32(std::vector<std::uint8_t>& out, float value) {
 std::uint32_t bits = 0;
 std::memcpy(&bits, &value, sizeof(bits));
 put_u32(out, bits);
}

std::uint32_t crc32(const std::uint8_t* data, std::size_t count) {
 std::uint32_t crc = 0xffffffffu;
 for (std::size_t i = 0; i < count; ++i) {
 crc ^= data[i];
 for (int bit = 0; bit < 8; ++bit) {
 crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
 }
 }
 return ~crc;
}

std::vector<std::uint8_t> frame(SensorKind kind, std::uint32_t boot,
 std::uint64_t sequence, std::int64_t stamp,
 const std::vector<std::uint8_t>& payload) {
 std::vector<std::uint8_t> out = {0x16, 0x4e, 0x54};
 put_u16(out, kWireVersion);
 out.push_back(static_cast<std::uint8_t>(kind));
 out.push_back(0); // flags
 put_u32(out, boot);
 put_u64(out, sequence);
 put_i64(out, stamp);
 put_u16(out, static_cast<std::uint16_t>(payload.size()));
 out.insert(out.end(), payload.begin(), payload.end());
 const std::uint32_t crc = crc32(out.data() + 3, out.size() - 3);
 put_u32(out, crc);
 return out;
}

std::vector<std::uint8_t> lidar_payload() {
 std::vector<std::uint8_t> payload;
 put_u16(payload, 3);
 put_u32(payload, static_cast<std::uint32_t>(-1'570'796));
 put_u32(payload, 1'570'796);
 put_u16(payload, 500);
 put_u16(payload, 1'250);
 put_u16(payload, 9'000);
 return payload;
}

std::vector<std::uint8_t> imu_payload() {
 std::vector<std::uint8_t> payload;
 put_f32(payload, 0.01f);
 put_f32(payload, 0.02f);
 put_f32(payload, 0.03f);
 put_f32(payload, 0.0f);
 put_f32(payload, 0.0f);
 put_f32(payload, 9.80665f);
 return payload;
}

} // namespace

int main() {
 // The receive side and parser side can be separate owners. The mutex-backed
 // byte ring never exposes a partially written byte sequence and reports
 // overflow instead of silently overwriting a control measurement.
 lost_signal::robot::transport::RingBuffer ring(8);
 const std::uint8_t burst[] = {1, 2, 3, 4, 5};
 assert(ring.push(burst, sizeof(burst)) == sizeof(burst));
 std::uint8_t peeked[3]{};
 assert(ring.peek(peeked, sizeof(peeked)) == sizeof(peeked));
 assert(peeked[0] == 1 && peeked[2] == 3);
 std::uint8_t drained[2]{};
 assert(ring.pop(drained, sizeof(drained)) == sizeof(drained));
 assert(drained[0] == 1 && drained[1] == 2);
 lost_signal::robot::transport::RingBuffer full_ring(4);
 assert(full_ring.push(burst, sizeof(burst)) == 4);
 assert(full_ring.size() == 4);
 assert(full_ring.push(burst, 1) == 0);
 assert(full_ring.discard(4) == 4 && full_ring.empty());

 SerialFrameDecoder decoder(4096);
 const auto lidar = frame(SensorKind::kLidar2d, 7, 1, 1'000'000,
 lidar_payload());
 // Split the frame at every possible byte boundary. A serial read is allowed
 // to return any prefix, and a subsequent read may contain the rest plus the
 // next frame (粘包).
 for (std::size_t split = 1; split < lidar.size(); ++split) {
 decoder.reset();
 assert(decoder.feed(lidar.data(), split));
 SerialPacket packet;
 assert(decoder.next(packet) == DecodeStatus::kNoFrame);
 assert(decoder.feed(lidar.data() + split, lidar.size() - split));
 assert(decoder.next(packet) == DecodeStatus::kAccepted);
 assert(packet.sequence == 1 && packet.stamp_ns == 1'000'000);
 SerialSensorConfig config;
 LidarMeasurement measurement;
 assert(decode_lidar(packet, config, measurement) == DecodeStatus::kAccepted);
 assert(measurement.beam_count == 3);
 assert(std::abs(measurement.ranges_m[1] - 1.25f) < 1e-6f);
 }

 // A CRC error consumes the corrupted frame and leaves the following valid
 // frame available for recovery.
 auto corrupted = lidar;
 corrupted[corrupted.size() - 1] ^= 0x40;
 const auto imu = frame(SensorKind::kImu, 7, 1, 2'000'000, imu_payload());
 std::vector<std::uint8_t> stream = corrupted;
 stream.insert(stream.end(), imu.begin(), imu.end());
 decoder.reset();
 assert(decoder.feed(stream.data(), stream.size()));
 SerialPacket packet;
 assert(decoder.next(packet) == DecodeStatus::kBadFrame);
 assert(decoder.next(packet) == DecodeStatus::kAccepted);
 ImuMeasurement imu_measurement;
 assert(decode_imu(packet, imu_measurement) == DecodeStatus::kAccepted);
 assert(imu_measurement.accel_z_mps2 > 9.8f);

 ObservationAdapter adapter;
 lost_signal::robot::iface::ObservationFrame observation;
 assert(adapter.make(packet, SerialSensorConfig{}, observation));
 assert(lost_signal::robot::iface::valid_observation_frame(observation));
 assert(lost_signal::robot::iface::imu_stamp_ns(observation) == 2'000'000);

 // Reordered data and a rebooted sequence domain are held until an explicit
 // reconnect/reset. No receive-time substitution is allowed.
 assert(decoder.feed(imu.data(), imu.size()));
 assert(decoder.next(packet) == DecodeStatus::kSequence);
 decoder.reset();
 assert(decoder.feed(imu.data(), imu.size()));
 assert(decoder.next(packet) == DecodeStatus::kAccepted);

 std::puts("serial_sensor_adapter ok");
 return 0;
}
