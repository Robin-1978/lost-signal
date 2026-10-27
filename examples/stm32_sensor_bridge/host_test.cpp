#include <cassert>
#include <cmath>
#include <cstdio>

#include "sensor_bridge.hpp"
#include "observation_adapter.hpp"

using namespace lost_signal::stm32_reference;

namespace {
PacketHeader header(SensorKind kind, std::uint64_t sequence,
 std::int64_t stamp = 1'000'000) {
 PacketHeader out;
 out.kind = kind;
 out.boot_id = 77;
 out.sequence = sequence;
 out.stamp_ns = stamp;
 return out;
}
} // namespace

int main() {
 SensorBridge bridge({.drive_count = 2, .steer_count = 0,
 .max_lidar_beams = 4, .max_range_mm = 10'000});

 Lidar2dPacket lidar;
 lidar.header = header(SensorKind::kLidar2d, 1);
 lidar.beam_count = 3;
 lidar.angle_min_urad = -1'570'796;
 lidar.angle_increment_urad = 1'570'796;
 lidar.range_mm[0] = 500;
 lidar.range_mm[1] = 1'250;
 lidar.range_mm[2] = 9'000;
 LidarMeasurement decoded_lidar;
 assert(bridge.decode(lidar, decoded_lidar) == DecodeStatus::kAccepted);
 assert(decoded_lidar.header.stamp_ns == lidar.header.stamp_ns);
 assert(decoded_lidar.beam_count == 3);
 assert(std::abs(decoded_lidar.ranges_m[1] - 1.25f) < 1e-6f);
 ObservationAdapter adapter;
 const auto lidar_observation = adapter.lidar(decoded_lidar);
 assert(lost_signal::robot::iface::valid_observation_frame(
 lidar_observation));
 assert(lost_signal::robot::iface::scan_stamp_ns(lidar_observation) ==
 decoded_lidar.header.stamp_ns);

 // Out-of-order data is rejected before it can reach the ROBOT observation
 // queue; acquisition time is never rewritten to receive time.
 LidarMeasurement stale;
 lidar.header.sequence = 1;
 assert(bridge.decode(lidar, stale) == DecodeStatus::kSequence);

 ImuPacket imu;
 imu.header = header(SensorKind::kImu, 1, 2'000'000);
 imu.accel_z_mps2 = 9.80665f;
 ImuMeasurement decoded_imu;
 assert(bridge.decode(imu, decoded_imu) == DecodeStatus::kAccepted);
 assert(decoded_imu.header.stamp_ns == 2'000'000);
 const auto imu_observation = adapter.imu(decoded_imu);
 assert(lost_signal::robot::iface::valid_observation_frame(imu_observation));

 JointPacket joints;
 joints.header = header(SensorKind::kJoints, 1, 3'000'000);
 joints.drive_count = 2;
 joints.drive_radps[0] = 1.0f;
 joints.drive_radps[1] = 1.1f;
 JointMeasurement decoded_joints;
 assert(bridge.decode(joints, decoded_joints) == DecodeStatus::kAccepted);
 assert(decoded_joints.drive_radps[1] == 1.1f);
 const auto joint_observation = adapter.joints(decoded_joints);
 assert(lost_signal::robot::iface::valid_observation_frame(
 joint_observation));

 // A reboot requires an explicit transport handshake/reset.  A packet from
 // the new boot cannot silently mix with the old sequence domain.
 imu.header.boot_id = 78;
 imu.header.sequence = 2;
 assert(bridge.decode(imu, decoded_imu) == DecodeStatus::kSequence);
 bridge.reset();
 assert(bridge.decode(imu, decoded_imu) == DecodeStatus::kAccepted);

 std::puts("stm32_sensor_bridge ok");
 return 0;
}
