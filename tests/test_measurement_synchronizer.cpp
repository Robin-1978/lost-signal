#include <cassert>
#include <cmath>
#include <cstdio>

#include "robot/autonomy/estimation/measurement_synchronizer.hpp"

namespace {

using lost_signal::robot::core::Frame;
using lost_signal::robot::core::MonotonicNs;
using lost_signal::robot::iface::MeasurementHeader;
using lost_signal::robot::iface::ObservationFrame;

MeasurementHeader header(std::uint64_t sequence, MonotonicNs stamp_ns,
 const char* source, Frame frame) {
 return {sequence, stamp_ns, source, frame};
}

ObservationFrame envelope(std::uint64_t sequence, MonotonicNs stamp_ns) {
 ObservationFrame observation;
 observation.sequence = sequence;
 observation.stamp_ns = stamp_ns;
 observation.source = "aggregate";
 observation.frame = Frame::kBaseLink;
 return observation;
}

} // namespace

int main() {
 using lost_signal::robot::autonomy::MeasurementSynchronizer;
 using lost_signal::robot::core::FrameTransformTree;
 using lost_signal::robot::core::quaternion_from_rpy;

 constexpr double kPi = 3.14159265358979323846;
 FrameTransformTree transforms;

 // Sections in one aggregate envelope retain independent acquisition order.
 {
 MeasurementSynchronizer synchronizer;
 ObservationFrame observation = envelope(20, 120'000'000);
 observation.scan_valid = true;
 observation.ranges_m = {1.0F, 2.0F};
 observation.scan_header =
 header(2, 100'000'000, "lidar", Frame::kLidarLink);
 observation.odometry_valid = true;
 observation.forward_mps = 0.4;
 observation.lateral_mps = -0.3;
 observation.motion_header =
 header(5, 120'000'000, "encoder", Frame::kBaseLink);
 observation.imu_valid = true;
 observation.imu_gyro_z_radps = 0.2;
 observation.imu_header =
 header(9, 110'000'000, "imu", Frame::kImuLink);

 const auto batch = synchronizer.ingest(observation, transforms);
 assert(batch.accepted);
 assert(batch.scan_event.has_value());
 assert(batch.scan_event->source == "lidar");
 assert(batch.scan_event->stamp_ns == 100'000'000);
 assert(batch.estimator_events.size() == 2);
 assert(batch.estimator_events[0].source == "imu");
 assert(batch.estimator_events[0].stamp_ns == 110'000'000);
 assert(batch.estimator_events[1].source == "encoder");
 assert(batch.estimator_events[1].stamp_ns == 120'000'000);
 assert(synchronizer.latest().scan_header.source == "lidar");
 assert(synchronizer.latest().motion_header.source == "encoder");
 assert(synchronizer.latest().imu_header.source == "imu");

 const ObservationFrame localization =
 synchronizer.take_localization_observation();
 assert(localization.scan_valid);
 assert(localization.source == "lidar");
 assert(localization.odometry_valid);
 assert(localization.lateral_mps == -0.3);
 const ObservationFrame no_repeat =
 synchronizer.take_localization_observation();
 assert(!no_repeat.scan_valid);
 assert(no_repeat.ranges_m.empty());
 assert(no_repeat.odometry_valid);

 const auto adaptation = synchronizer.take_adaptation_observation();
 assert(adaptation.has_value());
 assert(adaptation->odometry_valid && adaptation->imu_valid);
 assert(adaptation->stamp_ns == 120'000'000);
 assert(adaptation->source == "encoder");
 assert(!synchronizer.take_adaptation_observation().has_value());
 }

 // Equal-time motion and IMU form one fusion event. A later source may reset
 // its sequence; duplicate and late sections cannot replace cached evidence.
 {
 MeasurementSynchronizer synchronizer;
 ObservationFrame first = envelope(1, 20'000'000);
 first.odometry_valid = true;
 first.motion_header = header(50, 20'000'000, "encoder-a",
 Frame::kBaseLink);
 first.imu_valid = true;
 first.imu_header = header(80, 20'000'000, "imu-a", Frame::kImuLink);
 const auto first_batch = synchronizer.ingest(first, transforms);
 assert(first_batch.estimator_events.size() == 1);
 assert(first_batch.estimator_events[0].odometry_valid);
 assert(first_batch.estimator_events[0].imu_valid);

 ObservationFrame switched = envelope(2, 40'000'000);
 switched.odometry_valid = true;
 switched.forward_mps = 0.7;
 switched.motion_header = header(1, 40'000'000, "encoder-b",
 Frame::kBaseLink);
 const auto switched_batch = synchronizer.ingest(switched, transforms);
 assert(switched_batch.accepted);
 assert(synchronizer.latest().motion_header.source == "encoder-b");
 assert(synchronizer.latest().motion_header.sequence == 1);

 ObservationFrame equal_time_other_source = envelope(3, 40'000'000);
 equal_time_other_source.odometry_valid = true;
 equal_time_other_source.forward_mps = 9.0;
 equal_time_other_source.motion_header = header(
 2, 40'000'000, "encoder-c", Frame::kBaseLink);
 assert(!synchronizer.ingest(equal_time_other_source, transforms).accepted);
 assert(synchronizer.latest().forward_mps == 0.7);

 ObservationFrame late = envelope(4, 30'000'000);
 late.odometry_valid = true;
 late.forward_mps = 8.0;
 late.motion_header = header(99, 30'000'000, "encoder-b",
 Frame::kBaseLink);
 assert(!synchronizer.ingest(late, transforms).accepted);
 assert(synchronizer.latest().forward_mps == 0.7);
 }

 // A legacy single-section frame inherits its envelope identity. IMU values
 // are rotated into base_link before any estimator sees them.
 {
 MeasurementSynchronizer synchronizer;
 transforms.set_transform(
 "base_link", "imu_link",
 {{0.0, 0.0, 0.0}, quaternion_from_rpy(kPi * 0.5, 0.0, 0.0)});
 ObservationFrame imu = envelope(7, 70'000'000);
 imu.source = "legacy-imu";
 imu.frame = Frame::kImuLink;
 imu.imu_valid = true;
 imu.imu_gyro_y_radps = 0.5;
 const auto batch = synchronizer.ingest(imu, transforms);
 assert(batch.estimator_events.size() == 1);
 const ObservationFrame& event = batch.estimator_events.front();
 assert(event.imu_header.source == "legacy-imu");
 assert(event.imu_header.frame == Frame::kBaseLink);
 assert(std::abs(event.imu_gyro_z_radps - 0.5) < 1e-12);
 assert(std::abs(event.imu_gyro_y_radps) < 1e-12);
 }

 std::printf("measurement_synchronizer ok\n");
 return 0;
}
