#include "robot/autonomy/estimation/measurement_synchronizer.hpp"

#include <algorithm>

namespace lost_signal::robot::autonomy {
namespace {

bool newer_measurement(const iface::MeasurementHeader& candidate,
                       const iface::MeasurementHeader& current,
                       bool have_current) {
  if (!candidate.valid()) return false;
  if (!have_current) return true;
  if (candidate.stamp_ns != current.stamp_ns) {
    return candidate.stamp_ns > current.stamp_ns;
  }
  return candidate.source == current.source &&
         candidate.sequence > current.sequence;
}

void copy_scan_section(const iface::ObservationFrame& source,
                       iface::ObservationFrame& target) {
  target.scan_header = source.scan_header;
  target.ranges_m = source.ranges_m;
  target.scan_valid = source.scan_valid;
  target.scan_angle_min_rad = source.scan_angle_min_rad;
  target.scan_angle_increment_rad = source.scan_angle_increment_rad;
  target.intensities = source.intensities;
  target.intensity_valid = source.intensity_valid;
  target.time_offsets_s = source.time_offsets_s;
  target.time_offset_valid = source.time_offset_valid;
  target.static_ranges_m = source.static_ranges_m;
  target.scan_static_valid = source.scan_static_valid;
}

void clear_scan_section(iface::ObservationFrame& target) {
  target.scan_header = {};
  target.ranges_m.clear();
  target.scan_valid = false;
  target.intensities.clear();
  target.intensity_valid = false;
  target.time_offsets_s.clear();
  target.time_offset_valid = false;
  target.static_ranges_m.clear();
  target.scan_static_valid = false;
}

void copy_motion_section(const iface::ObservationFrame& source,
                         iface::ObservationFrame& target) {
  target.motion_header = source.motion_header;
  target.odometry_valid = source.odometry_valid;
  target.forward_mps = source.forward_mps;
 target.lateral_mps = source.lateral_mps;
  target.yaw_rate_radps = source.yaw_rate_radps;
  target.drive_count = source.drive_count;
  target.steer_count = source.steer_count;
  target.drive_radps = source.drive_radps;
  target.steer_rad = source.steer_rad;
  target.steer_vel_radps = source.steer_vel_radps;
 target.arm_count = source.arm_count;
 target.arm_rad = source.arm_rad;
 target.arm_vel_radps = source.arm_vel_radps;
}

void copy_imu_section(const iface::ObservationFrame& source,
                      iface::ObservationFrame& target) {
  target.imu_header = source.imu_header;
  target.imu_valid = source.imu_valid;
  target.imu_gyro_x_radps = source.imu_gyro_x_radps;
  target.imu_gyro_y_radps = source.imu_gyro_y_radps;
  target.imu_gyro_z_radps = source.imu_gyro_z_radps;
  target.imu_ax_mps2 = source.imu_ax_mps2;
  target.imu_ay_mps2 = source.imu_ay_mps2;
  target.imu_az_mps2 = source.imu_az_mps2;
}

void use_header(const iface::MeasurementHeader& header,
                iface::ObservationFrame& target) {
  target.sequence = header.sequence;
  target.stamp_ns = header.stamp_ns;
  target.source = header.source;
  target.frame = header.frame;
}

iface::ObservationFrame estimator_event(
    const iface::ObservationFrame& source, bool motion, bool imu) {
  iface::ObservationFrame event;
  if (motion) copy_motion_section(source, event);
  if (imu) copy_imu_section(source, event);
  use_header(motion ? source.motion_header : source.imu_header, event);
  return event;
}

const iface::MeasurementHeader& newest_header(
    const iface::MeasurementHeader& first,
    const iface::MeasurementHeader& second) {
  if (!first.valid()) return second;
  if (!second.valid()) return first;
  if (second.stamp_ns != first.stamp_ns) {
    return second.stamp_ns > first.stamp_ns ? second : first;
  }
  return second.sequence > first.sequence ? second : first;
}

}  // namespace

void MeasurementSynchronizer::reset() {
  has_observation_ = false;
  have_scan_ = false;
  have_motion_ = false;
  have_imu_ = false;
  localization_scan_pending_ = false;
  adaptation_pending_ = false;
  latest_ = {};
  adaptation_ = {};
}

MeasurementBatch MeasurementSynchronizer::ingest(
    const iface::ObservationFrame& observation,
    const core::FrameTransformTree& transforms) {
  MeasurementBatch batch;
  if (!iface::valid_observation_frame(observation)) return batch;

  // Legacy native providers may use the transport header for their only
  // section. Aggregate providers retain their explicit acquisition headers.
  iface::ObservationFrame shaped = observation;
  if (shaped.scan_valid && shaped.scan_header.empty()) {
    shaped.scan_header = iface::envelope_header(shaped);
  }
  if ((shaped.odometry_valid || shaped.has_joints()) &&
      shaped.motion_header.empty()) {
    shaped.motion_header = iface::envelope_header(shaped);
  }
  if (shaped.imu_valid && shaped.imu_header.empty()) {
    shaped.imu_header = iface::envelope_header(shaped);
  }

  const bool accept_scan =
      shaped.scan_valid &&
      newer_measurement(shaped.scan_header, latest_.scan_header, have_scan_);
  const bool accept_motion =
      (shaped.odometry_valid || shaped.has_joints()) &&
      newer_measurement(shaped.motion_header, latest_.motion_header,
                        have_motion_);
  const bool accept_imu =
      shaped.imu_valid &&
      newer_measurement(shaped.imu_header, latest_.imu_header, have_imu_);
  if (!accept_scan && !accept_motion && !accept_imu) return batch;

  if (accept_scan) {
    shaped.scan_angle_increment_rad = iface::scan_angle_increment_rad(shaped);
  }
  if (accept_imu) {
    if (const auto base_from_imu =
            transforms.lookup("base_link", "imu_link")) {
      const core::Vec3 gyro = core::rotate_vector(
          base_from_imu->rotation,
          {shaped.imu_gyro_x_radps, shaped.imu_gyro_y_radps,
           shaped.imu_gyro_z_radps});
      const core::Vec3 accel = core::rotate_vector(
          base_from_imu->rotation,
          {shaped.imu_ax_mps2, shaped.imu_ay_mps2,
           shaped.imu_az_mps2});
      shaped.imu_gyro_x_radps = gyro.x;
      shaped.imu_gyro_y_radps = gyro.y;
      shaped.imu_gyro_z_radps = gyro.z;
      shaped.imu_ax_mps2 = accel.x;
      shaped.imu_ay_mps2 = accel.y;
      shaped.imu_az_mps2 = accel.z;
      shaped.imu_header.frame = core::Frame::kBaseLink;
    }
  }

  if (accept_motion && accept_imu &&
      shaped.motion_header.stamp_ns == shaped.imu_header.stamp_ns) {
    batch.estimator_events.push_back(estimator_event(shaped, true, true));
  } else {
    const bool motion_first =
        accept_motion &&
        (!accept_imu ||
         shaped.motion_header.stamp_ns < shaped.imu_header.stamp_ns);
    if (motion_first) {
      batch.estimator_events.push_back(estimator_event(shaped, true, false));
    }
    if (accept_imu) {
      batch.estimator_events.push_back(estimator_event(shaped, false, true));
    }
    if (accept_motion && !motion_first) {
      batch.estimator_events.push_back(estimator_event(shaped, true, false));
    }
  }

  if (accept_scan) {
    iface::ObservationFrame scan;
    copy_scan_section(shaped, scan);
    use_header(shaped.scan_header, scan);
    batch.scan_event = std::move(scan);
    copy_scan_section(shaped, latest_);
    have_scan_ = true;
    localization_scan_pending_ = true;
  }
  if (accept_motion) {
    copy_motion_section(shaped, latest_);
    copy_motion_section(shaped, adaptation_);
    have_motion_ = true;
    adaptation_pending_ = true;
  }
  if (accept_imu) {
    copy_imu_section(shaped, latest_);
    copy_imu_section(shaped, adaptation_);
    have_imu_ = true;
    adaptation_pending_ = true;
  }

  // Keep the accepted transport identity for diagnostics. Per-section
  // identity remains authoritative for freshness, ordering and ownership.
  latest_.sequence = shaped.sequence;
  latest_.stamp_ns = shaped.stamp_ns;
  latest_.source = shaped.source;
  latest_.frame = shaped.frame;
  has_observation_ = true;
  batch.accepted = true;
  return batch;
}

iface::ObservationFrame
MeasurementSynchronizer::take_localization_observation() {
  iface::ObservationFrame observation = latest_;
  if (localization_scan_pending_) {
    use_header(observation.scan_header, observation);
    localization_scan_pending_ = false;
  } else {
    clear_scan_section(observation);
  }
  return observation;
}

std::optional<iface::ObservationFrame>
MeasurementSynchronizer::take_adaptation_observation() {
  if (!adaptation_pending_) return std::nullopt;
  iface::ObservationFrame observation = adaptation_;
  const iface::MeasurementHeader& header =
      newest_header(observation.motion_header, observation.imu_header);
  use_header(header, observation);
  adaptation_ = {};
  adaptation_pending_ = false;
  return observation;
}

}  // namespace lost_signal::robot::autonomy
