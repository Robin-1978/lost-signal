#pragma once

#include "autonomy/perception/dynamic_object.hpp"
#include "hw/chassis/diff_drive.hpp"
#include "hw/sensors/sensor.hpp"

#include <optional>
#include <vector>

namespace sim {

struct WorldTruth {
 DiffState robot;
 std::vector<DynamicObjectState> dynamic_objects;
};

struct EncoderObservation {
 double linear_velocity_mps{0.0};
 double angular_velocity_radps{0.0};
 double slip_scale{1.0};
 MeasurementHeader header;
};

struct ImuObservation {
 // Right-handed imu_link axes: +x forward, +y left, +z up. Keep the complete
 // six-axis measurement even while the planar estimator consumes only yaw.
 double roll_rate_radps{0.0};
 double pitch_rate_radps{0.0};
 double yaw_rate_radps{0.0};
 double forward_acceleration_mps2{0.0};
 double left_acceleration_mps2{0.0};
 double up_acceleration_mps2{0.0};
 double noise_sigma_radps{0.0};
 double acceleration_noise_sigma_mps2{0.0};
 MeasurementHeader header;
};

struct PoseObservation {
 Vec2 position;
 double heading_rad{0.0};
 double position_sigma_m{0.05};
 double heading_sigma_rad{0.01};
 MeasurementHeader header;
};

struct ObservationFrame {
 double time_s{0.0};
 std::optional<Scan> lidar;
 // Expected returns from known static 2.5D/3D geometry at the estimated
 // sensor pose. Simulation supplies this with a physics raycast; hardware can
 // produce the same contract from map raycasting.
 std::optional<Scan> lidar_static_reference;
 std::optional<EncoderObservation> encoders;
 std::optional<ImuObservation> imu;
 std::optional<PoseObservation> absolute_pose;
};

struct EstimatedState {
 Vec2 position;
 double heading_rad{0.0};
 double linear_velocity_mps{0.0};
 double angular_velocity_radps{0.0};
 double confidence{1.0};
 double timestamp_s{0.0};
};

} // namespace sim
