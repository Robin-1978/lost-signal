#pragma once

#include "robot/core/math.hpp"
#include "robot/runtime/sensor_messages.hpp"

#include <algorithm>
#include <optional>
#include <string>
#include <vector>

namespace sim {

// Transport-neutral observations shared by simulation, replay, and native
// hardware providers. No physics, mission, actor-truth, or rendering type is
// reachable through this contract.
struct EncoderObservation {
 // Body-frame planar twist reconstructed from wheel/track encoders and the
 // platform geometry: +x forward, +y left, +z counter-clockwise.
 double linear_velocity_mps{0.0};
 double lateral_velocity_mps{0.0};
 double angular_velocity_radps{0.0};
 double slip_scale{1.0};
 MeasurementHeader header;
};

struct ActuatorFeedbackObservation {
 struct Channel {
 std::string channel_id;
 double position_rad{0.0};
 double velocity_radps{0.0};
 };

 std::vector<Channel> channels;
 MeasurementHeader header;

 const Channel* find(const std::string& channel_id) const {
 const auto found = std::find_if(channels.begin(), channels.end(),
 [&](const auto& channel) { return channel.channel_id == channel_id; });
 return found == channels.end() ? nullptr : &*found;
 }
};

struct ImuObservation {
 // Right-handed imu_link axes: +x forward, +y left, +z up.
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
 std::optional<LidarPointCloud> lidar_point_cloud;
 // Expected returns from known static geometry at the estimated sensor pose.
 // Simulation may raycast them; hardware may raycast the deployed map.
 std::optional<Scan> lidar_static_reference;
 std::optional<EncoderObservation> encoders;
 std::optional<ActuatorFeedbackObservation> actuator_feedback;
 std::optional<ImuObservation> imu;
 std::optional<PoseObservation> absolute_pose;
};

struct EstimatedState {
 Vec2 position;
 double heading_rad{0.0};
 double linear_velocity_mps{0.0};
 double lateral_velocity_mps{0.0};
 double angular_velocity_radps{0.0};
 double confidence{1.0};
 double timestamp_s{0.0};
};

} // namespace sim
