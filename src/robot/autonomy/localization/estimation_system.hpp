#pragma once

#include "robot/autonomy/localization/dead_reckon.hpp"
#include "robot/config/motion_policy.hpp"
#include "robot/runtime/observation.hpp"

#include <cstdint>
#include <cmath>
#include <optional>

namespace sim {

class EstimationSystem {
 public:
 explicit EstimationSystem(std::uint32_t seed = 12345) : rng_(seed) {}

	// Deployed calibration for quality and validity thresholds. Composition
	// roots load it once from robot_data; unit tests keep the baseline defaults.
	void set_motion_policy(const MotionPolicy& policy) { motion_policy_ = policy; }
	const MotionPolicy& motion_policy() const { return motion_policy_; }

 void reset(double x, double y, double heading_rad, std::uint32_t seed);
 // Static calibration from imu_link coordinates into base_link. The mount
 // transform belongs to the robot description and is identical for simulated,
 // replayed, and native observations.
 void set_imu_mount_rpy(double roll_in_base_rad, double pitch_in_base_rad,
 double yaw_in_base_rad);
 // A map normal or a stationary alignment procedure may seed attitude before
 // the first IMU sample. This is an estimate, never simulator body truth.
 void initialize_attitude(double roll_rad, double pitch_rad);
 void update(const ObservationFrame& observation, double dt_s);
 void correct(const PoseObservation& observation, double time_s);
 const EstimatedState& state() const { return state_; }
 double position_error(Vec2 truth) const { return odometry_.driftErr(truth); }
 double heading_error_rad(double truth_heading_rad) const {
 return std::abs(wrap_angle(odometry_.th - truth_heading_rad));
 }
 double fused_position_error(Vec2 truth) const {
 return std::hypot(state_.position.x - truth.x, state_.position.y - truth.y);
 }
 double fused_heading_error_rad(double truth_heading_rad) const {
 return std::abs(wrap_angle(state_.heading_rad - truth_heading_rad));
 }
 Vec2 odometry_position() const { return odometry_.p; }
 double odometry_heading_rad() const { return odometry_.th; }
 Vec2 map_from_odom_translation() const { return map_from_odom_translation_; }
 double map_from_odom_heading_rad() const { return map_from_odom_heading_rad_; }

 private:
	MotionPolicy motion_policy_{};

 struct Vector3 {
 double x{0.0};
 double y{0.0};
 double z{0.0};
 };

 struct Quaternion {
 double w{1.0};
 double x{0.0};
 double y{0.0};
 double z{0.0};
 };

 void update_map_state();
 static Quaternion quaternion_from_euler(double roll, double pitch, double yaw);
 static Vector3 euler_from_quaternion(const Quaternion& value);
 static void normalize(Quaternion& value);
 Vector3 imu_to_base(Vector3 value) const;
 void update_attitude(const ImuObservation& imu, double dt_s);

 DeadReckon odometry_;
 Rng rng_;
 EstimatedState state_;
 Vec2 map_from_odom_translation_;
 double map_from_odom_heading_rad_{0.0};
 double imu_mount_roll_rad_{0.0};
 double imu_mount_pitch_rad_{0.0};
 double imu_mount_yaw_rad_{0.0};
 Quaternion attitude_;
 bool attitude_initialized_{false};
 std::optional<PoseObservation> velocity_pose_anchor_;
 Vec2 absolute_velocity_world_mps_;
 double absolute_yaw_rate_radps_{0.0};
 bool absolute_velocity_valid_{false};
 int pose_velocity_innovation_samples_{0};
 int pose_yaw_rate_innovation_samples_{0};
};

} // namespace sim
