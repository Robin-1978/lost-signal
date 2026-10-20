#include "robot/autonomy/localization/estimation_system.hpp"

#include <cmath>
#include <iostream>
#include <stdexcept>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

constexpr double kGravityMps2 = 9.81;

sim::ObservationFrame observation(double time_s, double roll_rate,
 double pitch_rate, double yaw_rate, double force_x, double force_y,
 double force_z) {
 sim::ObservationFrame frame;
 frame.time_s = time_s;
 frame.encoders.emplace();
 frame.encoders->header = {
 time_s, 0, "base_link", 0.0, sim::kMeasurementSchemaVersion, "test.encoder"};
 frame.imu.emplace();
 frame.imu->roll_rate_radps = roll_rate;
 frame.imu->pitch_rate_radps = pitch_rate;
 frame.imu->yaw_rate_radps = yaw_rate;
 frame.imu->forward_acceleration_mps2 = force_x;
 frame.imu->left_acceleration_mps2 = force_y;
 frame.imu->up_acceleration_mps2 = force_z;
 frame.imu->header = {
 time_s, 0, "imu_link", 0.0, sim::kMeasurementSchemaVersion, "test.imu"};
 return frame;
}

sim::PoseObservation pose(double time_s, double x, double y,
 double heading_rad = 0.0) {
 sim::PoseObservation value;
 value.position = {x, y};
 value.heading_rad = heading_rad;
 value.position_sigma_m = 0.01;
 value.heading_sigma_rad = 0.006;
 value.header = {
 time_s, 0, "map", 0.0, sim::kMeasurementSchemaVersion, "test.pose"};
 return value;
}

} // namespace

int main() {
 // Analytic gravity oracle for ZYX base attitude. At rest, specific force in
 // base_link is [-g sin(pitch), g sin(roll) cos(pitch),
 // g cos(roll) cos(pitch)].
 constexpr double roll = 0.24;
 constexpr double pitch = -0.17;
 const double base_x = -kGravityMps2 * std::sin(pitch);
 const double base_y = kGravityMps2 * std::sin(roll) * std::cos(pitch);
 const double base_z = kGravityMps2 * std::cos(roll) * std::cos(pitch);

 sim::EstimationSystem aligned(7);
 aligned.reset(1.0, 2.0, 0.3, 7);
 aligned.update(observation(0.02, 0.0, 0.0, 0.0,
 base_x, base_y, base_z), 0.02);
 CHECK(aligned.state().attitude_valid);
 CHECK(std::abs(aligned.state().roll_rad - roll) < 1e-12);
 CHECK(std::abs(aligned.state().pitch_rad - pitch) < 1e-12);

 // The same physical attitude through a +90 degree yawed imu_link must yield
 // the same base_link result. This independently verifies mount calibration
 // rather than assuming every device is aligned with the chassis.
 sim::EstimationSystem yawed(7);
 yawed.reset(1.0, 2.0, 0.3, 7);
 yawed.set_imu_mount_rpy(0.0, 0.0, 0.5 * sim::kPi);
 yawed.update(observation(0.02, 0.0, 0.0, 0.0,
 base_y, -base_x, base_z), 0.02);
 CHECK(yawed.state().attitude_valid);
 CHECK(std::abs(yawed.state().roll_rad - roll) < 1e-12);
 CHECK(std::abs(yawed.state().pitch_rad - pitch) < 1e-12);

 // Quaternion integration is an independent rate check: with gravity marked
 // invalid, one second at 0.2 rad/s about base X must produce 0.2 rad roll.
 sim::EstimationSystem integrated(11);
 integrated.reset(0.0, 0.0, 0.0, 11);
 integrated.initialize_attitude(0.0, 0.0);
 for (int sample = 1; sample <= 50; ++sample) {
 integrated.update(observation(sample * 0.02, 0.2, 0.0, 0.0,
 0.0, 0.0, 0.0), 0.02);
 }
 CHECK(std::abs(integrated.state().roll_rad - 0.2) < 1e-5);
 CHECK(std::abs(integrated.state().pitch_rad) < 1e-12);

 // A single horizontal acceleration can have the same magnitude as gravity.
 // It may gently correct drift, but it must not be interpreted as an
 // instantaneous 24-degree pitch change.
 sim::EstimationSystem impulse(13);
 impulse.reset(0.0, 0.0, 0.0, 13);
 impulse.initialize_attitude(0.0, 0.0);
 const double impulse_x = 4.0;
 const double impulse_z = std::sqrt(
 kGravityMps2 * kGravityMps2 - impulse_x * impulse_x);
 impulse.update(observation(0.02, 0.0, 0.0, 0.0,
 impulse_x, 0.0, impulse_z), 0.02);
 CHECK(std::abs(impulse.state().pitch_rad) < 0.03);

 // Independent pose motion must remain observable when wheel encoders say
 // zero, as happens with locked wheels sliding on a slope. The oracle is a
 // one metre-per-second map-frame translation with a stationary heading.
 sim::EstimationSystem sliding(17);
 sliding.reset(0.0, 0.0, 0.0, 17);
 sliding.correct(pose(0.0, 0.0, 0.0), 0.0);
 for (int sample = 1; sample <= 10; ++sample) {
 const double time_s = 0.02 * sample;
 sliding.update(observation(time_s, 0.0, 0.0, 0.0,
 0.0, 0.0, kGravityMps2), 0.02);
 sliding.correct(pose(time_s, time_s, 0.0), time_s);
 }
 CHECK(sliding.state().linear_velocity_mps > 0.55);
 CHECK(std::abs(sliding.state().lateral_velocity_mps) < 1e-12);
 CHECK(std::abs(sliding.state().angular_velocity_radps) < 1e-12);

 // Sub-threshold pose differentiation noise must not replace a quiet wheel
 // observation during ordinary rolling.
 sim::EstimationSystem quiet(19);
 quiet.reset(0.0, 0.0, 0.0, 19);
 quiet.correct(pose(0.0, 0.0, 0.0), 0.0);
 for (int sample = 1; sample <= 10; ++sample) {
 const double time_s = 0.02 * sample;
 auto quiet_observation = observation(time_s, 0.0, 0.0, 0.0,
 0.0, 0.0, kGravityMps2);
 quiet_observation.encoders->linear_velocity_mps = 0.5;
 quiet.update(quiet_observation, 0.02);
 // A 0.51 m/s differentiated pose agrees with the 0.50 m/s encoder.
 quiet.correct(pose(time_s, 0.51 * time_s, 0.0), time_s);
 }
 CHECK(std::abs(quiet.state().linear_velocity_mps - 0.5) < 1e-12);

 // One independent-pose jump is not enough to declare body motion against
 // stopped encoders. The next consistent window may confirm real sliding;
 // a quiet follow-up instead clears the pending innovation.
 sim::EstimationSystem pose_spike(23);
 pose_spike.reset(0.0, 0.0, 0.0, 23);
 pose_spike.correct(pose(0.0, 0.0, 0.0), 0.0);
 for (int sample = 1; sample <= 5; ++sample) {
 const double time_s = 0.02 * sample;
 pose_spike.update(observation(time_s, 0.0, 0.0, 0.0,
 0.0, 0.0, kGravityMps2), 0.02);
 pose_spike.correct(pose(time_s, sample == 5 ? 0.10 : 0.0, 0.0), time_s);
 }
 CHECK(std::abs(pose_spike.state().linear_velocity_mps) < 1e-12);
 for (int sample = 6; sample <= 10; ++sample) {
 const double time_s = 0.02 * sample;
 pose_spike.update(observation(time_s, 0.0, 0.0, 0.0,
 0.0, 0.0, kGravityMps2), 0.02);
 pose_spike.correct(pose(time_s, 0.10, 0.0), time_s);
 }
 CHECK(std::abs(pose_spike.state().linear_velocity_mps) < 1e-12);

 bool rejected = false;
 try {
 impulse.set_imu_mount_rpy(0.0, std::nan(""), 0.0);
 } catch (const std::invalid_argument&) {
 rejected = true;
 }
 CHECK(rejected);
 return 0;
}
