#include "robot/autonomy/diagnostics/vehicle_measurement.hpp"

#include <algorithm>
#include <cmath>

namespace sim::autonomy {
namespace {

constexpr double kMovingSpeedMps = 0.25;
constexpr double kZeroCommandMps = 0.05;
constexpr double kStoppedSpeedMps = 0.03;
constexpr double kCancelCommandMps = 0.15;

double distance(Vec2 first, Vec2 second) {
 return std::hypot(first.x - second.x, first.y - second.y);
}

} // namespace

void VehicleMeasurementSystem::reset() {
 braking_ = {};
 have_previous_ = false;
 previous_target_speed_mps_ = 0.0;
 previous_position_ = {};
 brake_start_time_s_ = 0.0;
 brake_direction_ = 0.0;
}

const BrakingMeasurement& VehicleMeasurementSystem::update(
 double time_s, Vec2 position, double speed_mps, double target_speed_mps) {
 if (!have_previous_) {
 have_previous_ = true;
 previous_target_speed_mps_ = target_speed_mps;
 previous_position_ = position;
 return braking_;
 }

 const bool stop_requested = !braking_.active &&
 std::abs(previous_target_speed_mps_) >= kMovingSpeedMps &&
 std::abs(target_speed_mps) <= kZeroCommandMps &&
 std::abs(speed_mps) >= kMovingSpeedMps;
 if (stop_requested) {
 braking_.active = true;
 braking_.valid = false;
 braking_.start_speed_mps = std::abs(speed_mps);
 braking_.elapsed_s = 0.0;
 braking_.distance_m = 0.0;
 brake_start_time_s_ = time_s;
 brake_direction_ = std::copysign(1.0, speed_mps);
 previous_position_ = position;
 }

 if (braking_.active) {
 braking_.distance_m += distance(previous_position_, position);
 braking_.elapsed_s = std::max(0.0, time_s - brake_start_time_s_);
 const bool stopped = std::abs(speed_mps) <= kStoppedSpeedMps ||
 speed_mps * brake_direction_ <= 0.0;
 const bool cancelled = std::abs(target_speed_mps) >= kCancelCommandMps &&
 target_speed_mps * brake_direction_ > 0.0;
 if (stopped) {
 braking_.active = false;
 braking_.valid = true;
 } else if (cancelled) {
 braking_.active = false;
 braking_.valid = false;
 }
 }

 previous_target_speed_mps_ = target_speed_mps;
 previous_position_ = position;
 return braking_;
}

} // namespace sim::autonomy
