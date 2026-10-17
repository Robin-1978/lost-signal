#pragma once

#include "sim/runtime_state.hpp"

#include <cstdint>

namespace sim {

class EstimationSystem {
 public:
 explicit EstimationSystem(std::uint32_t seed = 12345) : rng_(seed) {}

 void reset(double x, double y, double heading_rad, std::uint32_t seed);
 void update(const ObservationFrame& observation, double dt_s);
 void correct(const PoseObservation& observation, double time_s);
 const EstimatedState& state() const { return state_; }
 double position_error(Vec2 truth) const { return odometry_.driftErr(truth); }
 double heading_error_rad(double truth_heading_rad) const {
 return std::abs(wrap_angle(odometry_.th - truth_heading_rad));
 }
 Vec2 odometry_position() const { return odometry_.p; }
 double odometry_heading_rad() const { return odometry_.th; }
 Vec2 map_from_odom_translation() const { return map_from_odom_translation_; }
 double map_from_odom_heading_rad() const { return map_from_odom_heading_rad_; }

 private:
 void update_map_state();

 DeadReckon odometry_;
 Rng rng_;
 EstimatedState state_;
 Vec2 map_from_odom_translation_;
 double map_from_odom_heading_rad_{0.0};
};

} // namespace sim
