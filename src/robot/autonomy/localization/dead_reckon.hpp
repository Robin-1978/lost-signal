#pragma once

#include "robot/core/math.hpp"
#include "robot/core/random.hpp"

namespace sim {

class DeadReckon {
 public:
 Vec2 p{0, 0};
 double th{0};
 void reset(double x, double y, double heading) {
 p = {x, y};
 th = heading;
 }
 void update(double forward_velocity, double lateral_velocity,
 double angular_velocity, double dt, Rng& rng,
 double slip_scale = 1.0, double yaw_bias = 0.0, bool has_gyro = false,
 double gyro_rate = 0.0, double gyro_sigma = 0.0);
 double driftErr(Vec2 true_position) const;
};

} // namespace sim
