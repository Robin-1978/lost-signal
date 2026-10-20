#pragma once

#include "robot/core/math.hpp"

namespace sim::autonomy {

struct BrakingMeasurement {
 bool active{false};
 bool valid{false};
 double start_speed_mps{0.0};
 double elapsed_s{0.0};
 double distance_m{0.0};
};

// Passive ROBOT-side instrument. It observes commands and resulting body
// motion, but never changes either. A braking run starts on an explicit
// moving-to-zero command transition and ends at physical standstill.
class VehicleMeasurementSystem {
 public:
 void reset();
 const BrakingMeasurement& update(double time_s, Vec2 position,
 double speed_mps, double target_speed_mps);
 const BrakingMeasurement& braking() const { return braking_; }

 private:
 BrakingMeasurement braking_;
 bool have_previous_{false};
 double previous_target_speed_mps_{0.0};
 Vec2 previous_position_;
 double brake_start_time_s_{0.0};
 double brake_direction_{0.0};
};

} // namespace sim::autonomy
