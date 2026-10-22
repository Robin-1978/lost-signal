#pragma once

namespace lost_signal::robot::platform {

struct AckermannSteeringAngles {
 double left_rad{0.0};
 double right_rad{0.0};
};

struct AckermannRearWheelSpeeds {
 double left_mps{0.0};
 double right_mps{0.0};
};

// The command angle is the equivalent bicycle angle at the centre of the
// front axle. The two physical steering joints and rear wheel speeds are
// resolved about one instantaneous centre on the rear axle.
AckermannSteeringAngles ackermann_steering_angles(double center_angle_rad,
 double wheel_base_m, double track_width_m);
AckermannRearWheelSpeeds ackermann_rear_wheel_speeds(double center_speed_mps,
 double center_angle_rad, double wheel_base_m, double track_width_m);

} // namespace lost_signal::robot::platform
