#include "robot/autonomy/control/pid.hpp"
#include "robot/core/math.hpp"
namespace sim {
double PID::update(double err, double dt) {
 p_ = kp_ * err;
 double deriv = first_ ? 0 : (err - prev_err_) / dt;
 first_ = false;
 prev_err_ = err;
 d_ = kd_ * deriv;
 const double candidate_integral = clamp(integ_ + err * dt, -int_lim_, int_lim_);
 const double candidate_output = p_ + ki_ * candidate_integral + d_;
 const bool pushes_farther_into_high_saturation = candidate_output > out_lim_ && err > 0.0;
 const bool pushes_farther_into_low_saturation = candidate_output < -out_lim_ && err < 0.0;
 if (anti_windup_ != AntiWindupMode::ConditionalIntegration ||
 (!pushes_farther_into_high_saturation && !pushes_farther_into_low_saturation)) {
 integ_ = candidate_integral;
 }
 i_ = ki_ * integ_;
 return clamp(p_ + i_ + d_, -out_lim_, out_lim_);
}
} // namespace sim
