#include "pid.hpp"
#include "math.hpp"
namespace sim {
double PID::update(double err, double dt){
  p_ = kp_*err;
  integ_ += err*dt;
  integ_ = clamp(integ_, -int_lim_, int_lim_);
  i_ = ki_*integ_;
  double deriv = first_ ? 0 : (err-prev_err_)/dt;
  first_=false; prev_err_=err;
  d_ = kd_*deriv;
  return clamp(p_+i_+d_, -out_lim_, out_lim_);
}
}
