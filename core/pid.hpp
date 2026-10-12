#pragma once
namespace sim {
class PID {
public:
  PID(double kp=1.0,double ki=0.0,double kd=0.0,double out_lim=1.0,double int_lim=1.0)
    : kp_(kp),ki_(ki),kd_(kd),out_lim_(out_lim),int_lim_(int_lim) {}
  void setGains(double kp,double ki,double kd){ kp_=kp; ki_=ki; kd_=kd; }
  void reset(){ integ_=0; prev_err_=0; first_=true; p_=i_=d_=0; }
  double update(double err, double dt);
  double p()const{return p_;} double i()const{return i_;} double d()const{return d_;}
  double kp()const{return kp_;} double ki()const{return ki_;} double kd()const{return kd_;}
private:
  double kp_,ki_,kd_,out_lim_,int_lim_;
  double integ_{0},prev_err_{0}; bool first_{true};
  double p_{0},i_{0},d_{0};
};
}
