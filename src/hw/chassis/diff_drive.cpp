#include "diff_drive.hpp"
#include "algo/math.hpp"
namespace sim {
DiffState DiffDrive::step(const DiffState& s, const DiffCmd& cmd, double dt) {
 DiffState n = s;
 double des_v = clamp(cmd.target_v, p_.v_min, p_.v_max);
 double des_w = clamp(cmd.target_w, -p_.w_max, p_.w_max);
 n.v = s.v + clamp(des_v - s.v, -p_.accel_max * dt, p_.accel_max * dt);
 n.w = s.w + clamp(des_w - s.w, -p_.alpha_max * dt, p_.alpha_max * dt);
 n.x = s.x + n.v * std::cos(s.theta) * dt;
 n.y = s.y + n.v * std::sin(s.theta) * dt;
 n.theta = wrap_angle(s.theta + n.w * dt);
 return n;
}
void DiffDrive::twistToWheels(double v, double w, double base, double& vl, double& vr) {
 vl = v - w * base / 2.0;
 vr = v + w * base / 2.0;
}
void DiffDrive::wheelsToTwist(double vl, double vr, double base, double& v, double& w) {
 v = (vl + vr) / 2.0;
 w = (vr - vl) / base;
}
} // namespace sim
