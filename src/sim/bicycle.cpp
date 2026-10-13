#include "bicycle.hpp"
#include "math.hpp"
#include <cmath>
namespace sim {
BikeState Bicycle::step(const BikeState& s, const BikeCmd& cmd, double dt) {
 BikeState n = s;
 double des_v = clamp(cmd.target_v, p_.v_min, p_.v_max);
 double des_d = clamp(cmd.target_delta, -p_.delta_max, p_.delta_max);
 // accel limit
 double dv = clamp(des_v - s.v, -p_.accel_max * dt, p_.accel_max * dt);
 n.v = s.v + dv;
 // steer rate limit (actuator lag -> overshoot if PID too aggressive)
 double dd = clamp(des_d - s.delta, -p_.steer_rate_max * dt, p_.steer_rate_max * dt);
 n.delta = s.delta + dd;
 n.x = s.x + n.v * std::cos(s.theta) * dt;
 n.y = s.y + n.v * std::sin(s.theta) * dt;
 n.theta = wrap_angle(s.theta + n.v / p_.L * std::tan(n.delta) * dt);
 return n;
}
} // namespace sim
