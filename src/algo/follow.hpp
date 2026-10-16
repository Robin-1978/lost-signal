#pragma once
#include "math.hpp"
#include "grid_map.hpp"
#include "pid.hpp"
#include "hw/chassis/caps.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
// Shared path following: lookahead-Ld pure pursuit, corner speed scheduling, and reverse hysteresis.
// guide_step serves differential chassis and outputs target yaw rate, never fake motor PWM.
namespace sim {
// Advance idx past waypoints that are reached or already behind the robot, preventing a target-behind loop.
// First resynchronize forward when a later waypoint in the window is clearly closer after a cut corner or tracking loss.
// idx only advances; reverse recovery handles backtracking without decrementing it.
inline void advance_wp(const std::vector<Vec2>& path, size_t& idx, double ex, double ey, double eth,
                       double accept, double Ld) {
 (void)Ld;
 if (path.empty() || idx >= path.size())
  return;
 size_t hi = std::min(path.size(), idx + 24);
 size_t best = idx;
 double bd = std::hypot(path[idx].x - ex, path[idx].y - ey);
 for (size_t j = idx + 1; j < hi; j++) {
  double d = std::hypot(path[j].x - ex, path[j].y - ey);
  if (d < bd) {
   bd = d;
   best = j;
  }
 }
 if (best != idx && bd < 1.0)
  idx = best; // Resynchronize only when the robot is clearly farther along the track.
 while (idx < path.size()) {
  double dx = path[idx].x - ex, dy = path[idx].y - ey;
  double dist = std::hypot(dx, dy);
  double along = (dx * std::cos(eth) + dy * std::sin(eth));
  if (dist < accept)
   idx++;
  else if (along < -0.1 && dist < 1.0 && idx + 1 < path.size())
   idx++; // Already passed; the 1 m limit prevents skipping a corner in bulk.
  else
   break;
 }
}
// Check line of sight in the inflated costmap so clearance is preserved.
inline bool los_clear(const GridMap& m, Vec2 a, Vec2 b) {
 double d = std::hypot(b.x - a.x, b.y - a.y);
 int n = std::max(1, (int)(d / (m.res() * 0.5)));
 for (int i = 0; i <= n; i++) {
  double t = (double)i / n;
  if (!m.isFree(a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t))
   return false;
 }
 return true;
}
// Three-stage lookahead selection:
// 1) first path point outside the Ld circle, without skipping a segment;
// 2) if it lacks line of sight at a corner, use the farthest visible point inside Ld;
// 3) if none inside Ld is visible, search near idx, including behind; fall back to path[idx].
inline Vec2 lookahead_pt(const std::vector<Vec2>& path, size_t idx, double ex, double ey, double Ld,
                         const GridMap& cost) {
 Vec2 self{ex, ey};
 if (path.empty())
  return self;
 if (idx >= path.size())
  return path.back();
 size_t hi = std::min(path.size(), idx + 24);
 for (size_t i = idx; i < hi; i++) {
  if (std::hypot(path[i].x - ex, path[i].y - ey) >= Ld) {
   if (los_clear(cost, self, path[i]))
    return path[i];
   break; // Outside the circle but blocked: use stage 2 instead of jumping farther.
  }
 }
 Vec2 best = path.back();
 bool found = false;
 // Euclidean distance along a path is not monotonic after overshoot or a reversal; do not break at d > Ld.
 // Doing so oscillates at the Ld boundary. Use the last visible point with d <= Ld in the window.
 for (size_t i = idx; i < hi; i++) {
  double d = std::hypot(path[i].x - ex, path[i].y - ey);
  if (d <= Ld && los_clear(cost, self, path[i])) {
   best = path[i];
   found = true;
  }
 }
 if (!found) {
  size_t j0 = idx > 4 ? idx - 4 : 0;
  for (size_t j = idx; j-- > j0;) { // Nearest visible point behind.
   if (los_clear(cost, self, path[j])) {
    best = path[j];
    found = true;
    break;
   }
   if (j == 0)
    break;
  }
  if (!found && idx < path.size())
   best = path[idx];
 }
 (void)found;
 return best;
}
// Reduce speed with cosine error scheduling and retain 10% creep because a bicycle model cannot steer at v=0.
inline double sched_v(double err, double cruise) {
 double c = std::cos(err);
 if (c < 0.1)
  c = 0.1;
 return cruise * c;
}
// Anticipatory corner slowdown; watchdog reverse recovery only reacts after tracking trouble.
// Limit each previewed turn by lateral acceleration: v <= sqrt(a_lat * R),
// where R ~= L_out / delta. Straight segments are unlimited; sharp turns slow early.
// a_lat comes from the tire and surface combination supplied by the caller.
// The 0.5 traction reserve is a method constant for disturbances and model error.
// Distance accumulates along the path; turn angle comes from adjacent segment headings.
// This measures path curvature; sched_v separately handles current heading error.
inline double corner_speed_cap(const std::vector<Vec2>& path, size_t idx, double cruise, double alat_max,
                               double preview_m) {
 if (idx >= path.size() || cruise <= 0.0 || preview_m <= 0.0)
  return cruise;
 double cap = cruise;
 double along = 0.0;
 for (size_t i = idx + 1; i + 1 < path.size(); ++i) {
  along += std::hypot(path[i].x - path[i - 1].x, path[i].y - path[i - 1].y);
  if (along > preview_m)
   break;
  const double incoming = std::atan2(path[i].y - path[i - 1].y, path[i].x - path[i - 1].x);
  const double outgoing = std::atan2(path[i + 1].y - path[i].y, path[i + 1].x - path[i].x);
  const double turn = std::abs(wrap_angle(outgoing - incoming));
  if (turn < 1e-6)
   continue;
  const double leg = std::hypot(path[i + 1].x - path[i].x, path[i + 1].y - path[i].y);
  if (leg < 1e-6)
   continue;
  const double allowed = std::sqrt(std::max(0.0, alat_max) * leg / turn);
  if (allowed < cap)
   cap = allowed;
 }
 return std::max(cap, 0.1 * cruise);
}
inline double heading_err(Vec2 wp, double ex, double ey, double eth) {
 return wrap_angle(std::atan2(wp.y - ey, wp.x - ex) - eth);
}
// Persistent follower state: waypoint index, reverse latch, and reverse timer.
// Reverse needs hysteresis (enter at 1.4, leave at 0.7); one 1.2 threshold creates a limit cycle near -1.2.
// After three seconds of continuous reverse, the watchdog treats the target-behind latch as stuck and spins
// toward the target when the chassis supports in-place rotation.
struct GuideState {
 size_t idx{0};
 bool reversing{false};
 double reverse_time{0.0};
 bool spinning{false};
};
// ex/ey/eth: pose used to advance the path and compute error (truth or odometry estimate).
// steer units follow the caller PID output limit: bicycle delta in rad, differential yaw rate in rad/s.
// tv: forward speed in m/s, negative during reverse recovery.
struct GuideOut {
 double err{0}, steer{0}, tv{0};
 bool reached{false};
};
inline GuideOut guide_step(const std::vector<Vec2>& path, GuideState& st, double ex, double ey, double eth,
                           PID& pid, double Ld, double cruise, double dt, const GridMap& cost,
                           const ChassisCaps& caps) {
 advance_wp(path, st.idx, ex, ey, eth, 0.6, Ld);
 if (st.idx >= path.size())
  return {0, 0, 0, true};
 Vec2 la = lookahead_pt(path, st.idx, ex, ey, Ld, cost);
 double err = heading_err(la, ex, ey, eth);
 double steer_fwd = pid.update(err, dt);
 if (!st.reversing && std::abs(err) > 1.4)
  st.reversing = true;
 else if (st.reversing && std::abs(err) < 0.7)
  st.reversing = false;
 double tv, steer;
 if (st.spinning) {
  // After the watchdog, remain in the spin until roughly aligned or the next step would reverse again.
  // Only a chassis that can spin in place enters this state.
  if (std::abs(err) < 0.5) {
   st.spinning = false;
   st.reverse_time = 0.0;
  } else {
   return {err, steer_fwd, 0.0, false};
  }
 }
 if (st.reversing) {
  st.reverse_time += dt;
  if (st.reverse_time > 3.0) {
   // The watchdog stops reversing and spins when possible. A non-spinning chassis such as Ackermann
   // continues steering in reverse, its natural recovery. Planning a full K-turn
   // is future work and is not simulated here.
   st.reversing = false;
   st.reverse_time = 0.0;
   if (caps.can_spin) {
    st.spinning = true;
    return {err, steer_fwd, 0.0, false};
   }
  }
  tv = -0.4;
  steer = -steer_fwd;
 } // Target is behind: turn while reversing.
 else {
  // Leaky trouble memory, not a latch reset: reverse_time drains at the same
  // rate it fills, so flapping between reverse and forward (marginal lookahead
  // flips, slow-spin chassis that cannot damp them) still accumulates toward
  // the watchdog. The 3 s threshold keeps its meaning: 3 net-seconds of
  // reversing trouble. A completed spin recovery (above) still clears it.
  st.reverse_time = std::max(0.0, st.reverse_time - dt);
  steer = steer_fwd;
  tv = sched_v(err, cruise);
 }
 return {err, steer, tv, false};
}

// Pure Pursuit for a bicycle/Ackermann chassis. The output is front steering
// angle; the chassis' steering-servo PID and drive-wheel PID close the two
// actuator loops. Reverse recovery remains available because a car cannot
// spin in place.
inline GuideOut ackermann_guide_step(const std::vector<Vec2>& path, GuideState& st, double ex, double ey,
 double eth, double wheel_base_m, double max_steer_rad, double Ld,
 double cruise, double dt, const GridMap& cost) {
 advance_wp(path, st.idx, ex, ey, eth, 0.6, Ld);
 if (st.idx >= path.size())
 return {0, 0, 0, true};
 const Vec2 target = lookahead_pt(path, st.idx, ex, ey, Ld, cost);
 const double alpha = heading_err(target, ex, ey, eth);
 if (!st.reversing && std::abs(alpha) > 1.55)
 st.reversing = true;
 else if (st.reversing && std::abs(alpha) < 0.65)
 st.reversing = false;
 const double target_distance = std::max(0.15, std::hypot(target.x - ex, target.y - ey));
 const double pursuit_distance = std::max(0.2, std::min(Ld, target_distance));
 const double motion_alpha = st.reversing ? wrap_angle(alpha + (alpha > 0.0 ? -kPi : kPi)) : alpha;
 double steer = std::atan2(2.0 * std::max(0.05, wheel_base_m) * std::sin(motion_alpha),
 pursuit_distance);
 steer = clamp(steer, -max_steer_rad, max_steer_rad);
 if (st.reversing) {
 st.reverse_time += dt;
 return {alpha, -steer, -0.4, false};
 }
 st.reverse_time = std::max(0.0, st.reverse_time - dt);
 return {alpha, steer, sched_v(alpha, cruise), false};
}
} // namespace sim
