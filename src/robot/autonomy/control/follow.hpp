#pragma once
#include "robot/core/math.hpp"
#include "robot/autonomy/mapping/grid_map.hpp"
#include "robot/control/pid.hpp"
#include "robot/autonomy/planning/path.hpp"
#include "robot/platform/chassis_caps.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
// Shared path following: lookahead-Ld pure pursuit, corner speed scheduling, and reverse hysteresis.
// guide_step serves differential chassis and outputs target yaw rate, never fake motor PWM.
namespace sim {
// Advance idx past waypoints that are reached or already behind the robot, preventing a target-behind loop.
// First resynchronize forward when a later waypoint in the window is clearly closer after a cut corner or tracking loss.
// idx only advances; reverse recovery handles backtracking without decrementing it.
inline void advance_wp(const std::vector<RoutePoint>& path, size_t& idx, double ex, double ey, double eth,
                       double accept, double Ld) {
 (void)Ld;
 if (path.empty() || idx >= path.size())
  return;
 size_t hi = std::min(path.size(), idx + 24);
 size_t best = idx;
 double bd = std::hypot(path[idx].position.x - ex, path[idx].position.y - ey);
 for (size_t j = idx + 1; j < hi; j++) {
 double d = std::hypot(path[j].position.x - ex, path[j].position.y - ey);
  if (d < bd) {
   bd = d;
   best = j;
  }
 }
 if (best != idx && bd < 1.0)
  idx = best; // Resynchronize only when the robot is clearly farther along the track.
 while (idx < path.size()) {
 double dx = path[idx].position.x - ex, dy = path[idx].position.y - ey;
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

inline TravelDirection path_travel_direction(const std::vector<RoutePoint>& path, size_t idx) {
 if (idx >= path.size())
 return TravelDirection::Unspecified;
 if (path[idx].arrival_direction != TravelDirection::Unspecified)
 return path[idx].arrival_direction;
 for (size_t next = idx + 1; next < path.size(); ++next) {
 if (path[next].arrival_direction != TravelDirection::Unspecified)
 return path[next].arrival_direction;
 }
 return TravelDirection::Unspecified;
}

inline bool path_has_reverse(const std::vector<RoutePoint>& path) {
 return std::any_of(path.begin(), path.end(), [](const RoutePoint& point) {
 return point.arrival_direction == TravelDirection::Reverse;
 });
}

// The first point whose incoming gear differs starts the next motion section.
// Keeping lookahead inside this boundary prevents a controller from cutting
// across a Hybrid A* forward/reverse cusp.
inline size_t path_direction_section_end(const std::vector<RoutePoint>& path, size_t idx) {
 const TravelDirection direction = path_travel_direction(path, idx);
 if (direction == TravelDirection::Unspecified)
 return path.size();
 for (size_t next = idx + 1; next < path.size(); ++next) {
 const TravelDirection candidate = path[next].arrival_direction;
 if (candidate != TravelDirection::Unspecified && candidate != direction)
 return next;
 }
 return path.size();
}

inline void advance_wp_until(const std::vector<RoutePoint>& path, size_t& idx, size_t end_exclusive,
 double ex, double ey, double eth, double accept) {
 if (path.empty() || idx >= path.size())
 return;
 end_exclusive = std::min(end_exclusive, path.size());
 const size_t hi = std::min(end_exclusive, idx + 24);
 size_t best = idx;
 double best_distance = std::hypot(path[idx].position.x - ex, path[idx].position.y - ey);
 for (size_t candidate = idx + 1; candidate < hi; ++candidate) {
 const double candidate_distance =
 std::hypot(path[candidate].position.x - ex, path[candidate].position.y - ey);
 if (candidate_distance < best_distance) {
 best_distance = candidate_distance;
 best = candidate;
 }
 }
 if (best != idx && best_distance < 1.0)
 idx = best;
 while (idx < end_exclusive) {
 const double dx = path[idx].position.x - ex;
 const double dy = path[idx].position.y - ey;
 const double distance = std::hypot(dx, dy);
 const double along = dx * std::cos(eth) + dy * std::sin(eth);
 // The final point in a gear section is a cusp. Ordinary waypoint
 // lookahead tolerance must not consume it before the vehicle has executed
 // the incoming motion; otherwise a short Hybrid A* reverse primitive can
 // disappear at startup and the controller begins the forward section from
 // an infeasible pose.
 const bool cusp = idx + 1 == end_exclusive && end_exclusive < path.size();
 const double arrival_tolerance = cusp ? std::min(accept, 0.15) : accept;
 if (distance < arrival_tolerance)
 ++idx;
 else if (along < -0.1 && distance < 1.0 && idx + 1 < end_exclusive)
 ++idx;
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
inline Vec2 lookahead_pt(const std::vector<RoutePoint>& path, size_t idx, double ex, double ey, double Ld,
                         const GridMap& cost) {
 Vec2 self{ex, ey};
 if (path.empty())
  return self;
 if (idx >= path.size())
 return path.back().position;
 size_t hi = std::min(path.size(), idx + 24);
 for (size_t i = idx; i < hi; i++) {
 if (std::hypot(path[i].position.x - ex, path[i].position.y - ey) >= Ld) {
 if (los_clear(cost, self, path[i].position))
 return path[i].position;
   break; // Outside the circle but blocked: use stage 2 instead of jumping farther.
  }
 }
 Vec2 best = path.back().position;
 bool found = false;
 // Euclidean distance along a path is not monotonic after overshoot or a reversal; do not break at d > Ld.
 // Doing so oscillates at the Ld boundary. Use the last visible point with d <= Ld in the window.
 for (size_t i = idx; i < hi; i++) {
 double d = std::hypot(path[i].position.x - ex, path[i].position.y - ey);
 if (d <= Ld && los_clear(cost, self, path[i].position)) {
 best = path[i].position;
   found = true;
  }
 }
 if (!found) {
  size_t j0 = idx > 4 ? idx - 4 : 0;
  for (size_t j = idx; j-- > j0;) { // Nearest visible point behind.
 if (los_clear(cost, self, path[j].position)) {
 best = path[j].position;
    found = true;
    break;
   }
   if (j == 0)
    break;
  }
  if (!found && idx < path.size())
 best = path[idx].position;
 }
 (void)found;
 return best;
}

inline Vec2 lookahead_pt_until(const std::vector<RoutePoint>& path, size_t idx, size_t end_exclusive,
 double ex, double ey, double Ld, const GridMap& cost) {
 const Vec2 self{ex, ey};
 if (path.empty())
 return self;
 end_exclusive = std::min(end_exclusive, path.size());
 if (idx >= end_exclusive)
 return path[std::min(idx, path.size() - 1)].position;
 const size_t hi = std::min(end_exclusive, idx + 24);
 for (size_t point = idx; point < hi; ++point) {
 if (std::hypot(path[point].position.x - ex, path[point].position.y - ey) >= Ld) {
 if (los_clear(cost, self, path[point].position))
 return path[point].position;
 break;
 }
 }
 Vec2 best = path[idx].position;
 for (size_t point = idx; point < hi; ++point) {
 const double distance =
 std::hypot(path[point].position.x - ex, path[point].position.y - ey);
 if (distance <= Ld && los_clear(cost, self, path[point].position))
 best = path[point].position;
 }
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
inline double corner_speed_cap(const std::vector<RoutePoint>& path, size_t idx, double cruise, double alat_max,
                               double preview_m) {
 if (idx >= path.size() || cruise <= 0.0 || preview_m <= 0.0)
  return cruise;
 double cap = cruise;
 double along = 0.0;
 for (size_t i = idx + 1; i + 1 < path.size(); ++i) {
 along += std::hypot(path[i].position.x - path[i - 1].position.x,
 path[i].position.y - path[i - 1].position.y);
  if (along > preview_m)
   break;
 const double incoming = std::atan2(path[i].position.y - path[i - 1].position.y,
 path[i].position.x - path[i - 1].position.x);
 const double outgoing = std::atan2(path[i + 1].position.y - path[i].position.y,
 path[i + 1].position.x - path[i].position.x);
  const double turn = std::abs(wrap_angle(outgoing - incoming));
  if (turn < 1e-6)
   continue;
 const double leg = std::hypot(path[i + 1].position.x - path[i].position.x,
 path[i + 1].position.y - path[i].position.y);
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
inline GuideOut guide_step(const std::vector<RoutePoint>& path, GuideState& st, double ex, double ey, double eth,
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
// actuator loops. Forward/reverse direction comes from the global route; a
// car-like controller never guesses gear from target geometry.
inline GuideOut ackermann_guide_step(const std::vector<RoutePoint>& path,
 GuideState& st, double ex, double ey,
 double eth, double wheel_base_m, double max_steer_rad, double Ld,
 double cruise, double dt, const GridMap& cost) {
 size_t section_end = path_direction_section_end(path, st.idx);
 advance_wp_until(path, st.idx, section_end, ex, ey, eth, 0.6);
 if (st.idx >= path.size())
 return {0, 0, 0, true};
 section_end = path_direction_section_end(path, st.idx);
 const Vec2 target = lookahead_pt_until(path, st.idx, section_end, ex, ey, Ld, cost);
 const double alpha = heading_err(target, ex, ey, eth);
 const TravelDirection planned_direction = path_travel_direction(path, st.idx);
 st.reversing = planned_direction == TravelDirection::Reverse;
 const double target_distance = std::max(0.15, std::hypot(target.x - ex, target.y - ey));
 const double pursuit_distance = std::max(0.2, std::min(Ld, target_distance));
 const double motion_alpha = st.reversing ? wrap_angle(alpha + (alpha > 0.0 ? -kPi : kPi)) : alpha;
 double steer = std::atan2(2.0 * std::max(0.05, wheel_base_m) * std::sin(motion_alpha),
 pursuit_distance);
 steer = clamp(steer, -max_steer_rad, max_steer_rad);
 if (st.reversing) {
 st.reverse_time += dt;
 // steer was computed in the reverse motion frame. With signed bicycle
 // speed, the body-frame steering command has the opposite sign.
 return {alpha, -steer, -std::min(0.25, std::max(0.0, cruise)), false};
 }
 st.reverse_time = std::max(0.0, st.reverse_time - dt);
 return {alpha, steer, sched_v(alpha, cruise), false};
}

} // namespace sim
