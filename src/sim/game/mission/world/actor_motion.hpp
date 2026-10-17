#pragma once
#include "robot/autonomy/perception/dynamic_object.hpp"
#include <algorithm>
#include <vector>
#include <cmath>
#include <cstdint>
// A patrol actor moves between a and b as a pure time function, making it deterministic and reproducible.
// Its radius covers the actor body; collision checks add the robot radius separately.
namespace mission {
// NOTE: vehicle geometry lives in robot JSON (collision_radius_m); do not
// reintroduce a hardcoded robot radius here.
struct Patrol {
 sim::Vec2 a{0, 0}, b{0, 0};
 double speed{0.8};
 double radius{0.45};
};
// Pedestrian wander: closed waypoint loop at constant speed, pure time
// function like patrol_state (deterministic, reproducible). Pedestrians are
// kinematic social agents: proximity eats clearance, contact never fails the
// mission (that call lives in ScenarioSession, not here).
struct Wander {
 std::vector<sim::Vec2> points;
 double speed{0.5};
 double radius{0.25};
};
struct WanderState {
 sim::Vec2 p{0, 0}, v{0, 0};
 double radius{0.25};
};
inline WanderState wander_state(const Wander& w, double t) {
 WanderState s;
 s.radius = w.radius;
 if (w.points.size() < 2 || w.speed < 1e-9) {
  if (!w.points.empty()) s.p = w.points.front();
  s.v = {0, 0};
  return s;
 }
 double total = 0.0;
 std::vector<double> seg;
 seg.reserve(w.points.size());
 for (std::size_t i = 0; i < w.points.size(); ++i) {
  const double len = std::hypot(w.points[(i + 1) % w.points.size()].x - w.points[i].x,
                                w.points[(i + 1) % w.points.size()].y - w.points[i].y);
  seg.push_back(len);
  total += len;
 }
 if (total < 1e-9) {
  s.p = w.points.front();
  s.v = {0, 0};
  return s;
 }
 double d = std::fmod(t * w.speed, total);
 for (std::size_t i = 0; i < w.points.size(); ++i) {
  if (d <= seg[i]) {
   const sim::Vec2& a = w.points[i];
   const sim::Vec2& b = w.points[(i + 1) % w.points.size()];
   const double f = seg[i] < 1e-9 ? 0.0 : d / seg[i];
   s.p = {a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f};
   const double n = std::hypot(b.x - a.x, b.y - a.y);
   s.v = n < 1e-9 ? sim::Vec2{0, 0} : sim::Vec2{(b.x - a.x) / n * w.speed, (b.y - a.y) / n * w.speed};
   return s;
  }
  d -= seg[i];
 }
 s.p = w.points.front();
 s.v = {0, 0};
 return s;
}
// Seeded wander: the same closed loop, rotated by seed (start offset) with
// direction from a seed bit. Pure time function, zero reactivity: the same
// (points, seed, t) replays bit-identically, while distinct seeds
// desynchronize walkers that share one authored loop (town pedestrians).
// Degenerate inputs behave exactly like wander_state.
struct SeededWander {
 std::vector<sim::Vec2> points;
 std::uint64_t seed{0};
 double speed{0.5};
 double radius{0.25};
};
inline WanderState seeded_wander_state(const SeededWander& w, double t) {
 Wander loop;
 loop.speed = w.speed;
 loop.radius = w.radius;
 if (w.points.size() >= 2) {
  const std::size_t n = w.points.size();
  const std::size_t off = static_cast<std::size_t>(w.seed % n);
  const bool reverse = ((w.seed >> 5) & 1u) != 0u;
  loop.points.reserve(n);
  for (std::size_t i = 0; i < n; ++i) {
   loop.points.push_back(reverse ? w.points[(off + n - i) % n] : w.points[(off + i) % n]);
  }
 } else if (!w.points.empty()) {
  loop.points = w.points;
 }
 return wander_state(loop, t);
}
// State at time t, including analytic velocity for DWA extrapolation.
inline sim::DynamicObjectState patrol_state(const Patrol& pt, double t) {
 sim::DynamicObjectState s;
 s.radius = pt.radius;
 double L = std::hypot(pt.b.x - pt.a.x, pt.b.y - pt.a.y);
 if (L < 1e-9 || pt.speed < 1e-9) {
  s.p = pt.a;
  s.v = {0, 0};
  return s;
 }
 double dx = (pt.b.x - pt.a.x) / L, dy = (pt.b.y - pt.a.y) / L;
 double ph = std::fmod(t * pt.speed, 2 * L);
 if (ph < L) {
  s.p = {pt.a.x + dx * ph, pt.a.y + dy * ph};
  s.v = {dx * pt.speed, dy * pt.speed};
 } else {
  double q = ph - L;
  s.p = {pt.b.x - dx * q, pt.b.y - dy * q};
  s.v = {-dx * pt.speed, -dy * pt.speed};
 }
 return s;
}

// Dwell: a wander loop with timed stops (waits[i] seconds at points[i]).
// Pure time function like the rest: the cycle is legs plus dwells, position
// is exact, velocity is zero while dwelling. Empty waits == plain wander.
struct Dwell {
 std::vector<sim::Vec2> points;
 std::vector<double> waits;
 double speed{0.5};
 double radius{0.25};
};
struct DwellState {
 sim::Vec2 p{0, 0}, v{0, 0};
 double radius{0.25};
};
inline DwellState dwell_state(const Dwell& d, double t) {
 DwellState s;
 s.radius = d.radius;
 if (d.points.size() < 2 || d.speed < 1e-9) {
  if (!d.points.empty()) s.p = d.points.front();
  s.v = {0, 0};
  return s;
 }
 const std::size_t n = d.points.size();
 std::vector<double> leg(n, 0.0);
 double total = 0.0;
 for (std::size_t i = 0; i < n; ++i) {
  const sim::Vec2& a = d.points[i];
  const sim::Vec2& b = d.points[(i + 1) % n];
  leg[i] = std::hypot(b.x - a.x, b.y - a.y);
  total += leg[i] / d.speed + (i < d.waits.size() ? std::max(0.0, d.waits[i]) : 0.0);
 }
 if (total < 1e-9) {
  s.p = d.points.front();
  s.v = {0, 0};
  return s;
 }
 double ph = std::fmod(t, total);
 if (ph < 0.0) ph += total;
 for (std::size_t i = 0; i < n; ++i) {
  const double wait = i < d.waits.size() ? std::max(0.0, d.waits[i]) : 0.0;
  if (ph < wait) {
   s.p = d.points[i];
   s.v = {0, 0};
   return s;
  }
  ph -= wait;
  const double travel = leg[i] / d.speed;
  const sim::Vec2& a = d.points[i];
  const sim::Vec2& b = d.points[(i + 1) % n];
  if (ph < travel && leg[i] > 1e-9) {
   const double f = ph / travel;
   s.p = {a.x + (b.x - a.x) * f, a.y + (b.y - a.y) * f};
   s.v = {(b.x - a.x) / leg[i] * d.speed, (b.y - a.y) / leg[i] * d.speed};
   return s;
  }
  ph -= travel;
 }
 s.p = d.points.front();
 s.v = {0, 0};
 return s;
}
} // namespace mission
