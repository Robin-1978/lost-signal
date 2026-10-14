#include "predictor.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>

namespace sim {
namespace {

// Reflection inside the observed corridor. Unbounded axes (no direction flip
// seen yet) extrapolate linearly; bounded axes bounce off the observed
// min/max. The 0.05 m corridor floor and 8-reflection cap are method
// constants: narrow corridors carry no direction information, and the cap
// bounds work for pathological velocity/corridor ratios.
double reflected_position(double position, double velocity, double time_s, double minimum, double maximum,
                          bool bounded) {
 if (!bounded || maximum - minimum < 0.05)
  return position + velocity * time_s;
 double predicted = position + velocity * time_s;
 for (int reflection = 0; reflection < 8; ++reflection) {
  if (predicted < minimum) {
   predicted = minimum + (minimum - predicted);
  } else if (predicted > maximum) {
   predicted = maximum - (predicted - maximum);
  } else {
   break;
  }
 }
 return std::clamp(predicted, minimum, maximum);
}

// Point-to-segment distance: the workhorse. interval_min_dist below reduces
// the time-parameterized closest approach to this exact call.
double point_seg_dist(Vec2 p, Vec2 s0, Vec2 s1) {
 const double dx = s1.x - s0.x;
 const double dy = s1.y - s0.y;
 const double len2 = dx * dx + dy * dy;
 constexpr double kEps = 1e-12;
 if (len2 < kEps) {
  return std::hypot(p.x - s0.x, p.y - s0.y);
 }
 const double t = std::clamp(((p.x - s0.x) * dx + (p.y - s0.y) * dy) / len2, 0.0, 1.0);
 return std::hypot(p.x - (s0.x + dx * t), p.y - (s0.y + dy * t));
}

// Time-parameterized closest approach over one frame interval: both bodies
// move with the SAME time parameter (relative motion r + w*t), so this is a
// point-to-segment distance, not a segment-segment one. An independent-(s,t)
// segment distance would flag two bodies using the same space at different
// times as a collision — exactly what waiting separates. Reviewer gate:
// discrete frame sampling alone can miss a fast crosser slipping between
// frames; this sweeps every interval exactly (linear motion per interval is
// exact for our predictors) and is conservative nowhere.
double interval_min_dist(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1) {
 const Vec2 r{a0.x - b0.x, a0.y - b0.y};
 const Vec2 w{(a1.x - a0.x) - (b1.x - b0.x), (a1.y - a0.y) - (b1.y - b0.y)};
 return point_seg_dist({0.0, 0.0}, r, {r.x + w.x, r.y + w.y});
}

} // namespace

std::vector<std::vector<DynState>> predict_frames(const std::vector<PredictInput>& inputs, double time_s,
                                                  double horizon_s, double frame_dt_s, double max_age_s) {
 std::vector<std::vector<DynState>> frames;
 for (double future_s = 0.0; future_s <= horizon_s + 1e-9; future_s += frame_dt_s) {
  auto& frame = frames.emplace_back();
  for (const auto& input : inputs) {
   const double age = time_s - input.last_observed_s;
   if (age < -1e-9 || age > max_age_s)
    continue;
   auto predicted = input.state;
   const double prediction_time = age + future_s;
   predicted.p.x = reflected_position(predicted.p.x, predicted.v.x, prediction_time, input.min_observed.x,
                                      input.max_observed.x, input.bounded_x);
   predicted.p.y = reflected_position(predicted.p.y, predicted.v.y, prediction_time, input.min_observed.y,
                                      input.max_observed.y, input.bounded_y);
   frame.push_back(predicted);
  }
 }
 return frames;
}

double predicted_gap(const std::vector<Vec2>& path, std::size_t start_idx, double speed_mps,
                     const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                     double ego_radius_m, double depart_delay_s) {
 if (path.empty() || frames.empty()) {
  return std::numeric_limits<double>::infinity();
 }
 // Ego distance at frame k (absolute time k*dt), departing after the delay.
// Frame 0 is now: ego sits at start. Fixed from an off-by-one ((k+1)*dt)
// that compared the threat-now against the ego-in-one-step.
 const double dt = std::max(1e-6, frame_dt_s);
 const double speed = std::max(0.0, speed_mps);
 const double delay = std::max(0.0, depart_delay_s);
 auto ego_at = [&](double distance) {
  double walked = 0.0;
  Vec2 ego = path[std::min(start_idx, path.size() - 1)];
  for (std::size_t seg = start_idx; seg + 1 < path.size(); ++seg) {
   const double leg = std::hypot(path[seg + 1].x - path[seg].x, path[seg + 1].y - path[seg].y);
   if (leg < 1e-9) {
    continue;
   }
   if (walked + leg >= distance) {
    const double t = std::clamp((distance - walked) / leg, 0.0, 1.0);
    return Vec2{path[seg].x + (path[seg + 1].x - path[seg].x) * t,
                path[seg].y + (path[seg + 1].y - path[seg].y) * t};
   }
   walked += leg;
   ego = path[seg + 1];
  }
  return ego;
 };
 const auto ego_frame = [&](std::size_t k) {
  return ego_at(speed * std::max(0.0, static_cast<double>(k) * dt - delay));
 };
 double gap = std::numeric_limits<double>::infinity();
 if (frames.size() == 1) {
  // Degenerate single frame: point check (segment helper covers it, but be
  // explicit so the intent reads).
  for (const auto& threat : frames.front()) {
   gap = std::min(gap, std::hypot(threat.p.x - ego_frame(0).x, threat.p.y - ego_frame(0).y) -
                           threat.radius - ego_radius_m);
  }
  return gap;
 }
 // Swept intervals: both ego and threat move linearly between frames, so the
 // closed-form segment distance cannot miss an inter-frame crossing.
 for (std::size_t k = 0; k + 1 < frames.size(); ++k) {
  const Vec2 ego0 = ego_frame(k);
  const Vec2 ego1 = ego_frame(k + 1);
  const auto& fa = frames[k];
  const auto& fb = frames[k + 1];
  for (std::size_t i = 0; i < fa.size() && i < fb.size(); ++i) {
   gap = std::min(gap, interval_min_dist(ego0, ego1, fa[i].p, fb[i].p) - fa[i].radius - ego_radius_m);
  }
 }
 return gap;
}

double earliest_departure(const std::vector<Vec2>& path, std::size_t start_idx, double cruise_mps,
                           const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                           double ego_radius_m, double margin_m) {
 if (path.empty() || frames.empty() || cruise_mps <= 0.0) {
  return std::numeric_limits<double>::infinity();
 }
 const double dt = std::max(1e-6, frame_dt_s);
 // Last frame stands at (N-1)*dt: predict_frames emits future_s = 0..H
 // inclusive, so N frames cover H = (N-1)*dt seconds of absolute time.
 const double horizon =
   static_cast<double>(frames.size() > 0 ? frames.size() - 1 : 0) * dt;
 // Full-trajectory gate (reviewer requirement): a wait is only feasible if
 // the ego completes its whole remaining path inside the predicted horizon
 // AND the swept gap passes. A wait that merely pushes the collision past
 // the last frame is undecidable, not safe — skip it.
 double path_length = 0.0;
 for (std::size_t seg = start_idx; seg + 1 < path.size(); ++seg) {
  path_length += std::hypot(path[seg + 1].x - path[seg].x, path[seg + 1].y - path[seg].y);
 }
 constexpr double kSearchStepS = 0.2;
 for (double wait = 0.0; wait <= horizon + 1e-9; wait += kSearchStepS) {
  if (cruise_mps * (horizon - wait) < path_length - 1e-9) {
   continue;
  }
  const double gap = predicted_gap(path, start_idx, cruise_mps, frames, dt, ego_radius_m, wait);
  if (gap >= margin_m) {
   return wait;
  }
 }
 return std::numeric_limits<double>::infinity();
}

} // namespace sim
