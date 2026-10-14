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
                     double ego_radius_m) {
 if (path.empty() || frames.empty()) {
  return std::numeric_limits<double>::infinity();
 }
 const double step = std::max(0.0, speed_mps) * std::max(1e-6, frame_dt_s);
 double gap = std::numeric_limits<double>::infinity();
 double along = 0.0;
 std::size_t seg = start_idx;
 Vec2 ego = path[std::min(start_idx, path.size() - 1)];
 for (const auto& frame : frames) {
  along += step;
  while (seg + 1 < path.size()) {
   const double leg = std::hypot(path[seg + 1].x - path[seg].x, path[seg + 1].y - path[seg].y);
   if (along <= leg || leg < 1e-9) {
    break;
   }
   along -= leg;
   ++seg;
  }
  if (seg + 1 < path.size()) {
   const double leg = std::hypot(path[seg + 1].x - path[seg].x, path[seg + 1].y - path[seg].y);
   const double t = leg < 1e-9 ? 0.0 : std::clamp(along / leg, 0.0, 1.0);
   ego = {path[seg].x + (path[seg + 1].x - path[seg].x) * t,
          path[seg].y + (path[seg + 1].y - path[seg].y) * t};
  } else {
   ego = path.back();
  }
  for (const auto& threat : frame) {
   const double clearance =
     std::hypot(threat.p.x - ego.x, threat.p.y - ego.y) - threat.radius - ego_radius_m;
   gap = std::min(gap, clearance);
  }
 }
 return gap;
}

} // namespace sim
