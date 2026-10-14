#include "predictor.hpp"

#include <algorithm>
#include <cmath>

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

} // namespace sim
