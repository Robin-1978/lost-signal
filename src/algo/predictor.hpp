#pragma once

#include "dynobs.hpp"

#include <vector>

namespace sim {

// Predictor input: one tracked object plus what the tracker learned about its
// patrol corridor. The tracker owns observation/velocity/bounds estimation;
// the predictor owns only the future: constant-velocity extrapolation with
// reflection inside the observed corridor. Pure function of its arguments,
// no hidden state, so DWA and tests can call it without a tracker.
struct PredictInput {
 DynState state;
 double last_observed_s{0.0};
 Vec2 min_observed{0.0, 0.0};
 Vec2 max_observed{0.0, 0.0};
 bool bounded_x{false};
 bool bounded_y{false};
};

// Fixed-step future frames: frame k holds the predicted state at
// time_s + age + k * frame_dt_s for every input younger than max_age_s.
// Stale inputs (age > max_age_s) and future-dated inputs are skipped.
std::vector<std::vector<DynState>> predict_frames(const std::vector<PredictInput>& inputs, double time_s,
                                                  double horizon_s, double frame_dt_s, double max_age_s);

} // namespace sim
