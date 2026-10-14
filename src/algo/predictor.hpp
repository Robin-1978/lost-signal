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

// Phase-aware gap acceptance: the minimum predicted separation between the
// ego vehicle (advancing along its path at speed_mps, holding at the end)
// and every predicted threat frame, minus radii.
// PARKED (not wired into arbitration): gates (conjunction, maturity, veto)
// and scored-waiting all failed the pinned finale — in a two-attractor
// encounter any wait-timing change flips seeds, and stop-in-place waiting
// cannot fix graze passes (east measured 8mm center-overlap). Next touchpoint
// is wait-aware planning (plan §4 item 16), not another bolt-on.
// Unit tests pin the math meanwhile.
// Returns +infinity when there is nothing predicted.
double predicted_gap(const std::vector<Vec2>& path, std::size_t start_idx, double speed_mps,
                     const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                     double ego_radius_m);

} // namespace sim
