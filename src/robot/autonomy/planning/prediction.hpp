#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

#include "robot/autonomy/perception/perception.hpp"
#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {

// Deterministic track prediction for local avoidance. Same hypothesis math
// as the battle-tested main-stack predictor, expressed directly in the new
// chain types: DynamicTrack in, DynamicTrack frames out. No actor IDs, no
// truth; uncertainty is an explicit envelope, not a probability claim.
enum class PredictionModel {
  kStationary = 0,
  kConstantVelocity = 1,
  kReflectedCorridor = 2,
  kAlternateIntent = 3,
};

struct PredictedHypothesis {
  std::uint32_t track_id = 0;
  std::size_t hypothesis_id = 0;
  PredictionModel model = PredictionModel::kStationary;
  std::vector<DynamicTrack> states;  // one per future time
};

struct PredictionBatch {
  double stamp_s = 0.0;
  double frame_dt_s = 0.0;
  std::vector<double> future_times_s;
  std::vector<PredictedHypothesis> hypotheses;
};

// Predictor input: one track plus forecaster parameters. Bounds/dwell come
// from the tracker when it has endpoint history; zeros degrade gracefully
// to linear extrapolation (bounded=false) with no dwell.
struct PredictInput {
  std::uint32_t track_id = 0;
  DynamicTrack state;
  double position_uncertainty_growth_mps = 0.0;
  double position_uncertainty_cap_m = 2.0;
  double velocity_uncertainty_mps = 0.0;
  core::Vec2 alternate_velocity{0.0, 0.0};
  bool alternate_velocity_valid = false;
  double last_observed_s = 0.0;
  core::Vec2 min_observed{0.0, 0.0};
  core::Vec2 max_observed{0.0, 0.0};
  bool bounded_x = false;
  bool bounded_y = false;
  double endpoint_dwell_s = 0.0;
  double initial_dwell_remaining_s = 0.0;
};

PredictionBatch predict_hypotheses(const std::vector<PredictInput>& inputs,
                                   double time_s, double horizon_s,
                                   double frame_dt_s, double max_age_s);

// Compatibility projection for DWA-style consumers: frame k holds every
// hypothesis state at future_times_s[k]. Deterministic order (input, then
// hypothesis); identity must not be reconstructed from a frame index.
std::vector<std::vector<DynamicTrack>> flatten_prediction_frames(
    const PredictionBatch& prediction);

// Fixed-step future frames directly from tracks.
std::vector<std::vector<DynamicTrack>> predict_frames(
    const std::vector<PredictInput>& inputs, double time_s, double horizon_s,
    double frame_dt_s, double max_age_s);

inline PredictInput predict_input_from_track(const DynamicTrack& track,
                                             double last_observed_s) {
  PredictInput in;
  in.track_id = track.id;
  in.state = track;
  in.last_observed_s = last_observed_s;
  return in;
}

}  // namespace lost_signal::robot::autonomy
