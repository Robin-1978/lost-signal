#pragma once

#include <cstddef>
#include <cstdint>
#include <limits>
#include <optional>
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

// One ROBOT-side forecast policy shared by behavior, DWA and the independent
// validator. A single batch prevents three consumers from assigning different
// ages or horizons to the same sensor track.
struct DynamicPredictionConfig {
 double horizon_s = 2.5;
 double frame_dt_s = 0.1;
 double max_track_age_s = 0.75;
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

PredictionBatch predict_perception(
 const PerceptionResult& perception, double decision_time_s,
 DynamicPredictionConfig config = {});

// Continuous-time state and clearance queries over a discrete forecast.
// States interpolate within each hypothesis; clearance minimizes relative
// linear motion over the whole interval, so a fast crossing between frames
// cannot pass through a DWA rollout undetected.
std::optional<DynamicTrack> predicted_state_at(
 const PredictedHypothesis& hypothesis, const PredictionBatch& prediction,
 double future_time_s);

struct PredictedClearance {
 double clearance_m = std::numeric_limits<double>::infinity();
 double time_s = std::numeric_limits<double>::infinity();
 std::uint32_t track_id = 0;
 std::size_t hypothesis_id = 0;
};

PredictedClearance minimum_predicted_clearance(
 const PredictionBatch& prediction, core::Vec2 ego_start,
 core::Vec2 ego_end, double interval_start_s, double interval_end_s,
 double ego_radius_m);

inline PredictInput predict_input_from_track(const DynamicTrack& track,
                                             double last_observed_s) {
  PredictInput in;
  in.track_id = track.id;
  in.state = track;
  in.last_observed_s = last_observed_s;
 in.position_uncertainty_growth_mps =
 track.position_uncertainty_growth_mps;
 in.position_uncertainty_cap_m = track.position_uncertainty_cap_m;
 in.velocity_uncertainty_mps = track.velocity_uncertainty_mps;
 in.alternate_velocity = track.alternate_velocity;
 in.alternate_velocity_valid = track.alternate_velocity_valid;
 in.min_observed = track.min_observed;
 in.max_observed = track.max_observed;
 in.bounded_x = track.bounded_x;
 in.bounded_y = track.bounded_y;
 in.endpoint_dwell_s = track.endpoint_dwell_s;
 in.initial_dwell_remaining_s = track.initial_dwell_remaining_s;
  return in;
}

}  // namespace lost_signal::robot::autonomy
