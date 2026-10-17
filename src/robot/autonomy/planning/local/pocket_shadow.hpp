#pragma once

#include "robot/autonomy/planning/local/predictor.hpp"
#include "robot/autonomy/localization/state_estimator.hpp"

#include <cstddef>
#include <vector>

namespace sim {

// RO-2 pocket shadow runner (observation only, never wired to behavior).
// Answers one question the transit slice left open: does the assessed
// execution (A* geometry + Rmin fillet + servo-lag bicycle at instant cruise)
// agree with the PRODUCTION AckermannDrive rollout (drive PID + shared
// contact physics) on the same reference?
//
// Layering: this module is pure comparison. It takes a predicted trajectory
// (from assess_transit) and an actual trajectory (caller-supplied, e.g. a
// production-chassis rollout in the test) plus an estimator observation, and
// returns an episode. It never steps a chassis, never fuses a filter, never
// releases behavior.
//
// Vocabulary discipline: the verdict here is OBSERVED agreement
// (Tracked/Deviated/Invalid), never Feasible/Allowed. A Tracked episode is
// model-consistency evidence, not a release license — sim-to-real stays red
// until a real vehicle runs (architecture is production-SHAPED, not
// production-PROVEN).

enum class ShadowVerdict {
  kObservedTracked,  // model and production agree within the reporting bands.
  kObservedDeviated, // they disagree: start/apex/merge-time bias quantified.
  kInvalid,          // no comparison possible (empty trajectory).
};

struct TrajectoryError {
  double max_dev_m{0.0};     // max nearest-time Euclidean deviation.
  double rmse_m{0.0};        // root-mean-square over resampled pairs.
  double endpoint_err_m{0.0}; // predicted end vs actual end distance.
  double time_bias_s{0.0};   // actual_end - predicted_end (drive lag shows here).
};

struct PocketShadowEpisode {
  double predicted_end_s{0.0};
  double actual_end_s{0.0};
  TrajectoryError error;
  // Estimator shadow (single stream observation, no fusion):
  double latency_s{0.0};                 // producer-set delay at query time.
  EstimateStatus estimator_status{EstimateStatus::kInvalid};
  double estimator_age_s{0.0};            // now - stamp at query.
  double covariance_trace{0.0};           // sum of the 6 diagonal variances.
  // Reporting carrier, NOT a gate: 1/(1+trace) full when Fresh, halved when
  // PredictedStale, 0 when Invalid. Pinned and stated; no threshold acts on it.
  double state_confidence{0.0};
  double min_gap_pred_m{0.0};             // predicted leg gap (nominal evidence).
  ShadowVerdict verdict{ShadowVerdict::kInvalid};
};

// Nearest-time resampling of actual onto predicted stamps. Deterministic,
// no interpolation invention: each predicted sample pairs with the
// temporally closest actual sample. Empty either side -> Invalid errors.
TrajectoryError compare_trajectories(const std::vector<TimedPose>& predicted,
                                     const std::vector<TimedPose>& actual);

// Episode assembly: pure, no thresholds beyond the reporting bands below.
// Tracked iff max_dev <= 0.20 m (level score-bar clearance) AND endpoint <=
// 1.0 m (level arrive tolerance). These bands are CONTENT-derived observation
// bins, not tuned gates: nothing releases on them, Phase-2 stays frozen.
PocketShadowEpisode make_pocket_shadow_episode(const LegAssessment& predicted,
                                               const std::vector<TimedPose>& actual,
                                               const PropagatedEstimate& estimator_view,
                                               double estimator_age_s);

const char* shadow_verdict_name(ShadowVerdict verdict);

} // namespace sim
