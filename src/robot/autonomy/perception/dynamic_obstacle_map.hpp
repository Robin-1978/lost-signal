#pragma once

#include "robot/autonomy/perception/dynamic_object.hpp"
#include "robot/autonomy/planning/local/predictor.hpp"

#include <cstddef>
#include <deque>
#include <vector>

namespace sim {

struct DynamicObservation {
 std::size_t object_id{0};
 Vec2 position;
 double radius{0.0};
};

enum class TrackMotionClass {
 Unconfirmed,
 Moving,
 IntentAmbiguous,
};

struct DynamicTrack {
 std::size_t object_id{0};
 DynamicObjectState state;
 double last_observed_s{0.0};
 TrackMotionClass motion_class{TrackMotionClass::Unconfirmed};
 double learned_endpoint_dwell_s{0.0};
};

struct TrackMotionSample {
 double time_s{0.0};
 Vec2 position;
};

// One anonymous-track state proposed for data association at a measurement
// timestamp. A learned bounded-motion track may publish several intent
// hypotheses with the same object_id; association consumes them as alternate
// gates for one track, never as multiple physical objects.
struct DynamicAssociationHypothesis {
 std::size_t object_id{0};
 DynamicObjectState state;
 double last_observed_s{0.0};
};

// Transient local map for moving objects. Static A* occupancy remains separate
// so a moving object cannot leave permanent ghost cells in the global map.
class DynamicObstacleMap {
 public:
 void clear();
 void observe(const DynamicObservation& observation, double time_s);
 std::vector<DynamicTrack> active_tracks(double time_s, double max_age_s) const;
 std::vector<DynamicAssociationHypothesis> association_hypotheses(
 double time_s, double max_age_s) const;
 // Tracker-owned corridor snapshot for the predictor module. Exposed so the
 // parity test can feed predict_frames directly and prove bit-identity with
 // the delegating predict() below.
 std::vector<PredictInput> predict_inputs() const;
 PredictionBatch predict_typed(
 double time_s, double horizon_s, double frame_dt_s,
 double max_age_s) const;
 // Conservative view retains every intent hypothesis for behavior release;
 // nominal view keeps only the measured/primary mode for short-horizon
 // motion planning after behavior has committed.
 std::vector<std::vector<DynamicObjectState>> predict(double time_s, double horizon_s, double frame_dt_s,
                                                      double max_age_s) const;
 std::vector<std::vector<DynamicObjectState>> predict_nominal(double time_s, double horizon_s,
                                                              double frame_dt_s, double max_age_s) const;
 private:
 struct TrackState {
  DynamicTrack track;
  // Last resolved motion before a periodic target pauses at a learned
  // corridor endpoint. The public track still reports the measured stop;
  // only prediction uses this motion hypothesis to cover the return trip.
  Vec2 last_motion_velocity;
  bool motion_velocity_initialized{false};
 Vec2 motion_candidate_velocity;
 int motion_confirmation_samples{0};
  bool intent_ambiguous{false};
 int ambiguous_motion_samples{0};
  int decisive_motion_samples{0};
 double stationary_since_s{-1.0};
 double learned_endpoint_dwell_s{0.0};
 bool endpoint_dwell_observed{false};
  Vec2 min_observed;
  Vec2 max_observed;
 // A travelled extent is not an endpoint. Each side becomes a reflection
 // boundary only after a measured direction reversal at that side. Behavior
 // may call a periodic-motion model mature only after both sides of the
 // dominant axis have been observed independently.
 bool min_x_endpoint_observed{false};
 bool max_x_endpoint_observed{false};
 bool min_y_endpoint_observed{false};
 bool max_y_endpoint_observed{false};
  bool bounded_x{false};
  bool bounded_y{false};
 std::deque<TrackMotionSample> motion_samples;
 double position_error_ema_m{0.03};
 double velocity_error_ema_mps{0.04};
 };

 std::vector<TrackState> tracks_;
};

} // namespace sim
