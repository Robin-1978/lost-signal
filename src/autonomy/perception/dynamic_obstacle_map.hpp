#pragma once

#include "autonomy/perception/dynamic_object.hpp"
#include "autonomy/planning/local/predictor.hpp"

#include <cstddef>
#include <vector>

namespace sim {

struct DynamicObservation {
 std::size_t object_id{0};
 Vec2 position;
 double radius{0.0};
};

struct DynamicTrack {
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
 // Tracker-owned corridor snapshot for the predictor module. Exposed so the
 // parity test can feed predict_frames directly and prove bit-identity with
 // the delegating predict() below.
 std::vector<PredictInput> predict_inputs() const;
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
  Vec2 velocity_anchor_position;
  double velocity_anchor_time_s{0.0};
  bool velocity_initialized{false};
  // Last resolved motion before a periodic target pauses at a learned
  // corridor endpoint. The public track still reports the measured stop;
  // only prediction uses this motion hypothesis to cover the return trip.
  Vec2 last_motion_velocity;
  bool motion_velocity_initialized{false};
  bool intent_ambiguous{false};
  int decisive_motion_samples{0};
  Vec2 min_observed;
  Vec2 max_observed;
  bool bounded_x{false};
  bool bounded_y{false};
 };

 std::vector<TrackState> tracks_;
};

} // namespace sim
