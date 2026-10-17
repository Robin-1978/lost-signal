#include "autonomy/perception/dynamic_obstacle_map.hpp"
#include "autonomy/planning/local/predictor.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

// A deterministic alpha-beta (g-h) tracker is the constant-velocity form of
// a Kalman-style predictor/corrector.  Position is corrected more strongly
// than velocity so LiDAR cluster-centroid jitter cannot look like an actor
// reversing direction on every scan.  These dimensionless gains assume the
// configured sensor cadence is reasonably stable; dt remains explicit so
// replay and headless execution produce identical state estimates.
constexpr double kPositionGain = 0.35;
constexpr double kVelocityGain = 0.08;
constexpr double kMaxTrackedSpeedMps = 3.0;
constexpr double kReliableMotionSpeedMps = 0.25;
constexpr double kMotionMemoryBlend = 0.20;
constexpr double kIntentAmbiguousBelowMps = 0.40;
constexpr double kIntentDecisiveAboveMps = 0.48;
constexpr int kIntentDecisiveSamples = 2;
constexpr int kMotionConfirmationSamples = 3;
constexpr double kVelocityFitWindowS = 1.0;
constexpr double kUncertaintyBlend = 0.15;
constexpr double kPositionUncertaintyFloorM = 0.03;
// RO-1's cluster tracker does not yet expose a calibrated covariance. Keep the
// deterministic proxy bounded to the sensor-scale regime and document it as
// an engineering envelope; a future covariance provider can replace these
// values without changing the planning contract.
constexpr double kPositionUncertaintyCapM = 0.08;
constexpr double kVelocityUncertaintyFloorMps = 0.04;
constexpr double kVelocityUncertaintyCapMps = 0.12;
constexpr double kPositionPredictionEnvelopeCapM = 0.10;

Vec2 limited_velocity(Vec2 velocity) {
 const double speed = std::hypot(velocity.x, velocity.y);
 if (speed <= kMaxTrackedSpeedMps || speed < 1e-9)
  return velocity;
 const double scale = kMaxTrackedSpeedMps / speed;
 return {velocity.x * scale, velocity.y * scale};
}

Vec2 fitted_velocity(
 const std::deque<TrackMotionSample>& samples,
 Vec2 fallback) {
 if (samples.size() < 4)
 return fallback;
 double mean_time = 0.0;
 Vec2 mean_position;
 for (const auto& sample : samples) {
 mean_time += sample.time_s;
 mean_position.x += sample.position.x;
 mean_position.y += sample.position.y;
 }
 const double count = static_cast<double>(samples.size());
 mean_time /= count;
 mean_position.x /= count;
 mean_position.y /= count;
 double time_variance = 0.0;
 Vec2 covariance;
 for (const auto& sample : samples) {
 const double centered_time = sample.time_s - mean_time;
 time_variance += centered_time * centered_time;
 covariance.x += centered_time * (sample.position.x - mean_position.x);
 covariance.y += centered_time * (sample.position.y - mean_position.y);
 }
 if (time_variance < 1e-9)
 return fallback;
 return limited_velocity({covariance.x / time_variance,
 covariance.y / time_variance});
}

} // namespace

void DynamicObstacleMap::clear() {
 tracks_.clear();
}

void DynamicObstacleMap::observe(const DynamicObservation& observation, double time_s) {
 auto found = std::find_if(tracks_.begin(), tracks_.end(), [&](const TrackState& candidate) {
  return candidate.track.object_id == observation.object_id;
 });
 if (found == tracks_.end()) {
  TrackState created;
  created.track = {
    observation.object_id,
    {observation.position, {0.0, 0.0}, observation.radius},
    time_s,
  };
  created.min_observed = observation.position;
  created.max_observed = observation.position;
 created.motion_samples.push_back({time_s, observation.position});
  tracks_.push_back(created);
  return;
 }

 auto& track = *found;
 const double dt = time_s - track.track.last_observed_s;
 // Samples are state events, not arrival events.  A stale or duplicated
 // timestamp must not rewind the filter or create an unbounded 1/dt update.
 if (dt <= 1e-9)
 return;

 const Vec2 predicted_position{
 track.track.state.p.x + track.track.state.v.x * dt,
 track.track.state.p.y + track.track.state.v.y * dt,
 };
 const Vec2 innovation{
 observation.position.x - predicted_position.x,
 observation.position.y - predicted_position.y,
 };
 const Vec2 filtered_position{
 predicted_position.x + kPositionGain * innovation.x,
 predicted_position.y + kPositionGain * innovation.y,
 };
 const Vec2 alpha_beta_velocity = limited_velocity({
 track.track.state.v.x + kVelocityGain * innovation.x / dt,
 track.track.state.v.y + kVelocityGain * innovation.y / dt,
 });
 // Estimate motion from measurement-time positions rather than from the
 // inertial filter state. The position filter is deliberately smooth and can
 // coast past a shuttle endpoint for several low-rate scans; fitting that
 // lagging state would keep reporting "moving away" while the measured object
 // is already dwelling or reversing. Raw samples are noisy, but the bounded
 // least-squares window averages that noise and preserves the timestamped
 // evidence needed to classify intent as ambiguous before a crossing release.
 track.motion_samples.push_back({time_s, observation.position});
 while (!track.motion_samples.empty() &&
 time_s - track.motion_samples.front().time_s > kVelocityFitWindowS) {
 track.motion_samples.pop_front();
 }
 const Vec2 filtered_velocity = fitted_velocity(
 track.motion_samples, alpha_beta_velocity);
 const double position_residual_m = std::hypot(innovation.x, innovation.y);
 const double velocity_residual_mps = std::min(
 kVelocityUncertaintyCapMps, position_residual_m / dt);
 track.position_error_ema_m = std::max(
 kPositionUncertaintyFloorM,
 std::min(kPositionUncertaintyCapM,
 (1.0 - kUncertaintyBlend) * track.position_error_ema_m +
 kUncertaintyBlend * position_residual_m));
 track.velocity_error_ema_mps = std::max(
 kVelocityUncertaintyFloorMps,
 (1.0 - kUncertaintyBlend) * track.velocity_error_ema_mps +
 kUncertaintyBlend * velocity_residual_mps);

 // The reflection corridor represents observed support, so a filter's
 // inertial overshoot must never enlarge it.  In particular, a constant-
 // velocity estimate continues beyond a shuttle endpoint during its dwell;
 // treating that transient as measured space makes every later bounce leave
 // the physical lane. Observation uncertainty belongs in the safety envelope
 // (and eventually track covariance), not in a silently expanded endpoint.
 track.min_observed.x = std::min(track.min_observed.x, observation.position.x);
 track.min_observed.y = std::min(track.min_observed.y, observation.position.y);
 track.max_observed.x = std::max(track.max_observed.x, observation.position.x);
 track.max_observed.y = std::max(track.max_observed.y, observation.position.y);
 const double measured_speed = std::hypot(filtered_velocity.x, filtered_velocity.y);
 bool confirmed_reversal = false;
 if (measured_speed >= kReliableMotionSpeedMps) {
 if (!track.motion_velocity_initialized) {
 const double agreement = track.motion_candidate_velocity.x * filtered_velocity.x +
 track.motion_candidate_velocity.y * filtered_velocity.y;
 if (track.motion_confirmation_samples == 0 || agreement <= 0.0) {
 track.motion_candidate_velocity = filtered_velocity;
 track.motion_confirmation_samples = 1;
 } else {
 track.motion_candidate_velocity = {
 track.motion_candidate_velocity.x * (1.0 - kMotionMemoryBlend) +
 filtered_velocity.x * kMotionMemoryBlend,
 track.motion_candidate_velocity.y * (1.0 - kMotionMemoryBlend) +
 filtered_velocity.y * kMotionMemoryBlend,
 };
 ++track.motion_confirmation_samples;
 }
 if (track.motion_confirmation_samples >= kMotionConfirmationSamples) {
 track.last_motion_velocity = track.motion_candidate_velocity;
 track.motion_velocity_initialized = true;
 track.motion_confirmation_samples = 0;
 }
 } else {
 const double direction_agreement = track.last_motion_velocity.x * filtered_velocity.x +
 track.last_motion_velocity.y * filtered_velocity.y;
 if (direction_agreement < -0.02) {
 confirmed_reversal = true;
 // A confirmed sign change identifies a shuttle boundary. Keep the
 // filtered turning point and switch the remembered continuation mode.
 const double corridor_x = track.max_observed.x - track.min_observed.x;
 const double corridor_y = track.max_observed.y - track.min_observed.y;
 if (corridor_x >= corridor_y) {
 track.bounded_x = true;
 } else {
 track.bounded_y = true;
 }
 track.last_motion_velocity = filtered_velocity;
 if (track.stationary_since_s >= 0.0) {
 const double observed_dwell_s = time_s - track.stationary_since_s;
 if (observed_dwell_s >= 0.20) {
 track.learned_endpoint_dwell_s = track.learned_endpoint_dwell_s <= 0.0
 ? observed_dwell_s
 : std::min(track.learned_endpoint_dwell_s, observed_dwell_s);
 }
 track.stationary_since_s = -1.0;
 }
 } else {
 track.last_motion_velocity = {
 track.last_motion_velocity.x * (1.0 - kMotionMemoryBlend) + filtered_velocity.x * kMotionMemoryBlend,
 track.last_motion_velocity.y * (1.0 - kMotionMemoryBlend) + filtered_velocity.y * kMotionMemoryBlend,
 };
 }
 }
 } else if (!track.motion_velocity_initialized) {
 track.motion_confirmation_samples = 0;
 track.motion_candidate_velocity = {};
 }
  if (track.motion_velocity_initialized && measured_speed < kIntentAmbiguousBelowMps) {
   track.intent_ambiguous = true;
   track.decisive_motion_samples = 0;
  } else if (track.intent_ambiguous) {
   if (measured_speed >= kIntentDecisiveAboveMps) {
    if (++track.decisive_motion_samples >= kIntentDecisiveSamples) {
     track.intent_ambiguous = false;
     track.decisive_motion_samples = 0;
    }
   } else {
    track.decisive_motion_samples = 0;
   }
  }
 if (measured_speed < kReliableMotionSpeedMps) {
 if (track.stationary_since_s < 0.0)
 track.stationary_since_s = time_s;
 } else if (track.stationary_since_s >= 0.0 && !confirmed_reversal) {
 // A mid-corridor slowdown followed by motion in the same direction is not
 // endpoint dwell evidence.
 track.stationary_since_s = -1.0;
 }
 track.track.state.p = filtered_position;
 track.track.state.v = filtered_velocity;
 track.track.state.radius = observation.radius;
 track.track.last_observed_s = time_s;
}

std::vector<DynamicTrack> DynamicObstacleMap::active_tracks(double time_s, double max_age_s) const {
 std::vector<DynamicTrack> result;
 for (const auto& stored : tracks_) {
  const double age = time_s - stored.track.last_observed_s;
  if (age < -1e-9 || age > max_age_s)
   continue;
  auto track = stored.track;
  track.state.p.x += track.state.v.x * age;
  track.state.p.y += track.state.v.y * age;
 track.motion_class = !stored.motion_velocity_initialized
 ? TrackMotionClass::Unconfirmed
 : (stored.intent_ambiguous ? TrackMotionClass::IntentAmbiguous : TrackMotionClass::Moving);
 track.learned_endpoint_dwell_s = stored.learned_endpoint_dwell_s;
  result.push_back(track);
 }
 return result;
}

std::vector<PredictInput> DynamicObstacleMap::predict_inputs() const {
 std::vector<PredictInput> inputs;
 for (const auto& stored : tracks_) {
 if (!stored.motion_velocity_initialized)
 continue;
  PredictInput input;
  input.state = stored.track.state;
 input.state.position_uncertainty_m = stored.position_error_ema_m;
 input.position_uncertainty_growth_mps = stored.velocity_error_ema_mps;
 input.position_uncertainty_cap_m = kPositionPredictionEnvelopeCapM;
  // Deceleration or a stop makes continuation and reversal both valid until
  // two decisive motion samples resolve intent. Hysteresis prevents one noisy
  // scan from granting a crossing release during endpoint dwell.
  if (stored.motion_velocity_initialized && stored.intent_ambiguous) {
   input.state.v = stored.last_motion_velocity;
 if (stored.bounded_x)
 input.state.p.x = stored.last_motion_velocity.x >= 0.0
 ? stored.max_observed.x
 : stored.min_observed.x;
 if (stored.bounded_y)
 input.state.p.y = stored.last_motion_velocity.y >= 0.0
 ? stored.max_observed.y
 : stored.min_observed.y;
   input.alternate_velocity = {
     -stored.last_motion_velocity.x,
     -stored.last_motion_velocity.y,
   };
   input.alternate_velocity_valid = true;
  }
  input.last_observed_s = stored.track.last_observed_s;
  input.min_observed = stored.min_observed;
  input.max_observed = stored.max_observed;
  input.bounded_x = stored.bounded_x;
  input.bounded_y = stored.bounded_y;
 const double corridor_x = stored.max_observed.x - stored.min_observed.x;
 const double corridor_y = stored.max_observed.y - stored.min_observed.y;
 // Long, narrow motion histories define a data-derived motion manifold.
 // Project tiny cross-track velocity onto that manifold so centimeter-scale
 // LiDAR centroid jitter cannot grow into meters of lateral drift over a
 // long behavior horizon.
 if (corridor_x >= 0.5 && corridor_x >= 3.0 * std::max(0.05, corridor_y)) {
 input.state.v.y = 0.0;
 input.alternate_velocity.y = 0.0;
 } else if (corridor_y >= 0.5 && corridor_y >= 3.0 * std::max(0.05, corridor_x)) {
 input.state.v.x = 0.0;
 input.alternate_velocity.x = 0.0;
 }
 input.endpoint_dwell_s = stored.learned_endpoint_dwell_s;
 if (stored.intent_ambiguous && stored.stationary_since_s >= 0.0) {
 const double elapsed = stored.track.last_observed_s - stored.stationary_since_s;
 input.initial_dwell_remaining_s =
 std::max(0.0, stored.learned_endpoint_dwell_s - elapsed);
 }
  inputs.push_back(input);
 }
 return inputs;
}

std::vector<std::vector<DynamicObjectState>>
DynamicObstacleMap::predict(double time_s, double horizon_s, double frame_dt_s, double max_age_s) const {
 // Thin delegation: all future-frame math lives in the predictor module.
 return predict_frames(predict_inputs(), time_s, horizon_s, frame_dt_s, max_age_s);
}

std::vector<std::vector<DynamicObjectState>> DynamicObstacleMap::predict_nominal(double time_s,
                                                                                 double horizon_s,
                                                                                 double frame_dt_s,
                                                                                 double max_age_s) const {
 auto inputs = predict_inputs();
 for (auto& input : inputs)
  input.alternate_velocity_valid = false;
 return predict_frames(inputs, time_s, horizon_s, frame_dt_s, max_age_s);
}

} // namespace sim
