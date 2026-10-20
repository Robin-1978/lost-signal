#include "robot/autonomy/perception/dynamic_obstacle_map.hpp"
#include "robot/autonomy/planning/local/predictor.hpp"

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
// Confirmed corridor speed is a process state, not the latest one-second
// LiDAR regression. At 10 Hz this gain gives roughly a two-second time
// constant for same-direction speed changes; sign reversals still replace
// the state immediately below. The innovation is retained in the prediction
// envelope, so smoothing does not turn acceleration into false certainty.
constexpr double kMotionMemoryBlend = 0.05;
constexpr double kIntentAmbiguousBelowMps = 0.40;
constexpr double kIntentDecisiveAboveMps = 0.48;
constexpr int kIntentAmbiguousSamples = 2;
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
// A dwell is positive evidence only when consecutive raw observations remain
// at the same endpoint. Keep this tighter than the tracker uncertainty: a
// noisy or sparsely sampled turn may under-estimate dwell (conservative), but
// motion away from an endpoint must never be re-labelled as a promised pause.
constexpr double kEndpointDwellPositionToleranceM = 0.02;

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

double observed_endpoint_dwell(
 const std::deque<TrackMotionSample>& samples,
 bool use_x_axis, double endpoint) {
 double first_s = -1.0;
 double last_s = -1.0;
 for (const auto& sample : samples) {
 const double coordinate = use_x_axis ? sample.position.x : sample.position.y;
 if (std::abs(coordinate - endpoint) > kEndpointDwellPositionToleranceM)
 continue;
 if (first_s < 0.0)
 first_s = sample.time_s;
 last_s = sample.time_s;
 }
 return first_s < 0.0 ? 0.0 : std::max(0.0, last_s - first_s);
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
 // Phase uncertainty is the observed variation of the fitted velocity, not
 // raw position innovation divided by one scan interval. The latter turns a
 // few centimetres of LiDAR centroid noise into a persistent 0.5 m/s speed
 // error. Once a corridor is known, only the along-corridor component changes
 // arrival phase; cross-track residual remains represented by position
 // uncertainty.
 const Vec2 velocity_innovation{
 filtered_velocity.x - (track.motion_velocity_initialized
 ? track.last_motion_velocity.x : track.track.state.v.x),
 filtered_velocity.y - (track.motion_velocity_initialized
 ? track.last_motion_velocity.y : track.track.state.v.y),
 };
 const double velocity_residual_mps = std::min(
 kVelocityUncertaintyCapMps,
 track.bounded_x
 ? std::abs(velocity_innovation.x)
 : track.bounded_y
 ? std::abs(velocity_innovation.y)
 : std::hypot(velocity_innovation.x, velocity_innovation.y));
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
 const bool use_x_axis = corridor_x >= corridor_y;
 const double endpoint = use_x_axis
 ? (track.last_motion_velocity.x >= 0.0
 ? track.max_observed.x : track.min_observed.x)
 : (track.last_motion_velocity.y >= 0.0
 ? track.max_observed.y : track.min_observed.y);
 const double observed_dwell_s = observed_endpoint_dwell(
 track.motion_samples, use_x_axis, endpoint);
 if (use_x_axis) {
 if (track.last_motion_velocity.x >= 0.0)
 track.max_x_endpoint_observed = true;
 else
 track.min_x_endpoint_observed = true;
 track.bounded_x = track.min_x_endpoint_observed &&
 track.max_x_endpoint_observed;
 } else {
 if (track.last_motion_velocity.y >= 0.0)
 track.max_y_endpoint_observed = true;
 else
 track.min_y_endpoint_observed = true;
 track.bounded_y = track.min_y_endpoint_observed &&
 track.max_y_endpoint_observed;
 }
 // One scalar dwell model is shared by both corridor endpoints. Its only
 // safe interpretation is the minimum observed dwell: a pause learned at
 // one end cannot be promised at the other. An immediate reversal therefore
 // contributes zero and remains the conservative bound for this track. Use
 // raw endpoint occupancy rather than filtered low speed: filter lag can make
 // an immediate reversal look like a pause that never physically occurred.
 track.learned_endpoint_dwell_s = track.endpoint_dwell_observed
 ? std::min(track.learned_endpoint_dwell_s, observed_dwell_s)
 : observed_dwell_s;
 track.endpoint_dwell_observed = true;
 track.last_motion_velocity = filtered_velocity;
 track.stationary_since_s = -1.0;
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
 if (track.motion_velocity_initialized && !track.intent_ambiguous &&
 measured_speed < kIntentAmbiguousBelowMps) {
 if (++track.ambiguous_motion_samples >= kIntentAmbiguousSamples) {
 track.intent_ambiguous = true;
 track.ambiguous_motion_samples = 0;
 track.decisive_motion_samples = 0;
 }
  } else if (track.intent_ambiguous) {
 track.ambiguous_motion_samples = 0;
   if (measured_speed >= kIntentDecisiveAboveMps) {
    if (++track.decisive_motion_samples >= kIntentDecisiveSamples) {
     track.intent_ambiguous = false;
     track.decisive_motion_samples = 0;
    }
   } else {
    track.decisive_motion_samples = 0;
   }
 } else {
 track.ambiguous_motion_samples = 0;
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

std::vector<DynamicAssociationHypothesis>
DynamicObstacleMap::association_hypotheses(
 double time_s, double max_age_s) const {
 std::vector<DynamicAssociationHypothesis> result;
 const auto inputs = predict_inputs();
 std::size_t input_index = 0;
 for (const auto& stored : tracks_) {
 const PredictInput* input = nullptr;
 if (stored.motion_velocity_initialized) {
 if (input_index >= inputs.size())
 break;
 input = &inputs[input_index++];
 }
 const double age = time_s - stored.track.last_observed_s;
 if (age < -1e-9 || age > max_age_s)
 continue;
 if (!input) {
 auto state = stored.track.state;
 state.p.x += state.v.x * age;
 state.p.y += state.v.y * age;
 result.push_back({
 stored.track.object_id, state, stored.track.last_observed_s});
 continue;
 }
 // Reuse the production predictor so association and planning reflect at
 // exactly the same learned endpoints and honor the same dwell/intent modes.
 const auto frames = predict_frames({*input}, time_s, 0.0, 1.0, max_age_s);
 if (frames.empty())
 continue;
 for (const auto& state : frames.front()) {
 result.push_back({
 stored.track.object_id, state, stored.track.last_observed_s});
 }
 }
 return result;
}

std::vector<PredictInput> DynamicObstacleMap::predict_inputs() const {
 std::vector<PredictInput> inputs;
 for (const auto& stored : tracks_) {
  PredictInput input;
 input.track_id = stored.track.object_id;
  input.state = stored.track.state;
 input.state.position_uncertainty_m = stored.position_error_ema_m;
 const bool bounded_motion = stored.bounded_x || stored.bounded_y;
 const bool enough_history = stored.motion_samples.size() >= 5 &&
 stored.motion_samples.back().time_s -
 stored.motion_samples.front().time_s >= 0.4;
 // Ambiguous intent does not make the estimate unusable: the predictor
 // publishes each retained mode and downstream safety checks their union.
 // Readiness only asserts sufficient time history and must not collapse
 // stop/reverse ambiguity into a false absence of evidence.
 input.state.prediction_ready = enough_history;
 if (bounded_motion && stored.motion_velocity_initialized) {
 // Once a motion manifold is observed, velocity error changes phase along
 // that manifold. Slow/nominal/fast hypotheses represent it without
 // inventing equal lateral motion away from the lane.
 // Use the confirmed motion-model velocity as the forecast mean. The raw
 // one-second regression remains useful for reversal detection, but its
 // view-dependent centroid slope can move by more than 0.1 m/s from one
 // scan to the next. Feeding that value directly into every new forecast
 // changes a periodic actor's phase discontinuously and invalidates a
 // crossing proof even though the confirmed motion has not changed.
 input.state.v = stored.last_motion_velocity;
 input.velocity_uncertainty_mps = stored.velocity_error_ema_mps;
 } else {
 // Before a manifold is known, direction itself is uncertain. Radial growth
 // is conservative for short-horizon local avoidance. Stationary tracks are
 // retained too: a temporary stopped pallet or vehicle must not disappear
 // merely because no non-zero motion velocity has been initialized.
 input.position_uncertainty_growth_mps = stored.velocity_error_ema_mps;
 }
 // Do not truncate velocity uncertainty at a fixed radial distance. A
 // 10-centimetre cap made a six-second crossing look certain even when the
 // measured velocity error could move a shuttle's reflected phase by several
 // decimetres. The predictor already grows this deterministic envelope with
 // forecast time; a future covariance provider may replace it with a
 // calibrated confidence region, but horizon growth must remain visible to
 // every safety consumer.
  // Deceleration or a stop makes continuation and reversal both valid until
  // two decisive motion samples resolve intent. Hysteresis prevents one noisy
  // scan from granting a crossing release during endpoint dwell.
  if (stored.motion_velocity_initialized && stored.intent_ambiguous) {
   input.state.v = stored.last_motion_velocity;
 // Ambiguous intent changes the velocity hypotheses, never the measured
 // position. During reversal hysteresis the target may already be moving
 // through the corridor; snapping it to the endpoint selected by the last
 // velocity sign teleports every hypothesis across the scene. Only a raw
 // observation that is actually dwelling at a learned endpoint may anchor
 // the seed there to remove alpha-beta filter overshoot.
 if (stored.stationary_since_s >= 0.0 && !stored.motion_samples.empty()) {
 const Vec2 measured = stored.motion_samples.back().position;
 if (stored.bounded_x) {
 const double to_min = std::abs(measured.x - stored.min_observed.x);
 const double to_max = std::abs(measured.x - stored.max_observed.x);
 if (std::min(to_min, to_max) <= kEndpointDwellPositionToleranceM) {
 input.state.p.x = to_min <= to_max
 ? stored.min_observed.x
 : stored.max_observed.x;
 }
 }
 if (stored.bounded_y) {
 const double to_min = std::abs(measured.y - stored.min_observed.y);
 const double to_max = std::abs(measured.y - stored.max_observed.y);
 if (std::min(to_min, to_max) <= kEndpointDwellPositionToleranceM) {
 input.state.p.y = to_min <= to_max
 ? stored.min_observed.y
 : stored.max_observed.y;
 }
 }
 }
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
 return flatten_prediction_frames(
 predict_typed(time_s, horizon_s, frame_dt_s, max_age_s));
}

PredictionBatch DynamicObstacleMap::predict_typed(
 double time_s, double horizon_s, double frame_dt_s,
 double max_age_s) const {
 return predict_hypotheses(
 predict_inputs(), time_s, horizon_s, frame_dt_s, max_age_s);
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
