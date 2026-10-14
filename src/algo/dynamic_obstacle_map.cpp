#include "dynamic_obstacle_map.hpp"
#include "predictor.hpp"

#include <algorithm>
#include <cmath>

namespace sim {
namespace {

constexpr double kVelocityWindowS = 0.20;
constexpr double kVelocityBlend = 0.55;
constexpr double kMaxTrackedSpeedMps = 3.0;

Vec2 limited_velocity(Vec2 velocity) {
 const double speed = std::hypot(velocity.x, velocity.y);
 if (speed <= kMaxTrackedSpeedMps || speed < 1e-9)
  return velocity;
 const double scale = kMaxTrackedSpeedMps / speed;
 return {velocity.x * scale, velocity.y * scale};
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
  created.velocity_anchor_position = observation.position;
  created.velocity_anchor_time_s = time_s;
  created.min_observed = observation.position;
  created.max_observed = observation.position;
  tracks_.push_back(created);
  return;
 }

 auto& track = *found;
 track.min_observed.x = std::min(track.min_observed.x, observation.position.x);
 track.min_observed.y = std::min(track.min_observed.y, observation.position.y);
 track.max_observed.x = std::max(track.max_observed.x, observation.position.x);
 track.max_observed.y = std::max(track.max_observed.y, observation.position.y);
 const double velocity_dt = time_s - track.velocity_anchor_time_s;
 if (velocity_dt >= kVelocityWindowS) {
  const Vec2 measured_velocity = limited_velocity({
    (observation.position.x - track.velocity_anchor_position.x) / velocity_dt,
    (observation.position.y - track.velocity_anchor_position.y) / velocity_dt,
  });
  if (track.velocity_initialized) {
   const double direction_agreement =
     track.track.state.v.x * measured_velocity.x + track.track.state.v.y * measured_velocity.y;
   if (direction_agreement < -0.02) {
    if (std::abs(track.track.state.v.x) >= std::abs(track.track.state.v.y)) {
     track.bounded_x = true;
    } else {
     track.bounded_y = true;
    }
    track.track.state.v = measured_velocity;
   } else {
    track.track.state.v = {
      track.track.state.v.x * (1.0 - kVelocityBlend) + measured_velocity.x * kVelocityBlend,
      track.track.state.v.y * (1.0 - kVelocityBlend) + measured_velocity.y * kVelocityBlend,
    };
   }
  } else {
   track.track.state.v = measured_velocity;
   track.velocity_initialized = true;
  }
  track.velocity_anchor_position = observation.position;
  track.velocity_anchor_time_s = time_s;
 }
 track.track.state.p = observation.position;
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
  result.push_back(track);
 }
 return result;
}

std::vector<PredictInput> DynamicObstacleMap::predict_inputs() const {
 std::vector<PredictInput> inputs;
 for (const auto& stored : tracks_) {
  PredictInput input;
  input.state = stored.track.state;
  input.last_observed_s = stored.track.last_observed_s;
  input.min_observed = stored.min_observed;
  input.max_observed = stored.max_observed;
  input.bounded_x = stored.bounded_x;
  input.bounded_y = stored.bounded_y;
  inputs.push_back(input);
 }
 return inputs;
}

std::vector<std::vector<DynState>> DynamicObstacleMap::predict(double time_s, double horizon_s,
                                                               double frame_dt_s, double max_age_s) const {
 // Thin delegation: all future-frame math lives in the predictor module.
 return predict_frames(predict_inputs(), time_s, horizon_s, frame_dt_s, max_age_s);
}

} // namespace sim
