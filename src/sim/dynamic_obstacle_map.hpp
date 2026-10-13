#pragma once

#include "dynobs.hpp"

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
 DynState state;
 double last_observed_s{0.0};
};

// Transient local map for moving objects. Static A* occupancy remains separate
// so a moving object cannot leave permanent ghost cells in the global map.
class DynamicObstacleMap {
public:
 void clear();
 void observe(const DynamicObservation& observation, double time_s);
 std::vector<DynamicTrack> active_tracks(double time_s, double max_age_s) const;
 std::vector<std::vector<DynState>> predict(
 double time_s, double horizon_s, double frame_dt_s, double max_age_s) const;

private:
 struct TrackState {
 DynamicTrack track;
 Vec2 velocity_anchor_position;
 double velocity_anchor_time_s{0.0};
 bool velocity_initialized{false};
 Vec2 min_observed;
 Vec2 max_observed;
 bool bounded_x{false};
 bool bounded_y{false};
 };

 std::vector<TrackState> tracks_;
};

} // namespace sim
