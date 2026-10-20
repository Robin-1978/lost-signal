#pragma once

#include "robot/autonomy/perception/dynamic_object.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace sim {

struct DynamicClearance {
 double physical_m{std::numeric_limits<double>::infinity()};
 double conservative_m{std::numeric_limits<double>::infinity()};
};

// Interpolate every predicted object to the candidate's exact relative time.
// Predictor frames preserve object/mode order, so matching indices describe
// the same hypothesis in adjacent frames. Radius is conservatively bounded by
// the larger endpoint while position is linearly interpolated.
inline DynamicClearance dynamic_clearance_at(
 Vec2 ego, double time_s,
 const std::vector<std::vector<DynamicObjectState>>& frames,
 double frame_dt_s, double ego_radius_m) {
 DynamicClearance out;
 if (frames.empty())
 return out;
 const double frame_position = std::max(0.0, time_s) /
 std::max(1e-6, frame_dt_s);
 const std::size_t before = std::min(
 frames.size() - 1, static_cast<std::size_t>(std::floor(frame_position)));
 const std::size_t after = std::min(frames.size() - 1, before + 1);
 const double blend = std::clamp(
 frame_position - static_cast<double>(before), 0.0, 1.0);
 const std::size_t common = std::min(frames[before].size(), frames[after].size());
 for (std::size_t index = 0; index < common; ++index) {
 const auto& a = frames[before][index];
 const auto& b = frames[after][index];
 const Vec2 object{
 a.p.x + (b.p.x - a.p.x) * blend,
 a.p.y + (b.p.y - a.p.y) * blend,
 };
 const double center_distance = std::hypot(ego.x - object.x, ego.y - object.y);
 const double physical_radius = std::max(a.radius, b.radius);
 const double occupied_radius = std::max(
 a.occupied_radius_m(), b.occupied_radius_m());
 out.physical_m = std::min(
 out.physical_m, center_distance - ego_radius_m - physical_radius);
 out.conservative_m = std::min(
 out.conservative_m, center_distance - ego_radius_m - occupied_radius);
 }
 return out;
}

// Clearance of a brake-and-hold pose over the complete maintained forecast.
// A staged rotate-then-translate recovery may prefer to turn in place, but
// that preference cannot become a hard zero-speed constraint when a moving
// object is predicted to invade the stationary footprint.
inline DynamicClearance minimum_stationary_clearance(
 Vec2 ego, const std::vector<std::vector<DynamicObjectState>>& frames,
 double frame_dt_s, double ego_radius_m) {
 DynamicClearance out;
 for (std::size_t index = 0; index < frames.size(); ++index) {
 const auto sample = dynamic_clearance_at(
 ego, static_cast<double>(index) * frame_dt_s,
 frames, frame_dt_s, ego_radius_m);
 out.physical_m = std::min(out.physical_m, sample.physical_m);
 out.conservative_m = std::min(
 out.conservative_m, sample.conservative_m);
 }
 return out;
}

} // namespace sim
