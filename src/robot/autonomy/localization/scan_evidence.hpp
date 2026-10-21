#pragma once

#include "robot/runtime/map_provider.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

namespace sim::localization {

struct Beam { double range{0.0}; double angle{0.0}; bool hit{false}; };

inline std::vector<Beam> collect_beams(const Scan& scan, std::size_t maximum,
 std::size_t* hit_returns) {
 std::vector<Beam> beams;
 beams.reserve(scan.ranges.size());
 std::size_t hits = 0;
 for (std::size_t i = 0; i < scan.ranges.size(); ++i) {
 const double range = scan.ranges[i];
 if (!std::isfinite(range) || range <= 0.05) continue;
 const bool hit = scan.hit[i] && range < scan.max_range;
 beams.push_back({std::min(range, scan.max_range), scan.angle_min_rad +
 scan.angle_increment_rad * static_cast<double>(i), hit});
 if (hit) ++hits;
 }
 if (hit_returns) *hit_returns = hits;
 if (maximum > 0 && beams.size() > maximum) {
 const std::size_t stride = (beams.size() + maximum - 1) / maximum;
 std::vector<Beam> selected;
 selected.reserve(maximum);
 for (std::size_t i = 0; i < beams.size() && selected.size() < maximum; i += stride)
 selected.push_back(beams[i]);
 beams = std::move(selected);
 }
 return beams;
}

inline Vec2 rotate(Vec2 p, double heading) {
 const double c = std::cos(heading); const double s = std::sin(heading);
 return {c * p.x - s * p.y, s * p.x + c * p.y};
}

inline Vec2 sensor_position(Vec2 base, double heading, double sensor_x,
 double sensor_y) {
 const auto offset = rotate({sensor_x, sensor_y}, heading);
 return {base.x + offset.x, base.y + offset.y};
}

inline double endpoint_score(const runtime::MapGrid& grid, Vec2 point) {
 const double c = std::cos(grid.origin_yaw_rad); const double s = std::sin(grid.origin_yaw_rad);
 const double dx = point.x - grid.origin_m.x; const double dy = point.y - grid.origin_m.y;
 const int x = static_cast<int>(std::floor((c * dx + s * dy) / grid.resolution_m));
 const int y = static_cast<int>(std::floor((-s * dx + c * dy) / grid.resolution_m));
 if (x < 0 || y < 0 || x >= static_cast<int>(grid.width) || y >= static_cast<int>(grid.height)) return -0.25;
 const auto index = static_cast<std::size_t>(y) * grid.width + static_cast<std::size_t>(x);
 if (index >= grid.occupancy.size()) return -0.25;
 const int value = static_cast<int>(grid.occupancy[index]);
 if (value < 0) return 0.0;
 return value >= 50 ? 1.0 : -0.20;
}

struct PoseEvidence { double score{0.0}; double overlap{0.0}; std::size_t known{0}; };

inline PoseEvidence score_pose(const runtime::MapGrid& grid, const std::vector<Beam>& beams,
 Vec2 base, double heading, double sensor_x, double sensor_y, double sensor_yaw,
 std::size_t hit_returns, bool include_free_rays) {
 (void)include_free_rays;
 const auto sensor = sensor_position(base, heading, sensor_x, sensor_y);
 PoseEvidence result; std::size_t matches = 0;
 for (const auto& beam : beams) {
 if (!beam.hit) continue;
 const auto local = rotate({beam.range * std::cos(beam.angle + sensor_yaw),
 beam.range * std::sin(beam.angle + sensor_yaw)}, heading);
 const double value = endpoint_score(grid, {sensor.x + local.x, sensor.y + local.y});
 result.score += value; ++result.known;
 if (value > 0.0) ++matches;
 }
 if (result.known > 0) result.score /= static_cast<double>(result.known);
 result.overlap = hit_returns > 0 ? static_cast<double>(matches) / hit_returns : 0.0;
 return result;
}

} // namespace sim::localization
