#include "robot/autonomy/localization/global_map_localizer.hpp"
#include "robot/autonomy/localization/known_map_localizer.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ \
 << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

using sim::Vec2;
using sim::runtime::MapGrid;
using sim::runtime::MapSnapshot;

struct Lcg {
 std::uint32_t state{0x13572468U};

 double next() {
 state = state * 1664525U + 1013904223U;
 return static_cast<double>(state >> 8) / 16777216.0;
 }

 double uniform(double low, double high) {
 return low + (high - low) * next();
 }
};

MapSnapshot make_map(const std::string& id, double width_m, double height_m,
 double resolution_m) {
 MapSnapshot map;
 map.map_id = id;
 map.frame_id = "map";
 map.revision = 1;
 map.sample_time = sim::runtime::TimePoint::from_seconds(0.0);
 map.provenance = {"test", "localization-environments", "1"};
 MapGrid grid;
 grid.width = static_cast<std::uint32_t>(std::lround(width_m / resolution_m));
 grid.height = static_cast<std::uint32_t>(std::lround(height_m / resolution_m));
 grid.resolution_m = resolution_m;
 grid.origin_m = {0.0, 0.0};
 grid.occupancy.assign(static_cast<std::size_t>(grid.width) * grid.height, 0);
 map.grid = std::move(grid);
 return map;
}

void fill_box(MapGrid& grid, double x0, double y0, double x1, double y1) {
 const auto to_cell_x = [&](double value) {
 return static_cast<int>(std::floor((value - grid.origin_m.x) / grid.resolution_m));
 };
 const auto to_cell_y = [&](double value) {
 return static_cast<int>(std::floor((value - grid.origin_m.y) / grid.resolution_m));
 };
 const int gx0 = to_cell_x(x0);
 const int gx1 = to_cell_x(x1);
 const int gy0 = to_cell_y(y0);
 const int gy1 = to_cell_y(y1);
 for (int y = gy0; y <= gy1; ++y) {
 if (y < 0 || y >= static_cast<int>(grid.height)) continue;
 for (int x = gx0; x <= gx1; ++x) {
 if (x < 0 || x >= static_cast<int>(grid.width)) continue;
 grid.occupancy[static_cast<std::size_t>(y) * grid.width +
 static_cast<std::size_t>(x)] = 100;
 }
 }
}

sim::Scan scan_at(const MapGrid& grid, Vec2 pose, double heading, double stamp_s,
 double max_range = 8.0, int rays = 180) {
 sim::Scan scan;
 scan.n_rays = rays;
 scan.max_range = max_range;
 scan.angle_min_rad = -sim::kPi;
 scan.angle_increment_rad = 2.0 * sim::kPi / static_cast<double>(rays);
 scan.scan_period_s = 0.1;
 scan.header = {stamp_s, 1, "lidar_link", 0.0, sim::kMeasurementSchemaVersion,
 "test.localization-environments"};
 const double step = std::max(0.02, grid.resolution_m * 0.25);
 for (int index = 0; index < rays; ++index) {
 const double angle = scan.angle_min_rad + scan.angle_increment_rad * index;
 double range = scan.max_range;
 for (double distance = 0.05; distance < scan.max_range; distance += step) {
 const auto cell = sim::runtime::world_to_grid(
 grid, {pose.x + distance * std::cos(heading + angle),
 pose.y + distance * std::sin(heading + angle)});
 if (!cell) break;
 const auto value = grid.occupancy[static_cast<std::size_t>(cell->y) *
 grid.width + cell->x];
 if (value >= 50) {
 range = distance;
 break;
 }
 }
 scan.ranges.push_back(range);
 scan.hit.push_back(range < scan.max_range ? 1 : 0);
 scan.intensities.push_back(1.0);
 }
 return scan;
}

bool contains(const std::string& text, const std::string& token) {
 return text.find(token) != std::string::npos;
}

double position_error(const sim::autonomy::LocalizationResult& result,
 Vec2 truth) {
 return std::hypot(result.pose.position.x - truth.x,
 result.pose.position.y - truth.y);
}

double heading_error(const sim::autonomy::LocalizationResult& result,
 double truth_heading) {
 return std::abs(sim::wrap_angle(result.pose.heading_rad - truth_heading));
}

sim::KnownMapLocalizationConfig local_config() {
 sim::KnownMapLocalizationConfig config;
 config.search_translation_m = 0.75;
 config.search_heading_rad = 0.25;
 config.translation_step_m = 0.05;
 config.heading_step_rad = 0.035;
 config.minimum_overlap = 0.20;
 config.minimum_score_margin = 0.01;
 config.minimum_returns = 12;
 return config;
}

// Dense shelving with irregular gaps: a feature-rich indoor environment where
// a bounded local correction must converge onto the surveyed pose.
MapSnapshot warehouse_map() {
 auto map = make_map("warehouse-shelves", 14.0, 12.0, 0.1);
 auto& grid = *map.grid;
 fill_box(grid, 0.0, 0.0, 14.0, 0.2);
 fill_box(grid, 0.0, 11.8, 14.0, 12.0);
 fill_box(grid, 0.0, 0.0, 0.2, 12.0);
 fill_box(grid, 13.8, 0.0, 14.0, 12.0);
 fill_box(grid, 1.0, 3.0, 4.0, 3.2);
 fill_box(grid, 5.0, 3.0, 8.0, 3.2);
 fill_box(grid, 9.5, 3.0, 12.6, 3.2);
 fill_box(grid, 1.0, 6.0, 2.6, 6.2);
 fill_box(grid, 4.0, 6.0, 7.5, 6.2);
 fill_box(grid, 9.0, 6.0, 12.6, 6.2);
 fill_box(grid, 2.0, 9.0, 6.4, 9.2);
 fill_box(grid, 7.8, 9.0, 10.2, 9.2);
 fill_box(grid, 11.4, 9.0, 12.6, 9.2);
 fill_box(grid, 2.5, 7.2, 3.6, 8.4); // machine enclosure
 fill_box(grid, 10.2, 4.2, 10.7, 4.7); // stanchion
 fill_box(grid, 0.4, 4.4, 0.9, 5.1); // pallet
 return map;
}

int warehouse_shelves() {
 const auto map = warehouse_map();
 const Vec2 truth{6.0, 7.6};
 const double truth_heading = 0.62;
 const auto scan = scan_at(*map.grid, truth, truth_heading, 20.0);
 sim::KnownMapLocalizer localizer(local_config());
 const auto result =
 localizer.localize(scan, {6.28, 7.32}, 0.52, map, 20.0);
 std::cerr << "warehouse accepted=" << result.accepted
 << " pos_error=" << position_error(result, truth)
 << " heading_error=" << heading_error(result, truth_heading)
 << " overlap=" << result.overlap << " margin=" << result.score_margin
 << " reason=" << result.rejection_reason << "\n";
 CHECK(result.accepted);
 CHECK(result.state == sim::autonomy::LocalizationState::Localized);
 CHECK(position_error(result, truth) <= 0.12);
 CHECK(heading_error(result, truth_heading) <= 0.08);
 CHECK(result.usable_returns >= 60);
 CHECK(result.pose.header.source_id == "robot.known-map-lidar");
 return 0;
}

// A long corridor is translation invariant along its axis inside the sensor
// range. Sliding by a full search step produces the same scan, so a bounded
// matcher must report ambiguity instead of inventing a confident position.
int long_corridor() {
 auto map = make_map("long-corridor", 60.0, 6.0, 0.1);
 auto& grid = *map.grid;
 fill_box(grid, 0.0, 0.0, 60.0, 0.2);
 fill_box(grid, 0.0, 5.8, 60.0, 6.0);
 fill_box(grid, 0.0, 0.0, 0.2, 6.0);
 fill_box(grid, 59.8, 0.0, 60.0, 6.0);
 const Vec2 truth{12.0, 3.0};
 const auto scan = scan_at(grid, truth, 0.0, 21.0);
 sim::KnownMapLocalizer localizer(local_config());
 const auto result = localizer.localize(scan, {12.45, 3.10}, 0.05, map, 21.0);
 std::cerr << "corridor accepted=" << result.accepted
 << " overlap=" << result.overlap << " score=" << result.score
 << " margin=" << result.score_margin
 << " reason=" << result.rejection_reason << "\n";
 CHECK(!result.accepted);
 CHECK(result.state == sim::autonomy::LocalizationState::Degraded);
 CHECK(contains(result.rejection_reason, "ambiguous"));
 return 0;
}

// A hall that maps onto itself under a quarter turn: four identical pillars and
// a square boundary. Global recovery must not publish one of those equivalent
// poses as a unique map fix.
int symmetric_hall() {
 auto map = make_map("symmetric-hall", 12.0, 12.0, 0.1);
 auto& grid = *map.grid;
 fill_box(grid, 0.0, 0.0, 12.0, 0.2);
 fill_box(grid, 0.0, 11.8, 12.0, 12.0);
 fill_box(grid, 0.0, 0.0, 0.2, 12.0);
 fill_box(grid, 11.8, 0.0, 12.0, 12.0);
 fill_box(grid, 2.6, 2.6, 3.4, 3.4);
 fill_box(grid, 8.6, 2.6, 9.4, 3.4);
 fill_box(grid, 2.6, 8.6, 3.4, 9.4);
 fill_box(grid, 8.6, 8.6, 9.4, 9.4);
 const auto scan = scan_at(grid, {6.0, 6.0}, 0.40, 22.0, 8.0, 180);

 sim::GlobalMapLocalizationConfig config;
 config.coarse_field_resolution_m = 0.1;
 config.coarse_position_step_m = 0.30;
 config.coarse_heading_step_rad = sim::kPi / 6.0;
 config.minimum_overlap = 0.20;
 config.minimum_score_margin = 0.01;
 config.ambiguity_distance_m = 0.40;
 config.ambiguity_heading_rad = 0.40;
 config.minimum_returns = 12;
 config.maximum_returns = 96;
 config.maximum_evaluations = 400000;
 sim::GlobalMapLocalizer global(config);
 const auto result = global.localize_unknown_start(scan, map, 22.0);
 std::cerr << "symmetric accepted=" << result.accepted
 << " score=" << result.score << " margin=" << result.score_margin
 << " overlap=" << result.overlap
 << " reason=" << result.rejection_reason << "\n";
 CHECK(!result.accepted);
 CHECK(result.state == sim::autonomy::LocalizationState::Degraded);
 CHECK(contains(result.rejection_reason, "ambiguous"));
 return 0;
}

// An empty hangar: every beam runs past the maximum range. A matcher must
// report missing evidence instead of falling back to the odometry prior as a
// successful map fix.
int sparse_hangar() {
 auto map = make_map("sparse-hangar", 40.0, 40.0, 0.2);
 auto& grid = *map.grid;
 fill_box(grid, 0.0, 0.0, 40.0, 0.2);
 fill_box(grid, 0.0, 39.8, 40.0, 40.0);
 fill_box(grid, 0.0, 0.0, 0.2, 40.0);
 fill_box(grid, 39.8, 0.0, 40.0, 40.0);
 const auto scan = scan_at(grid, {20.0, 20.0}, 0.0, 23.0, 8.0, 180);
 sim::KnownMapLocalizer localizer(local_config());
 const auto result =
 localizer.localize(scan, {20.3, 19.8}, 0.1, map, 23.0);
 std::cerr << "sparse accepted=" << result.accepted
 << " returns=" << result.usable_returns
 << " reason=" << result.rejection_reason << "\n";
 CHECK(!result.accepted);
 CHECK(result.rejection_reason == "insufficient geometric returns");
 return 0;
}

// Dropouts, outliers and one unmodelled moving object must not pull an accepted
// fix away from the surveyed pose.
int occlusion_and_noise() {
 const auto map = warehouse_map();
 const Vec2 truth{6.0, 7.6};
 const double truth_heading = 0.62;
 auto scan = scan_at(*map.grid, truth, truth_heading, 24.0);
 Lcg rng;
 for (std::size_t index = 0; index < scan.ranges.size(); ++index) {
 const double draw = rng.next();
 if (draw < 0.15) {
 scan.hit[index] = 0;
 scan.ranges[index] = scan.max_range;
 } else if (draw < 0.21) {
 scan.ranges[index] = rng.uniform(0.6, 6.0);
 }
 }
 // A person-sized object crossing 2.1 m ahead of the robot in the front
 // quadrant. It is not in the surveyed map.
 const int centre = scan.n_rays / 2;
 for (int index = centre - 12; index < centre + 12; ++index) {
 if (scan.ranges[static_cast<std::size_t>(index)] > 2.1)
 scan.ranges[static_cast<std::size_t>(index)] = 2.1;
 }
 sim::KnownMapLocalizer localizer(local_config());
 const auto result = localizer.localize(scan, {6.24, 7.36}, 0.55, map, 24.0);
 std::cerr << "occlusion accepted=" << result.accepted
 << " pos_error=" << position_error(result, truth)
 << " heading_error=" << heading_error(result, truth_heading)
 << " overlap=" << result.overlap
 << " reason=" << result.rejection_reason << "\n";
 CHECK(result.accepted);
 CHECK(position_error(result, truth) <= 0.20);
 CHECK(heading_error(result, truth_heading) <= 0.10);

 // Severe corruption must never turn into a confident wrong pose: the
 // contract is either a rejection or a fix close to the surveyed pose.
 auto severe = scan_at(*map.grid, truth, truth_heading, 24.5);
 Lcg noisy;
 for (std::size_t index = 0; index < severe.ranges.size(); ++index) {
 const double draw = noisy.next();
 if (draw < 0.50) {
 severe.hit[index] = 0;
 severe.ranges[index] = severe.max_range;
 } else if (draw < 0.70) {
 severe.ranges[index] = noisy.uniform(0.5, 7.5);
 }
 }
 const auto severe_result =
 localizer.localize(severe, {6.3, 7.3}, 0.55, map, 24.5);
 std::cerr << "severe accepted=" << severe_result.accepted
 << " pos_error=" << position_error(severe_result, truth)
 << " reason=" << severe_result.rejection_reason << "\n";
 CHECK(!severe_result.accepted || position_error(severe_result, truth) <= 0.35);
 return 0;
}

// Multi-room facility: shelved storage on one side, a doorway, then a hall with
// repeated columns. Repeated structure is the environment most likely to invite
// a confident wrong match, so the gate counts wrong accepts explicitly.
MapSnapshot complex_facility() {
 auto map = make_map("complex-facility", 30.0, 24.0, 0.1);
 auto& grid = *map.grid;
 fill_box(grid, 0.0, 0.0, 30.0, 0.2);
 fill_box(grid, 0.0, 23.8, 30.0, 24.0);
 fill_box(grid, 0.0, 0.0, 0.2, 24.0);
 fill_box(grid, 29.8, 0.0, 30.0, 24.0);
 // Dividing wall with a 2 m doorway.
 fill_box(grid, 14.0, 0.2, 14.2, 8.0);
 fill_box(grid, 14.0, 10.0, 14.2, 23.8);
 // Aisles in the storage room.
 for (int row = 0; row < 5; ++row) {
 const double y = 4.0 + 4.0 * row;
 const double gap = 3.0 + 1.3 * row;
 fill_box(grid, 1.0, y, gap, y + 0.2);
 fill_box(grid, gap + 1.4, y, 13.4, y + 0.2);
 }
 fill_box(grid, 9.7, 5.6, 10.4, 6.6);
 fill_box(grid, 11.2, 13.4, 12.0, 14.3);
 // Column hall in the second room.
 for (int iy = 0; iy < 3; ++iy) {
 for (int ix = 0; ix < 4; ++ix) {
 const double x = 17.0 + 3.0 * ix;
 const double y = 4.0 + 4.0 * iy;
 fill_box(grid, x, y, x + 0.4, y + 0.4);
 }
 }
 fill_box(grid, 26.6, 20.4, 28.4, 21.8);
 fill_box(grid, 16.0, 17.6, 23.0, 17.9);
 return map;
}

int complex_facility_route() {
 const auto map = complex_facility();
 const std::vector<Vec2> route{
 {4.0, 2.5}, {6.5, 2.5}, {9.0, 2.5}, {11.5, 3.0}, {12.8, 6.0},
 {13.0, 9.0}, {15.0, 9.2}, {17.5, 9.5}, {20.5, 10.5}, {24.0, 11.0}};
 const std::vector<double> headings{0.0, 0.1, 0.2, 0.4, 0.9,
 1.5, 1.0, 0.4, 0.3, 0.2};
 sim::KnownMapLocalizer localizer(local_config());
 int accepted = 0;
 int rejected = 0;
 int wrong = 0;
 double worst_error = 0.0;
 for (std::size_t index = 0; index < route.size(); ++index) {
 const Vec2 truth = route[index];
 const double truth_heading = headings[index];
 const double stamp = 30.0 + 0.1 * static_cast<double>(index);
 const auto scan = scan_at(*map.grid, truth, truth_heading, stamp);
 const Vec2 prior{truth.x + 0.30, truth.y - 0.22};
 const auto result =
 localizer.localize(scan, prior, truth_heading + 0.10, map, stamp);
 if (!result.accepted) {
 ++rejected;
 std::cerr << "  facility pose " << index << " rejected: "
 << result.rejection_reason << " margin=" << result.score_margin
 << " overlap=" << result.overlap << "\n";
 continue;
 }
 ++accepted;
 const double error = position_error(result, truth);
 worst_error = std::max(worst_error, error);
 if (error > 0.50 || heading_error(result, truth_heading) > 0.35) {
 ++wrong;
 std::cerr << "  facility pose " << index << " WRONG accept error=" << error
 << " heading=" << heading_error(result, truth_heading) << "\n";
 }
 }
 // A kidnapped prior moves the whole scan outside the search window. The
 // matcher must not snap it onto a plausible-looking local minimum.
 const double last_stamp = 40.0;
 const auto kidnapped_scan = scan_at(*map.grid, {24.0, 11.0}, 0.2, last_stamp);
 const auto kidnapped = localizer.localize(
 kidnapped_scan, {29.0, 5.0}, 0.2, map, last_stamp);
 std::cerr << "facility accepted=" << accepted << " rejected=" << rejected
 << " wrong_accepts=" << wrong << " worst_error=" << worst_error
 << " kidnapped_accepted=" << kidnapped.accepted
 << " kidnapped_reason=" << kidnapped.rejection_reason << "\n";
 CHECK(wrong == 0);
 CHECK(accepted >= 8);
 CHECK(worst_error <= 0.35);
 CHECK(!kidnapped.accepted);
 return 0;
}

} // namespace

int main() {
 if (warehouse_shelves() != 0) return 1;
 if (long_corridor() != 0) return 1;
 if (symmetric_hall() != 0) return 1;
 if (sparse_hangar() != 0) return 1;
 if (occlusion_and_noise() != 0) return 1;
 if (complex_facility_route() != 0) return 1;
 std::cout << "localization_environments ok\n";
 return 0;
}

