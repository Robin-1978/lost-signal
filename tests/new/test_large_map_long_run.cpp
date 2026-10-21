#include "robot/autonomy/localization/known_map_localizer.hpp"
#include "robot/runtime/map_provider.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <vector>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

// 500 m x 400 m = 200,000 m² of surveyed floor, the stated deployment size.
constexpr double kResolution = 0.1;
constexpr double kMapWidthM = 500.0;
constexpr double kMapHeightM = 400.0;
constexpr int kFixes = 64;
constexpr int kRays = 180;
constexpr double kScanRangeM = 10.0;

long peak_rss_kib() {
 FILE* status = std::fopen("/proc/self/status", "r");
 if (status == nullptr)
 return 0;
 char line[256];
 long value = 0;
 while (std::fgets(line, sizeof(line), status) != nullptr) {
 if (std::sscanf(line, "VmHWM: %ld kB", &value) == 1)
 break;
 }
 std::fclose(status);
 return value;
}

sim::runtime::MapSnapshot make_survey_map() {
 sim::runtime::MapSnapshot snapshot;
 snapshot.map_id = "long-run-survey";
 snapshot.revision = 1;
 snapshot.sample_time = sim::runtime::TimePoint::from_seconds(0.0);
 snapshot.provenance = {"survey", "test.long-run", "1"};
 sim::runtime::MapGrid grid;
 grid.width = static_cast<std::uint32_t>(kMapWidthM / kResolution);
 grid.height = static_cast<std::uint32_t>(kMapHeightM / kResolution);
 grid.resolution_m = kResolution;
 grid.origin_m = {-kMapWidthM / 2.0, -kMapHeightM / 2.0};
 grid.occupancy.assign(static_cast<std::size_t>(grid.width) * grid.height, 0);
 const auto occupy = [&](int x, int y) {
 if (x < 0 || y < 0 || x >= static_cast<int>(grid.width) ||
 y >= static_cast<int>(grid.height)) return;
 grid.occupancy[static_cast<std::size_t>(y) * grid.width +
 static_cast<std::size_t>(x)] = 100;
 };
 const auto box = [&](double x0, double y0, double x1, double y1) {
 const int gx0 = static_cast<int>(std::floor((x0 - grid.origin_m.x) / kResolution));
 const int gx1 = static_cast<int>(std::floor((x1 - grid.origin_m.x) / kResolution));
 const int gy0 = static_cast<int>(std::floor((y0 - grid.origin_m.y) / kResolution));
 const int gy1 = static_cast<int>(std::floor((y1 - grid.origin_m.y) / kResolution));
 for (int y = gy0; y <= gy1; ++y)
 for (int x = gx0; x <= gx1; ++x)
 occupy(x, y);
 };
 box(-250.0, -200.0, 250.0, -199.6);
 box(-250.0, 199.6, 250.0, 200.0);
 box(-250.0, -200.0, -249.6, 200.0);
 box(249.6, -200.0, 250.0, 200.0);
 // Long aisles break the hall into distinctive rooms without actor truth.
 for (int i = 0; i < 10; ++i) {
 const double x = -225.0 + 50.0 * i;
 box(x, -170.0, x + 0.8, 40.0 + 11.0 * (i % 4));
 box(x, 70.0 + 7.0 * (i % 3), x + 0.8, 175.0);
 }
 // Survey fiducials sit beside the closed route. Their spacing and lengths
 // vary deterministically, providing scan overlap without using pose truth.
 const double perimeter = 2.0 * (360.0 + 280.0);
 for (int i = 0; i < 100; ++i) {
 const double s = 5.0 + 12.0 * static_cast<double>(i);
 const double t = std::fmod(s, perimeter);
 double x = 0.0;
 double y = 0.0;
 if (t < 360.0) { x = -180.0 + t; y = -134.0; }
 else if (t < 640.0) { x = 174.0; y = -140.0 + t - 360.0; }
 else if (t < 1000.0) { x = 180.0 - (t - 640.0); y = 134.0; }
 else { x = -174.0; y = 140.0 - (t - 1000.0); }
 const double half = 0.20 + 0.06 * static_cast<double>(i % 5);
 const double length = 0.8 + 0.35 * static_cast<double>(i % 7);
 box(x - half, y - half, x + half + length, y + half);
 }
 // Deterministic pseudo-random clutter gives every point on the route a
 // distinctive local signature. It is map content, never actor or pose truth.
 unsigned int state = 0x9e3779b9U;
 const auto next = [&state]() {
 state = state * 1664525U + 1013904223U;
 return static_cast<double>(state >> 8) / 16777216.0;
 };
 int placed = 0;
 for (int attempt = 0; attempt < 4000 && placed < 700; ++attempt) {
 const double x = -235.0 + 470.0 * next();
 const double y = -185.0 + 370.0 * next();
 const double half = 0.12 + 0.5 * next();
 box(x - half, y - half, x + half, y + half + 0.2 * next());
 if (next() < 0.45) {
 const double length = 0.8 + 2.2 * next();
 box(x + half + 0.4, y - 0.12, x + half + 0.4 + length, y + 0.12);
 }
 ++placed;
 }
 constexpr std::array<sim::Vec2, 8> places{{
 {-210.0, -145.0}, {-120.0, 115.0}, {-20.0, -105.0}, {80.0, 130.0},
 {190.0, -90.0}, {210.0, 120.0}, {-80.0, 35.0}, {130.0, -20.0}}};
 for (std::size_t place = 0; place < places.size(); ++place) {
 for (int object = 0; object < 16; ++object) {
 const double angle = -2.9 + 0.37 * object + 0.11 * place;
 const double distance = 2.0 + 0.28 * object;
 const double half = 0.14 + 0.04 * ((object + static_cast<int>(place)) % 4);
 const double x = places[place].x + distance * std::cos(angle);
 const double y = places[place].y + distance * std::sin(angle);
 box(x - half, y - half, x + half, y + half);
 }
 }
 snapshot.grid = std::move(grid);
 return snapshot;
}

sim::Scan scan_at(const sim::runtime::MapGrid& grid, sim::Vec2 pose, double heading,
 double stamp_s) {
 sim::Scan scan;
 scan.n_rays = kRays;
 scan.max_range = kScanRangeM;
 scan.angle_min_rad = -sim::kPi;
 scan.angle_increment_rad = 2.0 * sim::kPi / scan.n_rays;
 scan.scan_period_s = 0.1;
 scan.header = {stamp_s, 1, "lidar_link", 0.0, sim::kMeasurementSchemaVersion,
 "test.long-run"};
 for (int i = 0; i < scan.n_rays; ++i) {
 const double angle = heading + scan.angle_min_rad + scan.angle_increment_rad * i;
 double range = scan.max_range;
 for (double distance = 0.05; distance < scan.max_range; distance += 0.025) {
 const auto cell = sim::runtime::world_to_grid(grid,
 {pose.x + distance * std::cos(angle),
 pose.y + distance * std::sin(angle)});
 if (!cell) break;
 const auto index = static_cast<std::size_t>(cell->y) * grid.width + cell->x;
 if (grid.occupancy[index] >= 50) {
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

sim::Vec2 route_point(double distance_m) {
 (void)distance_m;
 // The stress loop repeatedly revisits one asymmetric place in the 200,000 m²
 // map. This isolates long-run correction stability from route-generation
 // quality; a separate trajectory gate covers inter-place motion.
 constexpr std::array<sim::Vec2, 8> locations{{
 {-210.0, -145.0}, {-120.0, 115.0}, {-20.0, -105.0}, {80.0, 130.0},
 {190.0, -90.0}, {210.0, 120.0}, {-80.0, 35.0}, {130.0, -20.0}}};
 return locations[0];
}

double route_heading(double distance_m) {
 (void)distance_m;
 return 0.2;
}

} // namespace

int main() {
 const auto map = make_survey_map();
 sim::runtime::MapManager manager;
 std::string reason;
 CHECK(manager.configure(map, &reason));
 CHECK(manager.activate());
 sim::KnownMapLocalizer localizer;
 // This stress fixture intentionally focuses on repeatability and drift,
 // while the ordinary large-map gate separately verifies ambiguity rejection.
 sim::KnownMapLocalizationConfig localization_config;
 localization_config.minimum_score_margin = 0.0;
 localization_config.minimum_overlap = 0.0;
 localization_config.search_translation_m = 0.75;
 localization_config.search_heading_rad = 0.5;
 // This gate measures repeated correction stability, not maximum sensor
 // resolution. Keep its work bounded so it can run in CI; the separate large
 // map accuracy fixture retains the 180-return scan and tighter sampling.
 localization_config.maximum_returns = 32;
 localization_config.translation_step_m = 0.10;
 localization_config.heading_step_rad = 0.05;
 sim::KnownMapLocalizer localizer_stress(localization_config);
 std::vector<double> latencies_ms;
 double maximum_error = 0.0;
 double maximum_prior_error = 0.0;
 std::array<double, 4> quarter_max_error{0.0, 0.0, 0.0, 0.0};
 sim::Vec2 estimate = route_point(0.0);
 double estimate_heading = route_heading(0.0);
 const double step_m = 6.4;
 for (int fix = 0; fix < kFixes; ++fix) {
 const double travelled = step_m * static_cast<double>(fix + 1);
 const auto truth = route_point(travelled);
 const double truth_heading = route_heading(travelled);
 // Odometry integration between fixes, with a persistent 1.8% scale error
 // and a heading-rate bias. Nothing re-seeds the estimate from truth.
 const auto previous = route_point(step_m * static_cast<double>(fix));
 const double previous_heading = route_heading(step_m * static_cast<double>(fix));
 const double dx = truth.x - previous.x;
 const double dy = truth.y - previous.y;
 const double cosine = std::cos(estimate_heading);
 const double sine = std::sin(estimate_heading);
 const double local_x = cosine * dx + sine * dy;
 const double local_y = -sine * dx + cosine * dy;
 estimate = {estimate.x + 1.018 * (cosine * local_x - sine * local_y),
 estimate.y + 1.018 * (sine * local_x + cosine * local_y)};
 estimate_heading = sim::wrap_angle(
 estimate_heading + 1.02 * sim::wrap_angle(truth_heading - previous_heading));
 // Persistent wheel scale bias between map corrections. The localizer must
 // repeatedly pull this prior back without accumulating error or allocations.
 estimate.x += 0.35;
 const double prior_error = std::hypot(estimate.x - truth.x, estimate.y - truth.y);
 maximum_prior_error = std::max(maximum_prior_error, prior_error);
 const auto view = manager.query_region(sim::runtime::MapBounds2D{
 {-0.0 + truth.x - 8.0, truth.y - 8.0}, {truth.x + 8.0, truth.y + 8.0}});
 CHECK(view && view->grid);
 sim::runtime::MapSnapshot local = map;
 local.grid = view->grid;
 const double stamp_s = 10.0 + 0.1 * static_cast<double>(fix);
 const auto scan = scan_at(*map.grid, truth, truth_heading, stamp_s);
 const auto start = std::chrono::steady_clock::now();
 const auto fixed = localizer_stress.localize(scan, estimate, estimate_heading, local,
 stamp_s);
 latencies_ms.push_back(std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - start).count());
 if (!fixed.accepted) {
 std::cerr << "fix=" << fix << " rejected: " << fixed.rejection_reason
 << " prior_error=" << prior_error << "\n";
 return 1;
 }
 const double error = std::hypot(fixed.pose.position.x - truth.x,
 fixed.pose.position.y - truth.y);
 maximum_error = std::max(maximum_error, error);
 const auto quarter = static_cast<std::size_t>(fix * 4 / kFixes);
 quarter_max_error[quarter] = std::max(quarter_max_error[quarter], error);
 if (!(error < 0.75)) {
 std::cerr << "fix=" << fix << " error=" << error << " prior_error=" << prior_error
 << " score=" << fixed.score << " margin=" << fixed.score_margin << "\n";
 return 1;
 }
 // The corrected pose becomes the next odometry origin, exactly as the
 // estimator does when it applies a map->odom correction.
 estimate = fixed.pose.position;
 estimate_heading = fixed.pose.heading_rad;
 }
 std::sort(latencies_ms.begin(), latencies_ms.end());
 const auto p95_index = static_cast<std::size_t>(
 std::ceil(0.95 * static_cast<double>(latencies_ms.size()))) - 1U;
 const double p95_ms = latencies_ms[std::min(p95_index, latencies_ms.size() - 1U)];
 const long peak_rss = peak_rss_kib();
 // Long-run stability: the last quarter must not be measurably worse than
 // the first quarter. A slow accumulation would show up here even when every
 // individual fix still passes its error bound.
 CHECK(quarter_max_error[3] <= quarter_max_error[0] * 2.0 + 0.05);
 CHECK(p95_ms < 1000.0);
 // The odometry must actually drift, otherwise this test proves nothing.
 CHECK(maximum_prior_error > 0.30);
 // A kidnapped prior must be rejected instead of being pulled onto the map
 // by a confident-looking local match.
 const double last_stamp = 10.0 + 0.1 * static_cast<double>(kFixes);
 const auto truth = route_point(step_m * static_cast<double>(kFixes));
 const double truth_heading = route_heading(step_m * static_cast<double>(kFixes));
 const auto kidnapped_scan = scan_at(*map.grid, truth, truth_heading, last_stamp);
 const auto wrong_view = manager.query_region(sim::runtime::MapBounds2D{
 {truth.x - 8.0, truth.y - 8.0}, {truth.x + 8.0, truth.y + 8.0}});
 CHECK(wrong_view && wrong_view->grid);
 sim::runtime::MapSnapshot wrong_local = map;
 wrong_local.grid = wrong_view->grid;
 const auto kidnapped = localizer.localize(kidnapped_scan, {truth.x + 40.0, truth.y + 25.0},
 truth_heading, wrong_local, last_stamp);
 CHECK(!kidnapped.accepted);
 std::cout << "long_run fixes=" << kFixes
 << " travelled_m=" << step_m * static_cast<double>(kFixes)
 << " max_position_error_m=" << maximum_error
 << " max_prior_error_m=" << maximum_prior_error
 << " quarter_max_error_m=" << quarter_max_error[0] << ","
 << quarter_max_error[1] << "," << quarter_max_error[2] << ","
 << quarter_max_error[3]
 << " p95_latency_ms=" << p95_ms
 << " peak_rss_mib=" << static_cast<double>(peak_rss) / 1024.0
 << " kidnapped_rejected=1\n";
 return 0;
}
