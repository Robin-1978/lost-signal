#include "robot/autonomy/localization/global_map_localizer.hpp"
#include "robot/autonomy/localization/known_map_localizer.hpp"
#include "robot/runtime/map_provider.hpp"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

constexpr double kResolution = 0.1;
constexpr int kRepeatsPerLocation = 12;
constexpr double kMapWidthM = 600.0;
constexpr double kMapHeightM = 400.0;
constexpr std::array<sim::Vec2, 8> kTestLocations{{
 { -170.0, 50.0 }, { -25.0, -60.0 }, { 115.0, 135.0 }, { 245.0, -110.0 },
 { -220.0, -130.0 }, { 40.0, 35.0 }, { 180.0, 70.0 }, { 260.0, 150.0 },
}};

// Peak resident set size in kibibytes, or zero where the platform does not
// expose it. The benchmark records this so a future regression cannot hide a
// per-fix allocation behind a passing latency number.
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

sim::runtime::MapSnapshot make_large_map() {
 sim::runtime::MapSnapshot snapshot;
 snapshot.map_id = "large-map-localization-gate";
 snapshot.revision = 1;
 snapshot.sample_time = sim::runtime::TimePoint::from_seconds(0.0);
 snapshot.provenance = {"survey", "test.large-map", "1"};
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
 for (int x = gx0; x <= gx1; ++x) occupy(x, y);
 };
 // The outer shell is 240,000 m². Internal walls and asymmetric blocks make
 // each localization window distinctive without using actor truth.
 box(-300.0, -200.0, 300.0, -199.6);
 box(-300.0, 199.6, 300.0, 200.0);
 box(-300.0, -200.0, -299.6, 200.0);
 box(299.6, -200.0, 300.0, 200.0);
 for (int i = 0; i < 9; ++i) {
 const double x = -260.0 + 64.0 * i;
 const double y0 = -160.0 + 11.0 * ((i * 7) % 9);
 const double y1 = y0 + 45.0 + 8.0 * (i % 5);
 box(x, y0, x + 0.8, std::min(y1, 150.0));
 box(x + 13.0, y0 + 60.0, x + 18.0 + 2.0 * (i % 4), y0 + 62.0);
 }
 for (int i = 0; i < 12; ++i) {
 const double x = -270.0 + 44.0 * i;
 const double y = (i % 2 == 0 ? -120.0 : 105.0);
 box(x, y, x + 7.0 + (i % 3) * 2.0, y + 2.5);
 box(x + 18.0, y + 13.0, x + 19.2, y + 33.0 + i);
 }
 // Objects around the tested locations include corners at different scales,
 // so the local matcher has real lever arms instead of a blank corridor.
 // Distinct local geometry supplies the lever arms and scan overlap needed
 // by repeated localization. The pattern is map data, never actor truth.
 for (std::size_t location_index = 0; location_index < kTestLocations.size();
 ++location_index) {
 const auto centre = kTestLocations[location_index];
 for (int object = 0; object < 14; ++object) {
 const double angle = -2.8 + 0.41 * object + 0.07 * location_index;
 const double distance = 2.4 + 0.42 * object;
 const double half_width = 0.12 + 0.05 * ((object + location_index) % 4);
 const double x = centre.x + distance * std::cos(angle);
 const double y = centre.y + distance * std::sin(angle);
 box(x - half_width, y - half_width, x + half_width, y + half_width);
 if ((object + static_cast<int>(location_index)) % 3 == 0)
 box(x + 0.8, y - 0.1, x + 2.1, y + 0.1);
 }
 }
 snapshot.grid = std::move(grid);
 return snapshot;
}

sim::Scan scan_at(const sim::runtime::MapGrid& grid, sim::Vec2 pose,
 double heading, double stamp_s) {
 sim::Scan scan;
 scan.n_rays = 180;
 scan.max_range = 10.0;
 scan.angle_min_rad = -sim::kPi;
 scan.angle_increment_rad = 2.0 * sim::kPi / scan.n_rays;
 scan.scan_period_s = 0.1;
 scan.header = {stamp_s, 1, "lidar_link", 0.0,
 sim::kMeasurementSchemaVersion, "test.large-map"};
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

} // namespace

int main() {
 auto map = make_large_map();
 CHECK(map.grid);
 CHECK(static_cast<double>(map.grid->width) * map.grid->resolution_m >= kMapWidthM);
 CHECK(static_cast<double>(map.grid->height) * map.grid->resolution_m >= kMapHeightM);
 CHECK(static_cast<double>(map.grid->width) * map.grid->height *
 map.grid->resolution_m * map.grid->resolution_m >= 200000.0);
 CHECK(map.grid->occupancy.size() ==
 static_cast<std::size_t>(map.grid->width) * map.grid->height);
 std::string reason;
 sim::runtime::MapManager manager;
 CHECK(manager.configure(map, &reason));
 CHECK(manager.activate());

 sim::KnownMapLocalizationConfig config;
 config.search_translation_m = 0.35;
 config.search_heading_rad = 0.16;
 config.translation_step_m = 0.05;
 config.heading_step_rad = 0.02;
 config.minimum_score_margin = 0.0;
 sim::KnownMapLocalizer localizer(config);
 const std::array<double, 8> headings = {0.3, 1.4, -2.1, 2.7, -0.6, 0.9, -1.3, 2.4};
 const std::size_t pose_count = kTestLocations.size();
 double maximum_position_error = 0.0;
 double maximum_heading_error = 0.0;
 double maximum_latency_ms = 0.0;
 std::vector<std::size_t> window_cells;
 std::vector<double> latencies_ms;
 for (std::size_t index = 0; index < pose_count; ++index) {
 const auto truth = kTestLocations[index];
 const double truth_heading = headings[index];
 const auto view = manager.query_region(sim::runtime::MapBounds2D{
 {truth.x - 8.0, truth.y - 8.0}, {truth.x + 8.0, truth.y + 8.0}});
 CHECK(view && view->grid);
 window_cells.push_back(view->grid->occupancy.size());
 // Exercise the same bounded provider result an online localization service
 // consumes. It must not require a second global-map allocation per fix.
 sim::runtime::MapSnapshot local_snapshot = map;
 local_snapshot.grid = view->grid;
 const double stamp_s = 10.0 + static_cast<double>(index);
 const auto scan = scan_at(*map.grid, truth, truth_heading, stamp_s);
 const sim::Vec2 noisy_prior{truth.x + 0.16, truth.y - 0.11};
 const double noisy_heading = sim::wrap_angle(truth_heading + 0.07);
 const auto start = std::chrono::steady_clock::now();
 const auto located = localizer.localize(scan, noisy_prior, noisy_heading,
 local_snapshot, stamp_s);
 const auto elapsed = std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - start).count();
 maximum_latency_ms = std::max(maximum_latency_ms, elapsed);
 latencies_ms.push_back(elapsed);
 if (!located.accepted) std::cerr << "index=" << index << " reason=" << located.rejection_reason << " returns=" << located.usable_returns << " score=" << located.score << " margin=" << located.score_margin << "\n";
 CHECK(located.accepted);
 CHECK(located.state == sim::autonomy::LocalizationState::Localized);
 const double position_error = std::hypot(
 located.pose.position.x - truth.x,
 located.pose.position.y - truth.y);
 const double heading_error = std::abs(sim::wrap_angle(
 located.pose.heading_rad - truth_heading));
 maximum_position_error = std::max(maximum_position_error, position_error);
 maximum_heading_error = std::max(maximum_heading_error, heading_error);
 CHECK(position_error < 0.16);
 CHECK(heading_error < 0.06);
 // Identical input must produce an identical pose. No hash-map iteration or
 // floating tie is allowed to change the selected correction.
 const auto repeat = localizer.localize(scan, noisy_prior, noisy_heading,
 local_snapshot, stamp_s);
 CHECK(repeat.accepted);
 CHECK(repeat.pose.position.x == located.pose.position.x);
 CHECK(repeat.pose.position.y == located.pose.position.y);
 CHECK(repeat.pose.heading_rad == located.pose.heading_rad);
 // Repeated fixes on the same bounded window must stay flat: a per-fix
 // allocation proportional to the map would show up here immediately.
 for (int repetition = 1; repetition < kRepeatsPerLocation; ++repetition) {
 const auto timing_start = std::chrono::steady_clock::now();
 const auto timed = localizer.localize(scan, noisy_prior, noisy_heading,
 local_snapshot, stamp_s);
 latencies_ms.push_back(std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - timing_start).count());
 CHECK(timed.accepted);
 }
 }
 CHECK(std::all_of(window_cells.begin(), window_cells.end(),
 [](std::size_t cells) { return cells <= 300U * 300U; }));
 CHECK(maximum_position_error < 0.16);
 CHECK(maximum_heading_error < 0.06);
 std::sort(latencies_ms.begin(), latencies_ms.end());
 CHECK(!latencies_ms.empty());
 const auto p95_index = static_cast<std::size_t>(
 std::ceil(0.95 * static_cast<double>(latencies_ms.size()))) - 1U;
 const double p95_latency_ms = latencies_ms[std::min(p95_index, latencies_ms.size() - 1U)];
 const long peak_rss = peak_rss_kib();
 // This is a generously bounded component regression, not a production
 // real-time certification. It runs on the development host; the deployment
 // profile must repeat it on target compute at 200,000 m² or larger.
 CHECK(maximum_latency_ms < 5000.0);
 CHECK(p95_latency_ms < 1000.0);
 // Cold start is a separate contract: no odometry prior or cropped map is
 // supplied. The coarse pass is deliberately bounded for this 240,000 m²
 // fixture, then the retained hypotheses return to the full-resolution map.
 sim::GlobalMapLocalizationConfig global_config;
 global_config.coarse_field_resolution_m = 0.5;
 global_config.coarse_position_step_m = 2.0;
 global_config.coarse_heading_step_rad = 0.5;
 global_config.coarse_returns = 16;
 global_config.coarse_hypotheses = 8;
 global_config.refine_translation_m = 1.0;
 global_config.refine_translation_step_m = 0.2;
 global_config.refine_heading_step_rad = 0.1;
 global_config.final_translation_m = 0.3;
 global_config.final_translation_step_m = 0.1;
 global_config.final_heading_rad = 0.1;
 global_config.final_heading_step_rad = 0.05;
 global_config.minimum_score_margin = 0.0;
 global_config.maximum_evaluations = 2000000;
 sim::GlobalMapLocalizer global_localizer(global_config);
 const auto cold_truth = kTestLocations.front();
 const double cold_heading = headings.front();
 const double cold_stamp = 40.0;
 const auto cold_scan = scan_at(*map.grid, cold_truth, cold_heading, cold_stamp);
 const auto cold_start = std::chrono::steady_clock::now();
 const auto cold = global_localizer.localize_unknown_start(
 cold_scan, map, cold_stamp);
 const double cold_latency_ms = std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - cold_start).count();
 CHECK(cold.accepted);
 CHECK(cold.state == sim::autonomy::LocalizationState::Localized);
 const double cold_position_error = std::hypot(
 cold.pose.position.x - cold_truth.x, cold.pose.position.y - cold_truth.y);
 const double cold_heading_error = std::abs(sim::wrap_angle(
 cold.pose.heading_rad - cold_heading));
 CHECK(cold_position_error < 1.0);
 CHECK(cold_heading_error < 0.25);
 CHECK(cold_latency_ms < 30000.0);
 std::vector<double> cold_latencies{cold_latency_ms};
 for (int repetition = 0; repetition < 2; ++repetition) {
 const auto repeat_start = std::chrono::steady_clock::now();
 const auto repeat = global_localizer.localize_unknown_start(
 cold_scan, map, cold_stamp);
 cold_latencies.push_back(std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - repeat_start).count());
 CHECK(repeat.accepted);
 CHECK(repeat.pose.position.x == cold.pose.position.x);
 CHECK(repeat.pose.position.y == cold.pose.position.y);
 CHECK(repeat.pose.heading_rad == cold.pose.heading_rad);
 }
 std::sort(cold_latencies.begin(), cold_latencies.end());
 const auto cold_p95_index = static_cast<std::size_t>(
 std::ceil(0.95 * static_cast<double>(cold_latencies.size()))) - 1U;
 const double cold_p95_latency_ms = cold_latencies[
 std::min(cold_p95_index, cold_latencies.size() - 1U)];
 CHECK(cold_p95_latency_ms < 30000.0);
 std::cout << "large_map area_m2="
 << static_cast<double>(map.grid->width) * map.grid->height *
 map.grid->resolution_m * map.grid->resolution_m
 << " cells=" << map.grid->occupancy.size()
 << " window_cells=" << window_cells.front()
 << " max_position_error_m=" << maximum_position_error
 << " max_heading_error_rad=" << maximum_heading_error
 << " max_latency_ms=" << maximum_latency_ms
 << " p95_latency_ms=" << p95_latency_ms
 << " samples=" << latencies_ms.size()
 << " peak_rss_mib=" << static_cast<double>(peak_rss) / 1024.0
 << " cold_start_position_error_m=" << cold_position_error
 << " cold_start_heading_error_rad=" << cold_heading_error
 << " cold_start_latency_ms=" << cold_latency_ms
 << " cold_start_p95_latency_ms=" << cold_p95_latency_ms << "\n";
 return 0;
}
