#include "robot/autonomy/mapping/observed_obstacle_layer.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace sim {

ObservedObstacleLayer::ObservedObstacleLayer(int width, int height, double resolution,
 ObservedObstacleLayerConfig config)
 : width_(width), height_(height), resolution_(resolution), config_(config),
 cells_(static_cast<std::size_t>(width) * static_cast<std::size_t>(height)) {
 if (width <= 0 || height <= 0 || !(resolution > 0.0) ||
 !(config.obstacle_persistence_s >= 0.0) || !(config.expected_update_period_s > 0.0)) {
 throw std::invalid_argument("invalid observed obstacle layer configuration");
 }
}

void ObservedObstacleLayer::reset() {
 cells_.assign(cells_.size(), {});
 last_observation_stamp_s_ = -std::numeric_limits<double>::infinity();
}

std::vector<std::size_t> ObservedObstacleLayer::ray_cells(Vec2 origin, Vec2 endpoint,
 bool include_endpoint) const {
 const double length = std::hypot(endpoint.x - origin.x, endpoint.y - origin.y);
 const int samples = std::max(1, static_cast<int>(std::ceil(length / (0.5 * resolution_))));
 std::vector<std::size_t> result;
 result.reserve(static_cast<std::size_t>(samples + 1));
 const int last_sample = include_endpoint ? samples : std::max(0, samples - 1);
 for (int sample = 0; sample <= last_sample; ++sample) {
 const double blend = static_cast<double>(sample) / static_cast<double>(samples);
 const double x = origin.x + (endpoint.x - origin.x) * blend;
 const double y = origin.y + (endpoint.y - origin.y) * blend;
 const int gx = static_cast<int>(std::floor(x / resolution_));
 const int gy = static_cast<int>(std::floor(y / resolution_));
 if (gx < 0 || gy < 0 || gx >= width_ || gy >= height_)
 continue;
 const auto index = static_cast<std::size_t>(gy * width_ + gx);
 if (result.empty() || result.back() != index)
 result.push_back(index);
 }
 return result;
}

bool ObservedObstacleLayer::observe(double stamp_s,
 const std::vector<ObservedRay2D>& rays,
 const std::vector<ObservedClearingRegion>& clearing_regions) {
 if (!std::isfinite(stamp_s) || stamp_s + 1e-9 < last_observation_stamp_s_)
 return false;

 std::vector<char> cleared(cells_.size(), 0);
 std::vector<char> marked(cells_.size(), 0);

 const auto distance_to_segment = [](Vec2 point, Vec2 from, Vec2 to) {
 const Vec2 delta{to.x - from.x, to.y - from.y};
 const double length_squared = delta.x * delta.x + delta.y * delta.y;
 if (length_squared <= 1e-12)
 return std::hypot(point.x - from.x, point.y - from.y);
 const double projection = std::clamp(
 ((point.x - from.x) * delta.x + (point.y - from.y) * delta.y) /
 length_squared, 0.0, 1.0);
 return std::hypot(
 point.x - (from.x + projection * delta.x),
 point.y - (from.y + projection * delta.y));
 };
 const double cell_half_diagonal = resolution_ / std::sqrt(2.0);
 for (const auto& region : clearing_regions) {
 if (!std::isfinite(region.from.x) || !std::isfinite(region.from.y) ||
 !std::isfinite(region.to.x) || !std::isfinite(region.to.y) ||
 !(region.radius_m >= 0.0) || !std::isfinite(region.radius_m)) {
 continue;
 }
 const double min_x = std::min(region.from.x, region.to.x) -
 region.radius_m - cell_half_diagonal;
 const double max_x = std::max(region.from.x, region.to.x) +
 region.radius_m + cell_half_diagonal;
 const double min_y = std::min(region.from.y, region.to.y) -
 region.radius_m - cell_half_diagonal;
 const double max_y = std::max(region.from.y, region.to.y) +
 region.radius_m + cell_half_diagonal;
 const int gx0 = std::max(0, static_cast<int>(std::floor(min_x / resolution_)));
 const int gx1 = std::min(width_ - 1, static_cast<int>(std::floor(max_x / resolution_)));
 const int gy0 = std::max(0, static_cast<int>(std::floor(min_y / resolution_)));
 const int gy1 = std::min(height_ - 1, static_cast<int>(std::floor(max_y / resolution_)));
 for (int gy = gy0; gy <= gy1; ++gy) {
 for (int gx = gx0; gx <= gx1; ++gx) {
 const Vec2 centre{
 (static_cast<double>(gx) + 0.5) * resolution_,
 (static_cast<double>(gy) + 0.5) * resolution_,
 };
 if (distance_to_segment(centre, region.from, region.to) <=
 region.radius_m + cell_half_diagonal) {
 cleared[static_cast<std::size_t>(gy * width_ + gx)] = 1;
 }
 }
 }
 }
 for (const auto& ray : rays) {
 if (!std::isfinite(ray.origin.x) || !std::isfinite(ray.origin.y) ||
 !std::isfinite(ray.endpoint.x) || !std::isfinite(ray.endpoint.y)) {
 continue;
 }
 for (const auto index : ray_cells(ray.origin, ray.endpoint, !ray.hit))
 cleared[index] = 1;
 if (ray.hit) {
 const int gx = static_cast<int>(std::floor(ray.endpoint.x / resolution_));
 const int gy = static_cast<int>(std::floor(ray.endpoint.y / resolution_));
 if (gx >= 0 && gy >= 0 && gx < width_ && gy < height_)
 marked[static_cast<std::size_t>(gy * width_ + gx)] = 1;
 }
 }

 for (std::size_t index = 0; index < cells_.size(); ++index) {
 if (cleared[index])
 cells_[index].cleared_s = stamp_s;
 if (marked[index])
 cells_[index].marked_s = stamp_s;
 }
 last_observation_stamp_s_ = stamp_s;
 return true;
}

bool ObservedObstacleLayer::fresh(double now_s) const {
 return std::isfinite(now_s) && std::isfinite(last_observation_stamp_s_) &&
 now_s + 1e-9 >= last_observation_stamp_s_ &&
 now_s - last_observation_stamp_s_ <= 2.0 * config_.expected_update_period_s + 1e-9;
}

bool ObservedObstacleLayer::active(const CellEvidence& cell, double now_s) const {
 return std::isfinite(now_s) && std::isfinite(cell.marked_s) && now_s + 1e-9 >= cell.marked_s &&
 cell.marked_s + 1e-9 >= cell.cleared_s &&
 now_s - cell.marked_s <= config_.obstacle_persistence_s + 1e-9;
}

std::size_t ObservedObstacleLayer::active_obstacle_cells(double now_s) const {
 return static_cast<std::size_t>(
 std::count_if(cells_.begin(), cells_.end(), [&](const auto& cell) { return active(cell, now_s); }));
}

std::vector<Vec2> ObservedObstacleLayer::active_obstacle_centres(
 double now_s) const {
 std::vector<Vec2> result;
 for (int gy = 0; gy < height_; ++gy) {
 for (int gx = 0; gx < width_; ++gx) {
 const auto index = static_cast<std::size_t>(gy * width_ + gx);
 if (active(cells_[index], now_s)) {
 result.push_back({
 (static_cast<double>(gx) + 0.5) * resolution_,
 (static_cast<double>(gy) + 0.5) * resolution_,
 });
 }
 }
 }
 return result;
}

GridMap ObservedObstacleLayer::compose(const GridMap& static_costmap, double now_s,
 double inflation_radius_m) const {
 if (static_costmap.w() != width_ || static_costmap.h() != height_ ||
 std::abs(static_costmap.res() - resolution_) > 1e-12 || inflation_radius_m < 0.0) {
 throw std::invalid_argument("observed obstacle layer does not match static costmap");
 }
 GridMap observed(width_, height_, resolution_);
 for (int gy = 0; gy < height_; ++gy) {
 for (int gx = 0; gx < width_; ++gx) {
 const auto index = static_cast<std::size_t>(gy * width_ + gx);
 if (active(cells_[index], now_s)) {
 const auto point = observed.gridToWorld(gx, gy);
 observed.setOccupied(point.x, point.y);
 }
 }
 }
 GridMap result = static_costmap;
 result.mergeOccupied(observed.inflated(
 inflation_radius_m, GridBoundaryPolicy::Ignore));
 return result;
}

} // namespace sim
