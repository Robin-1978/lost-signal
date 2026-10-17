#include "autonomy/mapping/observed_obstacle_layer.hpp"

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

bool ObservedObstacleLayer::observe(double stamp_s, const std::vector<ObservedRay2D>& rays) {
 if (!std::isfinite(stamp_s) || stamp_s + 1e-9 < last_observation_stamp_s_)
 return false;

 std::vector<char> cleared(cells_.size(), 0);
 std::vector<char> marked(cells_.size(), 0);
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
 result.mergeOccupied(observed.inflated(inflation_radius_m));
 return result;
}

} // namespace sim
