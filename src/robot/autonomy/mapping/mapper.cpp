#include "robot/autonomy/mapping/mapper.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace lost_signal::robot::autonomy {

namespace {

void world_to_grid(const LocalMap& m, double wx, double wy, int& gx, int& gy) {
  gx = static_cast<int>(
      std::floor((wx - m.origin_x_m) / m.resolution_m));
  gy = static_cast<int>(
      std::floor((wy - m.origin_y_m) / m.resolution_m));
}

double distance_to_cell_square(double wx, double wy, int gx, int gy,
                               double res, double ox, double oy) {
  const double x0 = ox + gx * res;
  const double x1 = x0 + res;
  const double y0 = oy + gy * res;
  const double y1 = y0 + res;
  const double dx = std::max({x0 - wx, 0.0, wx - x1});
  const double dy = std::max({y0 - wy, 0.0, wy - y1});
  return std::hypot(dx, dy);
}

}  // namespace

bool LocalMap::is_free(double wx, double wy) const {
  if (!valid || occupancy.size() != width * height) return false;
  int gx, gy;
  world_to_grid(*this, wx, wy, gx, gy);
  if (!inside(gx, gy)) return false;
  int8_t v = occupancy[gy * width + gx];
  return v >= 0 && v < 50;
}

bool LocalMap::is_blocked(double wx, double wy) const {
  if (!valid || occupancy.size() != width * height) return true;
  int gx, gy;
  world_to_grid(*this, wx, wy, gx, gy);
 if (!inside(gx, gy)) return true;
 // Unknown is conservative for planning as well as safety. A cell becomes
 // traversable only after a sensor ray or a rebuilt SLAM submap marks it
 // free; no mapless mission may infer free space from an empty window.
 return occupancy[gy * width + gx] < 0 || occupancy[gy * width + gx] >= 50;
}

bool LocalMap::segment_is_traversable(double x0, double y0, double x1,
                                               double y1) const {
  if (!valid || occupancy.size() != width * height) return false;
  const double length = std::hypot(x1 - x0, y1 - y0);
  const int subdivisions =
      std::max(1, static_cast<int>(std::ceil(length / (0.5 * resolution_m))));
  for (int step = 0; step <= subdivisions; ++step) {
    const double blend =
        static_cast<double>(step) / static_cast<double>(subdivisions);
    if (is_blocked(x0 + (x1 - x0) * blend, y0 + (y1 - y0) * blend)) {
      return false;
    }
  }
  return true;
}

bool LocalMap::segment_is_free(double x0, double y0, double x1,
                               double y1) const {
  const double length = std::hypot(x1 - x0, y1 - y0);
  const int subdivisions =
      std::max(1, static_cast<int>(std::ceil(length / (0.5 * resolution_m))));
  for (int step = 0; step <= subdivisions; ++step) {
    const double blend =
        static_cast<double>(step) / static_cast<double>(subdivisions);
    if (!is_free(x0 + (x1 - x0) * blend, y0 + (y1 - y0) * blend)) {
      return false;
    }
  }
  return true;
}

double LocalMap::clearance(double wx, double wy) const {
  if (!valid || occupancy.size() != width * height) return 0.0;
  int qx, qy;
  world_to_grid(*this, wx, wy, qx, qy);
  if (inside(qx, qy) &&
      occupancy[qy * width + qx] >= 50) {
    return 0.0;
  }
  double best = std::numeric_limits<double>::infinity();
  for (int gy = 0; gy < static_cast<int>(height); ++gy) {
    for (int gx = 0; gx < static_cast<int>(width); ++gx) {
      if (occupancy[gy * width + gx] >= 50) {
        best = std::min(best, distance_to_cell_square(wx, wy, gx, gy,
                                                      resolution_m, origin_x_m,
                                                      origin_y_m));
      }
    }
  }
  if (!inside(qx, qy)) return 0.0;  // outside the window counts as occupied
  if (best == std::numeric_limits<double>::infinity()) {
    return std::hypot(width * resolution_m, height * resolution_m);
  }
  return best;
}

LocalMap LocalMap::inflated(double radius_m) const {
  LocalMap out = *this;
  if (!valid || occupancy.size() != width * height) return out;
 if (!(radius_m > 0.0) || !(resolution_m > 0.0)) return out;
 // Planner states represent body centres while an occupied cell owns its
 // complete square. Centre-to-centre inflation underestimates corners by up
 // to half a cell and can leave a nominally inflated path in contact.
 const int r_cells = static_cast<int>(
 std::ceil(radius_m / resolution_m + 0.5));
  std::vector<int8_t> dst = occupancy;
  for (int gy = 0; gy < static_cast<int>(height); ++gy) {
    for (int gx = 0; gx < static_cast<int>(width); ++gx) {
      if (occupancy[gy * width + gx] < 50) continue;
      for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
          int nx = gx + dx, ny = gy + dy;
          if (!inside(nx, ny)) continue;  // ignore boundary: transient layer
 const double nearest_x = std::max(
 0.0, std::abs(dx) * resolution_m - 0.5 * resolution_m);
 const double nearest_y = std::max(
 0.0, std::abs(dy) * resolution_m - 0.5 * resolution_m);
 if (std::hypot(nearest_x, nearest_y) <= radius_m + 1e-12) {
            dst[ny * width + nx] = 100;
          }
        }
      }
    }
  }
  out.occupancy = std::move(dst);
  return out;
}

float LocalCostmap::cost_at(double wx, double wy) const {
 if (!valid()) return 1.0f;
 int gx, gy;
 world_to_grid(lethal, wx, wy, gx, gy);
 if (!lethal.inside(gx, gy)) return 1.0f;
 return linear_cost[static_cast<std::size_t>(gy) * lethal.width + gx];
}

LocalCostmap make_local_costmap(const LocalMap& source,
 double lethal_radius_m, double soft_radius_m) {
 LocalCostmap out;
 out.lethal = source.inflated(std::max(0.0, lethal_radius_m));
 out.soft_radius_m = std::max(0.0, soft_radius_m);
 if (!out.lethal.valid ||
 out.lethal.occupancy.size() != out.lethal.width * out.lethal.height) {
 return out;
 }
 out.linear_cost.assign(out.lethal.occupancy.size(), 0.0f);
 for (std::size_t index = 0; index < out.lethal.occupancy.size(); ++index) {
 if (out.lethal.occupancy[index] < 0 ||
 out.lethal.occupancy[index] >= 50) {
 out.linear_cost[index] = 1.0f;
 }
 }
 if (!(out.soft_radius_m > 0.0) ||
 !(out.lethal.resolution_m > 0.0)) {
 return out;
 }
 const int r_cells = static_cast<int>(std::ceil(
 out.soft_radius_m / out.lethal.resolution_m + 0.5));
 for (int gy = 0; gy < static_cast<int>(out.lethal.height); ++gy) {
 for (int gx = 0; gx < static_cast<int>(out.lethal.width); ++gx) {
 if (out.lethal.occupancy[
 static_cast<std::size_t>(gy) * out.lethal.width + gx] < 50) {
 continue;
 }
 for (int dy = -r_cells; dy <= r_cells; ++dy) {
 for (int dx = -r_cells; dx <= r_cells; ++dx) {
 const int nx = gx + dx, ny = gy + dy;
 if (!out.lethal.inside(nx, ny)) continue;
 const std::size_t index =
 static_cast<std::size_t>(ny) * out.lethal.width + nx;
 if (out.lethal.occupancy[index] < 0 ||
 out.lethal.occupancy[index] >= 50) {
 out.linear_cost[index] = 1.0f;
 continue;
 }
 const double nearest_x = std::max(
 0.0, std::abs(dx) * out.lethal.resolution_m -
 0.5 * out.lethal.resolution_m);
 const double nearest_y = std::max(
 0.0, std::abs(dy) * out.lethal.resolution_m -
 0.5 * out.lethal.resolution_m);
 const double distance = std::hypot(nearest_x, nearest_y);
 if (distance > out.soft_radius_m) continue;
 const float cost = static_cast<float>(
 1.0 - distance / out.soft_radius_m);
 out.linear_cost[index] = std::max(out.linear_cost[index], cost);
 }
 }
 }
 }
 return out;
}

LocalCostmap make_global_costmap(const iface::MapSnapshot& source,
 double lethal_radius_m, double soft_radius_m,
 bool unknown_traversable) {
 LocalCostmap out;
 if (!source.has_raster() || !std::isfinite(source.origin_x_m) ||
 !std::isfinite(source.origin_y_m)) {
 return out;
 }
 LocalMap obstacle_map;
 obstacle_map.valid = true;
 obstacle_map.resolution_m = source.resolution_m;
 obstacle_map.width = source.width;
 obstacle_map.height = source.height;
 obstacle_map.origin_x_m = source.origin_x_m;
 obstacle_map.origin_y_m = source.origin_y_m;
 obstacle_map.occupancy.resize(source.occupancy.size(), 0);
 for (std::size_t index = 0; index < source.occupancy.size(); ++index) {
 const int8_t cell = source.occupancy[index];
 if (cell >= 50) {
 obstacle_map.occupancy[index] = 100;
 } else if (cell < 0 && !unknown_traversable) {
 obstacle_map.occupancy[index] = -1;
 }
 }

 const double hard_radius = std::max(0.0, lethal_radius_m);
 const double soft_radius = std::max(0.0, soft_radius_m);
 out = make_local_costmap(obstacle_map, hard_radius, soft_radius);
 if (!out.valid()) return LocalCostmap{};

 // Outside the authored raster is occupied. Marking the outermost authored
 // cell occupied before inflation would make the boundary one complete cell
 // thicker than it is, so apply its exact distance at each cell centre.
 const double width_m = source.width * source.resolution_m;
 const double height_m = source.height * source.resolution_m;
 for (std::uint32_t y = 0; y < source.height; ++y) {
 for (std::uint32_t x = 0; x < source.width; ++x) {
 const double cx = (static_cast<double>(x) + 0.5) * source.resolution_m;
 const double cy = (static_cast<double>(y) + 0.5) * source.resolution_m;
 const double boundary = std::min({cx, cy, width_m - cx, height_m - cy});
 const std::size_t index = static_cast<std::size_t>(y) * source.width + x;
 if (boundary <= hard_radius + 1e-12) {
 out.lethal.occupancy[index] = 100;
 out.linear_cost[index] = 1.0f;
 } else if (soft_radius > 0.0 &&
 boundary < hard_radius + soft_radius) {
 const float cost = static_cast<float>(
 1.0 - (boundary - hard_radius) / soft_radius);
 out.linear_cost[index] = std::max(out.linear_cost[index], cost);
 }
 }
 }
 return out;
}

}  // namespace lost_signal::robot::autonomy
