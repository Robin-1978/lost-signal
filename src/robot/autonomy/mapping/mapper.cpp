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
  if (!inside(gx, gy)) return false;
  return occupancy[gy * width + gx] >= 50;
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
  const int r_cells =
      static_cast<int>(std::ceil(radius_m / resolution_m));
  std::vector<int8_t> dst = occupancy;
  for (int gy = 0; gy < static_cast<int>(height); ++gy) {
    for (int gx = 0; gx < static_cast<int>(width); ++gx) {
      if (occupancy[gy * width + gx] < 50) continue;
      for (int dy = -r_cells; dy <= r_cells; ++dy) {
        for (int dx = -r_cells; dx <= r_cells; ++dx) {
          int nx = gx + dx, ny = gy + dy;
          if (!inside(nx, ny)) continue;  // ignore boundary: transient layer
          double cx = origin_x_m + (nx + 0.5) * resolution_m;
          double cy = origin_y_m + (ny + 0.5) * resolution_m;
          double ox = origin_x_m + (gx + 0.5) * resolution_m;
          double oy = origin_y_m + (gy + 0.5) * resolution_m;
          if (std::hypot(cx - ox, cy - oy) <= radius_m) {
            dst[ny * width + nx] = 100;
          }
        }
      }
    }
  }
  out.occupancy = std::move(dst);
  return out;
}

}  // namespace lost_signal::robot::autonomy
