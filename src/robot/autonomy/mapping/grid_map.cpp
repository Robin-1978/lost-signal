#include "robot/autonomy/mapping/grid_map.hpp"
#include <algorithm>
#include <stdexcept>
#include <cmath>
#include <limits>
namespace sim {
namespace {

double distance_to_cell_square(
 double wx, double wy, int gx, int gy, double resolution) {
 const double x0 = static_cast<double>(gx) * resolution;
 const double x1 = x0 + resolution;
 const double y0 = static_cast<double>(gy) * resolution;
 const double y1 = y0 + resolution;
 const double dx = std::max({x0 - wx, 0.0, wx - x1});
 const double dy = std::max({y0 - wy, 0.0, wy - y1});
 return std::hypot(dx, dy);
}

} // namespace

bool GridMap::segmentIsFree(Vec2 from, Vec2 to) const {
 const double length = std::hypot(to.x - from.x, to.y - from.y);
 const int subdivisions = std::max(
 1, static_cast<int>(std::ceil(length / (0.5 * res_))));
 for (int step = 0; step <= subdivisions; ++step) {
 const double blend = static_cast<double>(step) /
 static_cast<double>(subdivisions);
 if (!isFree(
 from.x + (to.x - from.x) * blend,
 from.y + (to.y - from.y) * blend)) {
 return false;
 }
 }
 return true;
}

double GridMap::clearance(double wx, double wy) const {
 const double width_m = static_cast<double>(w_) * res_;
 const double height_m = static_cast<double>(h_) * res_;
 if (!(wx >= 0.0 && wy >= 0.0 && wx <= width_m && wy <= height_m) ||
 !isFree(wx, wy)) {
 return 0.0;
 }
 // The map boundary represents the nearest point of implicit outside
 // occupancy. Explicit cells then contribute their full closed squares.
 double best = std::min({wx, wy, width_m - wx, height_m - wy});
 for (int gy = 0; gy < h_; ++gy) {
 for (int gx = 0; gx < w_; ++gx) {
 if (isFreeCell(gx, gy))
 continue;
 best = std::min(best, distance_to_cell_square(wx, wy, gx, gy, res_));
 }
 }
 return std::max(0.0, best);
}

double GridMap::distanceToFreeSpace(double wx, double wy) const {
 if (isFree(wx, wy))
 return 0.0;
 double best = std::numeric_limits<double>::infinity();
 for (int gy = 0; gy < h_; ++gy) {
 for (int gx = 0; gx < w_; ++gx) {
 if (!isFreeCell(gx, gy))
 continue;
 best = std::min(best, distance_to_cell_square(wx, wy, gx, gy, res_));
 }
 }
 return best;
}

void GridMap::setElevation(double wx, double wy, double elevation_m) {
 if (!std::isfinite(elevation_m))
 throw std::invalid_argument("grid elevation must be finite");
 const auto [gx, gy] = worldToGrid(wx, wy);
 if (!inside(gx, gy))
 return;
 if (elevation_m_.empty()) {
 elevation_m_.assign(data_.size(), std::numeric_limits<float>::quiet_NaN());
 }
 elevation_m_[static_cast<std::size_t>(gy) * w_ + gx] =
 static_cast<float>(elevation_m);
}

std::optional<double> GridMap::elevation(double wx, double wy) const {
 const auto [gx, gy] = worldToGrid(wx, wy);
 if (!inside(gx, gy) || elevation_m_.empty())
 return std::nullopt;
 const float value = elevation_m_[static_cast<std::size_t>(gy) * w_ + gx];
 if (!std::isfinite(value))
 return std::nullopt;
 return static_cast<double>(value);
}

GridMap GridMap::inflated(double radius, GridBoundaryPolicy boundary_policy) const {
 GridMap out(w_, h_, res_);
 out.elevation_m_ = elevation_m_;
 // Outside the authored map is occupied. Inflate that implicit obstacle
 // inward as well as explicit occupied cells: planning a body centre in the
 // last free cell is unsafe when its footprint already crosses the world
 // boundary. Use cell-centre clearance because every planner state is a cell
 // centre and continuous trajectory validation samples between those states.
 if (boundary_policy == GridBoundaryPolicy::Occupied) {
 for (int gy = 0; gy < h_; ++gy) {
 for (int gx = 0; gx < w_; ++gx) {
   const double center_x = (static_cast<double>(gx) + 0.5) * res_;
   const double center_y = (static_cast<double>(gy) + 0.5) * res_;
   const double boundary_clearance =
    std::min({center_x, center_y, static_cast<double>(w_) * res_ - center_x,
     static_cast<double>(h_) * res_ - center_y});
   if (boundary_clearance <= radius + 1e-12)
    out.data_[gy * w_ + gx] = 100;
  }
 }
 }
 // Planning samples cell centres, while an occupied cell represents its full
 // square area. Measure centre-to-square distance so a circular robot cannot
 // cut through a rasterized obstacle corner. Centre-to-centre distance would
 // give zero inflation whenever radius < resolution and is therefore unsafe.
 int r = (int)std::ceil(radius / res_ + 0.5);
 for (int gy = 0; gy < h_; gy++)
  for (int gx = 0; gx < w_; gx++) {
   if (data_[gy * w_ + gx] >= 50) { // lethal: paint neighborhood
    for (int dy = -r; dy <= r; dy++)
     for (int dx = -r; dx <= r; dx++) {
      int nx = gx + dx, ny = gy + dy;
      if (!inside(nx, ny))
       continue;
 const double nearest_x = std::max(0.0, std::abs(dx) * res_ - res_ * 0.5);
 const double nearest_y = std::max(0.0, std::abs(dy) * res_ - res_ * 0.5);
 double d = std::hypot(nearest_x, nearest_y);
      if (d <= radius)
       out.data_[ny * w_ + nx] = 100;
     }
   }
  }
 return out;
}

void GridMap::mergeOccupied(const GridMap& other) {
 if (w_ != other.w_ || h_ != other.h_ || std::abs(res_ - other.res_) > 1e-12)
 throw std::invalid_argument("cannot merge grid maps with different geometry");
 for (std::size_t index = 0; index < data_.size(); ++index) {
 if (other.data_[index] >= 50)
 data_[index] = 100;
 }
}
} // namespace sim
