#include "robot/autonomy/mapping/grid_map.hpp"
#include <algorithm>
#include <stdexcept>
#include <cmath>
namespace sim {
GridMap GridMap::inflated(double radius, GridBoundaryPolicy boundary_policy) const {
 GridMap out(w_, h_, res_);
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
