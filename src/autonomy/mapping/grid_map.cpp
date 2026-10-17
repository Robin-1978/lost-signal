#include "autonomy/mapping/grid_map.hpp"
#include <cmath>
namespace sim {
GridMap GridMap::inflated(double radius) const {
 GridMap out(w_, h_, res_);
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
} // namespace sim
