#include "grid_map.hpp"
#include <cmath>
namespace sim {
GridMap GridMap::inflated(double radius) const {
 GridMap out(w_, h_, res_);
 int r = (int)std::ceil(radius / res_);
 for (int gy = 0; gy < h_; gy++)
  for (int gx = 0; gx < w_; gx++) {
   if (data_[gy * w_ + gx] >= 50) { // lethal: paint neighborhood
    for (int dy = -r; dy <= r; dy++)
     for (int dx = -r; dx <= r; dx++) {
      int nx = gx + dx, ny = gy + dy;
      if (!inside(nx, ny))
       continue;
      double d = std::hypot(dx * res_, dy * res_);
      if (d <= radius)
       out.data_[ny * w_ + nx] = 100;
     }
   }
  }
 return out;
}
} // namespace sim
