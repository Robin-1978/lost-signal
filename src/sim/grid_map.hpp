#pragma once
#include "math.hpp"
#include <vector>
namespace sim {
// Occupancy grid: 0 free, 100 occupied. World origin at (0,0), res = meters/cell.
class GridMap {
 public:
 GridMap(int w, int h, double res) : w_(w), h_(h), res_(res), data_(w * h, 0) {}
 int w() const { return w_; }
 int h() const { return h_; }
 double res() const { return res_; }
 bool inside(int gx, int gy) const { return gx >= 0 && gy >= 0 && gx < w_ && gy < h_; }
 void setOccupied(double wx, double wy) {
  auto [gx, gy] = worldToGrid(wx, wy);
  if (inside(gx, gy))
   data_[gy * w_ + gx] = 100;
 }
 bool isFree(double wx, double wy) const {
  auto [gx, gy] = worldToGrid(wx, wy);
  return inside(gx, gy) && data_[gy * w_ + gx] < 50;
 }
 bool isFreeCell(int gx, int gy) const { return inside(gx, gy) && data_[gy * w_ + gx] < 50; }
 // simple box obstacle in world coords, half-open [x0,x1)x[y0,y1): 不多占格子
 void addBox(double x0, double y0, double x1, double y1) {
  int gx0 = (int)std::floor(x0 / res_), gx1 = (int)std::ceil(x1 / res_);
  int gy0 = (int)std::floor(y0 / res_), gy1 = (int)std::ceil(y1 / res_);
  for (int gy = gy0; gy < gy1; gy++)
   for (int gx = gx0; gx < gx1; gx++)
    if (inside(gx, gy))
     data_[gy * w_ + gx] = 100;
 }
 // inflate occupied cells by radius (meters) -> out costmap (0 free, 100 lethal+inflated)
 GridMap inflated(double radius) const;
 std::pair<int, int> worldToGrid(double wx, double wy) const {
  return {(int)std::floor(wx / res_), (int)std::floor(wy / res_)};
 }
 Vec2 gridToWorld(int gx, int gy) const { return {(gx + 0.5) * res_, (gy + 0.5) * res_}; }
 const std::vector<int8_t>& data() const { return data_; }

 private:
 int w_, h_;
 double res_;
 std::vector<int8_t> data_;
};
} // namespace sim
