#pragma once
#include "robot/core/math.hpp"
#include <optional>
#include <vector>
namespace sim {
enum class GridBoundaryPolicy {
 Occupied,
 Ignore,
};

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
 void setElevation(double wx, double wy, double elevation_m);
 std::optional<double> elevation(double wx, double wy) const;
 bool hasElevation() const { return elevation_m_.size() == data_.size(); }
 // World-space box with half-open bounds [x0,x1)x[y0,y1), avoiding an extra cell.
 void addBox(double x0, double y0, double x1, double y1) {
  int gx0 = (int)std::floor(x0 / res_), gx1 = (int)std::ceil(x1 / res_);
  int gy0 = (int)std::floor(y0 / res_), gy1 = (int)std::ceil(y1 / res_);
  for (int gy = gy0; gy < gy1; gy++)
   for (int gx = gx0; gx < gx1; gx++)
    if (inside(gx, gy))
     data_[gy * w_ + gx] = 100;
 }
 // Inflate occupied cells by radius (meters) into a binary costmap. A static
 // world normally treats space outside the grid as occupied; a transient
 // observation layer must ignore its boundary because the base map owns that
 // policy and an empty sensor layer must contribute no obstacles.
 GridMap inflated(
 double radius,
 GridBoundaryPolicy boundary_policy = GridBoundaryPolicy::Occupied) const;
 // Union another occupancy layer with identical geometry. Layer-specific
 // clearing never erases obstacles owned by a different layer.
 void mergeOccupied(const GridMap& other);
 std::pair<int, int> worldToGrid(double wx, double wy) const {
  return {(int)std::floor(wx / res_), (int)std::floor(wy / res_)};
 }
 Vec2 gridToWorld(int gx, int gy) const { return {(gx + 0.5) * res_, (gy + 0.5) * res_}; }
 const std::vector<int8_t>& data() const { return data_; }

 private:
 int w_, h_;
 double res_;
 std::vector<int8_t> data_;
 // Optional 2.5D ground layer at cell centres. NaN is unknown. Occupancy and
 // elevation share coordinates but retain independent semantics.
 std::vector<float> elevation_m_;
};
} // namespace sim
