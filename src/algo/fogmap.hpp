#pragma once
#include "grid_map.hpp"
#include "sensor.hpp"
#include <vector>
#include <cmath>
// Headless fog-of-war map: the robot knows only cells observed by LiDAR.
// reveal() marches measured Scan data and never stores or reads world truth.
// planningGrid() treats unknown cells as free (optimistic) or occupied (conservative).
namespace sim {
class FogMap {
 public:
 FogMap(int w = 0, int h = 0, double res = 0.5) : w_(w), h_(h), res_(res), seen_(w * h, 0), occ_(w * h, 0) {}
 void reset(double x, double y) { markFree(x, y); }
 void reveal(Vec2 pos, double theta, const Scan& s) {
  markFree(pos.x, pos.y);
  for (int i = 0; i < s.n_rays; i++) {
   double a = theta + 2 * kPi * i / s.n_rays;
   double ca = std::cos(a), sa = std::sin(a);
   double r = s.ranges[i];
   for (double t = res_ * 0.5; t < r - res_ * 0.25; t += res_ * 0.5)
    markFree(pos.x + ca * t, pos.y + sa * t);
   if (s.hit[i])
    markOcc(pos.x + ca * r, pos.y + sa * r); // Only a reported hit may create a wall.
  }
 }
 GridMap planningGrid(bool unknown_free) const {
  GridMap g(w_, h_, res_);
  for (int gy = 0; gy < h_; gy++)
   for (int gx = 0; gx < w_; gx++) {
    int k = gy * w_ + gx;
    int8_t v = seen_[k] ? occ_[k] : (unknown_free ? 0 : 100);
    if (v >= 50) {
     Vec2 p = g.gridToWorld(gx, gy);
     g.setOccupied(p.x, p.y);
    }
   }
  return g;
 }
 bool seenAt(double x, double y) const {
  int gx = (int)std::floor(x / res_), gy = (int)std::floor(y / res_);
  if (gx < 0 || gy < 0 || gx >= w_ || gy >= h_)
   return false;
  return seen_[gy * w_ + gx];
 }
 int w() const { return w_; }
 int h() const { return h_; }
 double res() const { return res_; }

 private:
 void markFree(double x, double y) {
  int gx = (int)std::floor(x / res_), gy = (int)std::floor(y / res_);
  if (gx < 0 || gy < 0 || gx >= w_ || gy >= h_)
   return;
  seen_[gy * w_ + gx] = 1; // Mark free without overwriting an observed occupied endpoint.
  if (occ_[gy * w_ + gx] < 50)
   occ_[gy * w_ + gx] = 0;
 }
 void markOcc(double x, double y) {
  int gx = (int)std::floor(x / res_), gy = (int)std::floor(y / res_);
  if (gx < 0 || gy < 0 || gx >= w_ || gy >= h_)
   return;
  seen_[gy * w_ + gx] = 1;
  occ_[gy * w_ + gx] = 100;
 }
 int w_, h_;
 double res_;
 std::vector<char> seen_;
 std::vector<int8_t> occ_;
};
} // namespace sim
