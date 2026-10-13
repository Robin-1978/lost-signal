#pragma once
#include "diff_drive.hpp"
#include <vector>
#include <cmath>
// 确定性recorder(Core, 无图形): 记初态+每步(v,w)指令+结果位姿(+guard干预标记).
// replay()用同样DiffDrive/步长重放, 逐位姿比对 — 验证"给定种子逐字节复现".
namespace sim {
struct StepRec {
 double tv{0}, tw{0}, x{0}, y{0}, th{0}, v{0}, w{0};
 bool blocked{false};
};
class Recorder {
 public:
 void begin(const DiffState& s0) {
  s0_ = s0;
  steps_.clear();
 }
 void push(double tv, double tw, const DiffState& after, bool blocked) {
  steps_.push_back({tv, tw, after.x, after.y, after.theta, after.v, after.w, blocked});
 }
 size_t size() const { return steps_.size(); }
 // 重放: diff底盘纯运动学, 无随机, 同二进制下必须逐位姿一致
 bool replay(DiffDrive& dd, double dt, double tol = 1e-9) const {
  DiffState bs = s0_;
  for (auto& s : steps_) {
   if (s.blocked) {
    // runner语义: guard拦截, 位姿不动, v/w清零
    if (std::abs(bs.x - s.x) > tol || std::abs(bs.y - s.y) > tol || std::abs(bs.theta - s.th) > tol)
     return false;
    bs.v = 0;
    bs.w = 0;
    if (std::abs(bs.v - s.v) > tol || std::abs(bs.w - s.w) > tol)
     return false;
    continue;
   }
   bs = dd.step(bs, {s.tv, s.tw}, dt);
   if (std::abs(bs.x - s.x) > tol || std::abs(bs.y - s.y) > tol || std::abs(bs.theta - s.th) > tol ||
       std::abs(bs.v - s.v) > tol || std::abs(bs.w - s.w) > tol)
    return false;
  }
  return true;
 }

 private:
 DiffState s0_;
 std::vector<StepRec> steps_;
};
} // namespace sim
