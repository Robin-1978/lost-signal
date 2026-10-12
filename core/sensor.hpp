#pragma once
#include "grid_map.hpp"
#include <random>
#include <vector>
#include <cstdint>
namespace sim {
struct Rng {
  std::mt19937 gen;
  explicit Rng(uint32_t s=12345): gen(s) {}
  double gauss(double std=1.0){ std::normal_distribution<double> d(0.0,std); return d(gen); }
};
struct Scan {
  std::vector<double> ranges; // size n_rays (含噪声测量值, 仅显示/统计用)
  std::vector<char> hit;      // 每ray是否真命中(检测), 建图只能信这个, 不能信range<max
  int n_rays{0};
  double max_range{8.0};
};
// 等步长march + 高斯噪声. View节流调用(20~60Hz一次即可),Core里别每步都调.
Scan lidar_scan(const GridMap& m, Vec2 pos, double theta,
  int n_rays, double max_range, double noise_std, Rng& rng);

// 里程计航位推算: 用带噪的v/w积分,真值在外面. slip_scale=0干净,1正常,3很差.
class DeadReckon {
public:
  Vec2 p{0,0}; double th{0};
  void reset(double x,double y,double t){ p={x,y}; th=t; }
  // v,w为真值; dt固定步长. yaw_bias: 系统性偏航(rad/m, 如左右轮径差), 随机游走盖不住的真漂移源.
  void update(double v,double w,double dt,Rng& rng,double slip_scale=1.0,double yaw_bias=0.0);
  double driftErr(Vec2 true_p) const;
};
}
