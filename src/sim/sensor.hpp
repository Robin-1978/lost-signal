#pragma once

#include "dynobs.hpp"
#include "grid_map.hpp"

#include <cstdint>
#include <random>
#include <vector>

namespace sim {

struct Rng {
 std::mt19937 gen;
 explicit Rng(uint32_t seed = 12345) : gen(seed) {}
 double gauss(double standard_deviation = 1.0) {
  std::normal_distribution<double> distribution(0.0, standard_deviation);
  return distribution(gen);
 }
};

struct Scan {
 std::vector<double> ranges;
 std::vector<char> hit;
 // -1 means a static hit or no hit; otherwise this is the dynamic-object index.
 // The index is sensor-side association data used by the simplified tracker.
 std::vector<int> dynamic_hit;
 int n_rays{0};
 double max_range{8.0};
};

// Deterministic 2D ray query. Static occupancy and moving collision circles
// participate in the same nearest-hit and occlusion test.
Scan lidar_scan(const GridMap& map, Vec2 position, double heading, int n_rays, double max_range,
                double noise_std, Rng& rng, const std::vector<DynState>& dynamic_objects = {});

class DeadReckon {
 public:
 Vec2 p{0, 0};
 double th{0};
 void reset(double x, double y, double heading) {
  p = {x, y};
  th = heading;
 }
 void update(double velocity, double angular_velocity, double dt, Rng& rng, double slip_scale = 1.0,
             double yaw_bias = 0.0, bool has_gyro = false, double gyro_rate = 0.0, double gyro_sigma = 0.0);
 double driftErr(Vec2 true_position) const;
};

} // namespace sim
