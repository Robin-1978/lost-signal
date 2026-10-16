#pragma once

#include "algo/dynamic_obstacle_map.hpp"
#include "hal.hpp"
#include "sim/runtime_state.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace sim {

struct LidarConfig {
 int rays{0};
 double range_m{0.0};
 double scan_hz{0.0};
 LidarNoiseModel noise;
 std::string frame_id{"lidar_link"};
 double latency_s{0.0};
 Vec2 position_in_base;
 double yaw_in_base_rad{0.0};
};

struct PerceptionFrame {
 std::vector<Vec2> lidar_hits;
 bool static_obstacle_near{false};
 bool dynamic_object_detected{false};
};

class SensorSystem {
 public:
 SensorSystem(const GridMap& static_map, std::uint32_t noise_seed, std::uint32_t perception_seed);

 void reset(std::uint32_t noise_seed, std::uint32_t perception_seed);
 ObservationFrame capture_lidar(const WorldTruth& truth, double time_s, const LidarConfig& config);
 ObservationFrame capture_lidar(const WorldTruth& truth, double time_s, const LidarConfig& config,
                                bool include_pose_observation);
 ObservationFrame capture_lidar(const WorldTruth& truth, double time_s, const LidarConfig& config,
 const LidarRaycastTruth& raycast_truth, bool include_pose_observation = false);
 PoseObservation capture_pose(const WorldTruth& truth, double time_s,
 double position_sigma_m = 0.01,
                              double heading_sigma_rad = 0.006);
 PerceptionFrame process_lidar(const ObservationFrame& observation, const EstimatedState& estimate,
                               double track_max_age_s);
 const DynamicObstacleMap& dynamic_obstacles() const { return dynamic_obstacles_; }
 DynamicObstacleMap& dynamic_obstacles() { return dynamic_obstacles_; }

 private:
 struct PendingDetection {
  std::size_t id{0};
  Vec2 position;
  double radius_m{0.45};
  double last_seen_s{0.0};
  int confirmations{0};
 };

 const GridMap& static_map_;
 std::unique_ptr<LidarBackend> lidar_;
 Rng noise_rng_;
 Rng perception_rng_;
 Rng localization_rng_;
 DynamicObstacleMap dynamic_obstacles_;
 std::vector<PendingDetection> pending_;
 std::size_t next_track_id_{0};
 std::uint64_t lidar_sequence_{0};
 std::uint64_t pose_sequence_{0};
 LidarConfig last_config_;
};

} // namespace sim
