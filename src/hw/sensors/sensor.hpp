#pragma once

#include "algo/dynobs.hpp"
#include "algo/grid_map.hpp"

#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace sim {

struct Rng {
 std::mt19937 gen;
 explicit Rng(uint32_t seed = 12345) : gen(seed) {}
 double gauss(double standard_deviation = 1.0) {
  std::normal_distribution<double> distribution(0.0, standard_deviation);
  return distribution(gen);
 }
 double uniform01() {
  return std::generate_canonical<double, 53>(gen);
 }
};

// Header shared by every simulated or hardware measurement. stamp_s is the
// acquisition time in the simulation/robot monotonic clock; latency_s is the
// known delivery delay. sequence is per sensor stream and frame_id names the
// calibrated TF frame. Transport adapters may map this directly to ROS 2.
struct MeasurementHeader {
 double stamp_s{0.0};
 std::uint64_t sequence{0};
 std::string frame_id;
 double latency_s{0.0};
};

struct Scan {
 std::vector<double> ranges;
 std::vector<char> hit;
 // Optional normalized first-return strength. Empty means the backend does
 // not model intensity; consumers must not invent a value from hit alone.
 std::vector<double> intensities;
 int n_rays{0};
 double max_range{8.0};
 double angle_min_rad{0.0};
 double angle_increment_rad{0.0};
 // header.stamp_s is the acquisition end. A mechanical scan assigns each ray
 // an earlier sample time using time_increment_s; solid-state patterns use the
 // point-cloud contract below instead.
 double scan_period_s{0.0};
 double time_increment_s{0.0};
 MeasurementHeader header;
};

enum class LidarPatternKind { PlanarSpinning, MultiBeamSpinning, SolidState };

// Message contract reserved for 3D/multi-line and solid-state LiDAR. A scan
// pattern producer decides azimuth/elevation and per-point timing; the common
// geometry and return models fill this transport-neutral point cloud.
struct LidarPointReturn {
 double x_m{0.0};
 double y_m{0.0};
 double z_m{0.0};
 double intensity{0.0};
 double time_offset_s{0.0};
 std::uint16_t ring{0};
 std::uint8_t return_index{0};
};

struct LidarPointCloud {
 std::vector<LidarPointReturn> points;
 LidarPatternKind pattern{LidarPatternKind::MultiBeamSpinning};
 double scan_period_s{0.0};
 MeasurementHeader header;
};

// Ideal geometry output. The combined scan sees static geometry and actors;
// static_environment is the map-expected scan used for foreground
// differencing. SensorSystem adds measurement noise after raycasting.
struct LidarRaycastTruth {
 Scan combined;
 Scan static_environment;
};

// A LiDAR-specific observation model. These parameters deliberately live at
// the sensor boundary: geometry backends return ideal ranges, then simulation
// applies bias, white noise, range quantization and missed returns. A hardware
// backend publishes its measured Scan directly and never receives simulation
// noise a second time.
struct LidarNoiseModel {
 double range_stddev_m{0.0};
 double range_bias_m{0.0};
 double range_resolution_m{0.0};
 double dropout_probability{0.0};
};

Scan apply_lidar_noise(Scan ideal_scan, const LidarNoiseModel& model, Rng& rng);
Scan apply_lidar_noise(Scan ideal_scan, double noise_std, Rng& rng);

// Ideal deterministic geometry query. Static occupancy and moving collision
// circles participate in the same nearest-hit and occlusion test.
Scan lidar_raycast(const GridMap& map, Vec2 position, double heading, int n_rays,
 double max_range, const std::vector<DynState>& dynamic_objects = {});

// Compatibility helper for callers that need an immediately-noised scan.
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
