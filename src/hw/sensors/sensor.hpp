#pragma once

#include "autonomy/perception/dynamic_object.hpp"
#include "autonomy/mapping/grid_map.hpp"

#include <cmath>
#include <cstdint>
#include <random>
#include <string>
#include <vector>

namespace sim {

struct Rng {
 std::mt19937 gen;
 explicit Rng(uint32_t seed = 12345) : gen(seed) {}
 double gauss(double standard_deviation = 1.0) {
 // Marsaglia's polar transform over our fixed uniform mapping preserves the
 // established simulation sequence and makes it independent of the standard
 // library's normal_distribution implementation.
 double x = 0.0;
 double y = 0.0;
 double radius_squared = 0.0;
 do {
 x = 2.0 * uniform01() - 1.0;
 y = 2.0 * uniform01() - 1.0;
 radius_squared = x * x + y * y;
 } while (radius_squared > 1.0 || radius_squared == 0.0);
 const double scale = std::sqrt(-2.0 * std::log(radius_squared) / radius_squared);
 const double standard_normal = y * scale;
 return standard_normal * standard_deviation;
 }
 double uniform01() {
 // Two engine outputs fill a 64-bit fraction in a documented order. This is
 // the established project sequence without relying on generate_canonical.
 constexpr double range = 4294967296.0;
 const double low = static_cast<double>(gen());
 const double high = static_cast<double>(gen());
 return (low + high * range) / (range * range);
 }
};

// Header shared by every simulated or hardware measurement. stamp_s is the
// acquisition time in the simulation/robot monotonic clock; latency_s is the
// known delivery delay. sequence is per sensor stream and frame_id names the
// calibrated transform frame. Providers translate this contract to their
// native device representation at the platform boundary.
struct MeasurementHeader {
 double stamp_s{0.0};
 std::uint64_t sequence{0};
 std::string frame_id;
 double latency_s{0.0};
 // Version and source identity make the observation contract safe to bridge
 // across an in-process simulator, native device driver, replay, or IPC.
 // They trail the legacy fields so existing four-field aggregate initializers
 // keep their meaning.
 std::uint32_t schema_version{1};
 std::string source_id;
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

// Normalized first-return model used after ideal 3D geometry selects the
// nearest surface. Values describe a named sensor/material profile; they are
// observation parameters and never alter collision or tire friction.
struct LidarReturnModel {
 double terrain_reflectivity{0.30};
 double obstacle_reflectivity{0.68};
 double actor_reflectivity{0.52};
 double default_reflectivity{0.50};
 double incidence_floor{0.15};
 double distance_attenuation_per_m2{0.018};
};

Scan apply_lidar_noise(Scan ideal_scan, const LidarNoiseModel& model, Rng& rng);
Scan apply_lidar_noise(Scan ideal_scan, double noise_std, Rng& rng);

// Ideal deterministic geometry query. Static occupancy and moving collision
// circles participate in the same nearest-hit and occlusion test.
Scan lidar_raycast(const GridMap& map, Vec2 position, double heading, int n_rays,
 double max_range, const std::vector<DynamicObjectState>& dynamic_objects = {});

// Compatibility helper for callers that need an immediately-noised scan.
Scan lidar_scan(const GridMap& map, Vec2 position, double heading, int n_rays, double max_range,
                double noise_std, Rng& rng, const std::vector<DynamicObjectState>& dynamic_objects = {});

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
