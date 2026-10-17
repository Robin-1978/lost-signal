#pragma once

#include "content_catalog.hpp"

#include <any>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace mission {

// Runtime component contracts. JSON stays flexible at the content boundary;
// simulation code receives named, unit-bearing fields after one validation
// pass instead of repeatedly looking up string keys in the hot path.
struct TireSpec {
 double slip_scale{0.0};
 double brake_decel_mps2{0.0};
 double power_scale{0.0};
 double friction_mu{0.0};
 double rolling_resistance{0.0};
 double normal_stiffness_Npm{0.0};
 double normal_damping_Nspm{0.0};
};

struct LidarSpec {
 double min_range_m{0.0};
 double range_m{0.0};
 int rays{0};
 double scan_hz{0.0};
 double noise_sigma_m{0.0};
 double range_bias_m{0.0};
 double range_resolution_m{0.0};
 double dropout_probability{0.0};
 double latency_s{0.0};
 double terrain_reflectivity{0.0};
 double obstacle_reflectivity{0.0};
 double actor_reflectivity{0.0};
 double incidence_floor{0.0};
 double distance_attenuation_per_m2{0.0};
 double power_w{0.0};
 int vertical_channels{1};
 std::vector<double> vertical_angles_rad;
 double projection_min_height_m{-0.20};
 double projection_max_height_m{1.50};
 double projection_ground_seed_max_height_m{-0.25};
 double projection_ground_max_slope_rad{30.0 * sim::kPi / 180.0};
};

struct ComputeSpec {
 double decision_hz{0.0};
 double prediction_horizon_s{0.0};
 double power_w{0.0};
 double track_max_age_s{0.0};
};

struct MotorSpec {
 double max_wheel_speed_mps{0.0};
 double stall_force_N{0.0};
 double free_speed_mps{0.0};
 double power_scale{0.0};
 double controller_bandwidth_radps{0.0};
 double controller_integral_time_s{0.0};
 double controller_derivative_time_s{0.0};
 double coulomb_smoothing_speed_mps{0.0};
 double viscous_drag_N_per_mps{0.0};
};

struct ImuSpec {
 double gyro_noise_density_radps_sqrt_hz{0.0};
 double accel_noise_density_mps2_sqrt_hz{0.0};
 double noise_bandwidth_hz{0.0};
 double gyro_noise_radps{0.0};
 double gyro_bias_stddev_radps{0.0};
 double power_w{0.0};
 double accel_noise_mps2{0.0};
 double gyro_bias_random_walk_radps_sqrt_s{0.0};
};

// Resolvers store values behind std::any so an extension can introduce its
// own strong type without editing a central variant. ResolvedBuild::get<T>
// performs the checked cast at the consuming system boundary.
using ComponentSpec = std::any;

struct ResolvedComponent {
 const ComponentDefinition* definition{nullptr};
 ComponentSpec spec;
};

class ResolvedBuild {
 public:
 bool has(const std::string& slot) const;
 const ComponentDefinition& definition(const std::string& slot) const;

 template <typename Spec> const Spec* get(const std::string& slot) const {
  const auto found = components_.find(slot);
  if (found == components_.end()) {
   return nullptr;
  }
  const auto* result = std::any_cast<Spec>(&found->second.spec);
  if (!result) {
   throw ContentError("component resolver for slot '" + slot + "' returned the wrong runtime type");
  }
  return result;
 }

 const TireSpec& tires() const;
 const LidarSpec* lidar() const;
 const ComputeSpec* computer() const;
 const MotorSpec* motor() const;
 const ImuSpec* imu() const;

 private:
 friend class ComponentRegistry;

 std::map<std::string, ResolvedComponent> components_;
};

class ComponentRegistry {
 public:
 using Resolver = std::function<ComponentSpec(const ComponentDefinition&)>;

 // Returns a process-wide immutable registry containing the five shipped
 // component slots. Register extensions on a separate instance.
 static const ComponentRegistry& builtins();
 static ComponentRegistry with_builtins();

 void register_slot(std::string slot, Resolver resolver);
 bool supports(const std::string& slot) const;
 std::vector<std::string> supported_slots() const;
 ResolvedBuild resolve(const ContentCatalog& catalog, const BuildSelection& build) const;

 private:
 std::map<std::string, Resolver> resolvers_;
};

} // namespace mission
