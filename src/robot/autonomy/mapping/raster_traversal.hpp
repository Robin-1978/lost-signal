#pragma once

#include "robot/autonomy/mapping/terrain.hpp"
#include "robot/runtime/map_provider.hpp"
#include "robot/runtime/observation.hpp"

#include <cstdint>
#include <memory>
#include <unordered_map>

namespace sim {

// Robot-side terrain estimate built only from the published Robot Map and
// calibrated hardware priors. It never reads scenario materials, weather,
// contact coefficients or a physics-engine body state.
struct RasterTraversalConfig {
 double tire_friction_mu{0.8};
 double tire_rolling_resistance{0.01};
 double tire_contact_time_constant_s{0.006};
 double tire_contact_damping_ratio{1.0};
 double reference_surface_friction_mu{0.8};
 double default_surface_friction_mu{0.8};
 // Unknown terrain is deliberately conservative. This is a prior for the
 // planner, not a claim that the surface is rough; motion observations can
 // reduce or increase it as confidence becomes available.
 double default_roughness_m{0.02};
 double roughness_observation_floor_m{0.005};
 double roughness_observation_ceiling_m{0.20};
 double roughness_observation_deadband_mps2{0.35};
 double roughness_observation_min_speed_mps{0.20};
 double roughness_belief_time_constant_s{1.5};
 double roughness_belief_full_confidence_s{2.0};
 double roughness_belief_max_age_s{3.0};
 double roughness_support_base_m{0.435};
 double brake_decel_mps2{1.5};
 double max_wheel_accel_mps2{4.0};
};

struct TraversalContactEstimate {
 double friction_mu{0.8};
 double rolling_resistance{0.01};
 double accel_limit_mps2{4.0};
 double contact_time_constant_s{0.006};
 double contact_damping_ratio{1.0};
};

class RasterTraversalModel {
 public:
 explicit RasterTraversalModel(RasterTraversalConfig config = {});

 void set_config(RasterTraversalConfig config);
 void bind(std::shared_ptr<const runtime::MapSnapshot> map);
 // Update the ROBOT-side terrain belief from LSRI motion observations. No
 // simulator contact or authored terrain value is accepted here.
 void observe_motion(const EstimatedState& estimate,
 const ImuObservation& imu, double dt_s);
 bool ready() const { return map_ && map_->grid.has_value(); }
 std::uint64_t revision() const { return map_ ? map_->revision : 0; }

 TerrainSample sample(Vec2 position) const;
 TraversalContactEstimate contact(Vec2 position) const;
 double traversal_cost(Vec2 from, Vec2 to,
 TerrainCapability capability) const;

 private:
 double friction_prior(Vec2 position) const;
 double rolling_prior(Vec2 position) const;
 double local_elevation(double local_x, double local_y) const;
 double local_roughness(double local_x, double local_y) const;
 double prior_roughness(Vec2 position, double* confidence) const;

 RasterTraversalConfig config_;
 std::shared_ptr<const runtime::MapSnapshot> map_;
 struct RoughnessBelief {
  double roughness_m{0.0};
  double confidence{0.0};
  double observed_at_s{0.0};
 };
 std::unordered_map<std::uint64_t, RoughnessBelief> roughness_beliefs_;
 double latest_observation_time_s_{0.0};
 bool have_observation_time_{false};
};

} // namespace sim
