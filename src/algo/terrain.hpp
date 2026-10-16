#pragma once

#include "math.hpp"

#include <string>
#include <vector>

namespace sim {

struct HeightPatch {
 std::string id;
 Vec2 min;
 Vec2 max;
 double base_height_m{0.0};
 double grade_x{0.0};
 double grade_y{0.0};
 double roughness{0.0};
};

struct TerrainSample {
 double height_m{0.0};
 double grade_x{0.0};
 double grade_y{0.0};
 double roughness{0.0};
};

struct TerrainCapability {
 double mass_kg{1.0};
 double available_drive_force_N{0.0};
 double friction_mu{0.8};
 double max_cross_slope_rad{0.7};
 double uphill_cost_weight{4.0};
 double cross_slope_cost_weight{2.0};
 double roughness_cost_weight{1.0};
};

class TerrainModel {
 public:
 TerrainModel() = default;
 explicit TerrainModel(std::vector<HeightPatch> patches) : patches_(std::move(patches)) {}

 TerrainSample sample(Vec2 position) const;
 double traversal_cost(Vec2 from, Vec2 to, const TerrainCapability& capability) const;
 bool empty() const { return patches_.empty(); }
 const std::vector<HeightPatch>& patches() const { return patches_; }

 private:
 std::vector<HeightPatch> patches_;
};

double longitudinal_grade(const TerrainSample& terrain, double heading_rad);
double lateral_grade(const TerrainSample& terrain, double heading_rad);
double static_rollover_angle(double support_width_m, double center_of_mass_height_m);
double rollover_risk(double cross_grade, double support_width_m, double center_of_mass_height_m);
double roughness_lateral_acceleration(double roughness_m, double speed_mps, double support_width_m);
double dynamic_rollover_risk(double cross_grade, double lateral_accel_mps2, double support_width_m,
                             double center_of_mass_height_m);

} // namespace sim
