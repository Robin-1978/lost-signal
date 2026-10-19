#pragma once

#include "robot/core/math.hpp"

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
 double grade_xy_per_m{0.0};
};

// A bounded, oriented support surface published by construction or another
// runtime world provider.  It is separate from authored height patches so a
// bridge can be added atomically without rewriting the source map.  Height is
// measured at the surface centre and grade_along_local_x follows its long axis.
struct SupportSurface {
 std::string id;
 Vec2 center;
 double half_length_m{0.0};
 double half_width_m{0.0};
 double yaw_rad{0.0};
 double center_height_m{0.0};
 double grade_along_local_x{0.0};
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
 explicit TerrainModel(std::vector<HeightPatch> patches,
 std::vector<SupportSurface> supports = {})
 : patches_(std::move(patches)), supports_(std::move(supports)) {}

 TerrainSample sample(Vec2 position) const;
 double traversal_cost(Vec2 from, Vec2 to, const TerrainCapability& capability) const;
 bool empty() const { return patches_.empty(); }
 const std::vector<HeightPatch>& patches() const { return patches_; }
 const std::vector<SupportSurface>& supports() const { return supports_; }

 private:
 std::vector<HeightPatch> patches_;
 std::vector<SupportSurface> supports_;
};

double longitudinal_grade(const TerrainSample& terrain, double heading_rad);
double lateral_grade(const TerrainSample& terrain, double heading_rad);
double static_rollover_angle(double support_width_m, double center_of_mass_height_m);
double rollover_risk(double cross_grade, double support_width_m, double center_of_mass_height_m);
double roughness_lateral_acceleration(double roughness_m, double speed_mps, double support_width_m);
double dynamic_rollover_risk(double cross_grade, double lateral_accel_mps2, double support_width_m,
                             double center_of_mass_height_m);

// Symmetric command-acceleration ceiling that preserves a requested fraction
// of both front and rear longitudinal support margins on the current grade.
// Distances are measured from the combined COM projection to the support
// edges. The result limits target changes; tire/contact physics still decides
// the acceleration that is actually realized.
double longitudinal_stability_acceleration_limit(double longitudinal_grade,
 double front_support_m, double rear_support_m, double center_of_mass_height_m,
 double margin_fraction = 0.70);

} // namespace sim
