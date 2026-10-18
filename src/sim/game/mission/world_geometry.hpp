#pragma once

#include "content.hpp"

#include <string_view>
#include <vector>

namespace mission {

struct ResolvedWorldCollision {
 std::string object_id;
 Vec3 center_m;
 Vec3 size_m;
 double yaw_rad{0.0};
 std::string material;
 double friction_mu{0.8};
 double rolling_drag{0.0};
 double normal_stiffness_Npm{1.0e9};
 double normal_damping_Nspm{0.0};
 double lidar_reflectivity{0.30};
 sim::Vec2 aabb_min;
 sim::Vec2 aabb_max;
};

// Resolve one authoritative world-object representation for physics, LiDAR,
// navigation-map projection and collision monitoring.
std::vector<ResolvedWorldCollision> resolve_world_collisions(
 const MapDefinition& map, std::string_view level_id);

} // namespace mission
