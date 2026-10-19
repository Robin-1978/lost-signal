#include "sim/hardware/physics/raycast_geometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace sim::physics {

RaycastResult raycast_upright_finite_cylinder(
 const Ray& ray, BodyHandle body, Vec3 center_m, double radius_m, double height_m) {
 RaycastResult result;
 if (!(radius_m > 0.0) || !(height_m > 0.0) || !(ray.max_distance_m >= 0.0))
 return result;
 const double direction_norm = std::sqrt(
 ray.direction.x * ray.direction.x + ray.direction.y * ray.direction.y +
 ray.direction.z * ray.direction.z);
 if (!(direction_norm > 1e-12) || !std::isfinite(direction_norm))
 return result;
 const Vec3 direction{
 ray.direction.x / direction_norm,
 ray.direction.y / direction_norm,
 ray.direction.z / direction_norm,
 };
 const double offset_x = ray.origin_m.x - center_m.x;
 const double offset_y = ray.origin_m.y - center_m.y;
 const double half_height = height_m * 0.5;
 double best_distance = std::numeric_limits<double>::infinity();
 Vec3 best_normal;

 const auto accept = [&](double distance, Vec3 normal) {
 if (distance < -1e-12 || distance > ray.max_distance_m || distance >= best_distance)
 return;
 best_distance = std::max(0.0, distance);
 best_normal = normal;
 };

 // Curved side. Consider both roots because a ray may originate inside the
 // radial projection of the cylinder and exit through the far side.
 const double a = direction.x * direction.x + direction.y * direction.y;
 if (a > 1e-12) {
 const double b = 2.0 * (offset_x * direction.x + offset_y * direction.y);
 const double c = offset_x * offset_x + offset_y * offset_y - radius_m * radius_m;
 const double discriminant = b * b - 4.0 * a * c;
 if (discriminant >= 0.0) {
 const double root = std::sqrt(std::max(0.0, discriminant));
 const double roots[] = {(-b - root) / (2.0 * a), (-b + root) / (2.0 * a)};
 for (const double distance : roots) {
 const double z = ray.origin_m.z + direction.z * distance;
 if (z < center_m.z - half_height - 1e-12 ||
 z > center_m.z + half_height + 1e-12) {
 continue;
 }
 const double x = ray.origin_m.x + direction.x * distance - center_m.x;
 const double y = ray.origin_m.y + direction.y * distance - center_m.y;
 const double normal_norm = std::max(1e-12, std::hypot(x, y));
 accept(distance, {x / normal_norm, y / normal_norm, 0.0});
 }
 }
 }

 // Closed top and bottom caps. Their plane hit is accepted only inside the
 // circular disc, including the rim shared with the side.
 if (std::abs(direction.z) > 1e-12) {
 for (const double sign : {-1.0, 1.0}) {
 const double cap_z = center_m.z + sign * half_height;
 const double distance = (cap_z - ray.origin_m.z) / direction.z;
 const double x = ray.origin_m.x + direction.x * distance - center_m.x;
 const double y = ray.origin_m.y + direction.y * distance - center_m.y;
 if (x * x + y * y <= radius_m * radius_m + 1e-12)
 accept(distance, {0.0, 0.0, sign});
 }
 }

 if (!std::isfinite(best_distance))
 return result;
 result.hit = true;
 result.body = body;
 result.distance_m = best_distance;
 result.point_m = {
 ray.origin_m.x + direction.x * best_distance,
 ray.origin_m.y + direction.y * best_distance,
 ray.origin_m.z + direction.z * best_distance,
 };
 result.normal = best_normal;
 return result;
}

} // namespace sim::physics
