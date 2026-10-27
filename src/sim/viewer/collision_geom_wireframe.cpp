#include "sim/viewer/collision_geom_wireframe.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace lost_signal::sim::viewer {
namespace {

struct SimPoint3 {
 double x = 0.0;
 double y = 0.0;
 double z = 0.0;
};

bool finite(const FrameGeom& geom) {
 return geom.valid && std::isfinite(geom.size_x_m) &&
 std::isfinite(geom.size_y_m) && std::isfinite(geom.size_z_m) &&
 std::isfinite(geom.x_m) && std::isfinite(geom.y_m) &&
 std::isfinite(geom.z_m) && std::isfinite(geom.qw) &&
 std::isfinite(geom.qx) && std::isfinite(geom.qy) &&
 std::isfinite(geom.qz);
}

WirePoint3 to_display(const SimPoint3& point) {
 return {static_cast<float>(point.x), static_cast<float>(point.z),
 static_cast<float>(-point.y)};
}

SimPoint3 rotate_vector(const FrameGeom& geom, const SimPoint3& point) {
 const double norm = std::sqrt(geom.qw * geom.qw + geom.qx * geom.qx +
 geom.qy * geom.qy + geom.qz * geom.qz);
 if (!(norm > 0.0) || !std::isfinite(norm)) return {};
 const double w = geom.qw / norm;
 const double qx = geom.qx / norm;
 const double qy = geom.qy / norm;
 const double qz = geom.qz / norm;
 const double tx = 2.0 * (qy * point.z - qz * point.y);
 const double ty = 2.0 * (qz * point.x - qx * point.z);
 const double tz = 2.0 * (qx * point.y - qy * point.x);
 return {
 point.x + w * tx + qy * tz - qz * ty,
 point.y + w * ty + qz * tx - qx * tz,
 point.z + w * tz + qx * ty - qy * tx,
 };
}

SimPoint3 rotate(const FrameGeom& geom, const SimPoint3& point) {
 return rotate_vector(geom, point);
}

WirePoint3 rotate_display(const FrameGeom& geom, const SimPoint3& point) {
 return to_display(rotate(geom, point));
}

WirePoint3 add(WirePoint3 lhs, WirePoint3 rhs) {
 return {lhs.x + rhs.x, lhs.y + rhs.y, lhs.z + rhs.z};
}

WirePoint3 point_at(const FrameGeom& geom, const SimPoint3& local) {
 const SimPoint3 rotated = rotate(geom, local);
 return to_display(
 {geom.x_m + rotated.x, geom.y_m + rotated.y, geom.z_m + rotated.z});
}

void add_edge(CollisionGeomWireframe& out, const std::array<WirePoint3, 8>& c,
 int a, int b) {
 out.segments.push_back({c[static_cast<std::size_t>(a)],
 c[static_cast<std::size_t>(b)]});
}

} // namespace

CollisionGeomWireframe build_collision_geom_wireframe(const FrameGeom& geom) {
 CollisionGeomWireframe out;
 if (!finite(geom)) return out;
 const double sx = std::abs(geom.size_x_m);
 const double sy = std::abs(geom.size_y_m);
 const double sz = std::abs(geom.size_z_m);
 switch (geom.shape) {
 case 6: { // box half extents
 std::array<WirePoint3, 8> corners{};
 int index = 0;
 for (int x : {-1, 1}) {
 for (int y : {-1, 1}) {
 for (int z : {-1, 1}) {
 corners[static_cast<std::size_t>(index++)] = point_at(
 geom, {x * sx, y * sy, z * sz});
 }
 }
 }
 // Corners are indexed with z fastest, then y, then x.
 for (int x = 0; x < 2; ++x) {
 for (int y = 0; y < 2; ++y) {
 for (int z = 0; z < 2; ++z) {
 const int i = x * 4 + y * 2 + z;
 if (y == 0) add_edge(out, corners, i, i + 2);
 if (z == 0) add_edge(out, corners, i, i + 1);
 if (x == 0) add_edge(out, corners, i, i + 4);
 }
 }
 }
 break;
 }
 case 3: // capsule: radius + half segment along local Z
 case 5: { // cylinder: radius + half length along local Z
 if (!(sx > 0.0)) break;
 const double half = sy;
 const WirePoint3 center = to_display({geom.x_m, geom.y_m, geom.z_m});
 const WirePoint3 axis = rotate_display(geom, {0.0, 0.0, half});
 const WirePoint3 start{center.x - axis.x, center.y - axis.y,
 center.z - axis.z};
 const WirePoint3 end{
 center.x + axis.x, center.y + axis.y, center.z + axis.z};
 if (half > 0.0) {
 out.cylinders.push_back(
 {start, end, static_cast<float>(sx)});
 }
 if (geom.shape == 3) {
 out.spheres.push_back({start, static_cast<float>(sx)});
 out.spheres.push_back({end, static_cast<float>(sx)});
 }
 break;
 }
 case 2: { // sphere: size_x is radius
 const double radius = sx;
 if (radius > 0.0) {
 out.spheres.push_back(
 {{static_cast<float>(geom.x_m), static_cast<float>(geom.z_m),
 static_cast<float>(-geom.y_m)},
 static_cast<float>(radius)});
 }
 break;
 }
 default: {
 const double radius = std::max({sx, sy, sz});
 if (radius > 0.0) {
 out.spheres.push_back(
 {{static_cast<float>(geom.x_m), static_cast<float>(geom.z_m),
 static_cast<float>(-geom.y_m)},
 static_cast<float>(radius)});
 }
 break;
 }
 }
 return out;
}

CollisionGeomWireframe build_static_box_wireframe(double x0_m, double y0_m,
                                                  double x1_m, double y1_m,
                                                  double height_m) {
 if (!std::isfinite(x0_m) || !std::isfinite(y0_m) ||
 !std::isfinite(x1_m) || !std::isfinite(y1_m) ||
 !std::isfinite(height_m) || !(height_m > 0.0)) {
 return {};
 }
 const double min_x = std::min(x0_m, x1_m);
 const double max_x = std::max(x0_m, x1_m);
 const double min_y = std::min(y0_m, y1_m);
 const double max_y = std::max(y0_m, y1_m);
 if (!(max_x > min_x) || !(max_y > min_y)) return {};

 FrameGeom geom;
 geom.valid = true;
 geom.shape = 6;
 geom.size_x_m = (max_x - min_x) * 0.5;
 geom.size_y_m = (max_y - min_y) * 0.5;
 geom.size_z_m = height_m * 0.5;
 geom.x_m = (min_x + max_x) * 0.5;
 geom.y_m = (min_y + max_y) * 0.5;
 geom.z_m = height_m * 0.5;
 return build_collision_geom_wireframe(geom);
}

} // namespace lost_signal::sim::viewer
