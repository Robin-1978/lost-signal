#pragma once

#include <vector>

#include "sim/viewer/frame.hpp"

namespace lost_signal::sim::viewer {

// raylib display basis: X forward, Y up, Z right. These primitives are built
// without raylib so the world-to-display transform and shape expansion are
// testable in headless builds.
struct WirePoint3 {
 float x = 0.0f;
 float y = 0.0f;
 float z = 0.0f;
};

struct WireSegment3 {
 WirePoint3 start;
 WirePoint3 end;
};

struct WireSphere3 {
 WirePoint3 center;
 float radius_m = 0.0f;
};

struct WireCylinder3 {
 WirePoint3 start;
 WirePoint3 end;
 float radius_m = 0.0f;
};

struct CollisionGeomWireframe {
 std::vector<WireSegment3> segments;
 std::vector<WireSphere3> spheres;
 std::vector<WireCylinder3> cylinders;

 bool empty() const {
 return segments.empty() && spheres.empty() && cylinders.empty();
 }
};

// Converts one MuJoCo world geom into display-space wire primitives. Boxes
// become 12 edge segments; capsules get capped cylinders; unsupported shapes
// remain visible as their bounding sphere.
CollisionGeomWireframe build_collision_geom_wireframe(const FrameGeom& geom);

} // namespace lost_signal::sim::viewer
