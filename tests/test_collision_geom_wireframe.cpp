#include <cassert>
#include <cmath>
#include <cstdio>

#include "sim/viewer/collision_geom_wireframe.hpp"

namespace {

lost_signal::sim::viewer::FrameGeom make_geom(int shape) {
 using lost_signal::sim::viewer::FrameGeom;
 FrameGeom geom;
 geom.valid = true;
 geom.shape = shape;
 geom.size_x_m = 0.2;
 geom.size_y_m = 0.5;
 geom.size_z_m = 0.3;
 geom.x_m = 1.0;
 geom.y_m = 2.0;
 geom.z_m = 3.0;
 return geom;
}

bool near(double lhs, double rhs) {
 return std::abs(lhs - rhs) < 1e-5;
}

} // namespace

int main() {
 using namespace lost_signal::sim::viewer;

 // SIM (x, y, z) maps to display (x, z, -y).
 {
 const auto wires = build_collision_geom_wireframe(make_geom(2));
 assert(wires.spheres.size() == 1);
 assert(near(wires.spheres[0].center.x, 1.0));
 assert(near(wires.spheres[0].center.y, 3.0));
 assert(near(wires.spheres[0].center.z, -2.0));
 assert(near(wires.spheres[0].radius_m, 0.2));
 }

 // A MuJoCo cylinder is local-Z. Identity rotation therefore makes it
 // vertical in the raylib display basis.
 {
 auto geom = make_geom(5);
 geom.size_x_m = 0.1;
 geom.size_y_m = 0.4;
 const auto wires = build_collision_geom_wireframe(geom);
 assert(wires.cylinders.size() == 1);
 assert(near(wires.cylinders[0].start.x, 1.0));
 assert(near(wires.cylinders[0].start.y, 2.6));
 assert(near(wires.cylinders[0].start.z, -2.0));
 assert(near(wires.cylinders[0].end.y, 3.4));
 }

 // A capsule preserves the cylinder and exposes both collision rounded ends.
 {
 auto geom = make_geom(3);
 geom.qw = std::cos(3.14159265358979323846 / 4.0);
 geom.qx = std::sin(3.14159265358979323846 / 4.0);
 const auto wires = build_collision_geom_wireframe(geom);
 assert(wires.cylinders.size() == 1);
 assert(wires.spheres.size() == 2);
 // Rotating local Z by +90 degrees about SIM X gives SIM -Y, then display
 // +Z.
 assert(near(wires.cylinders[0].end.y, wires.cylinders[0].start.y));
 assert(std::abs(wires.cylinders[0].end.z -
 wires.cylinders[0].start.z) > 0.79);
 }

 // Boxes expand to 12 true edges. Every edge has its authored half-length on
 // one display axis, even with the non-identity SIM-to-display basis.
 {
 auto geom = make_geom(6);
 geom.size_x_m = 0.2;
 geom.size_y_m = 0.5;
 geom.size_z_m = 0.3;
 const auto wires = build_collision_geom_wireframe(geom);
 assert(wires.segments.size() == 12);
 int x_edges = 0;
 int y_edges = 0;
 int z_edges = 0;
 for (const auto& edge : wires.segments) {
 const double dx = std::abs(edge.end.x - edge.start.x);
 const double dy = std::abs(edge.end.y - edge.start.y);
 const double dz = std::abs(edge.end.z - edge.start.z);
 if (near(dx, 0.4) && near(dy, 0.0) && near(dz, 0.0)) ++x_edges;
 if (near(dx, 0.0) && near(dy, 0.6) && near(dz, 0.0)) ++y_edges;
 if (near(dx, 0.0) && near(dy, 0.0) && near(dz, 1.0)) ++z_edges;
 }
 assert(x_edges == 4);
 assert(y_edges == 4);
 assert(z_edges == 4);
 }

 // Invalid engine evidence never produces a plausible-looking proxy.
 {
 auto geom = make_geom(6);
 geom.qw = NAN;
 assert(build_collision_geom_wireframe(geom).empty());
 geom = make_geom(6);
 geom.size_z_m = -1.0;
 assert(!build_collision_geom_wireframe(geom).empty());
 }

 std::printf("collision_geom_wireframe ok\n");
 return 0;
}
