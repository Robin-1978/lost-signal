#include "robot/autonomy/planning/path_smoothing.hpp"

#include <cmath>
#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ \
 << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 // Independent analytic oracle: a 90-degree, one-metre-radius fillet has
 // tangent points (1,0) and (2,1), and every curved sample lies on the
 // circle centred at (1,1). This catches external/internal angle mix-ups.
 const auto right_angle = sim::fillet_polyline(
 {{0.0, 0.0}, {2.0, 0.0}, {2.0, 2.0}}, 1.0, 0.10);
 CHECK(right_angle.valid);
 CHECK(right_angle.fully_smoothed);
 CHECK(right_angle.points.size() > 4);
 CHECK(std::abs(right_angle.points[1].x - 1.0) < 1e-12);
 CHECK(std::abs(right_angle.points[1].y) < 1e-12);
 CHECK(std::abs(right_angle.points[right_angle.points.size() - 2].x - 2.0) < 1e-12);
 CHECK(std::abs(right_angle.points[right_angle.points.size() - 2].y - 1.0) < 1e-12);
 for (std::size_t index = 2; index + 1 < right_angle.points.size(); ++index) {
 const auto point = right_angle.points[index];
 CHECK(std::abs(std::hypot(point.x - 1.0, point.y - 1.0) - 1.0) < 1e-12);
 CHECK(std::abs(right_angle.curvature_inv_m[index] - 1.0) < 1e-12);
 }

 // A shallow grid corner must stay local to its two legs. The former wrong
 // external-bisector construction produced an excursion below x=14.5 for
 // these world-space points, more than a metre away from the route.
 const auto shallow = sim::fillet_polyline(
 {{15.349, 4.099}, {15.625, 4.375}, {17.125, 8.375}}, 0.50);
 CHECK(shallow.valid);
 CHECK(shallow.fully_smoothed);
 for (const auto point : shallow.points) {
 CHECK(point.x >= 15.349 - 1e-12);
 CHECK(point.x <= 17.125 + 1e-12);
 CHECK(point.y >= 4.099 - 1e-12);
 CHECK(point.y <= 8.375 + 1e-12);
 }

 // A corner with insufficient adjacent-leg room remains explicit. Callers
 // requiring continuous motion must see fully_smoothed=false and fall back
 // to a stop-turn-go or another proved path.
 const auto short_leg = sim::fillet_polyline(
 {{0.0, 0.0}, {0.10, 0.0}, {0.10, 1.0}}, 0.50);
 CHECK(short_leg.valid);
 CHECK(!short_leg.fully_smoothed);
 CHECK(short_leg.points.size() == 3);

 std::cout << "path smoothing green\n";
 return 0;
}
