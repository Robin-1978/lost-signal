#include "robot/autonomy/planning/path_smoothing.hpp"

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>

namespace sim {

FilletPath fillet_polyline(const std::vector<Vec2>& route,
 double radius_m, double max_chord_m) {
 FilletPath out;
 std::vector<Vec2> points;
 for (const auto& point : route) {
 if (points.empty() || std::hypot(point.x - points.back().x,
 point.y - points.back().y) >= 1e-9) {
 points.push_back(point);
 }
 }
 if (points.empty() || !(radius_m > 1e-9) || !(max_chord_m > 1e-9))
 return out;
 if (points.size() < 3) {
 out.points = std::move(points);
 out.curvature_inv_m.assign(out.points.size(), 0.0);
 out.valid = true;
 out.fully_smoothed = true;
 return out;
 }

 std::vector<double> leg_lengths;
 leg_lengths.reserve(points.size() - 1);
 for (std::size_t index = 0; index + 1 < points.size(); ++index) {
 leg_lengths.push_back(std::hypot(points[index + 1].x - points[index].x,
 points[index + 1].y - points[index].y));
 }

 out.points.push_back(points.front());
 out.curvature_inv_m.push_back(0.0);
 out.fully_smoothed = true;
 for (std::size_t index = 1; index + 1 < points.size(); ++index) {
 Vec2 incoming{points[index].x - points[index - 1].x,
 points[index].y - points[index - 1].y};
 Vec2 outgoing{points[index + 1].x - points[index].x,
 points[index + 1].y - points[index].y};
 const double incoming_length = std::hypot(incoming.x, incoming.y);
 const double outgoing_length = std::hypot(outgoing.x, outgoing.y);
 if (incoming_length <= 1e-12 || outgoing_length <= 1e-12)
 return {};
 incoming = {incoming.x / incoming_length, incoming.y / incoming_length};
 outgoing = {outgoing.x / outgoing_length, outgoing.y / outgoing_length};
 const double cosine = std::clamp(
 incoming.x * outgoing.x + incoming.y * outgoing.y, -1.0, 1.0);
 const double turn_angle = std::acos(cosine);
 if (turn_angle < 1e-6) {
 out.points.push_back(points[index]);
 out.curvature_inv_m.push_back(0.0);
 continue;
 }

 const double tangent_length = radius_m * std::tan(turn_angle / 2.0);
 if (!std::isfinite(tangent_length) ||
 tangent_length > 0.5 * leg_lengths[index - 1] + 1e-12 ||
 tangent_length > 0.5 * leg_lengths[index] + 1e-12) {
 out.points.push_back(points[index]);
 out.curvature_inv_m.push_back(0.0);
 out.fully_smoothed = false;
 continue;
 }

 const Vec2 toward_incoming{-incoming.x, -incoming.y};
 const Vec2 tangent_in{
 points[index].x + toward_incoming.x * tangent_length,
 points[index].y + toward_incoming.y * tangent_length,
 };
 const Vec2 tangent_out{
 points[index].x + outgoing.x * tangent_length,
 points[index].y + outgoing.y * tangent_length,
 };
 const double cross = incoming.x * outgoing.y - incoming.y * outgoing.x;
 if (std::abs(cross) < 1e-9) {
 out.points.push_back(points[index]);
 out.curvature_inv_m.push_back(0.0);
 out.fully_smoothed = false;
 continue;
 }
 const double turn_sign = cross > 0.0 ? 1.0 : -1.0;
 // For a tangent circular fillet the centre is exactly one signed normal
 // radius from either tangent point. Using the external-angle bisector here
 // sends a shallow turn's centre roughly R/sin(theta/2) away and creates a
 // large loop instead of a corner fillet.
 const Vec2 centre{
 tangent_in.x - turn_sign * incoming.y * radius_m,
 tangent_in.y + turn_sign * incoming.x * radius_m,
 };
 const double start_angle = std::atan2(
 tangent_in.y - centre.y, tangent_in.x - centre.x);
 double sweep = std::atan2(
 tangent_out.y - centre.y, tangent_out.x - centre.x) - start_angle;
 while (cross > 0.0 && sweep <= 0.0)
 sweep += 2.0 * kPi;
 while (cross < 0.0 && sweep >= 0.0)
 sweep -= 2.0 * kPi;
 const int steps = std::max(2, static_cast<int>(std::ceil(
 radius_m * std::abs(sweep) / max_chord_m)));
 const double signed_curvature = turn_sign / radius_m;
 out.points.push_back(tangent_in);
 out.curvature_inv_m.push_back(0.0);
 for (int step = 1; step < steps; ++step) {
 const double angle = start_angle + sweep * static_cast<double>(step) /
 static_cast<double>(steps);
 out.points.push_back({centre.x + radius_m * std::cos(angle),
 centre.y + radius_m * std::sin(angle)});
 out.curvature_inv_m.push_back(signed_curvature);
 }
 out.points.push_back(tangent_out);
 out.curvature_inv_m.push_back(signed_curvature);
 }
 out.points.push_back(points.back());
 out.curvature_inv_m.push_back(0.0);
 out.valid = out.points.size() == out.curvature_inv_m.size();
 if (!out.valid)
 return {};
 return out;
}

} // namespace sim
