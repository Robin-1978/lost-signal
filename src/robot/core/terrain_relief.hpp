#pragma once

#include "robot/core/math.hpp"

#include <algorithm>
#include <cmath>

namespace sim {

struct ReliefSample {
 double height_m{0.0};
 double gradient_x{0.0};
 double gradient_y{0.0};
};

// Shared map/world relief contract. It contains no planner or simulator
// policy: both the collision mesh and deployable map use this deterministic
// geometry primitive, so they cannot disagree about authored ground.
inline ReliefSample rough_relief(double roughness_m, double width_m, double depth_m,
 double local_x, double local_y) {
 ReliefSample output;
 const double width = std::max(0.1, width_m);
 const double depth = std::max(0.1, depth_m);
 if (!(roughness_m > 0.0) || local_x < 0.0 || local_x > width ||
 local_y < 0.0 || local_y > depth) {
 return output;
 }
 constexpr double wavelength_x_m = 1.35;
 constexpr double wavelength_y_m = 0.95;
 const double u = std::clamp(local_x / width, 0.0, 1.0);
 const double v = std::clamp(local_y / depth, 0.0, 1.0);
 const double fade = std::sin(kPi * u) * std::sin(kPi * v);
 const double kx = 2.0 * kPi / wavelength_x_m;
 const double ky = 2.0 * kPi / wavelength_y_m;
 const double wave_x = std::sin(kx * local_x);
 const double wave_y = std::sin(ky * local_y);
 output.height_m = roughness_m * fade * wave_x * wave_y;
 output.gradient_x = roughness_m *
 (kPi * std::cos(kPi * u) / width * std::sin(kPi * v) * wave_x * wave_y +
 fade * kx * std::cos(kx * local_x) * wave_y);
 output.gradient_y = roughness_m *
 (kPi * std::sin(kPi * u) * std::cos(kPi * v) / depth * wave_x * wave_y +
 fade * ky * std::cos(ky * local_y) * wave_x);
 return output;
}

} // namespace sim
