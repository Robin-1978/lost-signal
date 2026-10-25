#pragma once

#include <algorithm>
#include <cmath>

namespace lost_signal::robot::autonomy {

// Convert a declared probability into confidence radii without embedding an
// unexplained "N sigma" threshold. Position uses the radial quantile of an
// isotropic 2-D Gaussian. Heading uses the two-sided 1-D Gaussian quantile.
inline double position_confidence_radius(
 double sigma_m, double probability) {
 if (!(std::isfinite(sigma_m) && sigma_m >= 0.0) ||
 !(std::isfinite(probability) && probability > 0.0 &&
 probability < 1.0)) {
 return INFINITY;
 }
 return sigma_m * std::sqrt(-2.0 * std::log1p(-probability));
}

inline double heading_confidence_radius(
 double sigma_rad, double probability) {
 if (!(std::isfinite(sigma_rad) && sigma_rad >= 0.0) ||
 !(std::isfinite(probability) && probability > 0.0 &&
 probability < 1.0)) {
 return INFINITY;
 }
 const auto cdf = [](double radius) {
 return std::erf(radius / std::sqrt(2.0));
 };
 double low = 0.0;
 double high = 12.0;
 for (int iteration = 0; iteration < 64; ++iteration) {
 const double middle = 0.5 * (low + high);
 if (cdf(middle) < probability) low = middle;
 else high = middle;
 }
 return sigma_rad * high;
}

inline double confidence_bounded_tolerance(
 double tolerance, double confidence_radius) {
 if (!(std::isfinite(tolerance) && tolerance >= 0.0) ||
 !(std::isfinite(confidence_radius) && confidence_radius >= 0.0)) {
 return 0.0;
 }
 return std::max(0.0, tolerance - confidence_radius);
}

} // namespace lost_signal::robot::autonomy
