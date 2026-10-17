#pragma once

#include <cmath>
#include <cstdint>
#include <random>

namespace sim {

// Portable deterministic random stream shared by plant and autonomy models.
// It lives in core so neither layer depends on the other for seeded replay.
struct Rng {
 std::mt19937 gen;
 explicit Rng(std::uint32_t seed = 12345) : gen(seed) {}

 double uniform01() {
 constexpr double range = 4294967296.0;
 const double low = static_cast<double>(gen());
 const double high = static_cast<double>(gen());
 return (low + high * range) / (range * range);
 }

 double gauss(double standard_deviation = 1.0) {
 double x = 0.0;
 double y = 0.0;
 double radius_squared = 0.0;
 do {
 x = 2.0 * uniform01() - 1.0;
 y = 2.0 * uniform01() - 1.0;
 radius_squared = x * x + y * y;
 } while (radius_squared > 1.0 || radius_squared == 0.0);
 const double scale = std::sqrt(-2.0 * std::log(radius_squared) / radius_squared);
 return y * scale * standard_deviation;
 }
};

} // namespace sim
