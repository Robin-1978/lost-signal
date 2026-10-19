#pragma once

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <vector>

namespace sim {

struct CommandWindow {
 double minimum{0.0};
 double maximum{0.0};
};

// Reachable command interval for the next control update. If a newly reduced
// behavior cap lies entirely beyond that interval, return the closest
// rate-limited recovery command instead of pretending the actuator can clamp
// to the cap instantaneously.
inline CommandWindow next_command_window(double current, double desired_minimum,
 double desired_maximum, double max_delta) {
 const double reachable_minimum = current - std::max(0.0, max_delta);
 const double reachable_maximum = current + std::max(0.0, max_delta);
 if (reachable_maximum < desired_minimum)
 return {reachable_maximum, reachable_maximum};
 if (reachable_minimum > desired_maximum)
 return {reachable_minimum, reachable_minimum};
 return {
 std::max(reachable_minimum, desired_minimum),
 std::min(reachable_maximum, desired_maximum),
 };
}

inline double next_rate_limited_command(double current, double desired,
 double desired_minimum, double desired_maximum, double max_delta) {
 desired = std::clamp(desired, desired_minimum, desired_maximum);
 return current + std::clamp(desired - current, -std::max(0.0, max_delta),
 std::max(0.0, max_delta));
}

inline std::vector<double> command_samples(CommandWindow window, double spacing,
 const std::vector<double>& preferred) {
 std::vector<double> samples;
 const auto append = [&](double value) {
 value = std::clamp(value, window.minimum, window.maximum);
 if (std::none_of(samples.begin(), samples.end(),
 [&](double existing) { return std::abs(existing - value) <= 1e-9; })) {
 samples.push_back(value);
 }
 };
 append(window.minimum);
 if (spacing > 0.0) {
 const double first = std::ceil(window.minimum / spacing) * spacing;
 for (double value = first; value <= window.maximum + 1e-9; value += spacing)
 append(value);
 }
 for (double value : preferred) {
 if (value >= window.minimum - 1e-9 && value <= window.maximum + 1e-9)
 append(value);
 }
 append(window.maximum);
 std::sort(samples.begin(), samples.end());
 return samples;
}

inline std::vector<double> command_samples(CommandWindow window, double spacing,
 std::initializer_list<double> preferred) {
 return command_samples(window, spacing, std::vector<double>(preferred));
}

} // namespace sim
