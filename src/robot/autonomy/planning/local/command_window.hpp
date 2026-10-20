#pragma once

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
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

// Reachable interval for a signed longitudinal speed with distinct drive and
// brake rates. Forward and reverse route sections are each one-sided; when
// measured motion has the opposite sign, the next command first moves toward
// zero at the braking rate instead of pretending it can change gear in one
// update.
inline CommandWindow next_velocity_command_window(
 double current, double desired_minimum, double desired_maximum,
 double max_accel_delta, double max_decel_delta) {
 max_accel_delta = std::max(0.0, max_accel_delta);
 max_decel_delta = std::max(0.0, max_decel_delta);
 // Encoder/estimator noise cannot be required to cross mathematical zero
 // before a declared gear may engage. Inside this measured standstill band,
 // construct the command window from zero; rollout integration still starts
 // from the unmodified measured velocity and therefore preserves the real
 // braking/reversal transient.
 constexpr double kMeasuredStandstillMps = 0.005;
 const double window_current = std::abs(current) <= kMeasuredStandstillMps
 ? 0.0
 : current;
 double reachable_minimum = window_current;
 double reachable_maximum = window_current;
 if (desired_minimum >= 0.0) {
 if (window_current >= 0.0) {
 reachable_minimum = window_current - max_decel_delta;
 reachable_maximum = window_current + max_accel_delta;
 } else {
 reachable_minimum = std::min(0.0, window_current + max_decel_delta);
 reachable_maximum = reachable_minimum;
 }
 } else if (desired_maximum <= 0.0) {
 if (window_current <= 0.0) {
 reachable_minimum = window_current - max_accel_delta;
 reachable_maximum = window_current + max_decel_delta;
 } else {
 reachable_minimum = std::max(0.0, window_current - max_decel_delta);
 reachable_maximum = reachable_minimum;
 }
 } else {
 const double delta = std::max(max_accel_delta, max_decel_delta);
 reachable_minimum = window_current - delta;
 reachable_maximum = window_current + delta;
 }
 if (reachable_maximum < desired_minimum)
 return {reachable_maximum, reachable_maximum};
 if (reachable_minimum > desired_maximum)
 return {reachable_minimum, reachable_minimum};
 return {
 std::max(reachable_minimum, desired_minimum),
 std::min(reachable_maximum, desired_maximum),
 };
}

inline double velocity_transition_time(double current, double target,
 double accel_mps2, double decel_mps2) {
 accel_mps2 = std::max(1e-9, accel_mps2);
 decel_mps2 = std::max(1e-9, decel_mps2);
 if (current * target < 0.0)
 return std::abs(current) / decel_mps2 + std::abs(target) / accel_mps2;
 const bool increasing_magnitude = std::abs(target) > std::abs(current);
 return std::abs(target - current) /
 (increasing_magnitude ? accel_mps2 : decel_mps2);
}

inline double velocity_step_toward(double current, double target,
 double accel_mps2, double decel_mps2, double duration_s) {
 if (duration_s <= 0.0 || current == target)
 return current;
 accel_mps2 = std::max(0.0, accel_mps2);
 decel_mps2 = std::max(0.0, decel_mps2);
 if (current * target < 0.0) {
 const double stop_time_s = decel_mps2 > 0.0
 ? std::abs(current) / decel_mps2
 : std::numeric_limits<double>::infinity();
 if (duration_s <= stop_time_s)
 return current + std::copysign(decel_mps2 * duration_s, -current);
 const double remaining_s = duration_s - stop_time_s;
 return std::copysign(std::min(std::abs(target), accel_mps2 * remaining_s), target);
 }
 const double rate = std::abs(target) > std::abs(current)
 ? accel_mps2 : decel_mps2;
 return current + std::clamp(target - current, -rate * duration_s,
 rate * duration_s);
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
