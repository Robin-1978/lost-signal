#include "sim/viewer/physical_audio.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace lost_signal::sim::viewer {
namespace {

double nonnegative(double value) {
 return std::isfinite(value) ? std::max(0.0, value) : 0.0;
}

double unit_ratio(double value, double reference) {
 if (!(reference > 0.0) || !std::isfinite(reference)) return 0.0;
 return std::clamp(nonnegative(value) / reference, 0.0, 1.0);
}

double response_alpha(double elapsed_s, double response_time_s) {
 if (!(elapsed_s > 0.0)) return 0.0;
 if (!(response_time_s > 0.0)) return 1.0;
 return 1.0 - std::exp(-elapsed_s / response_time_s);
}

double smooth(double current, double target, double elapsed_s,
 const PhysicalAudioConfig& config) {
 const double response = target > current
 ? config.attack_time_s : config.release_time_s;
 return std::clamp(
 current + response_alpha(elapsed_s, response) * (target - current),
 0.0, 1.0);
}

struct InstantaneousEvidence {
 double wheel_speed_ratio = 0.0;
 double contact_load_ratio = 0.0;
 double motor = 0.0;
 double rolling = 0.0;
 double slip = 0.0;
 double brake = 0.0;
 double impact = 0.0;
};

InstantaneousEvidence reduce(const Frame& frame,
 const PhysicalAudioConfig& config) {
 InstantaneousEvidence out;
 const double free_speed = nonnegative(frame.body.drive_free_speed_mps);
 const double nominal_load = nonnegative(frame.body.nominal_normal_load_N);

 double driven_speed_sq = 0.0;
 std::size_t driven_count = 0;
 double contact_load = 0.0;
 double rolling_speed_sq_N = 0.0;
 double slip_speed_sq_N = 0.0;
 for (const FrameWheel& wheel : frame.body.wheels) {
 const double radius = nonnegative(wheel.radius_m);
 if (wheel.driven && radius > 0.0 &&
 std::isfinite(wheel.spin_rate_radps)) {
 const double surface_speed = radius * wheel.spin_rate_radps;
 driven_speed_sq += surface_speed * surface_speed;
 ++driven_count;
 }
 if (!wheel.contact_valid) continue;
 const double load = nonnegative(wheel.contact_normal_force_N);
 if (!(load > 0.0)) continue;
 double rolling_speed = std::hypot(
 nonnegative(std::abs(wheel.contact_motion_x_mps)),
 nonnegative(std::abs(wheel.contact_motion_y_mps)));
 rolling_speed = std::hypot(
 rolling_speed, nonnegative(std::abs(wheel.contact_motion_z_mps)));
 if (!(rolling_speed > 0.0)) {
 rolling_speed = nonnegative(std::abs(wheel.contact_speed_mps));
 }
 double slip_speed = std::hypot(
 nonnegative(std::abs(wheel.contact_slip_x_mps)),
 nonnegative(std::abs(wheel.contact_slip_y_mps)));
 slip_speed = std::hypot(
 slip_speed, nonnegative(std::abs(wheel.contact_slip_z_mps)));
 if (!(slip_speed > 0.0)) {
 slip_speed = nonnegative(wheel.slip_speed_mps);
 }
 contact_load += load;
 rolling_speed_sq_N += load * rolling_speed * rolling_speed;
 slip_speed_sq_N += load * slip_speed * slip_speed;
 }

 const double driven_speed_rms = driven_count > 0
 ? std::sqrt(driven_speed_sq / static_cast<double>(driven_count)) : 0.0;
 const double rolling_speed_rms = contact_load > 0.0
 ? std::sqrt(rolling_speed_sq_N / contact_load) : 0.0;
 const double slip_speed_rms = contact_load > 0.0
 ? std::sqrt(slip_speed_sq_N / contact_load) : 0.0;
 out.wheel_speed_ratio = unit_ratio(driven_speed_rms, free_speed);
 out.contact_load_ratio = unit_ratio(contact_load, nominal_load);

 const std::size_t drive_count = std::min<std::size_t>(
 frame.actuation.drive_count, frame.actuation.drive_duty.size());
 double duty_sq = 0.0;
 for (std::size_t i = 0; i < drive_count; ++i) {
 const double duty = std::isfinite(frame.actuation.drive_duty[i])
 ? std::clamp(frame.actuation.drive_duty[i], -1.0, 1.0) : 0.0;
 duty_sq += duty * duty;
 }
 const double duty_rms = drive_count > 0
 ? std::sqrt(duty_sq / static_cast<double>(drive_count)) : 0.0;
 // Motor/gear sound has two real sources: commanded electrical effort and
 // measured shaft motion. RMS combines them without a scene-specific gate.
 out.motor = std::clamp(
 std::hypot(duty_rms, out.wheel_speed_ratio) / std::sqrt(2.0),
 0.0, 1.0);
 out.rolling = unit_ratio(rolling_speed_rms, free_speed) *
 std::sqrt(out.contact_load_ratio);
 out.slip = unit_ratio(slip_speed_rms, free_speed) *
 std::sqrt(out.contact_load_ratio);
 const double brake_request = std::isfinite(frame.actuation.service_brake)
 ? std::clamp(frame.actuation.service_brake, 0.0, 1.0) : 0.0;
 out.brake = brake_request * std::max(
 out.wheel_speed_ratio, unit_ratio(rolling_speed_rms, free_speed));

 if (frame.impact.valid && frame.impact.normal_impulse_Ns > 0.0 &&
 frame.impact.relative_speed_mps > 0.0 && nominal_load > 0.0 &&
 free_speed > 0.0) {
 // Compare integrated contact force with one plant-step of static weight,
 // and relative speed with the chassis no-load speed. Both references carry
 // physical units and come from the current plant configuration.
 const double impulse_reference = nominal_load * config.sim_step_s;
 const double impulse_ratio = unit_ratio(
 frame.impact.normal_impulse_Ns, impulse_reference);
 const double speed_ratio = unit_ratio(
 frame.impact.relative_speed_mps, free_speed);
 out.impact = std::sqrt(impulse_ratio * speed_ratio);
 }
 return out;
}

} // namespace

PhysicalAudioEvidence::PhysicalAudioEvidence(PhysicalAudioConfig config)
 : config_(config) {
 if (!std::isfinite(config_.sim_step_s) || !(config_.sim_step_s > 0.0) ||
 !std::isfinite(config_.attack_time_s) || config_.attack_time_s < 0.0 ||
 !std::isfinite(config_.release_time_s) || config_.release_time_s < 0.0 ||
 !std::isfinite(config_.impact_decay_time_s) ||
 config_.impact_decay_time_s < 0.0) {
 throw std::invalid_argument("invalid physical audio configuration");
 }
}

void PhysicalAudioEvidence::reset() {
 sample_ = {};
 has_frame_ = false;
 impact_active_ = false;
}

const PhysicalAudioSample& PhysicalAudioEvidence::advance(
 const Frame& frame) {
 const bool rewound = has_frame_ && frame.step < sample_.step;
 const bool repeated = has_frame_ && frame.step == sample_.step;
 if (repeated) {
 sample_.advanced = false;
 sample_.rewound = false;
 sample_.impact_event = false;
 return sample_;
 }

 const InstantaneousEvidence instant = reduce(frame, config_);
 if (!has_frame_ || rewound) {
 sample_ = {};
 sample_.step = frame.step;
 sample_.rewound = rewound;
 sample_.wheel_speed_ratio = instant.wheel_speed_ratio;
 sample_.contact_load_ratio = instant.contact_load_ratio;
 sample_.motor_intensity = instant.motor;
 sample_.rolling_intensity = instant.rolling;
 sample_.slip_intensity = instant.slip;
 sample_.brake_intensity = instant.brake;
 // A frame selected by a rewind/scrub synchronizes state but never invents
 // a newly occurring impact. A first live frame may carry a real impact.
 sample_.impact_event = !rewound && frame.impact.valid && instant.impact > 0.0;
 sample_.impact_intensity = sample_.impact_event ? instant.impact : 0.0;
 sample_.advanced = !rewound;
 impact_active_ = frame.impact.valid;
 has_frame_ = true;
 return sample_;
 }

 const std::uint64_t elapsed_steps = frame.step - sample_.step;
 const double elapsed_s = static_cast<double>(elapsed_steps) *
 config_.sim_step_s;
 sample_.step = frame.step;
 sample_.advanced = true;
 sample_.rewound = false;
 sample_.wheel_speed_ratio = instant.wheel_speed_ratio;
 sample_.contact_load_ratio = instant.contact_load_ratio;
 sample_.motor_intensity = smooth(
 sample_.motor_intensity, instant.motor, elapsed_s, config_);
 sample_.rolling_intensity = smooth(
 sample_.rolling_intensity, instant.rolling, elapsed_s, config_);
 sample_.slip_intensity = smooth(
 sample_.slip_intensity, instant.slip, elapsed_s, config_);
 sample_.brake_intensity = smooth(
 sample_.brake_intensity, instant.brake, elapsed_s, config_);

 if (config_.impact_decay_time_s > 0.0) {
 sample_.impact_intensity *= std::exp(
 -elapsed_s / config_.impact_decay_time_s);
 } else {
 sample_.impact_intensity = 0.0;
 }
 sample_.impact_event = frame.impact.valid && !impact_active_ &&
 instant.impact > 0.0;
 if (sample_.impact_event) {
 sample_.impact_intensity = std::max(
 sample_.impact_intensity, instant.impact);
 }
 impact_active_ = frame.impact.valid;
 return sample_;
}

} // namespace lost_signal::sim::viewer
