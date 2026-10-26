#include "sim/viewer/particle_system.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>

namespace lost_signal::sim::viewer {
namespace {

double stable_unit(std::uint64_t step, std::size_t wheel, std::size_t sample) {
 std::uint64_t value = step * 0x9e3779b97f4a7c15ULL;
 value ^= (wheel + 17U) * 0xbf58476d1ce4e5b9ULL;
 value ^= (sample + 31U) * 0x94d049bb133111ebULL;
 value ^= value >> 30;
 value *= 0xbf58476d1ce4e5b9ULL;
 value ^= value >> 27;
 return static_cast<double>(value & 0xffffU) / 65535.0;
}

ParticleKind effect_for(const FrameWheel& wheel,
 const ParticlePresentationConfig& config, bool& enabled) {
 const bool water = wheel.contact_surface_effect == "water" &&
 wheel.contact_fluid_depth_m > 0.0;
 const bool mud = wheel.contact_surface_effect == "mud";
 const bool loose = mud || wheel.contact_surface_effect == "dust";
 enabled = water || loose ||
 (wheel.slipping && wheel.slip_speed_mps >=
 config.tire_smoke_slip_mps);
 if (water) return ParticleKind::kWater;
 if (mud) return ParticleKind::kMud;
 if (loose) return ParticleKind::kDust;
 return ParticleKind::kTireSmoke;
}

} // namespace

ParticleSystem::ParticleSystem(ParticlePresentationConfig config)
 : config_(config), particles_(config.capacity) {
 if (config.capacity == 0 || !std::isfinite(config.sim_step_s) ||
 !(config.sim_step_s > 0.0) ||
 !std::isfinite(config.motion_threshold_mps) ||
 config.motion_threshold_mps < 0.0 ||
 !std::isfinite(config.tire_smoke_slip_mps) ||
 config.tire_smoke_slip_mps < 0.0 ||
 !std::isfinite(config.loose_particles_per_m) ||
 config.loose_particles_per_m < 0.0 ||
 !std::isfinite(config.water_particles_per_m) ||
 config.water_particles_per_m < 0.0 ||
 !std::isfinite(config.water_reference_depth_m) ||
 !(config.water_reference_depth_m > 0.0) ||
 !std::isfinite(config.smoke_particles_per_m) ||
 config.smoke_particles_per_m < 0.0 ||
 !std::isfinite(config.impact_min_relative_speed_mps) ||
 config.impact_min_relative_speed_mps < 0.0 ||
 !std::isfinite(config.impact_particles_per_Ns) ||
 config.impact_particles_per_Ns < 0.0 ||
 config.maximum_impact_particles_per_step == 0 ||
 !std::isfinite(config.maximum_emission_catchup_s) ||
 config.maximum_emission_catchup_s < config.sim_step_s) {
 throw std::invalid_argument("invalid particle presentation config");
 }
}

void ParticleSystem::clear() {
 for (auto& particle : particles_) particle = Particle{};
 emission_carry_.fill(0.0);
 replacement_cursor_ = 0;
 last_step_ = 0;
 has_step_ = false;
}

std::size_t ParticleSystem::active_count() const {
 return static_cast<std::size_t>(std::count_if(
 particles_.begin(), particles_.end(),
 [](const Particle& particle) { return particle.active; }));
}

void ParticleSystem::integrate(double elapsed_s) {
 for (auto& particle : particles_) {
 if (!particle.active) continue;
 particle.age_s += elapsed_s;
 if (particle.age_s >= particle.lifetime_s) {
 particle.active = false;
 continue;
 }
 particle.position_m[0] += particle.velocity_mps[0] * elapsed_s;
 particle.position_m[1] += particle.velocity_mps[1] * elapsed_s;
 particle.position_m[2] += particle.velocity_mps[2] * elapsed_s;
 if (particle.kind == ParticleKind::kWater ||
 particle.kind == ParticleKind::kImpactSpark) {
 particle.velocity_mps[2] -=
 (particle.kind == ParticleKind::kImpactSpark ? 9.81 : 4.0) * elapsed_s;
 if (particle.position_m[2] <= particle.minimum_z_m) {
 particle.active = false;
 }
 } else {
 particle.velocity_mps[0] *= std::exp(-1.8 * elapsed_s);
 particle.velocity_mps[1] *= std::exp(-1.8 * elapsed_s);
 particle.velocity_mps[2] *= std::exp(-1.2 * elapsed_s);
 }
 }
}

void ParticleSystem::emit_impact(const Frame& frame) {
 const FrameImpact& impact = frame.impact;
 if (!impact.spark_eligible ||
 impact.spark_robot_material != "metal" ||
 impact.spark_static_material != "metal" ||
 !(impact.spark_normal_impulse_Ns > 0.0) ||
 impact.spark_relative_speed_mps < config_.impact_min_relative_speed_mps ||
 !(config_.impact_particles_per_Ns > 0.0)) {
 return;
 }
 const double requested =
 impact.spark_normal_impulse_Ns * config_.impact_particles_per_Ns;
 const std::size_t count = std::min(
 config_.maximum_impact_particles_per_step,
 static_cast<std::size_t>(std::max(1.0, std::floor(requested))));
 for (std::size_t index = 0; index < count; ++index) {
 const double azimuth = 6.283185307179586 *
 stable_unit(frame.step, 97U, index + 401U);
 const double speed_scale = 0.35 + 0.65 *
 stable_unit(frame.step, 97U, index + 503U);
 const double speed = std::clamp(
 impact.spark_relative_speed_mps * speed_scale, 0.25, 4.0);
 Particle particle;
 particle.kind = ParticleKind::kImpactSpark;
 particle.active = true;
 particle.position_m = {
 impact.spark_x_m, impact.spark_y_m, impact.spark_z_m};
 particle.velocity_mps = {
 std::cos(azimuth) * speed,
 std::sin(azimuth) * speed,
 0.25 * speed + 0.45 * stable_unit(frame.step, 97U, index + 607U)};
 particle.lifetime_s = 0.12 + 0.10 *
 stable_unit(frame.step, 97U, index + 701U);
 particle.minimum_z_m = 0.0;
 particle.start_size_m = 0.014;
 particle.end_size_m = 0.002;
 particles_[acquire_slot()] = particle;
 }
}

std::size_t ParticleSystem::acquire_slot() {
 for (std::size_t i = 0; i < particles_.size(); ++i) {
 const std::size_t slot = (replacement_cursor_ + i) % particles_.size();
 if (!particles_[slot].active) {
 replacement_cursor_ = (slot + 1) % particles_.size();
 return slot;
 }
 }
 const std::size_t slot = replacement_cursor_;
 replacement_cursor_ = (replacement_cursor_ + 1) % particles_.size();
 return slot;
}

void ParticleSystem::emit(const Frame& frame, std::size_t wheel_index,
 ParticleKind kind, const std::array<double, 3>& source_velocity_mps,
 double activity_mps, double elapsed_s) {
 if (wheel_index >= frame.body.wheels.size() ||
 wheel_index >= emission_carry_.size()) {
 return;
 }
 const FrameWheel& wheel = frame.body.wheels[wheel_index];
 double density = config_.loose_particles_per_m;
 const double water_depth_scale = kind == ParticleKind::kWater
 ? std::clamp(wheel.contact_fluid_depth_m /
 config_.water_reference_depth_m, 0.25, 2.0) : 1.0;
 if (kind == ParticleKind::kWater) {
 density = config_.water_particles_per_m * water_depth_scale;
 }
 if (kind == ParticleKind::kTireSmoke) density = config_.smoke_particles_per_m;
 emission_carry_[wheel_index] += activity_mps * density * elapsed_s;
 const int count = std::min(16, static_cast<int>(emission_carry_[wheel_index]));
 emission_carry_[wheel_index] -= count;
 if (count <= 0) return;

 const double direction_norm = std::sqrt(
 source_velocity_mps[0] * source_velocity_mps[0] +
 source_velocity_mps[1] * source_velocity_mps[1] +
 source_velocity_mps[2] * source_velocity_mps[2]);
 if (!(direction_norm > 1e-9)) return;
 const double forward_x = source_velocity_mps[0] / direction_norm;
 const double forward_y = source_velocity_mps[1] / direction_norm;
 const double forward_z = source_velocity_mps[2] / direction_norm;
 const double horizontal_norm = std::hypot(forward_x, forward_y);
 const double side_x = horizontal_norm > 1e-9
 ? -forward_y / horizontal_norm : 0.0;
 const double side_y = horizontal_norm > 1e-9
 ? forward_x / horizontal_norm : 1.0;
 for (int index = 0; index < count; ++index) {
 const double spread = stable_unit(frame.step, wheel_index,
 static_cast<std::size_t>(index) + 101U) - 0.5;
 const double speed_variation = 0.65 + 0.7 * stable_unit(
 frame.step, wheel_index, static_cast<std::size_t>(index) + 211U);
 Particle particle;
 particle.kind = kind;
 particle.active = true;
 particle.position_m = {wheel.contact_x_m + side_x * spread * wheel.radius_m,
 wheel.contact_y_m + side_y * spread * wheel.radius_m,
 wheel.contact_z_m + wheel.contact_fluid_depth_m + 0.008};
 particle.minimum_z_m = wheel.contact_z_m + wheel.contact_fluid_depth_m;
 const double backward_mps = activity_mps *
 (kind == ParticleKind::kWater ? 0.22 * water_depth_scale : 0.12) *
 speed_variation;
 const double sideways_mps = spread * activity_mps *
 (kind == ParticleKind::kWater ? 0.28 * water_depth_scale : 0.10);
 particle.velocity_mps = {
 -forward_x * backward_mps + side_x * sideways_mps,
 -forward_y * backward_mps + side_y * sideways_mps,
 -forward_z * backward_mps + (kind == ParticleKind::kWater
 ? (0.16 + 0.10 * speed_variation) * std::sqrt(water_depth_scale)
 : 0.06 + 0.08 * speed_variation)};
 if (kind == ParticleKind::kWater) {
 particle.lifetime_s = 0.34;
 particle.start_size_m = 0.018;
 particle.end_size_m = 0.045;
 } else if (kind == ParticleKind::kTireSmoke) {
 particle.lifetime_s = 0.72;
 particle.start_size_m = 0.035;
 particle.end_size_m = 0.18;
 } else {
 particle.lifetime_s = kind == ParticleKind::kMud ? 0.52 : 0.64;
 particle.start_size_m = kind == ParticleKind::kMud ? 0.026 : 0.032;
 particle.end_size_m = kind == ParticleKind::kMud ? 0.09 : 0.15;
 }
 particles_[acquire_slot()] = particle;
 }
}

void ParticleSystem::advance(const Frame& frame) {
 if (has_step_ && frame.step == last_step_) return;
 if (has_step_ && frame.step < last_step_) {
 clear();
 }
 std::uint64_t elapsed_steps = 1;
 if (has_step_ && frame.step > last_step_) {
 elapsed_steps = frame.step - last_step_;
 }
 const double elapsed_s = static_cast<double>(elapsed_steps) *
 config_.sim_step_s;
 integrate(elapsed_s);
 const double emission_s = std::min(
 elapsed_s, config_.maximum_emission_catchup_s);

 double normal_sum_N = 0.0;
 std::size_t loaded_wheels = 0;
 for (const auto& wheel : frame.body.wheels) {
 if (!wheel.contact_valid || !(wheel.contact_normal_force_N > 0.0)) continue;
 normal_sum_N += wheel.contact_normal_force_N;
 ++loaded_wheels;
 }
 const double average_normal_N = loaded_wheels > 0
 ? normal_sum_N / static_cast<double>(loaded_wheels) : 0.0;
 for (std::size_t index = 0;
 index < frame.body.wheels.size() && index < emission_carry_.size();
 ++index) {
 const FrameWheel& wheel = frame.body.wheels[index];
 if (!wheel.contact_valid || !(wheel.contact_normal_force_N > 0.0)) continue;
 bool enabled = false;
 const ParticleKind kind = effect_for(wheel, config_, enabled);
 if (!enabled) continue;
 const double load_scale = average_normal_N > 0.0
 ? std::clamp(std::sqrt(wheel.contact_normal_force_N / average_normal_N),
 0.5, 2.0) : 1.0;
 const std::array<double, 3> motion = {
 wheel.contact_motion_x_mps, wheel.contact_motion_y_mps,
 wheel.contact_motion_z_mps};
 const std::array<double, 3> slip = {
 wheel.contact_slip_x_mps, wheel.contact_slip_y_mps,
 wheel.contact_slip_z_mps};
 const double motion_speed = std::sqrt(
 motion[0] * motion[0] + motion[1] * motion[1] +
 motion[2] * motion[2]);
 const double slip_speed = std::sqrt(
 slip[0] * slip[0] + slip[1] * slip[1] + slip[2] * slip[2]);
 // Rolling through loose material displaces it along module travel. When
 // the tire surface moves faster than the module (wheelspin or a locked
 // slide), the measured contact slip owns both direction and activity.
 const std::array<double, 3>& source =
 kind == ParticleKind::kTireSmoke || slip_speed > motion_speed
 ? slip : motion;
 const double activity = load_scale *
 (kind == ParticleKind::kTireSmoke
 ? slip_speed : std::max(motion_speed, slip_speed));
 if (activity < config_.motion_threshold_mps) continue;
 emit(frame, index, kind, source, activity, emission_s);
 }
 emit_impact(frame);
 last_step_ = frame.step;
 has_step_ = true;
}

} // namespace lost_signal::sim::viewer
