#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

#include "sim/viewer/frame.hpp"

namespace lost_signal::sim::viewer {

enum class ParticleKind {
 kDust,
 kMud,
 kWater,
 kTireSmoke,
 kImpactSpark,
};

struct Particle {
 ParticleKind kind = ParticleKind::kDust;
 std::array<double, 3> position_m{};
 std::array<double, 3> velocity_mps{};
 double age_s = 0.0;
 double lifetime_s = 0.5;
 double minimum_z_m = 0.0;
 double start_size_m = 0.03;
 double end_size_m = 0.12;
 bool active = false;
};

// SIM presentation profile. These values control only visible density and
// lifetime; they never enter physics, observations, scoring, or ROBOT.
struct ParticlePresentationConfig {
 std::size_t capacity = 1024;
 double sim_step_s = 0.02;
 double motion_threshold_mps = 0.08;
 double tire_smoke_slip_mps = 0.25;
 double loose_particles_per_m = 32.0;
 double water_particles_per_m = 46.0;
 double water_reference_depth_m = 0.02;
 double smoke_particles_per_m = 24.0;
 double impact_min_relative_speed_mps = 0.35;
 double impact_particles_per_Ns = 18.0;
 std::size_t maximum_impact_particles_per_step = 24;
 double maximum_emission_catchup_s = 0.10;
};

// Fixed-capacity, deterministic presentation pool. advance() consumes one
// completed SIM frame. Repeated paused frames do not emit; a backward seek
// clears the pool rather than carrying particles across replay time.
class ParticleSystem {
 public:
 explicit ParticleSystem(ParticlePresentationConfig config = {});

 void advance(const Frame& frame);
 void clear();
 std::size_t active_count() const;
 std::size_t capacity() const { return particles_.size(); }
 const std::vector<Particle>& particles() const { return particles_; }

 private:
 void integrate(double elapsed_s);
 void emit(const Frame& frame, std::size_t wheel_index, ParticleKind kind,
 const std::array<double, 3>& source_velocity_mps,
 double activity_mps, double elapsed_s);
 void emit_impact(const Frame& frame);
 std::size_t acquire_slot();

 ParticlePresentationConfig config_;
 std::vector<Particle> particles_;
 std::array<double, 6> emission_carry_{};
 std::size_t replacement_cursor_ = 0;
 std::uint64_t last_step_ = 0;
 bool has_step_ = false;
};

} // namespace lost_signal::sim::viewer
