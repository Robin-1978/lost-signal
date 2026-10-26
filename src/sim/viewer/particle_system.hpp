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

enum class SurfaceTraceKind {
 kDust,
 kMud,
 kWater,
 kRubber,
};

struct SurfaceTrace {
 SurfaceTraceKind kind = SurfaceTraceKind::kDust;
 std::array<double, 3> start_m{};
 std::array<double, 3> end_m{};
 double half_width_m = 0.02;
 double intensity = 1.0;
 double age_s = 0.0;
 double lifetime_s = 10.0;
 bool active = false;
};

// Long-lived contact evidence for presentation. Every segment comes from two
// consecutive MuJoCo contact points. Loose surfaces mark ordinary rolling;
// paved surfaces mark only confirmed physical slip. This state is never read
// by physics, scoring, sensors, or ROBOT.
struct SurfaceTracePresentationConfig {
 std::size_t capacity = 2048;
 double sim_step_s = 0.02;
 double minimum_spacing_m = 0.012;
 double maximum_continuous_gap_m = 0.30;
 double loose_motion_threshold_mps = 0.04;
 double rubber_slip_threshold_mps = 0.25;
 double reference_activity_mps = 1.0;
 double width_scale = 0.88;
 double surface_offset_m = 0.002;
 double dust_lifetime_s = 14.0;
 double mud_lifetime_s = 36.0;
 double water_lifetime_s = 5.0;
 double rubber_lifetime_s = 60.0;
};

class SurfaceTraceSystem {
 public:
 explicit SurfaceTraceSystem(SurfaceTracePresentationConfig config = {});

 void advance(const Frame& frame);
 void clear();
 std::size_t active_count() const;
 std::size_t capacity() const { return traces_.size(); }
 const std::vector<SurfaceTrace>& traces() const { return traces_; }

 private:
 struct ContactAnchor {
 bool valid = false;
 SurfaceTraceKind kind = SurfaceTraceKind::kDust;
 std::array<double, 3> position_m{};
 };

 std::size_t acquire_slot();

 SurfaceTracePresentationConfig config_;
 std::vector<SurfaceTrace> traces_;
 std::array<ContactAnchor, 6> anchors_{};
 std::size_t replacement_cursor_ = 0;
 std::uint64_t last_step_ = 0;
 bool has_step_ = false;
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
