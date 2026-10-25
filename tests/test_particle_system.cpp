#include <cassert>
#include <cmath>
#include <stdexcept>

#include "sim/viewer/particle_system.hpp"

namespace {

lost_signal::sim::viewer::Frame contact_frame(
 std::uint64_t step, const char* material, bool slipping = false,
 const char* surface_effect = "none", double fluid_depth_m = 0.0) {
 lost_signal::sim::viewer::Frame frame;
 frame.step = step;
 frame.body.x_m = 10.0;
 frame.body.y_m = -3.0;
 frame.body.yaw_rad = 0.25;
 lost_signal::sim::viewer::FrameWheel wheel;
 wheel.contact_valid = true;
 wheel.contact_x_m = 10.4;
 wheel.contact_y_m = -2.8;
 wheel.contact_z_m = 0.02;
 wheel.contact_normal_force_N = 110.0;
 wheel.contact_speed_mps = 1.4;
 wheel.slip_speed_mps = slipping ? 0.6 : 0.02;
 wheel.slipping = slipping;
 wheel.steer_rad = -0.1;
 wheel.radius_m = 0.1;
 wheel.contact_material_id = material;
 wheel.contact_surface_effect = surface_effect;
 wheel.contact_fluid_depth_m = fluid_depth_m;
 frame.body.wheels.push_back(wheel);
 return frame;
}

} // namespace

int main() {
 using namespace lost_signal::sim::viewer;
 bool rejected = false;
 try {
 ParticlePresentationConfig bad;
 bad.capacity = 0;
 ParticleSystem invalid(bad);
 (void)invalid;
 } catch (const std::invalid_argument&) {
 rejected = true;
 }
 assert(rejected);

 ParticlePresentationConfig config;
 config.capacity = 12;
 config.loose_particles_per_m = 120.0;
 config.water_particles_per_m = 120.0;
 config.smoke_particles_per_m = 120.0;
 ParticleSystem dry(config);
 dry.advance(contact_frame(1, "paved"));
 assert(dry.active_count() == 0);
 dry.advance(contact_frame(2, "looks_wet_but_is_not_authored"));
 assert(dry.active_count() == 0);
 dry.advance(contact_frame(3, "paved", true));
 assert(dry.active_count() > 0);
 assert(dry.active_count() <= dry.capacity());

 ParticleSystem first(config);
 ParticleSystem second(config);
 for (std::uint64_t step = 10; step < 30; ++step) {
 const auto frame = contact_frame(step, "gravel", false, "dust");
 first.advance(frame);
 second.advance(frame);
 }
 assert(first.active_count() == second.active_count());
 assert(first.active_count() <= config.capacity);
 for (std::size_t i = 0; i < first.particles().size(); ++i) {
 const auto& a = first.particles()[i];
 const auto& b = second.particles()[i];
 assert(a.active == b.active);
 assert(a.kind == b.kind);
 for (int axis = 0; axis < 3; ++axis) {
 assert(std::abs(a.position_m[axis] - b.position_m[axis]) < 1e-12);
 assert(std::abs(a.velocity_mps[axis] - b.velocity_mps[axis]) < 1e-12);
 }
 }

 const std::size_t before_pause = first.active_count();
 first.advance(contact_frame(29, "gravel", false, "dust"));
 assert(first.active_count() == before_pause);
 first.advance(contact_frame(3, "standing_water", false, "water", 0.02));
 assert(first.active_count() > 0);
 bool found_water = false;
 for (const auto& particle : first.particles()) {
 if (particle.active && particle.kind == ParticleKind::kWater) {
 found_water = true;
 assert(particle.position_m[2] >= 0.04);
 }
 }
 assert(found_water);

 Frame impact;
 impact.step = 100;
 impact.impact.valid = true;
 impact.impact.x_m = 2.0;
 impact.impact.y_m = -1.0;
 impact.impact.z_m = 0.2;
 impact.impact.normal_impulse_Ns = 1.0;
 impact.impact.relative_speed_mps = 0.1;
 ParticleSystem sparks(config);
 sparks.advance(impact);
 assert(sparks.active_count() == 0);
 impact.step = 101;
 impact.impact.relative_speed_mps = 1.0;
 sparks.advance(impact);
 assert(sparks.active_count() > 0);
 bool found_spark = false;
 for (const auto& particle : sparks.particles()) {
 if (particle.active && particle.kind == ParticleKind::kImpactSpark) {
 found_spark = true;
 assert(particle.position_m[0] == 2.0);
 }
 }
 assert(found_spark);
 const std::size_t spark_count = sparks.active_count();
 sparks.advance(impact); // a paused/repeated frame never emits twice
 assert(sparks.active_count() == spark_count);

 ParticleSystem sparks_again(config);
 sparks_again.advance(impact);
 assert(sparks_again.active_count() == spark_count);
 for (std::size_t i = 0; i < sparks.particles().size(); ++i) {
 const auto& a = sparks.particles()[i];
 const auto& b = sparks_again.particles()[i];
 assert(a.active == b.active);
 assert(a.kind == b.kind);
 for (int axis = 0; axis < 3; ++axis) {
 assert(a.position_m[axis] == b.position_m[axis]);
 assert(a.velocity_mps[axis] == b.velocity_mps[axis]);
 }
 }
 return 0;
}
