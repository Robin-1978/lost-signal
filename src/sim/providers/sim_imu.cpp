#include "sim/providers/sim_imu.hpp"

#include <cmath>

#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

namespace {

std::uint64_t splitmix64(std::uint64_t& state) {
  std::uint64_t z = (state += 0x9E3779B97F4A7C15ULL);
  z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ULL;
  z = (z ^ (z >> 27)) * 0x94D049BB133111EBULL;
  return z ^ (z >> 31);
}

double rand01(std::uint64_t& state) {
  return static_cast<double>(splitmix64(state) >> 11) *
         (1.0 / 9007199254740992.0);
}

// Exact standard normal, fixed 2-draw consumption (Box-Muller, spare
// discarded). Deterministic per stream position by construction.
double randn(std::uint64_t& state) {
  const double u1 = std::max(rand01(state), 1e-300);
  const double u2 = rand01(state);
  return std::sqrt(-2.0 * std::log(u1)) *
         std::cos(2.0 * 3.14159265358979323846 * u2);
}

}  // namespace

SimImuProvider::SimImuProvider(world::World* world) : world_(world) {
  desc_.endpoint_id = "sim.imu";
  desc_.role = robot::iface::EndpointRole::kImu;
  desc_.backend = robot::iface::ProviderBackend::kSim;
  desc_.frame_id = "imu_link";
}

const robot::iface::EndpointDescriptor& SimImuProvider::descriptor() const {
  return desc_;
}

void SimImuProvider::set_realism(ImuRealismConfig config) {
  if (!(config.gyro_noise_radps >= 0.0) ||
      !std::isfinite(config.gyro_bias_radps) ||
      !(config.accel_noise_mps2 >= 0.0)) {
    return;  // fail closed: keep previous settings
  }
  realism_ = config;
  rng_state_ = config.seed ^ 0x9E3779B97F4A7C15ULL;
}

bool SimImuProvider::poll(robot::iface::ImuMessage& out) {
  if (world_ == nullptr) return false;
  seq_ += 1;
  out.seq = seq_;
  out.stamp = static_cast<robot::core::MonotonicNs>(world_->truth().step *
                                                    20'000'000);
  out.src = desc_.endpoint_id;
  out.frm = robot::core::Frame::kImuLink;
  // Yaw rate is truth here. It must be truthful, not zero: the estimator's
  // slip gate trusts the gyro over wheel odometry on disagreement, so a
  // hard zero would freeze the heading estimate through every turn.
  // Planar motion: no roll/pitch rate. Level: specific force opposes
  // gravity; noise rides on top when enabled.
  out.gyro_z_radps = world_->truth().yaw_rate_radps +
                     realism_.gyro_bias_radps +
                     realism_.gyro_noise_radps * randn(rng_state_);
  out.gyro_x_radps = realism_.gyro_noise_radps * randn(rng_state_);
  out.gyro_y_radps = realism_.gyro_noise_radps * randn(rng_state_);
  out.accel_x_mps2 = realism_.accel_noise_mps2 * randn(rng_state_);
  out.accel_y_mps2 = realism_.accel_noise_mps2 * randn(rng_state_);
  out.accel_z_mps2 = 9.81 + realism_.accel_noise_mps2 * randn(rng_state_);
  return out.valid();
}

}  // namespace lost_signal::sim::providers
