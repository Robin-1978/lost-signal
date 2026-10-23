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

void SimImuProvider::set_mount(const double xyz_m[3],
                               const double rpy_rad[3]) {
  for (int i = 0; i < 3; ++i) {
    if (!std::isfinite(xyz_m[i]) || !std::isfinite(rpy_rad[i])) {
      return;  // fail closed: keep previous mount
    }
  }
  for (int i = 0; i < 3; ++i) {
    mount_xyz_m_[i] = xyz_m[i];
    mount_rpy_rad_[i] = rpy_rad[i];
  }
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
  // Specific force is the full hardware quantity: body linear acceleration
  // (differentiated truth twist at poll cadence) plus gravity in the TRUE
  // body attitude, plus the yaw lever arm at the mount offset (exact planar
  // rigid-body kinematics). Everything below is expressed in the SENSOR
  // frame: body vectors rotate through the mount orientation. Noise rides
  // on top when enabled. The longitudinal traction observer closes on the
  // linear terms — without them straight-line wheelspin is unobservable.
  const double stamp_s =
      static_cast<double>(world_->truth().step) * 0.02;
  const double w = world_->truth().yaw_rate_radps;
  const double fwd = world_->truth().forward_mps;
  const double lat = world_->truth().lateral_mps;
  double alpha = 0.0, acc_f = 0.0, acc_l = 0.0;
  if (prev_stamp_s_ >= 0.0 && stamp_s > prev_stamp_s_) {
    alpha = (w - prev_yaw_rate_radps_) / (stamp_s - prev_stamp_s_);
    if (std::isfinite(fwd) && std::isfinite(lat) &&
        std::isfinite(prev_fwd_mps_) && std::isfinite(prev_lat_mps_)) {
      acc_f = (fwd - prev_fwd_mps_) / (stamp_s - prev_stamp_s_);
      acc_l = (lat - prev_lat_mps_) / (stamp_s - prev_stamp_s_);
    }
  }
  prev_yaw_rate_radps_ = w;
  prev_fwd_mps_ = fwd;
  prev_lat_mps_ = lat;
  prev_stamp_s_ = stamp_s;
  const robot::core::Vec3 body_w{0.0, 0.0, w};
  const robot::core::Quaternion mount_q = robot::core::quaternion_from_rpy(
      mount_rpy_rad_[0], mount_rpy_rad_[1], mount_rpy_rad_[2]);
  // Inverse mount rotation: body-frame vector -> sensor-frame vector.
  // (R^-1 v, computed as rotation by the conjugate.)
  const robot::core::Quaternion mount_q_inv{
      -mount_q.x, -mount_q.y, -mount_q.z, mount_q.w};
  const robot::core::Vec3 gyro_s =
      robot::core::rotate_vector(mount_q_inv, body_w);
  const double rx = mount_xyz_m_[0], ry = mount_xyz_m_[1];
  // Base-frame specific force: yaw lever arm + body linear accel.
  const robot::core::Vec3 lever_b{-alpha * ry - w * w * rx + acc_f,
                                  alpha * rx - w * w * ry + acc_l, 0.0};
  const robot::core::Vec3 lever_s =
      robot::core::rotate_vector(mount_q_inv, lever_b);
  // Gravity in the true body attitude (specific force opposes gravity):
  // level reads +g on z; pitch/roll project it onto x/y. Derived as
  // -(R_y(pitch) R_x(roll) (0,0,-g)): yaw never moves the gravity vector.
  const double pitch = world_->truth().body_pitch_rad;
  const double roll = world_->truth().body_roll_rad;
  const robot::core::Vec3 grav_b{9.81 * std::cos(roll) * std::sin(pitch),
                                 -9.81 * std::sin(roll),
                                 9.81 * std::cos(pitch) * std::cos(roll)};
  const robot::core::Vec3 grav_s =
      robot::core::rotate_vector(mount_q_inv, grav_b);
  out.gyro_z_radps = gyro_s.z + realism_.gyro_bias_radps +
                     realism_.gyro_noise_radps * randn(rng_state_);
  out.gyro_x_radps = gyro_s.x + realism_.gyro_noise_radps * randn(rng_state_);
  out.gyro_y_radps = gyro_s.y + realism_.gyro_noise_radps * randn(rng_state_);
  out.accel_x_mps2 = lever_s.x + grav_s.x +
                     realism_.accel_noise_mps2 * randn(rng_state_);
  out.accel_y_mps2 = lever_s.y + grav_s.y +
                     realism_.accel_noise_mps2 * randn(rng_state_);
  out.accel_z_mps2 = lever_s.z + grav_s.z +
                     realism_.accel_noise_mps2 * randn(rng_state_);
  return out.valid();
}

}  // namespace lost_signal::sim::providers
