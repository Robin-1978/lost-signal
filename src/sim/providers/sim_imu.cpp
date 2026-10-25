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
      !std::isfinite(config.gyro_trim_radps[0]) ||
      !std::isfinite(config.gyro_trim_radps[1]) ||
      !std::isfinite(config.gyro_trim_radps[2]) ||
      !(config.gyro_bias_stddev_radps >= 0.0) ||
      !(config.gyro_bias_walk_radps_sqrt_s >= 0.0) ||
      !(config.accel_noise_mps2 >= 0.0)) {
    return;  // fail closed: keep previous settings
  }
  realism_ = config;
  // Independent streams per measurement family (master's doctrine: draw
  // order in one family never perturbs another).
  gyro_state_ = config.seed ^ 0x9E3779B97F4A7C15ULL;
  accel_state_ = config.seed ^ 0x51ed2707ULL;
  bias_state_ = config.seed ^ 0x3c6ef372ULL;
  // Per-run bias draw, per axis (master's form: N(0, stddev) once per run).
  for (int i = 0; i < 3; ++i) {
    bias_draw_radps_[i] = config.gyro_bias_stddev_radps > 0.0
                              ? randn(bias_state_) *
                                    config.gyro_bias_stddev_radps
                              : 0.0;
  }
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
 out.stamp = world_->sim_time_ns();
  out.src = desc_.endpoint_id;
  out.frm = robot::core::Frame::kImuLink;
  // Yaw rate is truth here. It must be truthful, not zero: the estimator's
  // slip gate trusts the gyro over wheel odometry on disagreement, so a
  // hard zero would freeze the heading estimate through every turn.
  // Specific force is the full hardware quantity: body linear acceleration
  // (differentiated truth twist at poll cadence, vertical from the body
  // height) plus gravity in the TRUE body attitude, plus the full 3D lever
  // arm at the mount offset (a = a_body + alpha x r + w x (w x r), exact
  // rigid-body kinematics). Roll/pitch rates come from the attitude delta
  // (truth carries no roll/pitch rate channel); yaw rides the first-class
  // truth rate, so spinning without stepping still reads. Everything below
  // is expressed in the SENSOR frame: body vectors rotate through the mount
  // orientation. Bias (exact trim + drawn + walk) and white noise ride on
  // the gyro when enabled. The longitudinal traction observer closes on the
  // linear terms — without them straight-line wheelspin is unobservable.
 const double stamp_s = world_->sim_time_s();
  const double w = world_->truth().yaw_rate_radps;
  const double fwd = world_->truth().forward_mps;
  const double lat = world_->truth().lateral_mps;
  const double bz = world_->truth().body_z_m;
  const double pitch = world_->truth().body_pitch_rad;
  const double roll = world_->truth().body_roll_rad;
  const robot::core::Quaternion body_q =
      robot::core::quaternion_from_rpy(roll, pitch,
                                       world_->truth().body_yaw_rad);
  const double yaw = world_->truth().body_yaw_rad;
  const double cy = std::cos(yaw), sy = std::sin(yaw);
  const double world_vx = cy * fwd - sy * lat;
  const double world_vy = sy * fwd + cy * lat;
  double world_vz = 0.0;
  double alpha_z = 0.0;
  robot::core::Vec3 origin_accel_b{};
  double omegax = 0.0, omegay = 0.0, alphax = 0.0, alphay = 0.0;
  const bool stepped =
      has_prev_attitude_ && prev_stamp_s_ >= 0.0 && stamp_s > prev_stamp_s_;
  if (stepped) {
    const double dt = stamp_s - prev_stamp_s_;
    alpha_z = (w - prev_yaw_rate_radps_) / dt;
    if (std::isfinite(fwd) && std::isfinite(lat) &&
        std::isfinite(world_vx) && std::isfinite(world_vy) &&
        std::isfinite(prev_world_vx_mps_) &&
        std::isfinite(prev_world_vy_mps_)) {
      const robot::core::Vec3 accel_w{
          (world_vx - prev_world_vx_mps_) / dt,
          (world_vy - prev_world_vy_mps_) / dt, 0.0};
      const robot::core::Quaternion body_q_inv{
          -body_q.x, -body_q.y, -body_q.z, body_q.w};
      origin_accel_b = robot::core::rotate_vector(body_q_inv, accel_w);
    }
    if (std::isfinite(bz) && std::isfinite(prev_body_z_m_)) {
      world_vz = (bz - prev_body_z_m_) / dt;
      if (has_prev_vertical_velocity_ && std::isfinite(world_vz) &&
          std::isfinite(prev_world_vz_mps_)) {
        const robot::core::Vec3 vertical_accel_w{
            0.0, 0.0, (world_vz - prev_world_vz_mps_) / dt};
        const robot::core::Quaternion body_q_inv{
            -body_q.x, -body_q.y, -body_q.z, body_q.w};
        const robot::core::Vec3 vertical_accel_b =
            robot::core::rotate_vector(body_q_inv, vertical_accel_w);
        origin_accel_b.x += vertical_accel_b.x;
        origin_accel_b.y += vertical_accel_b.y;
        origin_accel_b.z += vertical_accel_b.z;
      }
      has_prev_vertical_velocity_ = true;
    }
    // Body-frame roll/pitch rates from the attitude delta: dq = q_prev^-1
    // q_now, axis-angle / dt. Small-angle form (poll cadence, MEMS grade).
    const robot::core::Quaternion dq{
        prev_body_q_.w * body_q.x - prev_body_q_.x * body_q.w -
            prev_body_q_.y * body_q.z + prev_body_q_.z * body_q.y,
        prev_body_q_.w * body_q.y + prev_body_q_.x * body_q.z -
            prev_body_q_.y * body_q.w - prev_body_q_.z * body_q.x,
        prev_body_q_.w * body_q.z - prev_body_q_.x * body_q.y +
            prev_body_q_.y * body_q.x - prev_body_q_.z * body_q.w,
        prev_body_q_.w * body_q.w + prev_body_q_.x * body_q.x +
            prev_body_q_.y * body_q.y + prev_body_q_.z * body_q.z};
    if (std::isfinite(dq.x) && std::isfinite(dq.y) &&
 std::isfinite(dq.z) && std::isfinite(dq.w)) {
 // Exact shortest-arc axis/angle. q and -q encode the same attitude;
 // choosing w >= 0 prevents a representation sign flip from appearing as
 // an almost-2pi angular-rate impulse.
 const double sign = dq.w < 0.0 ? -1.0 : 1.0;
 const double vx = sign * dq.x, vy = sign * dq.y, vz = sign * dq.z;
 const double vw = sign * dq.w;
 const double vector_norm = std::hypot(vx, vy, vz);
 const double rate_scale = vector_norm > 1e-12
 ? 2.0 * std::atan2(vector_norm, vw) / (vector_norm * dt)
 : 2.0 / dt;
 omegax = vx * rate_scale;
 omegay = vy * rate_scale;
      alphax = (omegax - prev_omega_x_) / dt;
      alphay = (omegay - prev_omega_y_) / dt;
    }
  }
  prev_yaw_rate_radps_ = w;
  prev_world_vx_mps_ = world_vx;
  prev_world_vy_mps_ = world_vy;
  if (stepped && std::isfinite(world_vz)) prev_world_vz_mps_ = world_vz;
  prev_body_z_m_ = bz;
  prev_body_q_ = body_q;
  prev_omega_x_ = omegax;
  prev_omega_y_ = omegay;
  has_prev_attitude_ = true;
  // Bias random walk (master's form: sigma * sqrt(dt) per poll, seeded).
  // Trim + draw add at output; the walk accumulates on the drawn bias.
  if (stepped) {
    const double walk_sigma = realism_.gyro_bias_walk_radps_sqrt_s *
                              std::sqrt(stamp_s - prev_stamp_s_);
    if (walk_sigma > 0.0) {
      for (int i = 0; i < 3; ++i) {
        bias_draw_radps_[i] += randn(bias_state_) * walk_sigma;
      }
    }
  }
  prev_stamp_s_ = stamp_s;
  const robot::core::Vec3 body_w{omegax, omegay, w};
  const robot::core::Quaternion mount_q = robot::core::quaternion_from_rpy(
      mount_rpy_rad_[0], mount_rpy_rad_[1], mount_rpy_rad_[2]);
  // Inverse mount rotation: body-frame vector -> sensor-frame vector.
  // (R^-1 v, computed as rotation by the conjugate.)
  const robot::core::Quaternion mount_q_inv{
      -mount_q.x, -mount_q.y, -mount_q.z, mount_q.w};
  const robot::core::Vec3 gyro_s =
      robot::core::rotate_vector(mount_q_inv, body_w);
  // Full 3D lever arm at the mount offset (exact rigid-body kinematics):
  // a = a_body + alpha x r + w x (w x r). Planar yaw-only motion reduces
  // to the legacy form exactly (-w^2*rx / alpha*rx terms).
  const double rx = mount_xyz_m_[0], ry = mount_xyz_m_[1],
               rz = mount_xyz_m_[2];
  const double wx = omegax, wy = omegay, wz = w;
  const double ax = alphax, ay = alphay, az = alpha_z;
  // alpha x r:
  const double er_x = ay * rz - az * ry;
  const double er_y = az * rx - ax * rz;
  const double er_z = ax * ry - ay * rx;
  // w x (w x r): inner = w x r, then w x inner.
  const double ix = wy * rz - wz * ry;
  const double iy = wz * rx - wx * rz;
  const double iz = wx * ry - wy * rx;
  const double ce_x = wy * iz - wz * iy;
  const double ce_y = wz * ix - wx * iz;
  const double ce_z = wx * iy - wy * ix;
  const robot::core::Vec3 lever_b{
      origin_accel_b.x + er_x + ce_x,
      origin_accel_b.y + er_y + ce_y,
      origin_accel_b.z + er_z + ce_z};
  const robot::core::Vec3 lever_s =
      robot::core::rotate_vector(mount_q_inv, lever_b);
 // Gravity in the true body attitude (specific force opposes gravity):
 // level reads +g on z; pitch/roll project it onto x/y. For
 // R = Rz(yaw) Ry(pitch) Rx(roll), body specific force is
 // R^T(0, 0, g). Yaw therefore drops out exactly.
 const robot::core::Vec3 grav_b{-9.81 * std::sin(pitch),
 9.81 * std::sin(roll) * std::cos(pitch),
                                 9.81 * std::cos(pitch) * std::cos(roll)};
  const robot::core::Vec3 grav_s =
      robot::core::rotate_vector(mount_q_inv, grav_b);
  // Gyro: exact trim per axis + drawn bias + walk, then white per axis on
  // independent streams. Accel: white per axis on its own stream.
  const double gx[3] = {gyro_s.x, gyro_s.y, gyro_s.z};
  double* out_g[3] = {&out.gyro_x_radps, &out.gyro_y_radps,
                      &out.gyro_z_radps};
  for (int i = 0; i < 3; ++i) {
    *out_g[i] = gx[i] + realism_.gyro_trim_radps[i] + bias_draw_radps_[i] +
                realism_.gyro_noise_radps * randn(gyro_state_);
  }
  out.accel_x_mps2 = lever_s.x + grav_s.x +
                     realism_.accel_noise_mps2 * randn(accel_state_);
  out.accel_y_mps2 = lever_s.y + grav_s.y +
                     realism_.accel_noise_mps2 * randn(accel_state_);
  out.accel_z_mps2 = lever_s.z + grav_s.z +
                     realism_.accel_noise_mps2 * randn(accel_state_);
  return out.valid();
}

}  // namespace lost_signal::sim::providers
