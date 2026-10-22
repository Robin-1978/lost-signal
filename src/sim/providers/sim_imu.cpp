#include "sim/providers/sim_imu.hpp"

namespace lost_signal::sim::providers {

SimImuProvider::SimImuProvider() {
  desc_.endpoint_id = "sim.imu";
  desc_.role = robot::iface::EndpointRole::kImu;
  desc_.backend = robot::iface::ProviderBackend::kSim;
  desc_.frame_id = "imu_link";
}

const robot::iface::EndpointDescriptor& SimImuProvider::descriptor() const {
  return desc_;
}

void SimImuProvider::set_step(std::uint64_t step,
                              robot::core::MonotonicNs stamp_ns) {
  (void)step;
  stamp_ns_ = stamp_ns;
}

bool SimImuProvider::poll(robot::iface::ImuMessage& out) {
  seq_ += 1;
  out.seq = seq_;
  out.stamp = stamp_ns_;
  out.src = desc_.endpoint_id;
  out.frm = robot::core::Frame::kImuLink;
  out.gyro_x_radps = 0.0;
  out.gyro_y_radps = 0.0;
  out.gyro_z_radps = 0.0;
  // Level hover: specific force opposes gravity (+z up in imu_link).
  out.accel_x_mps2 = 0.0;
  out.accel_y_mps2 = 0.0;
  out.accel_z_mps2 = 9.81;
  return out.valid();
}

}  // namespace lost_signal::sim::providers
