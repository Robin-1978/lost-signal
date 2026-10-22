#include "sim/robot/sim_robot.hpp"

namespace lost_signal::sim::robot {

namespace r = ::lost_signal::robot;

SimRobot::SimRobot()
    : platform_(&world_),
      lidar_("sim.lidar2d", 72, 10.0f),
      app_(nullptr) {
  app_.register_provider({"sim.platform",
                          r::iface::EndpointRole::kEncoder,
                          r::iface::ProviderBackend::kSim, "base_link"});
  app_.register_provider({"sim.platform",
                          r::iface::EndpointRole::kImu,
                          r::iface::ProviderBackend::kSim, "imu_link"});
  app_.register_provider(lidar_.descriptor());
  // Alias: platform also carries the packed scan under the lidar endpoint.
  app_.register_provider({"sim.platform",
                          r::iface::EndpointRole::kLidar,
                          r::iface::ProviderBackend::kSim, "lidar_link"});
  // Harness bringup: ordered configure -> activate; a failed module blocks.
  (void)app_.configure();
  (void)app_.activate();
}

r::core::Status SimRobot::configure() { return app_.configure(); }

r::core::Status SimRobot::activate() { return app_.activate(); }

void SimRobot::shutdown() { app_.shutdown(); }

r::lifecycle::ModuleState SimRobot::bringup_state() const {
  return app_.bringup_state();
}

r::core::Status SimRobot::submit_task(const r::iface::TaskRequest& req) {
  return app_.submit_task(req);
}

r::core::Status SimRobot::request_map(const std::string& map_id) {
  return app_.request_map(map_id);
}

bool SimRobot::register_provider(const r::iface::EndpointDescriptor& desc) {
  return app_.register_provider(desc);
}

bool SimRobot::publish_observation(const r::iface::ObservationFrame& obs) {
  return app_.publish_observation(obs);
}

r::iface::MotionCommand SimRobot::take_command(r::core::MonotonicNs now_ns) {
  return app_.take_command(now_ns);
}

r::iface::MotionCommand SimRobot::tick(r::core::MonotonicNs now_ns) {
  (void)now_ns;
  world_.step(0.02);
  lidar_.set_step(world_.truth().step, static_cast<r::core::MonotonicNs>(
                                           world_.truth().step * 20'000'000));
  r::iface::ObservationFrame obs;
  if (!platform_.poll_observation(obs)) {
    return r::iface::MotionCommand::SafeStop(
        static_cast<r::core::MonotonicNs>(world_.truth().step * 20'000'000));
  }
  // Message-level integration: the typed 2D scan is packed into the LSRI
  // envelope. No truth struct crosses; only ranges + measurement metadata.
  r::iface::Lidar2DScan scan;
  if (lidar_.poll(scan) && scan.valid()) {
    obs.ranges_m = scan.ranges_m;
    obs.scan_valid = true;
    obs.source = "sim.lidar2d";
    obs.frame = r::core::Frame::kLidarLink;
    obs.sequence = scan.seq;
    obs.stamp_ns = scan.stamp;
  }
  if (!app_.publish_observation(obs)) {
    return r::iface::MotionCommand::SafeStop(obs.stamp_ns);
  }
  r::iface::MotionCommand cmd = app_.tick(obs.stamp_ns);
  platform_.consume_command(cmd);
  return cmd;
}

r::iface::TaskStatus SimRobot::task_status() const {
  return app_.task_status();
}

r::iface::MapSnapshot SimRobot::active_map() const {
  return app_.active_map();
}

r::iface::ChannelHealth SimRobot::channel_health() const {
  return app_.channel_health();
}

}  // namespace lost_signal::sim::robot
