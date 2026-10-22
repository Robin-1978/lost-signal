#include "robot/app/robot_application.hpp"

namespace lost_signal::robot::app {

namespace {
class NullMapProvider : public iface::IMapProvider {
 public:
  iface::MapSnapshot active_snapshot() const override { return snapshot_; }
  bool select_map(const std::string& map_id) override {
    if (map_id.empty()) return false;
    snapshot_.map_id = map_id;
    snapshot_.revision += 1;
    return true;
  }

 private:
  iface::MapSnapshot snapshot_{.map_id = "void", .revision = 0};
};
}  // namespace

RobotApplication::RobotApplication(std::unique_ptr<iface::IMapProvider> maps,
                                   platform::RobotDescription desc)
    : maps_(maps ? std::move(maps) : std::make_unique<NullMapProvider>()),
      desc_(std::move(desc)) {
  channel_.activate();
}

core::Status RobotApplication::submit_task(const iface::TaskRequest& req) {
  return executive_.submit(req);
}

core::Status RobotApplication::request_map(const std::string& map_id) {
  if (!maps_->select_map(map_id)) return core::Status::Fail("unknown map");
  return core::Status::Ok();
}

bool RobotApplication::register_provider(const iface::EndpointDescriptor& desc) {
  return registry_.register_endpoint(desc);
}

bool RobotApplication::publish_observation(const iface::ObservationFrame& obs) {
  if (!registry_.owns(obs.source, iface::EndpointRole::kLidar) &&
      !registry_.owns(obs.source, iface::EndpointRole::kImu) &&
      !registry_.owns(obs.source, iface::EndpointRole::kEncoder)) {
    return false;  // unregistered source: never enters the decision path
  }
  return channel_.publish_observation(obs);
}

iface::MotionCommand RobotApplication::take_command(core::MonotonicNs now_ns) {
  return channel_.take_command(now_ns);
}

iface::MotionCommand RobotApplication::tick(core::MonotonicNs now_ns) {
  for (const auto& obs : channel_.drain_observations()) {
    coordinator_.ingest_observation(obs);
  }
  iface::MotionCommand cmd =
      coordinator_.update(maps_->active_snapshot(), now_ns);
  if (!iface::valid_motion_command(cmd, now_ns)) {
    cmd = iface::MotionCommand::SafeStop(now_ns);
  }
  channel_.publish_command(cmd);
  return cmd;
}

iface::TaskStatus RobotApplication::task_status() const {
  return executive_.status();
}

iface::MapSnapshot RobotApplication::active_map() const {
  return maps_->active_snapshot();
}

iface::ChannelHealth RobotApplication::channel_health() const {
  return channel_.health();
}

RobotApplication::Diagnostics RobotApplication::diagnostics() const {
  Diagnostics d;
  d.has_observation = coordinator_.has_observation();
  if (d.has_observation) {
    d.last_obs_stamp_ns = coordinator_.last_observation().stamp_ns;
    d.last_obs_seq = coordinator_.last_observation().sequence;
    d.estimate = coordinator_.estimate();
  }
  d.task = executive_.status();
  d.health = channel_.health();
  return d;
}

}  // namespace lost_signal::robot::app
