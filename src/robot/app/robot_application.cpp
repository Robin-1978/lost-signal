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

RobotApplication::RobotApplication(std::unique_ptr<iface::IMapProvider> maps)
    : maps_(maps ? std::move(maps) : std::make_unique<NullMapProvider>()) {}

core::Status RobotApplication::submit_task(const iface::TaskRequest& req) {
  return executive_.submit(req);
}

core::Status RobotApplication::request_map(const std::string& map_id) {
  if (!maps_->select_map(map_id)) return core::Status::Fail("unknown map");
  return core::Status::Ok();
}

void RobotApplication::publish_observation(const iface::ObservationFrame& obs) {
  coordinator_.ingest_observation(obs);
}

iface::MotionCommand RobotApplication::tick(core::MonotonicNs now_ns) {
  iface::MotionCommand cmd = coordinator_.update(maps_->active_snapshot(), now_ns);
  if (!iface::valid_motion_command(cmd, now_ns)) {
    return iface::MotionCommand::SafeStop(now_ns);
  }
  return cmd;
}

iface::TaskStatus RobotApplication::task_status() const {
  return executive_.status();
}

iface::MapSnapshot RobotApplication::active_map() const {
  return maps_->active_snapshot();
}

}  // namespace lost_signal::robot::app
