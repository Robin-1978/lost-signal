#include <cassert>
#include <cstdio>
#include <memory>

#include "robot/app/robot_application.hpp"

namespace {
class PlacesMap : public lost_signal::robot::iface::IMapProvider {
 public:
  PlacesMap() {
    snap_.map_id = "test";
    snap_.revision = 1;
    snap_.frame_id = "map";
    snap_.places.push_back({"dock", 5.0, 0.0, 0.0});
  }
 const lost_signal::robot::iface::MapSnapshot& active_snapshot() const override {
    return snap_;
  }
  bool select_map(const std::string& id) override {
    return id == "test" || id.empty();
  }

 private:
  lost_signal::robot::iface::MapSnapshot snap_;
};
}  // namespace

int main() {
  using namespace lost_signal::robot;

  app::RobotApplication app(std::make_unique<PlacesMap>());
  assert(app.configure().ok && app.activate().ok);
  app.register_provider({"t.lidar", iface::EndpointRole::kLidar,
                         iface::ProviderBackend::kNative, "lidar_link"});

  // Unknown place fails closed and arms nothing.
  iface::TaskRequest bad;
  bad.request_id = "b1";
  bad.place_id = "nowhere";
  assert(!app.request_goal(bad).valid());
  assert(app.current_goal().state != iface::GoalState::kActive);

  // Known place resolves to a coordinator goal through the map provider.
  iface::TaskRequest req;
  req.request_id = "t1";
  req.place_id = "dock";
  iface::GoalHandle h = app.request_goal(req);
  assert(h.valid());
  assert(app.goal_status(h).state == iface::GoalState::kActive);

  // With an observation flowing, the resolved goal becomes a real route and
  // the DWA drives toward it (place -> map -> route -> command).
  iface::ObservationFrame obs;
  obs.sequence = 1;
  obs.stamp_ns = 1'000'000;
  obs.source = "t.lidar";
  obs.frame = core::Frame::kLidarLink;
  obs.scan_valid = true;
  obs.ranges_m.assign(72, 10.0f);
  obs.odometry_valid = true;
  assert(app.publish_observation(obs));
  // Settling takes a few decision cycles (controlled-stop latch).
  iface::MotionCommand cmd = app.tick(1'000'000);
  for (int i = 0; i < 4 && cmd.forward_mps <= 0.0; ++i) {
 cmd = app.tick(1'000'000LL +
 static_cast<core::MonotonicNs>(i + 1) * 100'000'000LL);
  }
  assert(cmd.forward_mps > 0.0);

  std::printf("places ok\n");
  return 0;
}
