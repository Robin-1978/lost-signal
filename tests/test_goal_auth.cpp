#include <cassert>
#include <cstdio>
#include <memory>

#include "robot/app/robot_application.hpp"

namespace {
class PlacesMap : public lost_signal::robot::iface::IMapProvider {
 public:
  explicit PlacesMap(double dock_x_m = 5.0) {
    snap_.map_id = "test";
    snap_.revision = 1;
    snap_.frame_id = "map";
    snap_.places.push_back({"dock", dock_x_m, 0.0, 0.0});
  }
  lost_signal::robot::iface::MapSnapshot active_snapshot() const override {
    return snap_;
  }
  bool select_map(const std::string& id) override {
    return id == "test" || id.empty();
  }

 private:
  lost_signal::robot::iface::MapSnapshot snap_;
};

void feed_obs(lost_signal::robot::app::RobotApplication& app,
              int seq, lost_signal::robot::core::MonotonicNs stamp_ns) {
  using namespace lost_signal::robot;
  iface::ObservationFrame obs;
  obs.sequence = static_cast<std::uint64_t>(seq);
  obs.stamp_ns = stamp_ns;
  obs.source = "t.lidar";
  obs.frame = core::Frame::kLidarLink;
  obs.scan_valid = true;
  obs.ranges_m.assign(72, 10.0f);
  obs.odometry_valid = true;
  assert(app.publish_observation(obs));
}

// Monotone feeder: sequence and stamp strictly increase across the whole
// test (the channel rejects stale per-source stamps).
struct Feeder {
  int seq = 1;
  lost_signal::robot::core::MonotonicNs t_ns = 1'000'000;
  void feed(lost_signal::robot::app::RobotApplication& app) {
    feed_obs(app, seq++, t_ns);
  }
  lost_signal::robot::iface::MotionCommand tick(
      lost_signal::robot::app::RobotApplication& app) {
    feed(app);
    const auto cmd = app.tick(t_ns);
    t_ns += 100'000'000;
    return cmd;
  }
};
}  // namespace

int main() {
  using namespace lost_signal::robot;

  app::RobotApplication app(std::make_unique<PlacesMap>());
  assert(app.configure().ok && app.activate().ok);
  app.register_provider({"t.lidar", iface::EndpointRole::kLidar,
                         iface::ProviderBackend::kNative, "lidar_link"});

  // --- Pin 1: a task the Executive rejects never enters planning. ---
  // Valid place, but empty request id: the executive fails closed with an
  // invalid handle. The coordinator must arm nothing: ticks stay stopped.
  Feeder feed;
  {
    iface::TaskRequest rejected;
    rejected.request_id = "";  // executive rejects: no acceptance
    rejected.place_id = "dock";
    const iface::GoalHandle h = app.request_goal(rejected);
    assert(!h.valid());
    assert(app.current_goal().state != iface::GoalState::kActive);
    for (int i = 0; i < 6; ++i) {
      assert(feed.tick(app).is_safe_stop());
    }
  }

  // --- Accepted task plans and drives (positive control). ---
  iface::TaskRequest req;
  req.request_id = "t1";
  req.place_id = "dock";
  const iface::GoalHandle h = app.request_goal(req);
  assert(h.valid());
  assert(app.goal_status(h).state == iface::GoalState::kActive);
  bool driving = false;
  for (int i = 0; i < 8 && !driving; ++i) {
    driving = feed.tick(app).forward_mps > 0.0;
  }
  assert(driving);

  // --- Failed cancel leaves the route alone. ---
  assert(!app.cancel_goal(iface::GoalHandle{}));  // invalid handle
  assert(app.goal_status(h).state == iface::GoalState::kActive);
  assert(feed.tick(app).forward_mps > 0.0);  // still driving

  // --- Pin 2: successful cancel withdraws the coordinator's route. ---
  assert(app.cancel_goal(h));
  assert(app.goal_status(h).state == iface::GoalState::kCanceled);
  for (int i = 0; i < 6; ++i) {
    assert(feed.tick(app).is_safe_stop());  // dead goal drives nothing
  }

  // --- Hold withdraws the active route immediately. ---
 {
 iface::TaskRequest again;
 again.request_id = "t2";
 again.place_id = "dock";
 const iface::GoalHandle h2 = app.request_goal(again);
 assert(h2.valid());
 bool moving = false;
 for (int i = 0; i < 8 && !moving; ++i) {
 moving = feed.tick(app).forward_mps > 0.0;
 }
 assert(moving);
 iface::TaskRequest hold;
 hold.request_id = "hold1"; // empty place = hold
 const iface::GoalHandle hh = app.request_goal(hold);
 assert(hh.valid());
 assert(app.goal_status(hh).state == iface::GoalState::kDone);
 for (int i = 0; i < 6; ++i) {
 assert(feed.tick(app).is_safe_stop());
 }
 }

 // --- Arrival completes the goal and stops on the arrival tick. ---
 {
 app::RobotApplication near(std::make_unique<PlacesMap>(1.0));
 assert(near.configure().ok && near.activate().ok);
 near.register_provider({"t.lidar", iface::EndpointRole::kLidar,
 iface::ProviderBackend::kNative, "lidar_link"});
 iface::TaskRequest task;
 task.request_id = "arrive";
 task.place_id = "dock";
 const iface::GoalHandle h = near.request_goal(task);
 assert(h.valid());
 Feeder nf;
 bool arrived = false;
 bool settling_stop_seen = false;
 double measured_speed_mps = 0.5;
 for (int i = 0; i < 80; ++i) {
 iface::ObservationFrame obs;
 obs.sequence = static_cast<std::uint64_t>(nf.seq++);
 obs.stamp_ns = nf.t_ns;
 obs.source = "t.lidar";
 obs.frame = core::Frame::kLidarLink;
 obs.scan_valid = true;
 obs.ranges_m.assign(72, 10.0f);
 obs.odometry_valid = true;
 obs.forward_mps = measured_speed_mps;
 assert(near.publish_observation(obs));
 const auto cmd = near.tick(nf.t_ns);
 nf.t_ns += 100'000'000;
 // Model the plant response to the arrival hold. A moving observation must
 // never complete the task; once the command stops near the goal, publish
 // settled wheel motion on subsequent samples.
 if (near.diagnostics().estimate.x_m > 0.70 && cmd.is_safe_stop()) {
 settling_stop_seen = true;
 measured_speed_mps = 0.0;
 }
 if (near.current_goal().state == iface::GoalState::kDone) {
 assert(cmd.is_safe_stop());
 arrived = true;
 break;
 }
 }
 assert(settling_stop_seen);
 assert(arrived);
 assert(near.current_goal().state == iface::GoalState::kDone);
 }

 std::printf("goal_auth ok\n");
  return 0;
}
