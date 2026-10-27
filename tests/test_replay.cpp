#include <cassert>
#include <cstdio>
#include <filesystem>
#include <memory>
#include <string>
#include <vector>

#include "robot/app/robot_application.hpp"
#include "robot/core/json.hpp"
#include "sim/eval/recording.hpp"
#include "sim/providers/replay.hpp"
#include "sim/providers/sim_motion.hpp"
#include "sim/providers/static_map.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/world/world.hpp"

namespace {
  std::string tmp_path(const char* name) {
  return (std::filesystem::temp_directory_path() / name).string();
}

lost_signal::robot::iface::ObservationFrame make_obs() {
  using namespace lost_signal::robot::iface;
  ObservationFrame o;
  o.sequence = 7;
  o.stamp_ns = 123456789;
  o.source = "sim.motion";
  o.frame = lost_signal::robot::core::Frame::kBaseLink;
 o.scan_header = {3, 100'000'000, "sim.lidar2d",
 lost_signal::robot::core::Frame::kLidarLink};
 o.motion_header = {7, 123'456'789, "sim.motion",
 lost_signal::robot::core::Frame::kBaseLink};
 o.imu_header = {11, 120'000'000, "sim.imu",
 lost_signal::robot::core::Frame::kImuLink};
  o.scan_valid = true;
  o.ranges_m = {1.0f, 2.5f, 10.0f};
 o.intensity_valid = true;
 o.intensities = {0.1f, 0.2f, 0.3f};
 o.time_offset_valid = true;
 o.time_offsets_s = {-0.01f, 0.0f, 0.01f};
  o.scan_static_valid = true;
  o.static_ranges_m = {1.0f, 2.5f, 10.0f};
  o.scan_angle_min_rad = -3.141592653589793;
  o.scan_angle_increment_rad = 0.5;
  o.odometry_valid = true;
  o.forward_mps = 1.25;
 o.lateral_mps = -0.75;
  o.yaw_rate_radps = -0.5;
  o.imu_valid = true;
  o.imu_gyro_x_radps = 0.01;
  o.imu_gyro_y_radps = -0.02;
  o.imu_gyro_z_radps = 0.5;
  o.imu_ax_mps2 = 0.1;
  o.imu_ay_mps2 = 0.2;
  o.imu_az_mps2 = 9.81;
  o.drive_count = 2;
  o.drive_radps[0] = 11.0;
  o.drive_radps[1] = 13.5;
 o.arm_count = 2;
 o.arm_rad[0] = 0.3;
 o.arm_rad[1] = -0.2;
 o.arm_vel_radps[0] = 0.4;
 o.arm_vel_radps[1] = -0.1;
  return o;
}
}  // namespace

int main() {
  using namespace lost_signal;

  // 1. JSON subset: round trip incl. escapes, unicode, nesting.
  {
    robot::core::Json::Object inner;
    inner.emplace_back("x", robot::core::Json::Num(3.25));
    robot::core::Json::Object o;
    o.emplace_back("s", robot::core::Json::Str("a\"b\\c\n\xc3\xa9"));
    o.emplace_back("n", robot::core::Json::Null());
    o.emplace_back("b", robot::core::Json::Bool(true));
    o.emplace_back("nested", robot::core::Json::Obj(std::move(inner)));
    std::string text = robot::core::json_dump(robot::core::Json::Obj(o));
    std::string err;
    auto back = robot::core::json_parse(text, &err);
    assert(back && back->is_object());
    assert(back->find("s")->as_str() == "a\"b\\c\n\xc3\xa9");
    assert(robot::core::json_dump(*back) == text);  // canonical re-dump
  }

  // 2. Malformed documents are rejected with a reason, never throw.
  {
    std::string err;
    assert(!robot::core::json_parse("{bad", &err) && !err.empty());
    assert(!robot::core::json_parse("[1,]", &err));
    assert(!robot::core::json_parse("{\"a\":0x10}", &err));
    assert(!robot::core::json_parse("{\"a\":1e}", &err));
    assert(!robot::core::json_parse("{\"a\":\"\\q\"}", &err));
    assert(!robot::core::json_parse("{\"a\":1} trailing", &err));
    assert(!robot::core::json_parse("", &err));
    sim::eval::RecordedStep s;
    std::string rerr;
    assert(!sim::eval::step_from_json_line("not json", s, &rerr));
    assert(!sim::eval::step_from_json_line("{\"v\":2}", s, &rerr));
    assert(!sim::eval::step_from_json_line("{\"v\":1}", s, &rerr));
  }

  // 3. Observation round-trips bit-identical (float %.9g, double %.17g).
  {
    auto obs = make_obs();
    sim::eval::RecordedStep rec = sim::eval::record_step(obs);
    std::string err;
    std::string line = sim::eval::step_to_json_line(rec);
    sim::eval::RecordedStep back;
    assert(sim::eval::step_from_json_line(line, back, &err));
    auto obs2 = sim::eval::step_to_observation(back);
    assert(obs2.sequence == obs.sequence);
    assert(obs2.stamp_ns == obs.stamp_ns);
    assert(obs2.source == obs.source);
 assert(obs2.scan_header.sequence == obs.scan_header.sequence);
 assert(obs2.scan_header.stamp_ns == obs.scan_header.stamp_ns);
 assert(obs2.scan_header.source == obs.scan_header.source);
 assert(obs2.motion_header.source == obs.motion_header.source);
 assert(obs2.imu_header.source == obs.imu_header.source);
    assert(obs2.ranges_m == obs.ranges_m);
 assert(obs2.intensity_valid == obs.intensity_valid);
 assert(obs2.intensities == obs.intensities);
 assert(obs2.time_offset_valid == obs.time_offset_valid);
 assert(obs2.time_offsets_s == obs.time_offsets_s);
    assert(obs2.static_ranges_m == obs.static_ranges_m);
    assert(obs2.forward_mps == obs.forward_mps);
 assert(obs2.lateral_mps == obs.lateral_mps);
    assert(obs2.imu_gyro_z_radps == obs.imu_gyro_z_radps);
    assert(obs2.drive_count == 2);
    assert(obs2.drive_radps[0] == obs.drive_radps[0]);
    assert(obs2.drive_radps[1] == obs.drive_radps[1]);
 assert(obs2.arm_count == 2);
 assert(obs2.arm_rad[0] == obs.arm_rad[0]);
 assert(obs2.arm_rad[1] == obs.arm_rad[1]);
 assert(obs2.arm_vel_radps[0] == obs.arm_vel_radps[0]);
  }

  // 4. Deterministic closed-loop replay: record a live run, export it,
  // replay through a fresh stack + fresh plant, expect identical commands.
  {
    sim::scenario::ScenarioSession session;
    session.setup_world().add_place({"dock", 6.0, 0.0, 0.0});
    robot::iface::TaskRequest req;
    req.request_id = "replay";
    req.place_id = "dock";
    session.request_goal(req);
    for (int i = 0; i < 60; ++i) session.step_once();
    assert(session.episode().size() == 60);

    const std::string path = tmp_path("test_replay_episode.jsonl");
    std::string err;
    assert(sim::providers::export_episode(path, "sim-world", 2,
                                          session.episode(), &err));
    sim::providers::ReplayProvider replay;
    assert(replay.load(path, &err));
    assert(replay.size() == 60);

    // Fresh stack + fresh plant, same map content (empty world + dock place).
    sim::world::World map_world;
    map_world.add_place({"dock", 6.0, 0.0, 0.0});
    auto maps = std::make_unique<sim::providers::StaticMapProvider>(
        &map_world, -10.0, -10.0, 20.0, 20.0, 0.25);
    robot::app::RobotApplication app(std::move(maps));
    assert(app.configure().ok && app.activate().ok);
 for (const auto& descriptor : replay.descriptors()) {
 assert(app.register_provider(descriptor));
 }
    robot::iface::TaskRequest replay_req;
    replay_req.request_id = "replay";
    replay_req.place_id = "dock";
    assert(app.request_goal(replay_req).valid());
    sim::world::World plant;
    sim::providers::SimMotionEndpoint motion(&plant);

    // Re-record the live commands for comparison, then consume both.
    std::vector<robot::iface::MotionCommand> live_cmds;
    for (const auto& st : session.episode().steps()) live_cmds.push_back(st.cmd);

    std::vector<robot::iface::MotionCommand> replayed_cmds;
    robot::iface::ObservationFrame obs;
    int steps = 0;
    while (replay.poll(obs)) {
      assert(app.publish_observation(obs));
      robot::iface::MotionCommand cmd = app.tick(obs.stamp_ns);
      replayed_cmds.push_back(cmd);
      motion.consume_command(app.take_actuator(obs.stamp_ns));
      plant.step(0.02);
      ++steps;
    }
    assert(steps == 60);
    assert(replayed_cmds.size() == live_cmds.size());
    for (std::size_t i = 0; i < live_cmds.size(); ++i) {
      assert(replayed_cmds[i].space == live_cmds[i].space);
      assert(replayed_cmds[i].forward_mps == live_cmds[i].forward_mps);
      assert(replayed_cmds[i].yaw_rate_radps == live_cmds[i].yaw_rate_radps);
    }
    // Replay past the end reports exhaustion instead of inventing data.
    assert(!replay.poll(obs));
    std::filesystem::remove(path);
  }

  std::printf("replay ok\n");
  return 0;
}
