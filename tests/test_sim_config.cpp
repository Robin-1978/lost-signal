#include <cassert>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "sim/eval/sim_config.hpp"
#include "sim/scenario/scenario_session.hpp"

int main() {
  using namespace lost_signal;
  const std::string dir =
      std::string(TEST_CONFIG_DIR);  // CMake-injected configs path

  // Default seed loads with code-default values.
  {
    sim::eval::SimConfig cfg;
    std::string err;
    assert(sim::eval::load_sim_config(dir + "/sim/default.json", cfg, &err));
 assert(cfg.scan_period_s == 0.02 && cfg.motion_period_s == 0.02 &&
 cfg.imu_period_s == 0.02 && cfg.decision_period_s == 0.02 &&
 cfg.planning_period_s == 0.02 && cfg.record_period_s == 0.02);
    assert(cfg.judge_radius_m == 0.4);
 assert(cfg.objective_acceptance.version == 2);
 assert(cfg.objective_acceptance.position_tolerance_m == 0.25);
 assert(cfg.objective_acceptance.heading_tolerance_rad == 0.2);
 assert(cfg.objective_acceptance.maximum_linear_speed_mps == 0.08);
 assert(cfg.objective_acceptance.maximum_yaw_rate_radps == 0.08);
 assert(cfg.objective_acceptance.stable_steps == 2);
 assert(cfg.objective_acceptance.terminal_observation_s == 0.5);
    assert(cfg.lidar_beams == 72 && cfg.lidar_max_range_m == 10.0f);
    assert(cfg.lidar_rings == 16);
    assert(!cfg.lidar_composite);  // opt-in front end, default off
    assert(cfg.sensor_realism.version == 1);
    assert(cfg.sensor_realism.name == "nominal");
    assert(cfg.sensor_realism.seed == 20261030);
    assert(cfg.delivery.latency_steps == 0);
    assert(cfg.delivery.drop_probability == 0.0);
    assert(cfg.imu_realism.gyro_noise_radps == 0.0015);
    assert(cfg.lidar_realism.range_stddev_m == 0.003);
  }
  // Overrides + session apply: record disabled empties the episode,
  // lidar geometry reaches the provider.
  {
    sim::eval::SimConfig cfg;
    std::string err;
    assert(sim::eval::load_sim_config(dir + "/sim/default.json", cfg, &err));
    cfg.record_period_s = 0.0;  // record tap off
    cfg.lidar_beams = 36;
    sim::scenario::ScenarioSession session;
    session.setup_world().add_place({"dock", 6.0, 0.0, 0.0});
    robot::iface::TaskRequest req;
    req.request_id = "s1";
    req.place_id = "dock";
    assert(session.request_goal(req).valid());
    session.apply_sim_config(cfg);
    for (int i = 0; i < 10; ++i) session.step_once();
    assert(session.episode().steps().empty());  // nothing recorded
  }
 // Programmatic configs fail closed too; bypassing the JSON loader cannot
 // silently retain an earlier SIM acceptance contract.
 {
 sim::eval::SimConfig cfg;
 cfg.objective_acceptance.stable_steps = 0;
 sim::scenario::ScenarioSession session;
 bool threw = false;
 try {
 session.apply_sim_config(cfg);
 } catch (const std::invalid_argument&) {
 threw = true;
 }
 assert(threw);
 }
 // Fail-closed cases.
  {
    const std::string tmp =
        (std::filesystem::temp_directory_path() / "test_sim_cfg.json")
            .string();
    auto write = [&](const std::string& body) {
      std::FILE* fp = std::fopen(tmp.c_str(), "wb");
      std::fputs(body.c_str(), fp);
      std::fclose(fp);
    };
    sim::eval::SimConfig cfg;
    std::string err;
    write(R"({"cadence": {"scan_s": -1}})");
    assert(!sim::eval::load_sim_config(tmp, cfg, &err));
 write(R"({"cadence": {"motion_s": -1}})");
 assert(!sim::eval::load_sim_config(tmp, cfg, &err));
 write(R"({"cadence": {"imu_s": -1}})");
 assert(!sim::eval::load_sim_config(tmp, cfg, &err));
 write(R"({"cadence": {"motion_s": 0.05, "imu_s": 0.01}})");
 assert(sim::eval::load_sim_config(tmp, cfg, &err));
 assert(cfg.motion_period_s == 0.05 && cfg.imu_period_s == 0.01);
 write(R"({"cadence": {"scan_s": 0.0065}})");
 assert(!sim::eval::load_sim_config(tmp, cfg, &err));
 assert(err.find("1 ms physics tick") != std::string::npos);
 write(R"({"cadence": {"planning_s": -0.1}})");
 assert(!sim::eval::load_sim_config(tmp, cfg, &err));
    write(R"({"delivery": {"drop_probability": 1.5}})");
    assert(!sim::eval::load_sim_config(tmp, cfg, &err));
 write(R"({"objective_acceptance": {"version": 1}})");
 assert(!sim::eval::load_sim_config(tmp, cfg, &err));
 write(R"({"objective_acceptance": {"version": 2,
 "position_tolerance_m": 0.2, "heading_tolerance_rad": 0.2,
 "maximum_linear_speed_mps": 0.1, "maximum_yaw_rate_radps": 0.1,
 "stable_steps": 0, "terminal_observation_s": 0.5}})");
 assert(!sim::eval::load_sim_config(tmp, cfg, &err));
 write(R"({"objective_acceptance": {"version": 2,
 "position_tolerance_m": 0.2, "heading_tolerance_rad": 0.2,
 "maximum_linear_speed_mps": 0.1, "maximum_yaw_rate_radps": 0.1,
 "stable_steps": 2, "terminal_observation_s": 0.0}})");
 assert(!sim::eval::load_sim_config(tmp, cfg, &err));
    write(R"({"lidar": {"beams": 0}})");
    assert(!sim::eval::load_sim_config(tmp, cfg, &err));
    write(R"({"lidar": {"rings": 0}})");
    assert(!sim::eval::load_sim_config(tmp, cfg, &err));
    write(R"({"lidar": {"composite": 1}})");
    assert(!sim::eval::load_sim_config(tmp, cfg, &err));
    write(R"({"lidar": {"composite": true}})");
    assert(sim::eval::load_sim_config(tmp, cfg, &err));
    assert(cfg.lidar_composite);
    write(R"({"imu_realism": {"gyro_noise_radps": -0.1}})");
    assert(!sim::eval::load_sim_config(tmp, cfg, &err));
    write(R"({"sensor_realism": {"version": 1, "profile": "unknown"}})");
    assert(!sim::eval::load_sim_config(tmp, cfg, &err));
    write(R"({"sensor_realism": {"version": 2, "profile": "ideal"}})");
    assert(!sim::eval::load_sim_config(tmp, cfg, &err));
    write(R"({"sensor_realism": {"version": 1, "profile": "adverse"}, "delivery": {}})");
    assert(!sim::eval::load_sim_config(tmp, cfg, &err));
    write(R"({"sensor_realism": {"version": 1, "profile": "adverse", "seed": 9}})");
    assert(sim::eval::load_sim_config(tmp, cfg, &err));
    assert(cfg.sensor_realism.name == "adverse" && cfg.sensor_realism.seed == 9);
    assert(cfg.delivery.latency_steps == 2 && cfg.delivery.drop_probability == 0.08);
    assert(cfg.lidar_realism.dropout_probability == 0.12);
    sim::providers::DeliveryConfig d1, d2;
    sim::providers::ImuRealismConfig i1, i2;
    sim::providers::LidarDeviceModel l1, l2;
    sim::eval::SensorRealismProfileConfig p;
    p.name = "nominal";
    p.seed = 77;
    assert(sim::eval::resolve_sensor_realism_profile(p, d1, i1, l1, &err));
    assert(sim::eval::resolve_sensor_realism_profile(p, d2, i2, l2, &err));
    assert(d1.seed == d2.seed && i1.gyro_noise_radps == i2.gyro_noise_radps &&
           l1.range_stddev_m == l2.range_stddev_m);
    const auto old = d1;
    p.name = "bad";
    assert(!sim::eval::resolve_sensor_realism_profile(p, d1, i1, l1, &err));
    assert(d1.seed == old.seed);
    write(R"({bad json)");
    assert(!sim::eval::load_sim_config(tmp, cfg, &err));
    write(R"({})");  // empty = all defaults
    assert(sim::eval::load_sim_config(tmp, cfg, &err));
    assert(cfg.lidar_beams == 72 && cfg.judge_radius_m == 0.4);
    assert(cfg.lidar_rings == 16);
    assert(!cfg.lidar_composite);
    assert(cfg.sensor_realism.name == "ideal");
    std::filesystem::remove(tmp);
  }

  std::printf("sim_config ok\n");
  return 0;
}
