#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "sim/eval/scoring.hpp"
#include "sim/providers/sensor_suite.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/world/world.hpp"

using namespace lost_signal;

namespace {

sim::providers::DeliveryConfig delivery(std::uint64_t seed, int latency,
                                        double drop) {
  sim::providers::DeliveryConfig c;
  c.seed = seed;
  c.latency_steps = latency;
  c.drop_probability = drop;
  return c;
}

sim::providers::ImuRealismConfig imu(std::uint64_t seed, double noise,
                                     double trim_z, double accel) {
  sim::providers::ImuRealismConfig c;
  c.seed = seed;
  c.gyro_noise_radps = noise;
  c.gyro_trim_radps[2] = trim_z;
  c.accel_noise_mps2 = accel;
  return c;
}

}  // namespace

int main() {
  using sim::providers::SimSensorSuite;

  // Default off: passthrough, stamps track the plant step exactly.
  {
    sim::world::World w;
    SimSensorSuite suite(&w);
    robot::iface::ObservationFrame obs;
    for (int i = 0; i < 5; ++i) {
      w.step(0.02);
      assert(suite.poll_envelope(obs));
      assert(obs.stamp_ns == (w.truth().step * 20'000'000));
    }
  }

  // Latency shifts the stream by whole ticks, stamps preserved.
  {
    sim::world::World w;
    SimSensorSuite suite(&w);
    suite.set_delivery(delivery(1, 2, 0.0));
    robot::iface::ObservationFrame obs;
    w.step(0.02);
    assert(!suite.poll_envelope(obs));  // filling step 1
    w.step(0.02);
    assert(!suite.poll_envelope(obs));  // filling step 2
    w.step(0.02);
    assert(suite.poll_envelope(obs));  // delivers step 1
    assert(obs.stamp_ns == 1 * 20'000'000);
    w.step(0.02);
    assert(suite.poll_envelope(obs));  // delivers step 2
    assert(obs.stamp_ns == 2 * 20'000'000);
  }

  // Dropout is seeded-deterministic: same seed, identical accept pattern.
  {
    auto run = [](std::uint64_t seed) {
      sim::world::World w;
      SimSensorSuite suite(&w);
      suite.set_delivery(delivery(seed, 0, 0.5));
      std::vector<bool> accepted;
      robot::iface::ObservationFrame obs;
      for (int i = 0; i < 50; ++i) {
        w.step(0.02);
        accepted.push_back(suite.poll_envelope(obs));
      }
      return accepted;
    };
    const auto a = run(7);
    const auto b = run(7);
    assert(a == b);  // bit-identical pattern
    std::size_t hits = 0;
    for (bool v : a) {
      if (v) ++hits;
    }
    assert(hits > 10 && hits < 40);  // drops and passes both happen
    // Full loss: nothing ever delivers.
    sim::world::World w;
    SimSensorSuite suite(&w);
    suite.set_delivery(delivery(3, 0, 1.0));
    robot::iface::ObservationFrame obs;
    for (int i = 0; i < 5; ++i) {
      w.step(0.02);
      assert(!suite.poll_envelope(obs));
    }
  }

  // Invalid configs are ignored (previous settings kept).
  {
    sim::world::World w;
    SimSensorSuite suite(&w);
    suite.set_delivery(delivery(1, -1, 0.0));
    suite.set_delivery(delivery(1, 0, 1.5));
    robot::iface::ObservationFrame obs;
    w.step(0.02);
    assert(suite.poll_envelope(obs));  // still passthrough
  }

  // IMU realism: exact truth by default; bias offsets exactly; noise is
  // bounded, zero-mean, and seeded-deterministic.
  {
    sim::world::World w;
    SimSensorSuite suite(&w);
    w.step(0.02);
    robot::iface::ObservationFrame obs;
    assert(suite.poll_envelope(obs));
    assert(obs.imu_gyro_z_radps == 0.0);  // at rest, exact
    suite.set_imu_realism(imu(5, 0.0, 0.05, 0.0));
    assert(suite.poll_envelope(obs));
    assert(obs.imu_gyro_z_radps == 0.05);  // bias only: exact offset
    assert(obs.imu_gyro_x_radps == 0.0 && obs.imu_az_mps2 == 9.81);
    // Noise: bounded (6 sigma) and zero-mean over the stream.
    suite.set_imu_realism(imu(9, 0.02, 0.0, 0.01));
    double sum = 0.0;
    double max_abs = 0.0;
    for (int i = 0; i < 200; ++i) {
      assert(suite.poll_envelope(obs));
      const double e = obs.imu_gyro_z_radps;  // truth is 0 at rest
      max_abs = std::max(max_abs, std::abs(e));
      sum += e;
    }
    assert(max_abs <= 6.0 * 0.02);
    assert(std::abs(sum / 200.0) < 0.01);
    // Same seed replays the identical noise stream.
    SimSensorSuite suite2(&w);
    suite2.set_imu_realism(imu(9, 0.02, 0.0, 0.01));
    suite.set_imu_realism(imu(9, 0.02, 0.0, 0.01));
    for (int i = 0; i < 20; ++i) {
      robot::iface::ObservationFrame o1, o2;
      assert(suite.poll_envelope(o1));
      assert(suite2.poll_envelope(o2));
      assert(o1.imu_gyro_z_radps == o2.imu_gyro_z_radps);
    }
  }

  // Delivery latency is measured against the current plant clock. A
 // latency above the 500 ms freshness budget must produce safe stops.
 {
 sim::scenario::ScenarioSession stale;
 stale.setup_world().add_place({"dock", 5.0, 0.0, 0.0});
 stale.set_sensor_realism(delivery(3, 30, 0.0), imu(4, 0.0, 0.0, 0.0));
 stale.set_objectives({"dock"}, 1.0);
 bool saw_stale_stop = false;
 for (int i = 0; i < 80; ++i) {
 stale.step_once();
 const auto& step = stale.episode().steps().back();
 if (step.obs_accepted && step.cmd.is_safe_stop()) saw_stale_stop = true;
 }
 assert(saw_stale_stop);
 }

 // HIL with realism on: latency + dropout + IMU noise still completes a
  // straight mission with no collisions.
  {
    auto run = []() {
      sim::scenario::ScenarioSession session;
 session.setup_world().set_static_boxes({
 {-1.0, 2.5, 0.0, 3.5, 1.5},
 {2.0, -3.5, 3.0, -2.5, 1.5},
 {5.5, 2.5, 6.5, 3.5, 1.5},
 });
 session.set_initial_pose(0.0, 0.0, 0.0, 0.05, 0.02);
      session.setup_world().add_place({"dock", 5.0, 0.0, 0.0});
      session.set_sensor_realism(delivery(11, 2, 0.05),
                                 imu(13, 0.01, 0.005, 0.02));
      session.set_objectives({"dock"}, 1.0);
      std::vector<robot::iface::MotionCommand> cmds;
 for (int i = 0; i < 800 && !session.objectives_done(); ++i) {
        session.step_once();
        cmds.push_back(session.episode().steps().back().cmd);
      }
 if (!session.objectives_done()) {
 const auto& last = session.episode().steps().back();
 std::fprintf(stderr,
 "realism mission stalled: truth=(%.3f,%.3f,%.3f) "
 "estimate=(%.3f,%.3f,%.3f) sigma=%.3f goal=%d/%s "
 "decision=%s/%s\n",
 last.truth_after.body_x_m, last.truth_after.body_y_m,
 last.truth_after.body_yaw_rad, last.diag.estimate.x_m,
 last.diag.estimate.y_m, last.diag.estimate.yaw_rad,
 last.diag.estimate.position_sigma_m,
 static_cast<int>(last.diag.goal.state), last.diag.goal.detail.c_str(),
 last.diag.evidence.constraint_code.c_str(),
 last.diag.evidence.reason_code.c_str());
 }
 return std::make_pair(session.objectives_done(), cmds);
    };
    const auto first = run();
    const auto second = run();
    assert(first.first && second.first);
    assert(first.second.size() == second.second.size());
    for (std::size_t i = 0; i < first.second.size(); ++i) {
      assert(first.second[i].space == second.second[i].space);
      assert(first.second[i].forward_mps == second.second[i].forward_mps);
      assert(first.second[i].yaw_rate_radps == second.second[i].yaw_rate_radps);
    }
    // Score the run: arrived, no judge hits, observations mostly accepted.
    sim::scenario::ScenarioSession scored;
 scored.setup_world().set_static_boxes({
 {-1.0, 2.5, 0.0, 3.5, 1.5},
 {2.0, -3.5, 3.0, -2.5, 1.5},
 {5.5, 2.5, 6.5, 3.5, 1.5},
 });
 scored.set_initial_pose(0.0, 0.0, 0.0, 0.05, 0.02);
    scored.setup_world().add_place({"dock", 5.0, 0.0, 0.0});
    scored.set_sensor_realism(delivery(11, 2, 0.05), imu(13, 0.01, 0.005, 0.02));
    scored.set_objectives({"dock"}, 1.0);
 for (int i = 0; i < 800 && !scored.objectives_done(); ++i) {
      scored.step_once();
    }
    assert(scored.objectives_done());
    assert(scored.objectives().results()[0].truth_arrived);
    const auto score = sim::eval::score_episode(scored.episode());
    assert(score.collision_steps == 0);
    assert(score.obs_rejected > 0);  // dropout visibly exercised
    assert(score.obs_accepted > 100);
  }

  std::printf("realism ok\n");
  return 0;
}
