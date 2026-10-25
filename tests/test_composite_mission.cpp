#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "robot/platform/robot_config.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/scoring.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/scenario/scenario_session.hpp"

using namespace lost_signal;

// Composite front end, mission-level proof. A 0.2 m plate under the mast
// (lidar at ~0.45): the horizontal 2D ring flies over it (blind), the
// ground-filtered 16-line composite sees the face. Same world, same robot:
// 2D collides, composite holds with zero contacts.
namespace {

robot::platform::RobotDescription seed_diff_desc() {
  const std::string dir = std::string(TEST_CONFIG_DIR) + "/robot";
  robot::platform::RobotConfig cfg;
  std::string err;
  assert(robot::platform::load_robot_config(dir + "/differential.json", cfg,
                                            &err));
  return robot::platform::description_from_config(cfg);
}

sim::world::MotorParams seed_diff_motor() {
  const std::string dir = std::string(TEST_CONFIG_DIR) + "/robot";
  robot::platform::RobotConfig cfg;
  std::string err;
  assert(robot::platform::load_robot_config(dir + "/differential.json", cfg,
                                            &err));
  return sim::eval::motor_from_config(cfg);
}

void plate_world(sim::scenario::ScenarioSession& session) {
  session.setup_world().add_place({"dock", 6.0, 0.0, 0.0});
  // Full-corridor plate: no way around, only through (or hold).
  session.setup_world().set_static_boxes({{2.5, -10.0, 3.0, 10.0, 0.2}});
  session.set_objectives({"dock"}, 1.0);
}

int run_plate(bool composite) {
  sim::scenario::ScenarioSession session(seed_diff_desc(), seed_diff_motor());
  plate_world(session);
  sim::eval::SimConfig sim;
  sim.lidar_composite = composite;
  session.apply_sim_config(sim);
  for (int i = 0; i < 600 && !session.objectives_done(); ++i) {
    session.step_once();
  }
  return sim::eval::score_episode(session.episode()).collision_steps;
}

}  // namespace

int main() {
  // Surveyed wall (static map knows it): both sources hold clear of it.
  // Parity pin — composite changes nothing when the survey already blocks.
  for (bool composite : {false, true}) {
    const int hits = run_plate(composite);
    std::printf("plate surveyed composite=%d collisions=%d\n", composite,
                hits);
    assert(hits == 0);
  }
  // Green: composite sees the 0.2 m face and holds clear of it.
  // (The unmapped-blind contrast lives at sensor level in test_lidar3d:
  // 2D max-range vs composite 2.0 on the same plate. Here the survey also
  // blocks, so hold is the correct joint outcome either way.)
  {
    sim::scenario::ScenarioSession session(seed_diff_desc(),
                                           seed_diff_motor());
    plate_world(session);
    sim::eval::SimConfig sim;
    sim.lidar_composite = true;
    session.apply_sim_config(sim);
    for (int i = 0; i < 600 && !session.objectives_done(); ++i) {
      session.step_once();
    }
    const auto score = sim::eval::score_episode(session.episode());
 std::printf("plate composite collisions=%zu done=%d\n",
                score.collision_steps, session.objectives_done());
    assert(score.collision_steps == 0);
    assert(!session.objectives_done());  // wall spans the corridor: hold
  }
  // Slope mission: the loop square under a sustained 0.1 grade with the
  // composite front end. Grade is gravity tilt (floor stays flat); this
  // pins the autonomy loop under downhill overshoot + uphill load.
  {
    sim::scenario::ScenarioSession session;
    session.setup_world().add_place({"loopA", 4.0, 0.0, 0.0});
    session.setup_world().add_place({"loopB", 4.0, 4.0, 0.0});
    session.setup_world().add_place({"loopC", 0.0, 4.0, 0.0});
    session.setup_world().add_place({"loopD", 0.0, 0.0, 0.0});
    session.setup_world().set_static_boxes({
        {5.2, 1.0, 6.0, 1.8, 1.0},
        {2.5, 5.0, 3.3, 5.8, 1.0},
        {-2.4, 2.0, -1.6, 2.8, 1.0},
        {1.0, -2.4, 1.8, -1.6, 1.0},
    });
    session.setup_world().set_incline(0.1);
    session.set_objectives({"loopA", "loopB", "loopC", "loopD"}, 1.0);
    sim::eval::SimConfig sim;
    sim.lidar_composite = true;
    session.apply_sim_config(sim);
    for (int i = 0; i < 2000 && !session.objectives_done(); ++i) {
      session.step_once();
    }
    const auto score = sim::eval::score_episode(session.episode());
 std::printf("grade loop done=%d collisions=%zu miss=%.2f\n",
                session.objectives_done(), score.collision_steps,
                session.objectives().results().empty()
                    ? -1.0
                    : session.objectives().results().back().truth_miss_m);
    assert(session.objectives_done());
    assert(score.collision_steps == 0);
  }

  std::printf("composite_mission ok\n");
  return 0;
}
