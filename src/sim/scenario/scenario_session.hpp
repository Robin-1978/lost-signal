#pragma once

#include <string>
#include <vector>

#include "robot/app/robot_application.hpp"
#include "sim/eval/episode.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/providers/sensor_suite.hpp"
#include "sim/providers/static_map.hpp"
#include "sim/scenario/collision_judge.hpp"
#include "sim/scenario/objectives.hpp"
#include "sim/scenario/scheduler.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::scenario {

// SIM composition root. Owns world truth and presentation state; drives the
// robot only through RobotApplication + LSRI. Must never call autonomy,
// estimator, planner, or safety code directly.
//
// Truth isolation: WorldTruth is recorded into EpisodeRecorder for offline
// comparison / scoring / RL labels. It is never passed to publish_observation,
// tick, or any ROBOT input.
class ScenarioSession {
 public:
  ScenarioSession();
  // Config-driven bring-up: the ROBOT side runs the given description, the
  // SIM side the given motor numbers (both from one RobotConfig file).
  ScenarioSession(robot::platform::RobotDescription desc,
                  world::MotorParams motor);

  robot::iface::GoalHandle request_goal(const robot::iface::TaskRequest& req);
  robot::iface::GoalStatus goal_status(robot::iface::GoalHandle h) const;
  bool cancel_goal(robot::iface::GoalHandle h);
  void step_once();

  const eval::EpisodeRecorder& episode() const { return episode_; }
  void clear_episode() { episode_.clear(); }
  // Test/scenario setup only: configure geometry, actors, and places before
  // the run. Never called by ROBOT code; never during a live tick.
  world::World& setup_world() { return world_; }
  // Multi-rate cadences (sim seconds; 0 disables a task). Scan and record
  // are scheduler taps; the decision tap is currently inert — the ROBOT
  // decision branch rate is gated inside AutonomyCoordinator by its own
  // decision_period_ns (default 10 Hz), not by decision_period_s here.
  // Scenarios may slow the scan/record taps without touching
  // wall-clock-free determinism.
  void set_cadence(double scan_period_s, double decision_period_s,
                   double record_period_s);
  // Objective list: place_ids resolved by the ROBOT map provider through
  // the operator seam; arrival counted truth-side for scoring only.
  void set_objectives(std::vector<std::string> place_ids,
                      double arrive_tol_m = 0.5);
  bool objectives_done() const { return driver_.done(); }
  // Truth-side judge radius (scenario config, not a ROBOT parameter).
  void set_judge_radius_m(double radius_m) {
    judge_radius_m_ = radius_m >= 0.0 ? radius_m : 0.4;
  }
  // Truth-side belly clearance for the terrain-contact judge (scenario
  // config, not a ROBOT parameter): the engine body must stay this far
  // above the authored surface. Non-positive falls back to the default.
  void set_terrain_clearance_m(double clearance_m) {
    terrain_clearance_m_ = clearance_m > 0.0 ? clearance_m : 0.05;
  }
  // Sensor realism switches (S7): delivery shaping + IMU noise/bias.
  // Default off (exact truth); scenario-owned orchestration config.
  void set_sensor_realism(providers::DeliveryConfig delivery,
                          providers::ImuRealismConfig imu) {
    suite_.set_delivery(delivery);
    suite_.set_imu_realism(imu);
  }
  // Whole SIM-side setup from one sim config file (cadence, judge,
  // realism, lidar geometry). Pre-run only: lidar geometry must precede
  // the first step.
  void apply_sim_config(const eval::SimConfig& sim) {
    set_cadence(sim.scan_period_s, sim.decision_period_s,
                sim.record_period_s);
    set_judge_radius_m(sim.judge_radius_m);
    set_sensor_realism(sim.delivery, sim.imu_realism);
    suite_.lidar().set_beams(sim.lidar_beams);
    suite_.lidar().set_max_range(sim.lidar_max_range_m);
    suite_.lidar3d().set_rings(sim.lidar_rings);
    suite_.lidar3d().set_azimuths(sim.lidar_beams);
    suite_.lidar3d().set_max_range(sim.lidar_max_range_m);
    suite_.set_composite_scan(sim.lidar_composite);
  }
  const ObjectiveDriver& objectives() const { return driver_; }

 private:
  world::World world_;
  providers::SimSensorSuite suite_;
  robot::app::RobotApplication app_;
  eval::EpisodeRecorder episode_;
  DeterministicScheduler scheduler_;
  ObjectiveDriver driver_;
  double sim_time_s_ = 0.0;
  double judge_radius_m_ = 0.4;
  double terrain_clearance_m_ = 0.05;
};

}  // namespace lost_signal::sim::scenario
