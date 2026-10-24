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
 robot::iface::GoalStatus current_goal() const { return app_.current_goal(); }
 robot::iface::MapSnapshot active_map() const { return app_.active_map(); }
 robot::autonomy::diagnostics::DecisionEvidence last_evidence() const {
 return app_.last_evidence();
 }
  bool cancel_goal(robot::iface::GoalHandle h);
  void step_once();

  const eval::EpisodeRecorder& episode() const { return episode_; }
  void clear_episode() { episode_.clear(); }
  // Test/scenario setup only: configure geometry, actors, and places before
  // the run. Never called by ROBOT code; never during a live tick.
  world::World& setup_world() { return world_; }
  // Multi-rate cadences (sim seconds; 0 disables a task). Scan and record
 // are independent scheduler taps. The decision tap gates ROBOT ticks while
 // scan delivery remains independent, so stale observations exercise the
 // same freshness path as deployment. All timing stays wall-clock-free.
  void set_cadence(double scan_period_s, double decision_period_s,
                   double record_period_s);
 // Independent provider taps. The legacy set_cadence() keeps motion and IMU
 // at scan cadence; this overload lets sparse LiDAR coexist with faster
 // encoder/IMU observations without fabricating repeated measurements.
 void set_sensor_cadence(double scan_period_s, double motion_period_s,
 double imu_period_s, double decision_period_s,
 double record_period_s);
 // Objective list: place_ids resolved by the ROBOT map provider through
 // the operator seam. ROBOT completion uses its estimate; SIM acceptance
 // independently requires the physical body to arrive.
  void set_objectives(std::vector<std::string> place_ids,
                      double arrive_tol_m = 0.5);
  // Startup mode passthrough (SIM orchestration, pre-run): mapless
  // missions skip the global-retry cadence; SLAM still establishes the
  // frame through loop closures.
  void set_startup_mode(robot::autonomy::StartupMode mode) {
    app_.set_startup_mode(mode);
  }
  bool objectives_done() const { return driver_.done(); }
 bool objectives_passed() const { return driver_.passed(); }
  // Per-objective step budget (0 = none, legacy): a live goal older than
  // this is abandoned (recorded + canceled, next dispatched). Pre-run.
  void set_objective_step_budget(std::uint64_t steps) {
    driver_.set_step_budget(steps);
  }
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
 set_sensor_cadence(sim.scan_period_s, sim.motion_period_s,
 sim.imu_period_s, sim.decision_period_s, sim.record_period_s);
    set_judge_radius_m(sim.judge_radius_m);
    set_sensor_realism(sim.delivery, sim.imu_realism);
    suite_.lidar().set_beams(sim.lidar_beams);
    suite_.lidar().set_max_range(sim.lidar_max_range_m);
    suite_.lidar3d().set_rings(sim.lidar_rings);
    suite_.lidar3d().set_azimuths(sim.lidar_beams);
    suite_.lidar3d().set_max_range(sim.lidar_max_range_m);
    suite_.set_composite_scan(sim.lidar_composite);
  }
  // Survey re-window (SIM orchestration, pre-run): fits the static-map
  // raster to the authored world (plus margin_m) so far-field goals on
  // big maps get a route. Default 20x20 window is kept unless this runs —
  // every legacy mission behaves bit-identically without it.
  void fit_map_window_to_world(double margin_m = 4.0);
  const ObjectiveDriver& objectives() const { return driver_; }
  // Applied lidar mount for presentation (viewer ray origins, same source
  // the suite generates from — never a separately authored copy).
  void lidar_mount(double xyz_m[3], double rpy_rad[3]) const {
    suite_.lidar_mount(xyz_m, rpy_rad);
  }

 private:
  world::World world_;
  providers::SimSensorSuite suite_;
  // Declared BEFORE app_: the ctors capture the provider address in
  // app_'s initializer, and members init in declaration order — after
  // app_ the default nullptr would overwrite the captured address.
  providers::StaticMapProvider* map_provider_ = nullptr;
  robot::app::RobotApplication app_;
  // Owned by the app (moved at construction); the session keeps the raw
  // handle for SIM-side orchestration only (survey re-window, never tick).
  eval::EpisodeRecorder episode_;
  DeterministicScheduler scheduler_;
  ObjectiveDriver driver_;
  double sim_time_s_ = 0.0;
 robot::iface::MotionCommand last_cmd_ =
 robot::iface::MotionCommand::SafeStop(0);
  double judge_radius_m_ = 0.4;
  double terrain_clearance_m_ = 0.05;
};

}  // namespace lost_signal::sim::scenario
