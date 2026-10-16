#include "game/campaign_progress.hpp"
#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"
#include "algo/predictor.hpp"

#include <cmath>
#include <limits>
#include <algorithm>
#include <filesystem>
#include <iostream>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

int main() {
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 const auto build = catalog.presentation_for_level("lost_signal").recommended_build;

 {
 game::MissionSimulation manual(
 catalog, "boot", catalog.presentation_for_level("boot").recommended_build);
 manual.start();
 manual.set_manual_command(1.0, 0.0);
 for (int step = 0; step < 50; ++step)
 manual.step();
 CHECK(manual.result().path_length_m > 0.1);
 CHECK(manual.lookup_transform("map", "odom").has_value());
 CHECK(manual.lookup_transform("odom", "base_footprint").has_value());
 CHECK(manual.lookup_transform("base_footprint", "base_link").has_value());
 CHECK(manual.lookup_transform("base_link", "lidar_link").has_value());
 const auto base_from_imu = manual.lookup_transform("base_link", "imu_link");
 CHECK(base_from_imu.has_value());
 CHECK(std::abs(base_from_imu->translation_m.z - 0.24) < 1e-12);
 CHECK(manual.lookup_transform("base_link", "left_wheel_link").has_value());
 CHECK(std::abs(manual.frame_timestamp_s() - manual.snapshot().time_s) < 1e-9);
 }

 // A field expedition is a bounded-memory autonomous runtime mode. It keeps
 // stepping beyond the level timeout and never enters a debrief state.
 {
 game::MissionSimulation endless(
 catalog, "motor_trial", catalog.presentation_for_level("motor_trial").recommended_build);
 endless.set_endless_mode(true);
 endless.start();
 for (int step = 0; step < 3050; ++step)
 endless.step();
 CHECK(endless.snapshot().time_s > endless.level().objective.time_limit_s);
 CHECK(endless.snapshot().status == game::MissionStatus::Running);
 CHECK(endless.replay().size() < 2000);
 CHECK(endless.field_dispatch().active || endless.field_dispatch().completed_count > 0);
 if (endless.field_dispatch().active) {
 CHECK(endless.field_dispatch().distance_m > 0.0);
 CHECK(endless.field_dispatch().target.x >= 0.0);
 CHECK(endless.field_dispatch().target.x <= endless.map_definition().size_m.x);
 }
 CHECK(std::any_of(endless.events().begin(), endless.events().end(),
 [](const auto& event) { return event.code == "field-dispatch-opened"; }));
 }

 // Autonomous watch mode must close the product loop: an issued field job
 // creates a chassis-aware route and produces real Bullet motion without
 // keyboard input. The session remains open after a job completes.
 {
 game::MissionSimulation autonomous(
 catalog, "boot", catalog.presentation_for_level("boot").recommended_build);
 autonomous.set_endless_mode(true);
 autonomous.start();
 for (int step = 0; step < 9000 && autonomous.field_dispatch().completed_count == 0; ++step)
 autonomous.step();
 CHECK(autonomous.snapshot().status == game::MissionStatus::Running);
 CHECK(autonomous.result().path_length_m > 1.0);
 if (autonomous.field_dispatch().completed_count == 0) {
 std::cerr << "dispatch stalled: marker=" << autonomous.field_dispatch().marker_id
 << " distance=" << autonomous.field_dispatch().distance_m
 << " robot=(" << autonomous.snapshot().robot.x << ',' << autonomous.snapshot().robot.y
 << ") incident=" << autonomous.world_activity().incident.marker_id
 << " route_points=" << autonomous.planned_path().size() << '\n';
 }
 CHECK(autonomous.field_dispatch().completed_count >= 1);
 CHECK(!autonomous.planned_path().empty() || autonomous.field_dispatch().completed_count > 0);
 CHECK(std::any_of(autonomous.events().begin(), autonomous.events().end(),
 [](const auto& event) { return event.code == "field-dispatch-opened"; }));
 }

 for (const auto& level_id : catalog.campaign().levels) {
  const auto& level_presentation = catalog.presentation_for_level(level_id);
  game::MissionSimulation chapter_mission(catalog, level_id, level_presentation.recommended_build);
  chapter_mission.run_to_completion();
 if (chapter_mission.result().status != game::MissionStatus::Succeeded) {
 std::cerr << "recommended build failed: level=" << level_id
 << " reason=" << chapter_mission.result().failure_reason
 << " time=" << chapter_mission.result().time_s
 << " collisions=" << chapter_mission.result().collisions << '\n';
 }
  CHECK(chapter_mission.result().status == game::MissionStatus::Succeeded);
 }

 CHECK(sim::physics::bullet_physics_available());
 {
 const auto localization_physics = game::PhysicsMode::Bullet;
 game::MissionSimulation drifting(catalog, "localization",
 catalog.presentation_for_level("localization").default_build, std::nullopt, {},
 game::ComponentRegistry::builtins(), {}, localization_physics);
 drifting.run_to_completion();
 CHECK(drifting.result().status == game::MissionStatus::Succeeded);
 game::MissionSimulation localized(catalog, "localization",
 catalog.presentation_for_level("localization").recommended_build, std::nullopt, {},
 game::ComponentRegistry::builtins(), {}, localization_physics);
 localized.run_to_completion();
 CHECK(localized.result().status == game::MissionStatus::Succeeded);
 CHECK(localized.result().max_odometry_error_m < drifting.result().max_odometry_error_m);
 CHECK(!drifting.result().safe_score);
 CHECK(drifting.result().max_odometry_error_m > drifting.level().max_localization_error_m);
 CHECK(localized.result().safe_score);
 CHECK(localized.result().max_odometry_error_m < localized.level().max_localization_error_m);
 }

 // Seed override: same effective seed reproduces bit-identical time; the
 // override path with the JSON seed equals the default path.
 game::MissionSimulation first(catalog, "lost_signal", build);
 game::MissionSimulation first_repeat(catalog, "lost_signal", build, first.level().seed);
 CHECK(first_repeat.effective_seed() == first.effective_seed());
 CHECK(!first.planned_path().empty());
 first.run_to_completion();
 CHECK(first.result().status == game::MissionStatus::Succeeded);
 CHECK(first.result().via_reached);
 CHECK(first.result().collisions == 0);
 CHECK(first.result().safe_score);
 CHECK(first.result().min_dynamic_clearance_m >= first.level().safe_min_clearance_m);
 CHECK(first.result().astar_expanded > 0);
 CHECK(first.result().local_decisions > 0);
 first_repeat.run_to_completion();
 CHECK(first_repeat.result().time_s == first.result().time_s);
 CHECK(first.replay().size() > 10);
 const double replay_target_s = first.result().time_s * 0.5;
 first.seek_replay(replay_target_s);
 CHECK(first.snapshot().time_s >= replay_target_s);
 CHECK(first.snapshot().time_s <= replay_target_s + 0.051);
 first.seek_replay(first.result().time_s);
 CHECK(first.snapshot().status == game::MissionStatus::Succeeded);
 CHECK(std::any_of(first.events().begin(), first.events().end(),
                   [](const auto& event) { return event.code == "dynamic-detected"; }));
 CHECK(std::any_of(first.events().begin(), first.events().end(),
                   [](const auto& event) { return event.code == "waiting"; }));
 // Gate-3 shadow isolation: the observer assessed departures during the finale
 // (snapshot fields populated) and logged wait episodes (first/last/actual)
 // without changing physics — time_s bit-identical above is the proof.
 CHECK(std::any_of(first.replay().begin(), first.replay().end(),
                   [](const auto& snapshot) { return snapshot.control.shadow_assessed; }));
 // (FF shadow log removed with the stage-4 negative verdict: mapping +
 // sensitivity evidence stays parked in test_feedforward/test_ff_sensitivity.)
 CHECK(std::any_of(first.events().begin(), first.events().end(),
                   [](const auto& event) { return event.code == "shadow_wait"; }));
 // Gate-B spatial shadow: pocket sampling ran while waiting (read-only).
 {
  int sampled = 0;
  int feasible = 0;
  for (const auto& snapshot : first.replay()) {
   if (snapshot.control.shadow_pocket_total > 0) {
    ++sampled;
    feasible += snapshot.control.shadow_pocket_feasible;
   }
  }
  CHECK(sampled > 0);
  std::cerr << "spatial shadow: " << sampled << " waiting frames sampled, " << feasible
            << " feasible pocket-frames\n";
  // Reason bins integrity: every sampled frame accounts 3 leg reasons per
  // candidate, and the finale distribution is reported (Gate B data quality).
  int bins[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  for (const auto& snapshot : first.replay()) {
   if (snapshot.control.shadow_pocket_total > 0) {
    int sum = 0;
    for (const int bin : snapshot.control.shadow_pocket_reasons) {
     CHECK(bin >= 0);
     sum += bin;
    }
    CHECK(sum == 3 * snapshot.control.shadow_pocket_total);
    for (int r = 0; r < 9; ++r) {
     bins[r] += snapshot.control.shadow_pocket_reasons[r];
    }
   }
  }
  std::cerr << "pocket leg reasons:";
  for (int r = 0; r < 9; ++r) {
   std::cerr << " " << sim::depart_reason_name(static_cast<sim::DepartReason>(r)) << "=" << bins[r];
  }
  std::cerr << "\n";
 }
 // Shadow data integrity (reviewer step 1): every episode opened is closed by
 // mission end, structured records pair 1:1 with log events, timestamps and
 // reasons are in range. Distribution (reviewer step 2) is reported, not
 // averaged away: the finale's micro-waits must show as WaitUnsafe (grazing
 // geometry), never silently reclassified.
 {
  CHECK(!first.shadow_episode_open());
  const auto& episodes = first.shadow_episodes();
  std::size_t logged = 0;
  for (const auto& event : first.events()) {
   if (event.code == "shadow_wait") {
    ++logged;
   }
  }
  CHECK(logged == episodes.size());
  CHECK(!episodes.empty());
  int counts[6] = {0, 0, 0, 0, 0, 0};
  for (const auto& episode : episodes) {
   CHECK(episode.start_s <= episode.end_s);
   CHECK(episode.waited_s >= 0.0);
   CHECK(std::abs(episode.waited_s - (episode.end_s - episode.start_s)) < 1e-9);
   CHECK(episode.first_reason >= 0 && episode.first_reason <= 5);
   CHECK(episode.last_reason >= 0 && episode.last_reason <= 5);
   CHECK((episode.first_pred_s >= 0.0 || episode.first_pred_s == std::numeric_limits<double>::infinity()) &&
         (episode.last_pred_s >= 0.0 || episode.last_pred_s == std::numeric_limits<double>::infinity()));
   CHECK(std::isfinite(episode.entry_threat_speed_mps) && episode.entry_threat_speed_mps >= 0.0);
   CHECK(std::isfinite(episode.entry_nearest_m));
   ++counts[episode.first_reason];
   std::cerr << "shadow episode: start=" << episode.start_s << " first=" << episode.first_pred_s << "("
             << sim::depart_reason_name(static_cast<sim::DepartReason>(episode.first_reason))
             << ") last=" << episode.last_pred_s << "("
             << sim::depart_reason_name(static_cast<sim::DepartReason>(episode.last_reason))
             << ") waited=" << episode.waited_s << "\n";
  }
  CHECK(counts[static_cast<int>(sim::DepartReason::kWaitUnsafe)] > 0);
 }
 CHECK(std::any_of(first.replay().begin(), first.replay().end(),
                   [](const auto& snapshot) { return !snapshot.dynamic_obstacles.empty(); }));
 CHECK(std::any_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
  return !snapshot.dynamic_obstacles.empty() &&
         std::hypot(snapshot.dynamic_obstacles.front().estimated_velocity.x,
                    snapshot.dynamic_obstacles.front().estimated_velocity.y) > 0.2;
 }));
 CHECK(std::any_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
  return snapshot.control.dwa_active && !snapshot.control.dwa_selected.empty();
 }));
 CHECK(std::any_of(first.replay().begin(), first.replay().end(),
                   [](const auto& snapshot) { return std::abs(snapshot.control.heading_pid_p) > 0.01; }));
 CHECK(std::any_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
  return std::abs(snapshot.control.left_target_rpm) > 1.0 ||
         std::abs(snapshot.control.right_target_rpm) > 1.0;
 }));
 CHECK(std::any_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
  return std::abs(snapshot.control.left_pwm) > 0.01 || std::abs(snapshot.control.right_pwm) > 0.01;
 }));
 CHECK(std::all_of(first.replay().begin(), first.replay().end(), [](const auto& snapshot) {
  return std::abs(snapshot.control.left_pwm) <= 1.0 && std::abs(snapshot.control.right_pwm) <= 1.0;
 }));

 const auto progress_path = std::filesystem::temp_directory_path() / "lost-signal-progress-test.json";
 std::filesystem::remove(progress_path);
 game::CampaignProgress progress;
 progress.record_completion("lost_signal", first.result(), catalog);
 progress.save(progress_path, catalog);
 const auto loaded_progress = game::CampaignProgress::load(progress_path, catalog);
 CHECK(loaded_progress.completed_levels().size() == 1);
 CHECK(loaded_progress.completed_levels().front() == "lost_signal");
 CHECK(loaded_progress.best_scores().at("lost_signal").safe);
 std::filesystem::remove(progress_path);

 // DIY unlock chain: fresh saves start at the starting inventory, component
 // rewards merge idempotently, and last builds survive a save/load roundtrip.
 {
 game::CampaignProgress diy = game::CampaignProgress::load(
   std::filesystem::temp_directory_path() / "lost-signal-progress-missing.json", catalog);
 CHECK(diy.inventory().size() == 6);
 game::MissionSimulation boot_mission(catalog, "boot",
                                      catalog.presentation_for_level("boot").recommended_build);
 boot_mission.run_to_completion();
 CHECK(boot_mission.result().status == game::MissionStatus::Succeeded);
 diy.record_completion("boot", boot_mission.result(), catalog);
 CHECK(diy.inventory().size() == 8);
 diy.record_completion("boot", boot_mission.result(), catalog);
 CHECK(diy.inventory().size() == 8);
 CHECK(diy.last_build("boot") == nullptr);
 game::BuildSelection custom = boot_mission.build();
 custom.style = 0.2;
 diy.record_build("boot", custom);
 diy.save(progress_path, catalog);
 const auto diy_loaded = game::CampaignProgress::load(progress_path, catalog);
 CHECK(diy_loaded.inventory().size() == 8);
 CHECK(diy_loaded.last_build("boot") != nullptr);
 CHECK(std::abs(diy_loaded.last_build("boot")->style - 0.2) < 1e-12);
 CHECK(diy_loaded.last_build("lost_signal") == nullptr);
 std::filesystem::remove(progress_path);
 }

 // The aggressive slow-decision default is now a visible counterexample:
 // rolling LiDAR plus a short horizon cannot resolve the crossing in time.
 // The recommended fast-decision build above succeeds with safe clearance.
 {
 game::MissionSimulation default_attempt(catalog, "lost_signal",
                                         catalog.presentation_for_level("lost_signal").default_build);
 default_attempt.run_to_completion();
 CHECK(default_attempt.result().status == game::MissionStatus::Failed);
 CHECK(default_attempt.result().failure_reason == "collision-dynamic");
 CHECK(!default_attempt.result().safe_score);
 CHECK(default_attempt.result().min_dynamic_clearance_m < catalog.level("lost_signal").safe_min_clearance_m);
 // More motor cannot compensate for a slow sensor and short prediction
 // horizon. The actor is observed, but the aggressive build reacts too late.
 game::BuildSelection hot_build;
 hot_build.parts["tires"] = "tires.eco";
 hot_build.parts["sensor"] = "sensor.long_slow_lidar";
 hot_build.parts["computer"] = "computer.light";
 hot_build.parts["motor"] = "motor.torque";
 hot_build.parts["imu"] = "imu.basic";
 hot_build.style = 1.0;
 game::MissionSimulation hot_attempt(catalog, "lost_signal", hot_build);
 hot_attempt.run_to_completion();
 CHECK(hot_attempt.result().status == game::MissionStatus::Failed);
 CHECK(hot_attempt.result().failure_reason == "collision-dynamic");
 bool actor_detected = false;
 for (const auto& event : hot_attempt.events()) {
  if (event.code == "dynamic-detected") {
   actor_detected = true;
  }
 }
 CHECK(actor_detected);
 CHECK(hot_attempt.result().collisions == 1);
 }
 // Traction saturation is live: a hot motor on eco tires spins through the
 // traction limit (27 episodes here). Official builds rarely saturate, which
 // is why their odometry comes from the slip-scale random walk instead.
 {
 game::BuildSelection spin_build;
 spin_build.parts["tires"] = "tires.eco";
 spin_build.parts["sensor"] = "sensor.long_slow_lidar";
 spin_build.parts["computer"] = "computer.light";
 spin_build.parts["motor"] = "motor.torque";
 spin_build.parts["imu"] = "imu.basic";
 spin_build.style = 1.0;
 game::MissionSimulation spin_run(catalog, "blocked_route", spin_build);
 spin_run.run_to_completion();
 CHECK(spin_run.result().slip_events > 0);
 CHECK(spin_run.result().status == game::MissionStatus::Succeeded);
 game::MissionSimulation spin_repeat(catalog, "blocked_route", spin_build);
 spin_repeat.run_to_completion();
 CHECK(std::abs(spin_repeat.result().time_s - spin_run.result().time_s) < 1e-12);
 }

 {
 game::BuildSelection mid_style = build;
 mid_style.style = 0.37;
 game::MissionSimulation mid_first(catalog, "lost_signal", mid_style);
 mid_first.run_to_completion();
 CHECK(mid_first.result().status == game::MissionStatus::Succeeded);
 game::MissionSimulation mid_second(catalog, "lost_signal", mid_style);
 mid_second.run_to_completion();
 CHECK(std::abs(mid_second.result().time_s - mid_first.result().time_s) < 1e-12);
 }

 {
 game::MissionSimulation second(catalog, "lost_signal", build);
 second.run_to_completion();
 CHECK(second.result().status == first.result().status);
 CHECK(std::abs(second.result().time_s - first.result().time_s) < 1e-12);
 CHECK(std::abs(second.result().path_length_m - first.result().path_length_m) < 1e-12);
 CHECK(second.events().size() == first.events().size());
 }

 // Ackermann content car: per-presentation robot, no spin capability and
 // live steering-servo telemetry for the recommended build.
 {
  const auto& presentation = catalog.presentation_for_level("return_route");
  CHECK(presentation.robot_id == "robot.ro2");
  game::MissionSimulation debut(catalog, "return_route", presentation.recommended_build);
  CHECK(debut.robot().drive_type == "ackermann");
  CHECK(!debut.robot().id.empty());
  debut.run_to_completion();
  CHECK(debut.result().status == game::MissionStatus::Succeeded);
  CHECK(debut.result().safe_score);
  CHECK(std::any_of(debut.replay().begin(), debut.replay().end(),
                    [](const auto& snapshot) { return snapshot.control.steer_angle_rad != 0.0; }));
 CHECK(std::any_of(debut.replay().begin(), debut.replay().end(), [](const auto& snapshot) {
 return snapshot.control.mount_actuations.size() == 2 &&
 snapshot.control.mount_actuations[0].mount_id == "front_left" &&
 snapshot.control.mount_actuations[1].mount_id == "front_right" &&
 (std::abs(snapshot.control.mount_actuations[0].yaw_rad) > 1e-6 ||
 std::abs(snapshot.control.mount_actuations[1].yaw_rad) > 1e-6);
 }));
  game::MissionSimulation debut_default(catalog, "return_route", presentation.default_build);
  debut_default.run_to_completion();
  CHECK(debut_default.result().status == game::MissionStatus::Succeeded);
  CHECK(debut_default.result().fast_score);
 }
 // Tracked content car: twin-track layout, spin capability, scrub-limited
 // turns. The recommended build sweeps all three badges; the aggressive
 // low-accuracy build cuts the inside of the staged return and hits a shelf.
 {
  const auto& presentation = catalog.presentation_for_level("track_turn");
  CHECK(presentation.robot_id == "robot.ro3");
  game::MissionSimulation slide(catalog, "track_turn", presentation.recommended_build);
  CHECK(slide.robot().drive_type == "tracked");
  slide.run_to_completion();
  CHECK(slide.result().status == game::MissionStatus::Succeeded);
  CHECK(slide.result().safe_score);
  CHECK(slide.result().fast_score);
  CHECK(slide.result().efficient_score);
  game::MissionSimulation slide_default(catalog, "track_turn", presentation.default_build);
  slide_default.run_to_completion();
 CHECK(slide_default.result().status == game::MissionStatus::Failed);
 CHECK(slide_default.result().failure_reason == "collision-static");
 }
 // Loose ground: sand/gravel materials (library + per-level gate) decide the
 // race through odometry. Eco/swift loses safe on drift, sand/careful takes
 // it; speed and thrift go the other way.
 {
  const auto& presentation = catalog.presentation_for_level("loose_ground");
  game::MissionSimulation grit(catalog, "loose_ground", presentation.recommended_build);
  grit.run_to_completion();
  CHECK(grit.result().status == game::MissionStatus::Succeeded);
  CHECK(grit.result().safe_score);
  game::MissionSimulation grit_default(catalog, "loose_ground", presentation.default_build);
  grit_default.run_to_completion();
  CHECK(grit_default.result().status == game::MissionStatus::Succeeded);
  CHECK(!grit_default.result().safe_score);
  CHECK(grit_default.result().fast_score);
 }
 // Tracked detour: rushing tracks gets lost into a wall (robust crash-teaching
 // case, 5/5 over seeds: scrub drift 17 m), careful tracks arrive. This closes
 // the open need for an honest failure level left by the feedforward session.
 {
  const auto& presentation = catalog.presentation_for_level("scrub_climb");
  CHECK(presentation.robot_id == "robot.ro3");
  game::MissionSimulation climb(catalog, "scrub_climb", presentation.recommended_build);
  climb.run_to_completion();
  CHECK(climb.result().status == game::MissionStatus::Succeeded);
  CHECK(climb.result().safe_score);
  CHECK(climb.result().fast_score);
  CHECK(climb.result().efficient_score);
  game::MissionSimulation climb_default(catalog, "scrub_climb", presentation.default_build);
  climb_default.run_to_completion();
  CHECK(climb_default.result().status == game::MissionStatus::Failed);
 CHECK(climb_default.result().failure_reason == "collision-static");
 }
 // Sand plus tracks: both builds can finish, but compound slip makes the
 // default slow, inefficient and unsafe while the careful build sweeps.
 {
  const auto& presentation = catalog.presentation_for_level("sand_track");
  CHECK(presentation.robot_id == "robot.ro3");
  game::MissionSimulation grit_track(catalog, "sand_track", presentation.recommended_build);
  grit_track.run_to_completion();
  CHECK(grit_track.result().status == game::MissionStatus::Succeeded);
  CHECK(grit_track.result().safe_score);
  CHECK(grit_track.result().fast_score);
  CHECK(grit_track.result().efficient_score);
  game::MissionSimulation grit_track_default(catalog, "sand_track", presentation.default_build);
  grit_track_default.run_to_completion();
 CHECK(grit_track_default.result().status == game::MissionStatus::Succeeded);
 CHECK(!grit_track_default.result().safe_score);
 CHECK(!grit_track_default.result().fast_score);
 CHECK(!grit_track_default.result().efficient_score);
 }
 // Sand plus steering: Hybrid A* and the grip build complete safely; the
 // low-grip aggressive build cuts into fixed geometry.
 {
  const auto& presentation = catalog.presentation_for_level("sand_steer");
  CHECK(presentation.robot_id == "robot.ro2");
  game::MissionSimulation sandy(catalog, "sand_steer", presentation.recommended_build);
  sandy.run_to_completion();
  CHECK(sandy.result().status == game::MissionStatus::Succeeded);
  CHECK(sandy.result().safe_score);
  game::MissionSimulation sandy_default(catalog, "sand_steer", presentation.default_build);
  sandy_default.run_to_completion();
 CHECK(sandy_default.result().status == game::MissionStatus::Failed);
 CHECK(sandy_default.result().failure_reason == "collision-static");
 }
 // Wet plus tracks: the careful build sweeps; the low-grip default accumulates
 // enough dead-reckoning error to hit a fixed obstacle.
 {
  const auto& presentation = catalog.presentation_for_level("wet_scrub");
  CHECK(presentation.robot_id == "robot.ro3");
  game::MissionSimulation scrub_wet(catalog, "wet_scrub", presentation.recommended_build);
  scrub_wet.run_to_completion();
  CHECK(scrub_wet.result().status == game::MissionStatus::Succeeded);
  CHECK(scrub_wet.result().safe_score);
  CHECK(scrub_wet.result().fast_score);
  CHECK(scrub_wet.result().efficient_score);
  game::MissionSimulation scrub_wet_default(catalog, "wet_scrub", presentation.default_build);
  scrub_wet_default.run_to_completion();
  CHECK(scrub_wet_default.result().status == game::MissionStatus::Failed);
 CHECK(scrub_wet_default.result().failure_reason == "timeout");
 }
 // Wet descent is a stopping task, not a drive-through waypoint: the grip
 // build settles inside the unload zone while the low-grip build slides past
 // it and reaches the boundary.
 {
 const auto& presentation = catalog.presentation_for_level("rain_descent");
 game::MissionSimulation careful(catalog, "rain_descent", presentation.recommended_build,
 std::nullopt, {}, game::ComponentRegistry::builtins(), {}, game::PhysicsMode::Bullet);
 careful.run_to_completion();
 CHECK(careful.result().status == game::MissionStatus::Succeeded);
 game::MissionSimulation fast(catalog, "rain_descent", presentation.default_build,
 std::nullopt, {}, game::ComponentRegistry::builtins(), {}, game::PhysicsMode::Bullet);
 fast.run_to_completion();
 CHECK(fast.result().status == game::MissionStatus::Failed);
 CHECK(fast.result().failure_reason == "collision-static");
 CHECK(fast.result().slip_events > 0);
 }
 // Cross-slope cargo is a Bullet-authored physical failure: both builds can
 // plan the lane, then the fast standard-tire build rolls after several
 // visible seconds while the careful grip build retains margin.
 {
  const auto& presentation = catalog.presentation_for_level("cross_slope");
 game::MissionSimulation careful(catalog, "cross_slope", presentation.recommended_build,
 std::nullopt, {}, game::ComponentRegistry::builtins(), {}, game::PhysicsMode::Bullet);
  careful.run_to_completion();
  CHECK(careful.result().status == game::MissionStatus::Succeeded);
  CHECK(careful.result().max_rollover_risk < 0.85);
 game::MissionSimulation fast(catalog, "cross_slope", presentation.default_build,
 std::nullopt, {}, game::ComponentRegistry::builtins(), {}, game::PhysicsMode::Bullet);
  fast.run_to_completion();
  CHECK(fast.result().status == game::MissionStatus::Failed);
 CHECK(fast.result().failure_reason == "rollover");
 CHECK(fast.result().time_s >= 5.0);
 CHECK(fast.result().time_s <= 10.0);
 }
 // Continuous field sessions keep stepping beyond authored mission limits and
 // share one deterministic environment snapshot across weather, incidents,
 // dispatch, physics and replay.
 {
 const auto& field_presentation = catalog.presentation_for_level("boot");
 game::MissionSimulation field(catalog, "boot", field_presentation.recommended_build);
 field.set_endless_mode(true);
 field.start();
 for (int step = 0; step < 3000; ++step)
 field.step();
 CHECK(field.snapshot().status == game::MissionStatus::Running);
 CHECK(field.snapshot().time_s > 59.0);
 CHECK(field.world_activity().weather_revision >= 1);
 CHECK(field.world_activity().incident.active);
 CHECK(field.field_dispatch().active);
 CHECK(std::any_of(field.events().begin(), field.events().end(),
 [](const auto& event) { return event.code == "incident-opened"; }));
 }
 return 0;
}
