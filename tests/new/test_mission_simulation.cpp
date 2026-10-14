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

 game::MissionSimulation manual(catalog, "boot", catalog.presentation_for_level("boot").recommended_build);
 manual.start();
 manual.set_manual_command(1.0, 0.0);
 for (int step = 0; step < 50; ++step)
  manual.step();
 CHECK(manual.result().path_length_m > 0.1);

 for (const auto& level_id : catalog.campaign().levels) {
  const auto& level_presentation = catalog.presentation_for_level(level_id);
  game::MissionSimulation chapter_mission(catalog, level_id, level_presentation.recommended_build);
  chapter_mission.run_to_completion();
  CHECK(chapter_mission.result().status == game::MissionStatus::Succeeded);
 }

 game::MissionSimulation drifting(catalog, "localization",
                                  catalog.presentation_for_level("localization").default_build);
 drifting.run_to_completion();
 CHECK(drifting.result().status == game::MissionStatus::Succeeded);
 CHECK(!drifting.result().safe_score);

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
   CHECK((episode.first_pred_s >= 0.0 ||
          episode.first_pred_s == std::numeric_limits<double>::infinity()) &&
         (episode.last_pred_s >= 0.0 ||
          episode.last_pred_s == std::numeric_limits<double>::infinity()));
   CHECK(std::isfinite(episode.entry_threat_speed_mps) && episode.entry_threat_speed_mps >= 0.0);
   CHECK(std::isfinite(episode.entry_nearest_m));
   ++counts[episode.first_reason];
   std::cerr << "shadow episode: start=" << episode.start_s << " first=" << episode.first_pred_s
             << "(" << sim::depart_reason_name(static_cast<sim::DepartReason>(episode.first_reason))
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

 // Official default: survives by 2.5 cm (near miss), earns no safety score.
 // The contrast with recommended (0.62 m) is structural: DWA margins 0.4 vs
 // 0.95 plus the bigger honest robot radius.
 game::MissionSimulation default_attempt(catalog, "lost_signal",
                                         catalog.presentation_for_level("lost_signal").default_build);
 default_attempt.run_to_completion();
 CHECK(default_attempt.result().status == game::MissionStatus::Succeeded);
 CHECK(!default_attempt.result().safe_score);
 CHECK(default_attempt.result().min_clearance_m < catalog.level("lost_signal").safe_min_clearance_m);
 // Hot motor on a slow brain still crashes: the dynamic-collision path stays
 // covered, and the motor/computer synergy is pinned, not just documented.
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
 CHECK(hot_attempt.result().collisions == 1);
 // Traction saturation is live: a hot motor on eco tires spins through the
 // traction limit (27 episodes here). Official builds rarely saturate, which
 // is why their odometry comes from the slip-scale random walk instead.
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

 game::BuildSelection mid_style = build;
 mid_style.style = 0.37;
 game::MissionSimulation mid_first(catalog, "lost_signal", mid_style);
 mid_first.run_to_completion();
 CHECK(mid_first.result().status == game::MissionStatus::Succeeded);
 game::MissionSimulation mid_second(catalog, "lost_signal", mid_style);
 mid_second.run_to_completion();
 CHECK(std::abs(mid_second.result().time_s - mid_first.result().time_s) < 1e-12);

 game::MissionSimulation second(catalog, "lost_signal", build);
 second.run_to_completion();
 CHECK(second.result().status == first.result().status);
 CHECK(std::abs(second.result().time_s - first.result().time_s) < 1e-12);
 CHECK(std::abs(second.result().path_length_m - first.result().path_length_m) < 1e-12);
 CHECK(second.events().size() == first.events().size());

 // Ackermann content car: per-presentation robot, no spin capability, servo
 // telemetry on the ackermann side, recommended sweep with a safe contrast.
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
  game::MissionSimulation debut_default(catalog, "return_route", presentation.default_build);
  debut_default.run_to_completion();
  CHECK(debut_default.result().status == game::MissionStatus::Succeeded);
  CHECK(!debut_default.result().safe_score);
 }
 // Tracked content car: twin-track layout, spin capability, scrub-limited
 // turns. The default build drifts wide (odometry/time/energy all lost);
 // the recommended build sweeps. Default success also covers the docking
 // controller: waypoints exhaust before arrival tolerance, the robot used to
 // park short until timeout.
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
  CHECK(slide_default.result().status == game::MissionStatus::Succeeded);
  CHECK(!slide_default.result().safe_score);
  CHECK(!slide_default.result().fast_score);
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
 // Sand plus tracks: compound slip (5/5 over seeds both sides). Default gets
 // lost into timeout, recommended arrives; odometry lies catastrophically
 // for both, so safe grades contact only here.
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
  CHECK(grit_track_default.result().status != game::MissionStatus::Succeeded);
 }
 // Sand plus steering: grit separates (5/5 over seeds), wide arcs cost time.
 {
  const auto& presentation = catalog.presentation_for_level("sand_steer");
  CHECK(presentation.robot_id == "robot.ro2");
  game::MissionSimulation sandy(catalog, "sand_steer", presentation.recommended_build);
  sandy.run_to_completion();
  CHECK(sandy.result().status == game::MissionStatus::Succeeded);
  CHECK(sandy.result().safe_score);
  game::MissionSimulation sandy_default(catalog, "sand_steer", presentation.default_build);
  sandy_default.run_to_completion();
  CHECK(sandy_default.result().status == game::MissionStatus::Succeeded);
  CHECK(!sandy_default.result().safe_score);
 }
 // Wet plus tracks: short dry approach, drunk but arriving (deterministic
 // systematic bias both sides). Recommended sweeps, default loses everything.
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
  CHECK(scrub_wet_default.result().status == game::MissionStatus::Succeeded);
  CHECK(!scrub_wet_default.result().safe_score);
 }
 return 0;
}
