#include <cassert>
#include <cmath>
#include <cstdio>
#include <string>

#include "sim/experiment/experience_runtime.hpp"
#include "sim/viewer/experience_manifest.hpp"

int main() {
  using namespace lost_signal;
  sim::viewer::ExperienceManifest experience;
  std::string error;
  assert(sim::viewer::load_experience_manifest(
      std::string(TEST_SOURCE_DIR) +
          "/configs/experiences/dynamic_safety_lab.json",
      experience, &error));

  auto config = sim::experiment::experience_runtime_config(
      experience, std::string(TEST_SOURCE_DIR) + "/assets/models/manifest.json");
  sim::experiment::ExperienceRuntime runtime;
  assert(sim::experiment::build_experience_runtime(config, runtime, &error));
  assert(runtime.session != nullptr);
  assert(runtime.session->setup_world().actors().size() == 1);

  bool saw_anonymous_track = false;
  bool saw_predictive_response = false;
  bool resumed_after_response = false;
  for (std::uint64_t step = 0;
       step < experience.time_limit_steps && !runtime.session->objectives_done();
       ++step) {
    runtime.session->step_once();
    const auto* sample = runtime.session->last_eval_step();
    if (sample == nullptr) continue;
    if (!sample->diag.dynamic_tracks.empty()) {
      saw_anonymous_track = true;
      // The ROBOT diagnostic carries anonymous track labels only.  The SIM
      // actor id and authored route remain in the scenario/presentation side.
      for (const auto& track : sample->diag.dynamic_tracks) {
        assert(track.id != 0);
      }
    }
    const auto& evidence = sample->diag.evidence;
    const bool predictive_response =
        evidence.constraint_code == "dynamic_escape" ||
        evidence.constraint_code == "dynamic_clearance" ||
        evidence.reason_code == "route-conflict-hold" ||
        evidence.reason_code == "route-conflict-speed-cap" ||
        evidence.reason_code == "route-conflict-no-safe-hold";
    saw_predictive_response |= predictive_response;
    resumed_after_response |=
        saw_predictive_response && !predictive_response &&
        sample->cmd.forward_mps > 0.2;
  }

  const auto score = runtime.session->score();
  if (!runtime.session->objectives_done() ||
      !runtime.session->objectives_passed() || score.collision_steps != 0 ||
      !saw_anonymous_track || !saw_predictive_response ||
      !resumed_after_response) {
    const auto* last = runtime.session->last_eval_step();
    std::fprintf(stderr,
                 "dynamic safety failed: done=%d passed=%d collisions=%zu "
                 "track=%d response=%d resumed=%d truth=(%.3f,%.3f)\n",
                 runtime.session->objectives_done(),
                 runtime.session->objectives_passed(), score.collision_steps,
                 saw_anonymous_track, saw_predictive_response,
                 resumed_after_response,
                 last != nullptr ? last->truth_after.body_x_m : 0.0,
                 last != nullptr ? last->truth_after.body_y_m : 0.0);
  }
  assert(runtime.session->objectives_done());
  assert(runtime.session->objectives_passed());
  assert(score.collision_steps == 0);
  assert(saw_anonymous_track);
  assert(saw_predictive_response);
  assert(resumed_after_response);
  return 0;
}
