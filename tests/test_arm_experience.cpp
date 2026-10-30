#include <array>
#include <cassert>
#include <string>

#include "sim/experiment/experience_runtime.hpp"
#include "sim/viewer/experience_manifest.hpp"

int main() {
  using namespace lost_signal;
  sim::viewer::ExperienceManifest manifest;
  std::string error;
  assert(sim::viewer::load_experience_manifest(
      std::string(TEST_SOURCE_DIR) + "/configs/experiences/arm_lab.json",
      manifest, &error));
  assert(manifest.robot_path.find("ro6_arm.json") != std::string::npos);

  const auto config = sim::experiment::experience_runtime_config(manifest);
  sim::experiment::ExperienceRuntime runtime;
  assert(sim::experiment::build_experience_runtime(config, runtime, &error));
  assert(runtime.robot_config.has_value());
  assert(runtime.robot_config->profile.arm.enabled);
  assert(runtime.session != nullptr);
  assert(runtime.session->setup_world().arm_count() == 4);

  // Exercise the same operator effort seam used by the Viewer.  The command
  // enters ROBOT validation/watchdog first; it is not a direct World write.
  const std::array<double, 4> effort{0.45, -0.25, 0.20, -0.15};
  assert(runtime.session->set_operator_arm_effort(4, effort));
  const double initial = runtime.session->setup_world().arm_angle(0);
  for (int step = 0; step < 80; ++step) runtime.session->step_once();
  runtime.session->clear_operator_arm_effort();
  for (int step = 0; step < 80; ++step) runtime.session->step_once();
  assert(runtime.session->setup_world().arm_angle(0) != initial);

  for (int step = 0; step < 1400 && !runtime.session->objectives_done();
       ++step) {
    runtime.session->step_once();
  }
  assert(runtime.session->objectives_done());
  assert(runtime.session->objectives_passed());
  const auto score = runtime.session->score();
  assert(score.collision_steps == 0);
  assert(!runtime.session->episode().steps().empty());
  const auto& presentation =
      runtime.session->episode().steps().back().presentation;
  assert(presentation.arm_links.size() == 4);
  for (const auto& link : presentation.arm_links) {
    assert(link.valid);
    assert(link.pose.valid);
    assert(link.joint.valid);
  }
  return 0;
}
