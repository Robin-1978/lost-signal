#include <cassert>
#include <iterator>
#include <string>

#include "sim/viewer/viewer_options.hpp"

int main() {
  {
    const char* argv[] = {"sim_viewer", "--experience", "chapter.json",
                          "--3d", "--rate", "8", "--quality", "fast",
                          "--camera-target", "1.25", "-2.5", "--shot",
                          "4", "frame.png", "--physics-overlay"};
    lost_signal::sim::viewer::ViewerOptions options;
    std::string error;
    assert(lost_signal::sim::viewer::parse_viewer_options(
        static_cast<int>(std::size(argv)), const_cast<char**>(argv), options,
        &error));
    assert(error.empty());
    assert(options.experience_path == "chapter.json");
    assert(options.view3d);
    assert(options.render_quality ==
           lost_signal::sim::viewer::render::RenderQuality::kFast);
    assert(options.initial_playback_rate == 8.0);
    assert(options.camera_target_set);
    assert(options.camera_target_x == 1.25);
    assert(options.camera_target_y == -2.5);
    assert(options.shot_frames == 4);
    assert(options.shot_path == "frame.png");
    assert(options.initial_physics_overlay);
  }
  {
    const char* argv[] = {"sim_viewer", "--rate", "64.1"};
    lost_signal::sim::viewer::ViewerOptions options;
    std::string error;
    assert(!lost_signal::sim::viewer::parse_viewer_options(
        static_cast<int>(std::size(argv)), const_cast<char**>(argv), options,
        &error));
    assert(error.find("--rate") != std::string::npos);
  }
  {
    const char* argv[] = {"sim_viewer", "--map"};
    lost_signal::sim::viewer::ViewerOptions options;
    std::string error;
    assert(!lost_signal::sim::viewer::parse_viewer_options(
        static_cast<int>(std::size(argv)), const_cast<char**>(argv), options,
        &error));
    assert(error.find("missing value") != std::string::npos);
  }
  return 0;
}
