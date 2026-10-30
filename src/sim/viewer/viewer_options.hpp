#pragma once

#include <string>

#include "sim/viewer/scene_renderer.hpp"

namespace lost_signal::sim::viewer {

inline constexpr double kDefaultSandboxHistorySeconds = 120.0;

struct ViewerOptions {
  std::string robot_path;
  std::string sim_path;
  std::string map_path;
  std::string assets_path;
  std::string scene_path;
  std::string experience_path;
  std::string experiment_output_path;
  std::string chapter_id;
  std::string profile_path;
  std::string requested_locale = "auto";

  double initial_playback_rate = 1.0;
  double sandbox_history_seconds = kDefaultSandboxHistorySeconds;
  bool initial_turbo = false;
  bool initial_maximized = false;
  render::RenderQuality render_quality = render::RenderQuality::kHigh;
  bool sandbox_requested = false;
  bool play_requested = false;
  bool source_requested = false;
  bool mute_audio = false;
  bool initial_show_lidar = false;
  bool initial_show_robot_evidence = false;
  bool list_worlds = false;
  bool list_experiences = false;
  bool catalog_requested = false;
  bool initial_physics_overlay = false;
  bool view3d = false;
  int shot_frames = -1;
  std::string shot_path;
  float camera_dist = 0.0f;
  float camera_yaw = 0.0f;
  float camera_pitch = 0.0f;
  bool camera_angle_set = false;
  bool camera_target_set = false;
  double camera_target_x = 0.0;
  double camera_target_y = 0.0;
  bool help_requested = false;
};

bool parse_viewer_options(int argc, char** argv, ViewerOptions& options,
                          std::string* error = nullptr);
void print_viewer_usage();

}  // namespace lost_signal::sim::viewer
