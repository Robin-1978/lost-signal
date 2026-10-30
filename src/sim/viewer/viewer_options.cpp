#include "sim/viewer/viewer_options.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>

namespace lost_signal::sim::viewer {
namespace {

bool next_value(int& index, int argc, char** argv, std::string& value,
               const char* option, std::string* error) {
  if (index + 1 >= argc) {
    if (error != nullptr) *error = std::string("missing value for ") + option;
    return false;
  }
  value = argv[++index];
  return true;
}

bool parse_double(const std::string& value, double& output, double minimum,
                  double maximum, const char* option, std::string* error) {
  char* end = nullptr;
  const double parsed = std::strtod(value.c_str(), &end);
  if (end == value.c_str() || *end != '\0' || !std::isfinite(parsed) ||
      parsed < minimum || parsed > maximum) {
    if (error != nullptr) {
      *error = std::string("bad ") + option + ": expected a value in [" +
               std::to_string(minimum) + ", " + std::to_string(maximum) +
               "]";
    }
    return false;
  }
  output = parsed;
  return true;
}

bool parse_float(const std::string& value, float& output, float minimum,
                 float maximum, const char* option, std::string* error) {
  double parsed = 0.0;
  if (!parse_double(value, parsed, minimum, maximum, option, error)) return false;
  output = static_cast<float>(parsed);
  return true;
}

}  // namespace

void print_viewer_usage() {
  std::puts(
      "usage: sim_viewer [--play | --sandbox | --experience FILE]\n"
      " [--robot FILE] [--sim FILE] [--map FILE] [--scene FILE]\n"
      " [--rate 0.05..64 | --turbo] [--quality high|fast] [--3d]\n"
      " [--lidar] [--robot-evidence] [--shot N FILE]\n"
      " [--physics-overlay]\n"
      " [--maximized]\n"
      " [--locale auto|en|zh-CN] [--mute] [--history-seconds N]\n"
      " [--experiment-out REPORT.json]\n"
      " [--profile FILE]\n"
      " [--chapter ID | --catalog | --list-worlds | --list-experiences]\n"
      " [--camera-dist M] [--camera-target X Y] [--camera-yaw DEG]"
      " [--camera-pitch DEG]\n"
      "\n"
      "No arguments starts the First Contact field lab. --sandbox keeps the\n"
      "open engineering demo. During a run: Space pause, N step, F follow,\n"
      "Q quality, +/- rate, T turbo, F10 maximize, F11 fullscreen, V 2D/3D,\n"
      "right-drag orbit, wheel zoom. Sandbox: H manual, WASD/arrows drive.");
}

bool parse_viewer_options(int argc, char** argv, ViewerOptions& options,
                          std::string* error) {
  if (error != nullptr) error->clear();
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--help" || argument == "-h") {
      options.help_requested = true;
      continue;
    }
    if (argument == "--play") {
      options.play_requested = true;
    } else if (argument == "--sandbox") {
      options.sandbox_requested = true;
    } else if (argument == "--mute") {
      options.mute_audio = true;
    } else if (argument == "--lidar") {
      options.initial_show_lidar = true;
    } else if (argument == "--robot-evidence") {
      options.initial_show_robot_evidence = true;
    } else if (argument == "--physics-overlay") {
      options.initial_physics_overlay = true;
    } else if (argument == "--list-worlds") {
      options.list_worlds = true;
    } else if (argument == "--list-experiences") {
      options.list_experiences = true;
    } else if (argument == "--catalog") {
      options.catalog_requested = true;
      options.list_worlds = true;
      options.list_experiences = true;
    } else if (argument == "--turbo") {
      options.initial_turbo = true;
    } else if (argument == "--maximized") {
      options.initial_maximized = true;
    } else if (argument == "--3d") {
      options.view3d = true;
    } else if (argument == "--chapter") {
      if (!next_value(index, argc, argv, options.chapter_id, "--chapter", error)) return false;
      options.source_requested = true;
    } else if (argument == "--robot") {
      if (!next_value(index, argc, argv, options.robot_path, "--robot", error)) return false;
      options.source_requested = true;
    } else if (argument == "--sim") {
      if (!next_value(index, argc, argv, options.sim_path, "--sim", error)) return false;
      options.source_requested = true;
    } else if (argument == "--map") {
      if (!next_value(index, argc, argv, options.map_path, "--map", error)) return false;
      options.source_requested = true;
    } else if (argument == "--assets") {
      if (!next_value(index, argc, argv, options.assets_path, "--assets", error)) return false;
    } else if (argument == "--scene") {
      if (!next_value(index, argc, argv, options.scene_path, "--scene", error)) return false;
      options.source_requested = true;
    } else if (argument == "--experience") {
      if (!next_value(index, argc, argv, options.experience_path, "--experience", error)) return false;
      options.source_requested = true;
    } else if (argument == "--experiment-out") {
      if (!next_value(index, argc, argv, options.experiment_output_path,
                     "--experiment-out", error)) return false;
    } else if (argument == "--profile") {
      if (!next_value(index, argc, argv, options.profile_path, "--profile", error)) return false;
    } else if (argument == "--locale") {
      if (!next_value(index, argc, argv, options.requested_locale, "--locale", error)) return false;
    } else if (argument == "--history-seconds") {
      std::string value;
      if (!next_value(index, argc, argv, value, "--history-seconds", error) ||
          !parse_double(value, options.sandbox_history_seconds, 0.0,
                        std::numeric_limits<double>::max(), "--history-seconds", error)) return false;
    } else if (argument == "--quality") {
      std::string value;
      if (!next_value(index, argc, argv, value, "--quality", error)) return false;
      if (value == "fast") options.render_quality = render::RenderQuality::kFast;
      else if (value == "high") options.render_quality = render::RenderQuality::kHigh;
      else {
        if (error != nullptr) *error = "bad --quality: expected fast or high";
        return false;
      }
    } else if (argument == "--rate") {
      std::string value;
      if (!next_value(index, argc, argv, value, "--rate", error) ||
          !parse_double(value, options.initial_playback_rate, 0.05, 64.0,
                        "--rate", error)) return false;
    } else if (argument == "--camera-dist") {
      std::string value;
      if (!next_value(index, argc, argv, value, "--camera-dist", error) ||
          !parse_float(value, options.camera_dist, 0.6f, 60.0f,
                       "--camera-dist", error)) return false;
    } else if (argument == "--camera-target") {
      std::string x, y;
      if (!next_value(index, argc, argv, x, "--camera-target", error) ||
          !next_value(index, argc, argv, y, "--camera-target", error) ||
          !parse_double(x, options.camera_target_x,
                        -std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max(), "--camera-target", error) ||
          !parse_double(y, options.camera_target_y,
                        -std::numeric_limits<double>::max(),
                        std::numeric_limits<double>::max(), "--camera-target", error)) return false;
      options.camera_target_set = true;
    } else if (argument == "--camera-yaw") {
      std::string value;
      if (!next_value(index, argc, argv, value, "--camera-yaw", error) ||
          !parse_float(value, options.camera_yaw, -360.0f, 360.0f,
                       "--camera-yaw", error)) return false;
      options.camera_angle_set = true;
    } else if (argument == "--camera-pitch") {
      std::string value;
      if (!next_value(index, argc, argv, value, "--camera-pitch", error) ||
          !parse_float(value, options.camera_pitch, 10.0f, 85.0f,
                       "--camera-pitch", error)) return false;
      options.camera_angle_set = true;
    } else if (argument == "--shot") {
      std::string frame_count;
      if (!next_value(index, argc, argv, frame_count, "--shot", error) ||
          !next_value(index, argc, argv, options.shot_path, "--shot", error)) return false;
      double parsed = 0.0;
      if (!parse_double(frame_count, parsed, 1.0,
                        static_cast<double>(std::numeric_limits<int>::max()),
                        "--shot", error)) return false;
      options.shot_frames = static_cast<int>(parsed);
    } else {
      if (error != nullptr) *error = "unknown option: " + argument;
      return false;
    }
  }
  return true;
}

}  // namespace lost_signal::sim::viewer
