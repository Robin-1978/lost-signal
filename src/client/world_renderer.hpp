#pragma once

#include "game/mission_simulation.hpp"

#include <raylib.h>

#include <filesystem>
#include <memory>
#include <string>

namespace client {

class SceneRenderer3D;

class WorldRenderer {
 public:
 WorldRenderer(const std::filesystem::path& font_path, const std::filesystem::path& assets_root,
               const game::ContentCatalog& catalog);
 ~WorldRenderer();

 WorldRenderer(const WorldRenderer&) = delete;
 WorldRenderer& operator=(const WorldRenderer&) = delete;

 void draw(const game::MissionSimulation& mission, int screen_width, int screen_height,
           const std::string& chapter_title);
 void toggle_analysis_mode() { analysis_mode_ = !analysis_mode_; }
 bool analysis_mode() const { return analysis_mode_; }

 private:
 Vector2 world_to_screen(sim::Vec2 point) const;
 void draw_text(const std::string& text, Vector2 position, float size, Color color) const;
 void draw_header(const game::MissionSimulation& mission, int width, const std::string& chapter_title) const;
 void draw_world(const game::MissionSimulation& mission) const;
 void draw_robot(const game::MissionSimulation& mission) const;
 void draw_telemetry(const game::MissionSimulation& mission, int screen_height) const;
 void draw_view_mode() const;

 Font font_{};
 bool owns_font_{false};
 const game::ContentCatalog& catalog_;
 std::unique_ptr<SceneRenderer3D> scene_3d_;
 bool analysis_mode_{false};
 Rectangle viewport_{};
 float scale_{1.0f};
 Vector2 world_origin_{};
};

std::filesystem::path find_ui_font(const std::filesystem::path& assets = {});

} // namespace client
