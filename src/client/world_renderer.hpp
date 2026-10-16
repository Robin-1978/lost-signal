#pragma once

#include "game/mission_simulation.hpp"

#include "map_editor.hpp"

#include <raylib.h>

#include <filesystem>
#include <memory>
#include <optional>
#include <string>

namespace client {

class SceneRenderer3D;

class WorldRenderer {
 public:
 enum class TerrainLayer { Elevation = 0, Slope = 1, VehicleRisk = 2, FinalCost = 3 };

 WorldRenderer(const std::filesystem::path& font_path, const std::filesystem::path& assets_root,
               const game::ContentCatalog& catalog);
 ~WorldRenderer();

 WorldRenderer(const WorldRenderer&) = delete;
 WorldRenderer& operator=(const WorldRenderer&) = delete;

 void draw(const game::MissionSimulation& mission, int screen_width, int screen_height,
 const std::string& chapter_title, bool assembly_view = false);
 void toggle_analysis_mode() { analysis_mode_ = !analysis_mode_; }
 bool analysis_mode() const { return analysis_mode_; }
 void set_terrain_layer(TerrainLayer layer) { terrain_layer_ = layer; }
 TerrainLayer terrain_layer() const { return terrain_layer_; }
 std::optional<sim::Vec2> screen_to_world(Vector2 screen) const;
 std::optional<std::string> garage_slot_at(Vector2 screen) const;
 void draw_edit_overlay(const game::MissionSimulation& mission, const MapEditor& editor) const;

 private:
 Vector2 world_to_screen(sim::Vec2 point) const;
 void draw_text(const std::string& text, Vector2 position, float size, Color color) const;
 void draw_header(const game::MissionSimulation& mission, int width, const std::string& chapter_title) const;
 void draw_world(const game::MissionSimulation& mission) const;
 void draw_terrain_layer(const game::MissionSimulation& mission) const;
 void draw_terrain_layer_panel(const game::MissionSimulation& mission) const;
 void draw_field_operator(const game::MissionSimulation& mission) const;
 void draw_robot(const game::MissionSimulation& mission) const;
 void draw_dust(const game::MissionSimulation& mission) const;
 void draw_telemetry(const game::MissionSimulation& mission, int screen_height) const;
 void draw_view_mode() const;
 void draw_garage_overlay(const game::MissionSimulation& mission);

 Font font_{};
 bool owns_font_{false};
 const game::ContentCatalog& catalog_;
 std::unique_ptr<SceneRenderer3D> scene_3d_;
 // Engine art (not content): dust sprite for transient puffs.
 Texture2D dust_sprite_{};
 // Ground albedo tiles by material texture path (2D analysis view).
 std::unordered_map<std::string, Texture2D> ground_textures_;
 // Dust puff pool (visual only: capped, fading, no physics in/out).
 struct DustPuff {
  sim::Vec2 pos{0.0, 0.0};
  float age_s{0.0f};
  float life_s{0.9f};
  float size_px{5.0f};
 };
 mutable std::vector<DustPuff> dust_;
 bool analysis_mode_{false};
 TerrainLayer terrain_layer_{TerrainLayer::FinalCost};
 Rectangle viewport_{};
 float scale_{1.0f};
 Vector2 world_origin_{};
 bool drawn_once_{false};
 struct GarageHotspot {
 std::string slot;
 Vector2 center{};
 Rectangle label{};
 float radius{28.0f};
 };
 std::vector<GarageHotspot> garage_hotspots_;
 // Camera focus smoothing (display only): at 5Hz the raw estimate steps and
 // the whole frame shudders. Exponential ease toward the target each frame.
 sim::Vec2 focus_smooth_{0.0, 0.0};
 bool focus_init_{false};
 // Threat-blend hold for focus hysteresis (display only, mission-time based).
 sim::Vec2 focus_hold_{0.0, 0.0};
 double focus_hold_until_{-1.0};
};

std::filesystem::path find_ui_font(const std::filesystem::path& assets = {});

} // namespace client
