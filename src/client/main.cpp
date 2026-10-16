#include "game_ui.hpp"
#include "world_renderer.hpp"

#include "game/campaign_progress.hpp"
#include "game/content_catalog.hpp"
#include "map_editor.hpp"

#include <optional>
#include "game/mission_simulation.hpp"

#include <TGUI/Backend/raylib.hpp>
#include <TGUI/TGUI.hpp>
#include <raylib.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {

struct Options {
 std::filesystem::path assets;
 std::string language;
 bool demo{false};
 bool analysis_view{false};
 bool chapter_view{false};
 bool garage_view{false};
 std::string level;
 std::filesystem::path screenshot;
 double screenshot_time_s{-1.0};
 std::filesystem::path progress_file;
 bool no_save{false};
 game::PhysicsMode physics_mode{game::PhysicsMode::Bullet};
};

std::filesystem::path default_progress_path() {
#if defined(_WIN32)
 if (const char* app_data = std::getenv("APPDATA")) {
  return std::filesystem::path(app_data) / "Lost Signal" / "progress.json";
 }
#elif defined(__APPLE__)
 if (const char* user_home = std::getenv("HOME")) {
  return std::filesystem::path(user_home) / "Library" / "Application Support" / "Lost Signal" /
         "progress.json";
 }
#else
 if (const char* data_home = std::getenv("XDG_DATA_HOME")) {
  return std::filesystem::path(data_home) / "lost-signal" / "progress.json";
 }
 if (const char* user_home = std::getenv("HOME")) {
  return std::filesystem::path(user_home) / ".local" / "share" / "lost-signal" / "progress.json";
 }
#endif
 return ".lost-signal-progress.json";
}

std::filesystem::path find_assets(const char* executable, const std::filesystem::path& requested) {
 if (!requested.empty())
  return requested;
 const std::filesystem::path candidates[] = {
   "assets",
   std::filesystem::absolute(executable).parent_path() / "assets",
   LOST_SIGNAL_SOURCE_ASSETS,
 };
 for (const auto& candidate : candidates) {
  if (std::filesystem::is_regular_file(candidate / "campaign.json"))
   return candidate;
 }
 throw std::runtime_error("cannot find assets/campaign.json; use --assets PATH");
}

Options parse_options(int argc, char** argv) {
 Options options;
 for (int index = 1; index < argc; ++index) {
  const std::string argument = argv[index];
  if (argument == "--assets" && index + 1 < argc)
   options.assets = argv[++index];
 else if (argument == "--language" && index + 1 < argc)
 options.language = argv[++index];
  else if (argument == "--demo")
   options.demo = true;
  else if (argument == "--analysis-view")
   options.analysis_view = true;
  else if (argument == "--chapter-view")
   options.chapter_view = true;
  else if (argument == "--garage-view")
   options.garage_view = true;
  else if (argument == "--level" && index + 1 < argc)
   options.level = argv[++index];
  else if (argument == "--screenshot" && index + 1 < argc)
   options.screenshot = argv[++index];
  else if (argument == "--screenshot-time" && index + 1 < argc)
   options.screenshot_time_s = std::stod(argv[++index]);
  else if (argument == "--save" && index + 1 < argc)
   options.progress_file = argv[++index];
  else if (argument == "--no-save")
   options.no_save = true;
 else if (argument == "--physics" && index + 1 < argc)
 options.physics_mode = game::parse_physics_mode(argv[++index]);
  else if (argument == "--help" || argument == "-h") {
 std::cout << "Usage: lost_signal [--assets PATH] [--language en|zh-CN] [--level ID] [--demo]"
                " [--analysis-view] [--chapter-view] [--garage-view]"
 " [--screenshot FILE] [--screenshot-time SECONDS] [--save FILE|--no-save]"
 " [--physics domain|shadow|bullet|auto]\n";
   std::exit(0);
  } else
   throw std::runtime_error("unknown argument: " + argument);
 }
 if (!std::isfinite(options.screenshot_time_s) ||
     (options.screenshot_time_s < 0.0 && options.screenshot_time_s != -1.0)) {
  throw std::runtime_error("--screenshot-time must be non-negative");
 }
 return options;
}

// The catalog is owned by value so the map editor can reload it after a
// save: references (ui, renderers) stay bound to this object while missions
// and the editor are rebuilt from the fresh contents.
int run_client(game::ContentCatalog catalog, const Options& options) {
 SetTraceLogLevel(LOG_ERROR);
 SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
 InitWindow(1440, 900, catalog.tr("app.window_title").c_str());
 if (!IsWindowReady()) {
  std::cerr << "error: could not initialize the graphics window\n";
  return 2;
 }
 SetWindowMinSize(1100, 720);
 SetTargetFPS(60);

 {
  tgui::Gui gui;
  const auto font_path = client::find_ui_font(options.assets);
  if (!font_path.empty())
   gui.setFont(tgui::Font(font_path.string().c_str()));

  const std::string initial_level =
    !options.level.empty() ? options.level
                           : ((options.demo || !options.screenshot.empty()) && !options.chapter_view
                                ? catalog.campaign().levels.back()
                                : catalog.campaign().starting_level);
  const auto& presentation = catalog.presentation_for_level(initial_level);
  client::GameUi ui(gui, catalog, presentation);
  const bool persist_progress = !options.no_save && !options.demo && options.screenshot.empty();
  const auto progress_path = options.progress_file.empty() ? default_progress_path() : options.progress_file;
  game::CampaignProgress progress;
  if (persist_progress) {
   try {
    progress = game::CampaignProgress::load(progress_path, catalog);
    ui.restore_completed(progress.completed_levels());
    ui.set_inventory(progress.inventory());
 ui.set_abilities(progress.abilities());
 ui.set_contract_status(progress.reputation(), progress.claimed_contracts());
   } catch (const std::exception& error) {
    std::cerr << "warning: ignoring unreadable progress: " << error.what() << '\n';
   }
  }
  client::WorldRenderer renderer(font_path, options.assets, catalog);
  std::optional<client::MapEditor> editor;
  if (options.analysis_view)
   renderer.toggle_analysis_mode();
 const auto make_mission = [&](const std::string& level, const game::BuildSelection& build) {
 auto created = std::make_unique<game::MissionSimulation>(catalog, level, build, std::nullopt,
 progress.abilities(), game::ComponentRegistry::builtins(),
 std::shared_ptr<const game::IPathPlanner>{}, options.physics_mode,
 std::shared_ptr<sim::IHighLevelPolicy>{},
 ui.endless_mode() ? std::string{} : ui.contract_id());
 created->set_endless_mode(ui.endless_mode());
 if (created->endless_mode() && progress.regional_world())
 created->restore_endless_world(*progress.regional_world());
 if (created->endless_mode() && progress.field_expedition() &&
 progress.field_expedition()->level_id == level)
 created->restore_field_expedition(*progress.field_expedition());
 return created;
 };
 auto mission = make_mission(presentation.level, presentation.default_build);
 double next_region_autosave_s = 5.0;
 const auto persist_regional_world = [&] {
 if (!persist_progress || !mission || !mission->endless_mode())
 return;
 progress.set_regional_world(mission->endless_world_checkpoint());
 progress.set_field_expedition(mission->field_expedition_checkpoint());
 progress.save(progress_path, catalog);
 };

  bool paused = false;
  double speed = 1.0;
  double accumulator = 0.0;
  bool screenshot_taken = false;
  int rendered_frames = 0;
  if ((options.demo || !options.screenshot.empty()) && !options.chapter_view && !options.garage_view) {
   ui.use_recommended_build();
 mission = make_mission(presentation.level, ui.build());
   mission->start();
   ui.set_screen(client::Screen::Running, mission.get());
   speed = options.screenshot.empty() ? 2.0 : 8.0;
  } else if (!options.level.empty()) {
   ui.set_screen(client::Screen::Garage, mission.get());
  }

  while (!WindowShouldClose()) {
   gui.handleEvents();
   while (const int character = GetCharPressed())
    gui.handleCharPressed(character);
   while (const int key = GetKeyPressed())
    gui.handleKeyPressed(key);

   if (IsWindowResized())
    ui.resize(GetScreenWidth(), GetScreenHeight());
   if (IsKeyPressed(KEY_SPACE))
    paused = !paused;
 if (IsKeyPressed(KEY_TAB))
 renderer.toggle_analysis_mode();
 if (ui.screen() == client::Screen::Garage && !renderer.analysis_mode() &&
 IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
 if (const auto slot = renderer.garage_slot_at(GetMousePosition())) {
 ui.cycle_component(*slot);
 }
 }
 if (renderer.analysis_mode()) {
 if (IsKeyPressed(KEY_F1))
 renderer.set_terrain_layer(client::WorldRenderer::TerrainLayer::Elevation);
 if (IsKeyPressed(KEY_F2))
 renderer.set_terrain_layer(client::WorldRenderer::TerrainLayer::Slope);
 if (IsKeyPressed(KEY_F3))
 renderer.set_terrain_layer(client::WorldRenderer::TerrainLayer::VehicleRisk);
 if (IsKeyPressed(KEY_F4))
 renderer.set_terrain_layer(client::WorldRenderer::TerrainLayer::FinalCost);
 }
 // The editor works in both 3D and 2D. In 3D, placement uses the camera ray
 // intersection and the renderer shows a translucent working copy.
 if (IsKeyPressed(KEY_E)) {
    if (!editor || editor->level_id() != ui.level_id()) {
     editor.emplace(catalog, ui.level_id());
    }
    editor->toggle();
   }
 if (editor && editor->active()) {
    if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
     if (const auto world = renderer.screen_to_world(GetMousePosition())) {
      editor->press(world->x, world->y);
     }
    }
    if (IsMouseButtonDown(MOUSE_BUTTON_LEFT)) {
     if (const auto world = renderer.screen_to_world(GetMousePosition())) {
      editor->drag(world->x, world->y);
     }
    }
    if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) {
     editor->release();
    }
    if (IsKeyPressed(KEY_N)) {
     if (const auto world = renderer.screen_to_world(GetMousePosition())) {
      editor->add_obstacle(world->x, world->y);
     }
    }
    if (IsKeyPressed(KEY_B)) {
     if (const auto world = renderer.screen_to_world(GetMousePosition())) {
      editor->add_surface(world->x, world->y);
     }
    }
    if (IsKeyPressed(KEY_M)) {
     if (const auto world = renderer.screen_to_world(GetMousePosition())) {
      editor->add_marker(world->x, world->y);
     }
    }
 if (IsKeyPressed(KEY_R)) {
 if (const auto world = renderer.screen_to_world(GetMousePosition())) {
 editor->add_ramp(world->x, world->y);
 }
 }
 if (IsKeyPressed(KEY_P)) {
 if (const auto world = renderer.screen_to_world(GetMousePosition())) {
 editor->add_platform(world->x, world->y);
 }
 }
 if (IsKeyPressed(KEY_EQUAL))
 editor->adjust_selected_height(0.10);
 if (IsKeyPressed(KEY_MINUS))
 editor->adjust_selected_height(-0.10);
 if (IsKeyPressed(KEY_RIGHT_BRACKET))
 editor->adjust_selected_slope(0.05);
 if (IsKeyPressed(KEY_LEFT_BRACKET))
 editor->adjust_selected_slope(-0.05);
 if (IsKeyPressed(KEY_PERIOD))
 editor->adjust_selected_roughness(0.01);
 if (IsKeyPressed(KEY_COMMA))
 editor->adjust_selected_roughness(-0.01);
 if (IsKeyPressed(KEY_G))
 editor->rotate_selected_terrain();
    if (IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE)) {
     editor->remove_selected();
    }
    if (IsKeyPressed(KEY_S)) {
     const auto assets = options.assets.string();
     if (editor->save(assets)) {
      const auto kept_build = ui.build();
 catalog = game::ContentCatalog::load(options.assets, catalog.locale());
      ui.select_level(ui.level_id());
      ui.set_build(kept_build);
 mission = make_mission(ui.level_id(), ui.build());
      editor.emplace(catalog, ui.level_id());
     }
    }
   }
   if (IsKeyPressed(KEY_ONE))
    speed = 1.0;
   if (IsKeyPressed(KEY_TWO))
    speed = 2.0;
   if (IsKeyPressed(KEY_FOUR))
    speed = 4.0;

   const auto actions = ui.consume_actions();
 if (!actions.purchase_component.empty()) {
 try {
 if (progress.purchase_component(actions.purchase_component, catalog)) {
 if (persist_progress)
 progress.save(progress_path, catalog);
 ui.set_inventory(progress.inventory());
 ui.set_contract_status(progress.reputation(), progress.claimed_contracts());
 }
 } catch (const std::exception& error) {
 std::cerr << "warning: could not purchase component: " << error.what() << '\n';
 }
 }
   if (!actions.selected_level.empty()) {
 if (mission->endless_mode()) {
 try { persist_regional_world(); }
 catch (const std::exception& error) {
 std::cerr << "warning: could not save regional world: " << error.what() << '\n';
 }
 }
    ui.select_level(actions.selected_level);
    if (const auto* last = progress.last_build(ui.level_id()))
     ui.set_build(*last);
 mission = make_mission(ui.level_id(), ui.build());
    ui.set_screen(client::Screen::Garage, mission.get());
    paused = false;
    accumulator = 0.0;
 next_region_autosave_s = 5.0;
   }
 if (actions.open_prediction) {
 if (ui.endless_mode()) {
 mission = make_mission(ui.level_id(), ui.build());
 mission->start();
 ui.set_screen(client::Screen::Running, mission.get());
 paused = false;
 accumulator = 0.0;
 next_region_autosave_s = 5.0;
 } else {
 ui.set_screen(client::Screen::Prediction, mission.get());
 }
 }
   if (actions.open_chapter)
    ui.set_screen(client::Screen::Chapter, mission.get());
   // Garage preview follows the assembly: rebuild the render-only mission without
   // touching widgets (a slider drag emits build_changed every tick).
 // The garage permits a temporary over-budget assembly so the player can
 // swap either part in either order. Do not construct a validated mission
 // until the combination fits; the Next button is disabled meanwhile.
 if (actions.build_changed && ui.screen() == client::Screen::Garage && ui.build_within_budget()) {
 mission = make_mission(ui.level_id(), ui.build());
   }
   if (actions.open_garage) {
 if (mission->endless_mode()) {
 try { persist_regional_world(); }
 catch (const std::exception& error) {
 std::cerr << "warning: could not save regional world: " << error.what() << '\n';
 }
 }
 mission = make_mission(ui.level_id(), ui.build());
    ui.set_screen(client::Screen::Garage, mission.get());
    paused = false;
    accumulator = 0.0;
 next_region_autosave_s = 5.0;
   }
   if (actions.launch || actions.retry) {
    if (persist_progress) {
     try {
      progress.record_build(ui.level_id(), ui.build());
      progress.save(progress_path, catalog);
     } catch (const std::exception& error) {
      std::cerr << "warning: could not save progress: " << error.what() << '\n';
     }
    }
 mission = make_mission(ui.level_id(), ui.build());
    mission->start();
    ui.set_screen(client::Screen::Running, mission.get());
    paused = false;
    accumulator = 0.0;
 next_region_autosave_s = 5.0;
   }
 if (actions.toggle_pause)
 paused = !paused;
 if (actions.reset_simulation && mission->endless_mode()) {
 try { persist_regional_world(); }
 catch (const std::exception& error) {
 std::cerr << "warning: could not save regional world: " << error.what() << '\n';
 }
 mission = make_mission(ui.level_id(), ui.build());
 mission->start();
 ui.set_screen(client::Screen::Running, mission.get());
 paused = false;
 accumulator = 0.0;
 next_region_autosave_s = 5.0;
 }
   if (actions.speed > 0.0)
    speed = actions.speed;
   if (actions.replay_time_s && ui.screen() == client::Screen::Debrief) {
    mission->seek_replay(*actions.replay_time_s);
    ui.refresh_debrief(*mission);
   }

 if (ui.screen() == client::Screen::Running && mission->endless_mode() &&
 (!editor || !editor->active()) && !options.demo && options.screenshot.empty()) {
 const double world_x = (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT) ? 1.0 : 0.0) -
 (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT) ? 1.0 : 0.0);
 const double world_y = (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP) ? 1.0 : 0.0) -
 (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN) ? 1.0 : 0.0);
 mission->set_field_operator_command(world_x, world_y);
 if (IsKeyPressed(KEY_F))
 mission->issue_field_intent(game::FieldIntentKind::Follow);
 if (IsKeyPressed(KEY_G)) {
 if (const auto target = renderer.screen_to_world(GetMousePosition()))
 mission->issue_field_intent(game::FieldIntentKind::GoTo, *target);
 }
 if (IsKeyPressed(KEY_Q)) {
 if (const auto target = renderer.screen_to_world(GetMousePosition()))
 mission->issue_field_intent(game::FieldIntentKind::Search, *target);
 }
 if (IsKeyPressed(KEY_X))
 mission->issue_field_intent(game::FieldIntentKind::Wait);
 if (IsKeyPressed(KEY_R))
 mission->issue_field_intent(game::FieldIntentKind::Return);
 } else if (ui.screen() == client::Screen::Running &&
 mission->level().control_mode == "manual_calibration" &&
 !options.demo && options.screenshot.empty()) {
    const double forward = (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP) ? 1.0 : 0.0) -
                           (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN) ? 1.0 : 0.0);
    const double turn = (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT) ? 1.0 : 0.0) -
                        (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT) ? 1.0 : 0.0);
    mission->set_manual_command(forward, turn);
   }

   if (ui.screen() == client::Screen::Running && !paused &&
       mission->snapshot().status == game::MissionStatus::Running) {
    if (!options.screenshot.empty()) {
     // Automated art checks advance by a fixed number of simulation ticks per frame so
     // screenshots are byte-for-byte reproducible on different machines.
     for (int step = 0; step < 12 && mission->snapshot().status == game::MissionStatus::Running; ++step) {
      mission->step();
     }
    } else {
     accumulator += std::min(0.1, static_cast<double>(GetFrameTime())) * speed;
     int steps = 0;
     while (accumulator >= game::MissionSimulation::fixed_step_s && steps < 80) {
      mission->step();
      accumulator -= game::MissionSimulation::fixed_step_s;
      ++steps;
     }
    }
   }
   if (ui.screen() == client::Screen::Running) {
    ui.refresh_running(*mission, paused, speed);
    if (mission->snapshot().status == game::MissionStatus::Succeeded ||
        mission->snapshot().status == game::MissionStatus::Failed) {
     if (mission->snapshot().status == game::MissionStatus::Succeeded) {
      ui.mark_completed(mission->level().id);
      if (persist_progress) {
       try {
        progress.record_completion(mission->level().id, mission->result(), catalog);
        progress.save(progress_path, catalog);
        ui.set_inventory(progress.inventory());
 ui.set_abilities(progress.abilities());
 ui.set_contract_status(progress.reputation(), progress.claimed_contracts());
       } catch (const std::exception& error) {
        std::cerr << "warning: could not save progress: " << error.what() << '\n';
       }
      }
     }
     ui.set_screen(client::Screen::Debrief, mission.get());
    }
   }

 if (persist_progress && mission->endless_mode() &&
 mission->snapshot().time_s + 1e-9 >= next_region_autosave_s) {
 try {
 persist_regional_world();
 } catch (const std::exception& error) {
 std::cerr << "warning: could not autosave regional world: " << error.what() << '\n';
 }
 next_region_autosave_s = mission->snapshot().time_s + 5.0;
 }

   BeginDrawing();
   // Chapter header follows the level, not the campaign: chapters.json title
   // by level.chapter plus the level's section number within that chapter.
   // (It used to print the campaign title for every level, so the town's
   // third chapter showed up as the first.)
   std::string chapter_title = catalog.campaign().title;
   for (const auto& chapter : catalog.chapters()) {
    if (chapter.id == mission->level().chapter) {
     chapter_title = chapter.title;
     break;
    }
   }
   int section = 1;
   for (const auto& level_id : catalog.campaign().levels) {
    if (level_id == mission->level().id)
     break;
    if (catalog.level(level_id).chapter == mission->level().chapter)
     ++section;
   }
   renderer.draw(*mission, GetScreenWidth(), GetScreenHeight(),
                 chapter_title + "  /  " + catalog.trf("world.section", {std::to_string(section)}),
                 ui.screen() == client::Screen::Garage);
   if (editor && editor->active()) {
    renderer.draw_edit_overlay(*mission, *editor);
   }
   gui.draw();
   EndDrawing();
   ++rendered_frames;

   const bool terminal = mission->snapshot().status == game::MissionStatus::Succeeded ||
                         mission->snapshot().status == game::MissionStatus::Failed;
   const bool target_frame = options.screenshot_time_s >= 0.0
                               ? mission->snapshot().time_s >= options.screenshot_time_s || terminal
                               : (mission->snapshot().actors.empty() && mission->snapshot().time_s >= 2.5) ||
                                   mission->snapshot().waiting_for_clearance || terminal;
   if (!options.screenshot.empty() && !screenshot_taken &&
       ((options.chapter_view && rendered_frames >= 3) || (options.garage_view && rendered_frames >= 3) ||
        (!options.chapter_view && !options.garage_view && target_frame))) {
    auto screenshot_path = options.screenshot;
    if (screenshot_path.is_absolute()) {
     screenshot_path = screenshot_path.lexically_relative(std::filesystem::current_path());
    }
    TakeScreenshot(screenshot_path.string().c_str());
    screenshot_taken = true;
   }
   if (screenshot_taken)
    break;
  }
 if (mission->endless_mode()) {
 try { persist_regional_world(); }
 catch (const std::exception& error) {
 std::cerr << "warning: could not save regional world at exit: " << error.what() << '\n';
 }
 }
 }
 CloseWindow();
 return 0;
}

} // namespace

int main(int argc, char** argv) {
 try {
  auto options = parse_options(argc, argv);
  options.assets = find_assets(argv[0], options.assets);
 const auto catalog = game::ContentCatalog::load(options.assets, options.language);
  return run_client(catalog, options);
 } catch (const std::exception& error) {
  std::cerr << "error: " << error.what() << '\n';
  return 1;
 }
}
