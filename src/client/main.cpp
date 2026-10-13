#include "game_ui.hpp"
#include "world_renderer.hpp"

#include "game/campaign_progress.hpp"
#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"

#include <TGUI/Backend/raylib.hpp>
#include <TGUI/TGUI.hpp>
#include <raylib.h>

#include <algorithm>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <memory>
#include <string>

namespace {

struct Options {
 std::filesystem::path assets;
 bool demo{false};
 bool analysis_view{false};
 bool chapter_view{false};
 std::string level;
 std::filesystem::path screenshot;
 std::filesystem::path progress_file;
 bool no_save{false};
};

std::filesystem::path default_progress_path() {
#if defined(_WIN32)
 if (const char* app_data = std::getenv("APPDATA")) {
 return std::filesystem::path(app_data) / "Lost Signal" / "progress.json";
 }
#elif defined(__APPLE__)
 if (const char* user_home = std::getenv("HOME")) {
 return std::filesystem::path(user_home) / "Library" / "Application Support"
 / "Lost Signal" / "progress.json";
 }
#else
 if (const char* data_home = std::getenv("XDG_DATA_HOME")) {
 return std::filesystem::path(data_home) / "lost-signal" / "progress.json";
 }
 if (const char* user_home = std::getenv("HOME")) {
 return std::filesystem::path(user_home) / ".local" / "share"
 / "lost-signal" / "progress.json";
 }
#endif
 return ".lost-signal-progress.json";
}

std::filesystem::path find_assets(const char* executable, const std::filesystem::path& requested) {
 if (!requested.empty()) return requested;
 const std::filesystem::path candidates[] = {
 "assets",
 std::filesystem::absolute(executable).parent_path() / "assets",
 LOST_SIGNAL_SOURCE_ASSETS,
 };
 for (const auto& candidate : candidates) {
 if (std::filesystem::is_regular_file(candidate / "campaign.json")) return candidate;
 }
 throw std::runtime_error("cannot find assets/campaign.json; use --assets PATH");
}

Options parse_options(int argc, char** argv) {
 Options options;
 for (int index = 1; index < argc; ++index) {
 const std::string argument = argv[index];
 if (argument == "--assets" && index + 1 < argc) options.assets = argv[++index];
 else if (argument == "--demo") options.demo = true;
 else if (argument == "--analysis-view") options.analysis_view = true;
 else if (argument == "--chapter-view") options.chapter_view = true;
 else if (argument == "--level" && index + 1 < argc) options.level = argv[++index];
 else if (argument == "--screenshot" && index + 1 < argc) options.screenshot = argv[++index];
 else if (argument == "--save" && index + 1 < argc) options.progress_file = argv[++index];
 else if (argument == "--no-save") options.no_save = true;
 else if (argument == "--help" || argument == "-h") {
 std::cout << "Usage: lost_signal [--assets PATH] [--level ID] [--demo]"
 " [--analysis-view] [--chapter-view]"
 " [--screenshot FILE] [--save FILE|--no-save]\n";
 std::exit(0);
 } else throw std::runtime_error("unknown argument: " + argument);
 }
 return options;
}

int run_client(const game::ContentCatalog& catalog, const Options& options) {
 SetTraceLogLevel(LOG_ERROR);
 SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT | FLAG_VSYNC_HINT);
 InitWindow(1440, 900, "失落的信号 / Lost Signal");
 SetWindowMinSize(1100, 720);
 SetTargetFPS(60);

 {
 tgui::Gui gui;
 const auto font_path = client::find_ui_font(options.assets);
 if (!font_path.empty()) gui.setFont(tgui::Font(font_path.string().c_str()));

 const std::string initial_level = !options.level.empty() ? options.level
 : ((options.demo || !options.screenshot.empty()) && !options.chapter_view
 ? catalog.campaign().levels.back()
 : catalog.campaign().starting_level);
 const auto& presentation = catalog.presentation_for_level(initial_level);
 client::GameUi ui(gui, catalog, presentation);
 const bool persist_progress = !options.no_save && !options.demo && options.screenshot.empty();
 const auto progress_path = options.progress_file.empty()
 ? default_progress_path() : options.progress_file;
 game::CampaignProgress progress;
 if (persist_progress) {
 try {
 progress = game::CampaignProgress::load(progress_path, catalog);
 ui.restore_completed(progress.completed_levels());
 ui.set_inventory(progress.inventory());
 } catch (const std::exception& error) {
 std::cerr << "warning: ignoring unreadable progress: " << error.what() << '\n';
 }
 }
 client::WorldRenderer renderer(font_path, options.assets, catalog);
 if (options.analysis_view) renderer.toggle_analysis_mode();
 auto mission = std::make_unique<game::MissionSimulation>(
 catalog, presentation.level, presentation.default_build);

 bool paused = false;
 double speed = 1.0;
 double accumulator = 0.0;
 bool screenshot_taken = false;
 int rendered_frames = 0;
 if ((options.demo || !options.screenshot.empty()) && !options.chapter_view) {
 ui.use_recommended_build();
 mission = std::make_unique<game::MissionSimulation>(catalog, presentation.level, ui.build());
 mission->start();
 ui.set_screen(client::Screen::Running, mission.get());
 speed = options.screenshot.empty() ? 2.0 : 8.0;
 } else if (!options.level.empty()) {
 ui.set_screen(client::Screen::Garage, mission.get());
 }

 while (!WindowShouldClose()) {
 gui.handleEvents();
 while (const int character = GetCharPressed()) gui.handleCharPressed(character);
 while (const int key = GetKeyPressed()) gui.handleKeyPressed(key);

 if (IsWindowResized()) ui.resize(GetScreenWidth(), GetScreenHeight());
 if (IsKeyPressed(KEY_SPACE)) paused = !paused;
 if (IsKeyPressed(KEY_TAB)) renderer.toggle_analysis_mode();
 if (IsKeyPressed(KEY_ONE)) speed = 1.0;
 if (IsKeyPressed(KEY_TWO)) speed = 2.0;
 if (IsKeyPressed(KEY_FOUR)) speed = 4.0;

 const auto actions = ui.consume_actions();
 if (!actions.selected_level.empty()) {
 ui.select_level(actions.selected_level);
 if (const auto* last = progress.last_build(ui.level_id())) ui.set_build(*last);
 mission = std::make_unique<game::MissionSimulation>(
 catalog, ui.level_id(), ui.build());
 ui.set_screen(client::Screen::Garage, mission.get());
 paused = false;
 accumulator = 0.0;
 }
 if (actions.open_prediction) ui.set_screen(client::Screen::Prediction, mission.get());
 if (actions.open_chapter) ui.set_screen(client::Screen::Chapter, mission.get());
 if (actions.open_garage) {
 mission = std::make_unique<game::MissionSimulation>(catalog, ui.level_id(), ui.build());
 ui.set_screen(client::Screen::Garage, mission.get());
 paused = false;
 accumulator = 0.0;
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
 mission = std::make_unique<game::MissionSimulation>(catalog, ui.level_id(), ui.build());
 mission->start();
 ui.set_screen(client::Screen::Running, mission.get());
 paused = false;
 accumulator = 0.0;
 }
 if (actions.toggle_pause) paused = !paused;
 if (actions.speed > 0.0) speed = actions.speed;

 if (ui.screen() == client::Screen::Running
 && mission->level().control_mode == "manual_calibration"
 && !options.demo && options.screenshot.empty()) {
 const double forward = (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP) ? 1.0 : 0.0)
 - (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN) ? 1.0 : 0.0);
 const double turn = (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT) ? 1.0 : 0.0)
 - (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT) ? 1.0 : 0.0);
 mission->set_manual_command(forward, turn);
 }

 if (ui.screen() == client::Screen::Running && !paused
 && mission->snapshot().status == game::MissionStatus::Running) {
 if (!options.screenshot.empty()) {
 // Automated art checks advance by a fixed number of simulation ticks per frame so
 // screenshots are byte-for-byte reproducible on different machines.
 for (int step = 0; step < 12
 && mission->snapshot().status == game::MissionStatus::Running; ++step) {
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
 if (mission->snapshot().status == game::MissionStatus::Succeeded
 || mission->snapshot().status == game::MissionStatus::Failed) {
 if (mission->snapshot().status == game::MissionStatus::Succeeded) {
 ui.mark_completed(mission->level().id);
 if (persist_progress) {
 try {
 progress.record_completion(mission->level().id, mission->result(), catalog);
 progress.save(progress_path, catalog);
 ui.set_inventory(progress.inventory());
 } catch (const std::exception& error) {
 std::cerr << "warning: could not save progress: " << error.what() << '\n';
 }
 }
 }
 ui.set_screen(client::Screen::Debrief, mission.get());
 }
 }

 BeginDrawing();
 renderer.draw(*mission, GetScreenWidth(), GetScreenHeight(), catalog.campaign().title);
 gui.draw();
 EndDrawing();
 ++rendered_frames;

 if (!options.screenshot.empty() && !screenshot_taken
 && ((options.chapter_view && rendered_frames >= 3)
 || (!options.chapter_view
 && ((mission->snapshot().actors.empty() && mission->snapshot().time_s >= 2.5)
 || mission->snapshot().waiting_for_clearance
 || mission->snapshot().status == game::MissionStatus::Succeeded
 || mission->snapshot().status == game::MissionStatus::Failed)))) {
 auto screenshot_path = options.screenshot;
 if (screenshot_path.is_absolute()) {
 screenshot_path = screenshot_path.lexically_relative(std::filesystem::current_path());
 }
 TakeScreenshot(screenshot_path.string().c_str());
 screenshot_taken = true;
 }
 if (screenshot_taken) break;
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
 const auto catalog = game::ContentCatalog::load(options.assets);
 return run_client(catalog, options);
 } catch (const std::exception& error) {
 std::cerr << "error: " << error.what() << '\n';
 return 1;
 }
}
