// SIM-side live viewer (2D top-down). Reads truth + recorded episode state
// through ScenarioSession (SIM composition root) and draws it with raylib.
// Never publishes to ROBOT: the session steps headless exactly as without
// a window; the viewer only observes. Headless CI never runs this binary
// (needs a display); frame logic is unit-tested in test_viewer_frame.

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <filesystem>
#include <limits>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <utility>
#include <vector>
#include "robot/core/json.hpp"
#include "robot/platform/robot_config.hpp"
#include "sim/experiment/experience_runtime.hpp"
#include "sim/experiment/episode_pair.hpp"
#include "sim/eval/robot_config.hpp"
#include "sim/eval/sim_config.hpp"
#include "sim/eval/world_io.hpp"
#include "sim/scenario/actor_sidecar.hpp"
#include "sim/scenario/scenario_session.hpp"
#include "sim/viewer/asset_manifest.hpp"
#include "sim/viewer/actor_render_policy.hpp"
#include "sim/viewer/collision_geom_wireframe.hpp"
#include "sim/viewer/experience_flow.hpp"
#include "sim/viewer/experience_manifest.hpp"
#include "sim/viewer/experience_overlay.hpp"
#include "sim/viewer/frame_builder.hpp"
#include "sim/viewer/keyframe_review.hpp"
#include "sim/viewer/map_editor.hpp"
#include "sim/viewer/particle_system.hpp"
#include "sim/viewer/playback_controller.hpp"
#include "sim/viewer/player_profile.hpp"
#include "sim/viewer/player_progress_ui.hpp"
#include "sim/viewer/scene_renderer.hpp"
#include "sim/viewer/scene_manifest.hpp"
#include "sim/viewer/view_transform.hpp"
#include "sim/viewer/viewer_audio.hpp"
#include "sim/viewer/viewer_app.hpp"
#include "sim/viewer/viewer_options.hpp"
#include "sim/world/actor_behavior.hpp"
#include "sim/world/mujoco_backend.hpp"
#include "sim/world/terrain.hpp"

namespace {
using lost_signal::sim::viewer::Frame;
using lost_signal::sim::viewer::FrameActor;
using lost_signal::sim::viewer::FrameBody;
using lost_signal::sim::viewer::FrameGeom;
using lost_signal::sim::viewer::FrameLinkPose;
using lost_signal::sim::viewer::FramePlace;
using lost_signal::sim::viewer::FrameWheel;
using lost_signal::sim::viewer::ViewTransform;
using lost_signal::sim::viewer::AssetDefinition;
using lost_signal::sim::viewer::AssetManifest;
using lost_signal::sim::viewer::AssemblySelection;
using lost_signal::sim::viewer::ActorRenderPath;
using lost_signal::sim::viewer::choose_actor_render_path;
using lost_signal::sim::viewer::ExperienceFlow;
using lost_signal::sim::viewer::HeroSequence;
using lost_signal::sim::viewer::ExperienceHeroShot;
using lost_signal::sim::viewer::ExperienceManifest;
using lost_signal::sim::viewer::experience_variable_spec;
using lost_signal::sim::viewer::ExperiencePhase;
using lost_signal::sim::viewer::ExperienceRunKind;
using lost_signal::sim::viewer::ExperienceViewLayer;
using lost_signal::sim::viewer::ExperienceViewPolicy;
using lost_signal::sim::viewer::PredictionAssessment;
using lost_signal::sim::viewer::PlayerProfile;
using lost_signal::sim::viewer::ChapterCatalogEntry;
using lost_signal::sim::viewer::ExperienceHitRegions;
using lost_signal::sim::viewer::ExperimentUiState;
using lost_signal::sim::viewer::VisibleRunSummary;
using lost_signal::sim::viewer::render::CameraState;
using lost_signal::sim::viewer::render::RenderQuality;
using lost_signal::sim::viewer::render::VisualAssets;
using lost_signal::sim::viewer::render::WalkaroundView;
using lost_signal::sim::viewer::render::blend_camera;
using lost_signal::sim::viewer::render::camera_from_state;
using lost_signal::sim::viewer::render::draw_frame_2d;
using lost_signal::sim::viewer::render::draw_frame_3d_direct;
using lost_signal::sim::viewer::render::draw_frame_3d_postprocessed;
using lost_signal::sim::viewer::render::draw_baseline_comparison_ghost_2d;
using lost_signal::sim::viewer::render::draw_baseline_comparison_ghost_3d;
using lost_signal::sim::viewer::render::load_visual_assets;
using lost_signal::sim::viewer::render::map_bounds;
using lost_signal::sim::viewer::render::render_quality_name;
using lost_signal::sim::viewer::render::screen_to_ground_3d;
using lost_signal::sim::viewer::render::scene_background;
using lost_signal::sim::viewer::render::unload_visual_assets;
using lost_signal::sim::viewer::render::walkaround_view;
using lost_signal::sim::viewer::draw_experience_overlay;
using lost_signal::sim::viewer::draw_chapter_catalog_panel;
using lost_signal::sim::viewer::draw_player_profile_panel;
using lost_signal::sim::viewer::load_chapter_catalog;
using lost_signal::sim::viewer::ParticleKind;
using lost_signal::sim::viewer::ParticleSystem;
using lost_signal::sim::viewer::SurfaceTraceKind;
using lost_signal::sim::viewer::SurfaceTraceSystem;
using lost_signal::sim::viewer::ReviewKeyframe;
using lost_signal::sim::viewer::SceneInstance;
using lost_signal::sim::viewer::SceneManifest;
using lost_signal::sim::world::ActorLinkKind;

constexpr int kScreenW = 1100;
constexpr int kScreenH = 750;
constexpr int kMinimumScreenW = 960;
constexpr int kMinimumScreenH = 640;
// Presentation text is intentionally larger than the simulation HUD. It
// grows with the window so a maximized 1080p/ultrawide view remains legible
// without changing any ROBOT or SIM timing.
constexpr float kMinimumUiTextScale = 1.15f;
constexpr float kMaximumUiTextScale = 1.45f;
// Open-ended sandbox runs retain a local replay window. Authored experiences
// keep their complete, independently scored episode for debrief and A/B.
Font g_ui_font{};
bool g_owns_ui_font = false;
bool g_zh_cn = false;

bool use_zh_cn_locale(const std::string& requested) {
 if (requested == "zh-CN") return true;
 if (requested == "en") return false;
 const char* locale = std::getenv("LC_ALL");
 if (locale == nullptr || *locale == '\0') locale = std::getenv("LC_MESSAGES");
 if (locale == nullptr || *locale == '\0') locale = std::getenv("LANG");
 return locale != nullptr && std::string(locale).rfind("zh", 0) == 0;
}

const char* tr(const char* english, const char* simplified_chinese) {
 return g_zh_cn ? simplified_chinese : english;
}

Font ui_font() {
 return g_owns_ui_font ? g_ui_font : GetFontDefault();
}

float ui_text_scale() {
 const float width_scale = static_cast<float>(GetScreenWidth()) /
 static_cast<float>(kScreenW);
 const float height_scale = static_cast<float>(GetScreenHeight()) /
 static_cast<float>(kScreenH);
 return std::clamp(kMinimumUiTextScale *
 std::min(width_scale, height_scale),
 kMinimumUiTextScale, kMaximumUiTextScale);
}

void draw_ui_text(const char* text, int x, int y, int font_size,
 Color color) {
 DrawTextEx(ui_font(), text, {static_cast<float>(x), static_cast<float>(y)},
 static_cast<float>(font_size) * ui_text_scale(), 0.5f, color);
}

float measure_ui_text(const char* text, float font_size) {
 return MeasureTextEx(ui_font(), text, font_size * ui_text_scale(), 0.5f).x;
}

void unload_ui_font();

void load_ui_font(const ExperienceManifest* experience) {
 unload_ui_font();
 const std::filesystem::path path =
 "assets/fonts/NotoSansCJKsc-Regular.otf";
 if (!std::filesystem::is_regular_file(path)) return;
 std::string glyph_text =
 "机器人现场实验室任务简报部件检查开始实验检查全部先做预测点击选项"
 "或按数字键开始实时证据数据权限观察估计规划移动制动稳定物理真值"
 "运行中隐藏复盘时显示离线评估不会向机器人发布证据姿态来源"
 "本次运行证明了什么关键帧受控对比重试下一章通过安全门后解锁"
 "暂停单步激光估计真值调试跟随画质适配速度二维三维编辑轨道缩放"
 "物理部件已检查时间用尽操作失败碰撞最小净空平均误差重新运行基线"
 "变量种子预测得到本局支持需要修正开始对照实验重试通过安全验收后"
 "失落信号初次接触真实物理刚体与关节姿态回车空格点击跳过因果复盘"
 "仿真离线验收净空估计误差安全任务实验为什么本次运行止于这里步骤"
 "左右键回放记录首次有效观测制动命令稳定停车报告完成接受拒绝未记录"
 "机器人记录证据变体下一步只改变扫描周期预测得到支持被证据反驳将在"
 "两次运行完成后验证运行可视通过验收后继续从未收到仿真真值教学视图"
 "因果链任务视图等待测量建立估计按已验证命令移动任务完成验证安全锁"
 "返回轮速位置不确定度已定位跟踪中足够传感器决策独立停车并观察局部"
 "轨迹运行时隐藏查看工程解释原因条件慢扫描游戏当前任务状态规划决策"
 "这些证据不会发布回机器人必看查看驱动轮仿真实验室"
 "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789"
 "：，。；！？、（）《》【】 /.-+_%#=<>";
 if (experience != nullptr) {
 auto append = [&](const std::string& value) {
 glyph_text.push_back('\n');
 glyph_text += value;
 };
 append(experience->title);
 append(experience->subtitle);
 append(experience->prediction_question);
 append(experience->running_prompt);
 append(experience->success_title);
 append(experience->failure_title);
 for (const auto& value : experience->briefing) append(value);
 for (const auto& value : experience->lessons) append(value);
 for (const auto& shot : experience->intro) {
 append(shot.component_id);
 append(shot.title);
 append(shot.caption);
 }
 for (const auto& stop : experience->walkaround) {
 append(stop.component_id);
 append(stop.title);
 append(stop.lesson);
 }
 for (const auto& choice : experience->choices) {
 append(choice.label);
 append(choice.explanation);
 }
 for (const auto& experiment : experience->experiments) {
 append(experiment.title);
 append(experience_variable_spec(experiment.controlled_variable).id);
 }
 }
 int decoded_count = 0;
 int* decoded = LoadCodepoints(glyph_text.c_str(), &decoded_count);
 if (decoded == nullptr || decoded_count <= 0) {
 UnloadCodepoints(decoded);
 return;
 }
 std::vector<int> codepoints(decoded, decoded + decoded_count);
 UnloadCodepoints(decoded);
 std::sort(codepoints.begin(), codepoints.end());
 codepoints.erase(std::unique(codepoints.begin(), codepoints.end()),
 codepoints.end());
 g_ui_font = LoadFontEx(path.string().c_str(), 48, codepoints.data(),
 static_cast<int>(codepoints.size()));
 g_owns_ui_font = g_ui_font.texture.id != 0;
 if (g_owns_ui_font) {
 SetTextureFilter(g_ui_font.texture, TEXTURE_FILTER_BILINEAR);
 }
}

void unload_ui_font() {
 if (g_owns_ui_font) UnloadFont(g_ui_font);
 g_ui_font = {};
 g_owns_ui_font = false;
}

struct PlaybackHitRegions {
 std::array<Rectangle, 4> rates{};
 Rectangle turbo{};
};

const char* view_layer_name(ExperienceViewLayer layer) {
 switch (layer) {
 case ExperienceViewLayer::kPlayer:
  return "PLAYER";
 case ExperienceViewLayer::kLearning:
  return "LEARNING";
 case ExperienceViewLayer::kEngineering:
  return "ENGINEERING";
 }
 return "PLAYER";
}

PlaybackHitRegions playback_hit_regions() {
 PlaybackHitRegions regions;
 constexpr float kYFromBottom = 62.0f;
 constexpr float kX = 12.0f;
 constexpr float kWidth = 46.0f;
 constexpr float kGap = 6.0f;
 const float y = static_cast<float>(GetScreenHeight()) - kYFromBottom;
 for (std::size_t index = 0; index < regions.rates.size(); ++index) {
 regions.rates[index] = {
 kX + static_cast<float>(index) * (kWidth + kGap), y, kWidth, 30.0f};
 }
 regions.turbo = {
 kX + static_cast<float>(regions.rates.size()) * (kWidth + kGap),
 y, 76.0f, 30.0f};
 return regions;
}

void draw_panel_button(Rectangle bounds, const char* label, bool primary,
 Color accent, Color text, bool enabled = true) {
 const bool hovered = enabled &&
 CheckCollisionPointRec(GetMousePosition(), bounds);
 if (!enabled) {
 accent = Color{70, 88, 84, 255};
 text = Color{112, 127, 121, 255};
 }
 const Color fill = primary
 ? (hovered ? Color{67, 235, 216, 255} : accent)
 : (hovered ? Color{49, 72, 71, 255} : Color{29, 48, 48, 255});
 DrawRectangleRounded(bounds, 0.16f, 8, fill);
 if (!primary) DrawRectangleRoundedLinesEx(bounds, 0.16f, 8, 1.0f, accent);
 const float text_size = 14.0f * ui_text_scale();
 const float width = measure_ui_text(label, 14.0f);
 draw_ui_text(label,
 static_cast<int>(bounds.x + (bounds.width - width) * 0.5f),
 static_cast<int>(bounds.y + (bounds.height - text_size) * 0.5f), 14,
 primary ? Color{7, 18, 18, 255} : text);
}

void draw_playback_controls(const PlaybackHitRegions& regions,
 const lost_signal::sim::viewer::PlaybackController& playback) {
 constexpr std::array<double, 4> kRates{1.0, 2.0, 4.0, 8.0};
 constexpr std::array<const char*, 4> kLabels{"1x", "2x", "4x", "8x"};
 constexpr Color kCyan{55, 218, 202, 255};
 constexpr Color kText{226, 235, 230, 255};
 for (std::size_t index = 0; index < kRates.size(); ++index) {
 const bool selected = !playback.turbo() &&
 std::abs(playback.requested_rate() - kRates[index]) < 1e-9;
 draw_panel_button(regions.rates[index], kLabels[index], selected,
 kCyan, kText);
 }
 draw_panel_button(regions.turbo, "TURBO", playback.turbo(),
 Color{246, 169, 61, 255}, kText);
}

float draw_wrapped_text(const std::string& text, float x, float y,
 float width, float font_size, float line_gap, Color color) {
 const Font font = ui_font();
 const float text_scale = ui_text_scale();
 font_size *= text_scale;
 line_gap *= text_scale;
 if (std::any_of(text.begin(), text.end(), [](unsigned char value) {
 return value >= 0x80;
 })) {
 const float spacing = 1.0f;
 std::string line;
 for (std::size_t offset = 0; offset < text.size();) {
 int bytes = 0;
 (void)GetCodepoint(text.c_str() + offset, &bytes);
 if (bytes <= 0) bytes = 1;
 const std::size_t glyph_bytes = std::min<std::size_t>(
 static_cast<std::size_t>(bytes), text.size() - offset);
 const std::string glyph = text.substr(offset, glyph_bytes);
 offset += glyph_bytes;
 const std::string candidate = line + glyph;
 if (!line.empty() &&
 MeasureTextEx(font, candidate.c_str(), font_size, spacing).x > width) {
 DrawTextEx(font, line.c_str(), {x, y}, font_size, spacing, color);
 y += font_size + line_gap;
 line = glyph;
 } else {
 line = candidate;
 }
 }
 if (!line.empty()) {
 DrawTextEx(font, line.c_str(), {x, y}, font_size, spacing, color);
 y += font_size + line_gap;
 }
 return y;
 }
 std::istringstream input(text);
 std::string word;
 std::string line;
 const float spacing = 1.0f;
 while (input >> word) {
 const std::string candidate = line.empty() ? word : line + " " + word;
 if (!line.empty() &&
 MeasureTextEx(font, candidate.c_str(), font_size, spacing).x > width) {
 DrawTextEx(font, line.c_str(), {x, y}, font_size, spacing, color);
 y += font_size + line_gap;
 line = word;
 } else {
 line = candidate;
 }
 }
 if (!line.empty()) {
 DrawTextEx(font, line.c_str(), {x, y}, font_size, spacing, color);
 y += font_size + line_gap;
 }
 return y;
}

void draw_hero_overlay(const ExperienceManifest& experience,
 const ExperienceHeroShot& shot, const HeroSequence& sequence) {
 constexpr Color kText{236, 244, 240, 255};
 constexpr Color kMuted{166, 186, 178, 255};
 constexpr Color kCyan{55, 218, 202, 255};
 const float screen_width = static_cast<float>(GetScreenWidth());
 const float screen_height = static_cast<float>(GetScreenHeight());
 const float bar_height = std::clamp(screen_height * 0.075f, 42.0f, 72.0f);
 DrawRectangle(0, 0, GetScreenWidth(), static_cast<int>(bar_height),
 Color{3, 8, 9, 238});
 DrawRectangle(0, static_cast<int>(screen_height - bar_height),
 GetScreenWidth(), static_cast<int>(bar_height), Color{3, 8, 9, 238});

 const std::string chapter_label =
     std::string(g_zh_cn ? "失落信号 / " : "LOST SIGNAL / ") +
     experience.experience_id;
 draw_ui_text(chapter_label.c_str(), 28, 20, 14, kCyan);
 draw_ui_text(tr("REAL PHYSICS / BODY + JOINT POSE",
 "真实物理 / 刚体与关节姿态"), 28,
 static_cast<int>(screen_height - bar_height + 18.0f), 13, kCyan);
 draw_ui_text(tr("ENTER / SPACE / CLICK  SKIP",
 "回车 / 空格 / 点击  跳过"),
 static_cast<int>(screen_width - 220.0f), 20, 12, kMuted);

 const float card_width = std::min(660.0f, screen_width - 56.0f);
 const float card_y = screen_height - bar_height - 155.0f;
 DrawRectangleRounded({28.0f, card_y, card_width, 126.0f},
 0.035f, 10, Color{5, 13, 14, 210});
 draw_ui_text(shot.title.c_str(), 48, static_cast<int>(card_y + 20.0f),
 23, kText);
 draw_wrapped_text(shot.caption, 48.0f, card_y + 54.0f,
 card_width - 40.0f, 14.0f, 4.0f, kMuted);

 const std::size_t shot_count = experience.intro.size();
 const float gap = 7.0f;
 const float segment_width = (card_width -
 gap * static_cast<float>(shot_count > 0 ? shot_count - 1 : 0)) /
 static_cast<float>(std::max<std::size_t>(1, shot_count));
 for (std::size_t index = 0; index < shot_count; ++index) {
 const float progress = index < sequence.shot_index()
 ? 1.0f
 : index == sequence.shot_index()
 ? static_cast<float>(sequence.shot_progress()) : 0.0f;
 const Rectangle track{28.0f + index * (segment_width + gap),
 bar_height - 7.0f, segment_width, 3.0f};
 DrawRectangleRec(track, Color{75, 96, 89, 180});
 DrawRectangleRec({track.x, track.y, track.width * progress, track.height},
 kCyan);
 }

 const double progress = sequence.total_progress();
 const double fade = progress < 0.08
 ? 1.0 - progress / 0.08
 : progress > 0.96 ? (progress - 0.96) / 0.04 : 0.0;
 if (fade > 0.0) {
 DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(),
 Color{0, 0, 0, static_cast<unsigned char>(
 std::clamp(fade, 0.0, 1.0) * 255.0)});
 }
}

// Map extents over every authored layer (auto-fit on --map load).
void setup_demo(lost_signal::sim::scenario::ScenarioSession& session) {
  session.setup_world().set_static_boxes({
      {5.2, 1.0, 6.0, 1.8, 0.0},
      {2.5, 5.0, 3.3, 5.8, 0.0},
      {-2.4, 2.0, -1.6, 2.8, 0.0},
      {1.0, -2.4, 1.8, -1.6, 0.0},
  });
  session.setup_world().add_place({"loopA", 4.0, 0.0, 0.0});
  session.setup_world().add_place({"loopB", 4.0, 4.0, 0.0});
  session.setup_world().add_place({"loopC", 0.0, 4.0, 0.0});
  session.setup_world().add_place({"loopD", 0.0, 0.0, 0.0});
  session.set_objectives({"loopA", "loopB", "loopC", "loopD"});
  lost_signal::robot::iface::TaskRequest req;
  req.request_id = "view";
  req.place_id = "loopA";
  session.request_goal(req);
}

}  // namespace

static lost_signal::sim::viewer::ExperienceFlow::Config experience_flow_config(
 const lost_signal::sim::viewer::ExperienceManifest& experience) {
 lost_signal::sim::viewer::ExperienceFlow::Config config;
 config.walkaround_count = experience.walkaround.size();
 config.choice_count = experience.choices.size();
 config.time_limit_steps = experience.time_limit_steps;
 config.required_walkaround.reserve(experience.walkaround.size());
 for (const auto& stop : experience.walkaround) {
 config.required_walkaround.push_back(stop.required);
 }
 if (!experience.experiments.empty()) {
 config.variant_time_limit_steps = static_cast<std::uint64_t>(
 experience.experiments.front().step_budget);
 }
 return config;
}

static HeroSequence hero_sequence_for(const ExperienceManifest& experience) {
 std::vector<double> durations;
 durations.reserve(experience.intro.size());
 for (const auto& shot : experience.intro) {
 durations.push_back(shot.duration_s);
 }
 return HeroSequence(std::move(durations));
}

int lost_signal::sim::viewer::run_viewer(int argc, char** argv) {
  using namespace lost_signal;
 sim::viewer::ViewerOptions options;
 std::string option_error;
 if (!sim::viewer::parse_viewer_options(argc, argv, options, &option_error)) {
   std::fprintf(stderr, "%s\n", option_error.c_str());
   return 1;
 }
 if (options.help_requested) {
   sim::viewer::print_viewer_usage();
   return 0;
 }
 std::string robot_path = options.robot_path;
 if (!options.chassis_kind.empty()) {
   robot_path = (std::filesystem::path("configs/robot") /
                 (options.chassis_kind + ".json")).string();
 }
 std::string sim_path = options.sim_path;
 std::string map_path = options.map_path;
 std::string assets_path = options.assets_path;
 std::string scene_path = options.scene_path;
 std::string experience_path = options.experience_path;
 std::string experiment_output_path = options.experiment_output_path;
 std::string chapter_id = options.chapter_id;
 std::string profile_path = options.profile_path;
 const std::string requested_locale = options.requested_locale;
 const double initial_playback_rate = options.initial_playback_rate;
 const double sandbox_history_seconds = options.sandbox_history_seconds;
 const bool initial_turbo = options.initial_turbo;
 const bool initial_maximized = options.initial_maximized;
 RenderQuality render_quality = options.render_quality;
 const bool sandbox_requested = options.sandbox_requested;
 const bool play_requested = options.play_requested;
 const bool source_requested = options.source_requested;
 const bool mute_audio = options.mute_audio;
 const bool initial_show_lidar = options.initial_show_lidar;
 const bool initial_show_robot_evidence = options.initial_show_robot_evidence;
 const bool list_worlds = options.list_worlds;
 const bool list_experiences = options.list_experiences;
 const bool catalog_requested = options.catalog_requested;
 const bool initial_physics_overlay = options.initial_physics_overlay;
 if (!chapter_id.empty()) {
 std::filesystem::path chapter_path(chapter_id);
 // The short form is deliberately resolved against the authored catalog;
 // arbitrary paths remain available through --experience for engineering
 // replay. This keeps the chapter picker discoverable and deterministic.
 if (chapter_path.extension() != ".json") {
 chapter_path = std::filesystem::path("configs/experiences") /
 (chapter_id + ".json");
 }
 if (!std::filesystem::is_regular_file(chapter_path)) {
 std::fprintf(stderr, "unknown chapter: %s\n", chapter_id.c_str());
 return 1;
 }
 experience_path = chapter_path.string();
 }
 if (list_worlds || list_experiences) {
 if (list_worlds) {
 std::vector<std::filesystem::path> worlds;
 const std::filesystem::path root = "configs/worlds";
 if (std::filesystem::is_directory(root)) {
 for (const auto& entry : std::filesystem::directory_iterator(root)) {
 if (!entry.is_regular_file() || entry.path().extension() != ".json" ||
 entry.path().filename().string().find(".actors.") != std::string::npos ||
 entry.path().filename().string().find(".scene.") != std::string::npos) {
 continue;
 }
 worlds.push_back(entry.path());
 }
 }
 std::sort(worlds.begin(), worlds.end());
 for (const auto& path : worlds) {
 sim::eval::WorldFileContent world;
 std::string error;
 if (!sim::eval::load_world_file(path.string(), world, &error)) {
 std::fprintf(stderr, "WORLD INVALID %s: %s\n",
 path.string().c_str(), error.c_str());
 continue;
 }
	 const char* lab = !world.patches.empty() && !world.zones.empty()
	 ? "terrain/material" : !world.patches.empty() ? "terrain"
	 : !world.zones.empty() ? "material" : "geometry";
	 const std::filesystem::path authored_scene =
	 path.parent_path() / (path.stem().string() + ".scene.json");
	 const bool has_authored_scene =
	 std::filesystem::is_regular_file(authored_scene);
	 const bool has_fallback_scene = std::filesystem::is_regular_file(
	 path.parent_path() / "lab_default.scene.json");
	 const char* scene_kind = has_authored_scene ? "authored"
	 : has_fallback_scene ? "fallback" : "none";
	 std::printf("WORLD %-24s kind=%-16s places=%zu boxes=%zu terrain=%zu actors=%s scene=%s\n",
	 path.stem().string().c_str(), lab, world.places.size(), world.boxes.size(),
	 world.patches.size(),
 std::filesystem::is_regular_file(path.parent_path() /
 (path.stem().string() + ".actors.json")) ? "yes" : "no",
	 scene_kind);
 }
 }
 if (list_experiences) {
 std::vector<std::filesystem::path> experiences;
 const std::filesystem::path root = "configs/experiences";
 if (std::filesystem::is_directory(root)) {
 for (const auto& entry : std::filesystem::directory_iterator(root)) {
 if (entry.is_regular_file() && entry.path().extension() == ".json") {
 experiences.push_back(entry.path());
 }
 }
 }
 std::sort(experiences.begin(), experiences.end());
 for (const auto& path : experiences) {
 sim::viewer::ExperienceManifest manifest;
 std::string error;
 if (!sim::viewer::load_experience_manifest(path.string(), manifest,
 &error)) {
 std::fprintf(stderr, "EXPERIENCE INVALID %s: %s\n",
 path.string().c_str(), error.c_str());
 continue;
 }
 std::printf("EXPERIENCE %-24s objectives=%zu title=%s\n",
 manifest.experience_id.c_str(), manifest.objectives.size(),
 manifest.title.c_str());
 }
 }
 if (catalog_requested) {
 std::puts("SANDBOX manual_control       mode=operator body_command=WASD/arrows "
           "arm_effort=I/K O/L P/; [/'] safety=validator+raw_scan+watchdog");
 }
 return 0;
 }
 if (requested_locale != "auto" && requested_locale != "en" &&
 requested_locale != "zh-CN") {
 std::fprintf(stderr, "bad --locale: expected auto, en, or zh-CN\n");
 return 1;
 }
 g_zh_cn = use_zh_cn_locale(requested_locale);
 // Explicit source selections (`--experience`, `--chapter`, or `--map`) own
 // the runtime. `--play` is only the convenient default when no source was
 // supplied; previously it silently replaced an explicit experience with
 // `first_contact`, so `--experience signal_town_big --play` appeared to
 // stall in the wrong chapter and map.
 if (experience_path.empty() && !source_requested &&
 (play_requested || !sandbox_requested)) {
 experience_path = g_zh_cn
 ? "configs/experiences/zh-CN/first_contact.json"
 : "configs/experiences/first_contact.json";
 }
 ExperienceManifest experience;
 bool experience_loaded = false;
 if (!experience_path.empty()) {
 std::string experience_error;
 if (!lost_signal::sim::viewer::load_experience_manifest(
 experience_path, experience, &experience_error)) {
 std::fprintf(stderr, "bad experience file: %s\n",
 experience_error.c_str());
 return 1;
 }
 experience_loaded = true;
 if (robot_path.empty()) robot_path = experience.robot_path;
 if (sim_path.empty()) sim_path = experience.sim_path;
 if (map_path.empty()) map_path = experience.map_path;
 if (scene_path.empty()) scene_path = experience.scene_path;
 std::fprintf(stderr, "experience: %s\n", experience.experience_id.c_str());
 }
 PlayerProfile player_profile;
 std::string player_profile_error;
 if (experience_loaded) {
   if (profile_path.empty()) {
     profile_path = lost_signal::sim::viewer::default_player_profile_path();
   }
   if (!lost_signal::sim::viewer::load_player_profile(
           profile_path, player_profile, &player_profile_error)) {
     // A corrupt profile is deliberately non-fatal: load_player_profile has
     // already returned an empty profile, so the player can continue and the
     // next published verdict can replace it atomically.
     std::fprintf(stderr, "player profile reset: %s (%s)\n",
                  profile_path.c_str(), player_profile_error.c_str());
   }
 }
 std::vector<ChapterCatalogEntry> chapter_catalog;
 if (experience_loaded) {
   chapter_catalog = load_chapter_catalog(player_profile, experience);
 }
 if (assets_path.empty() && std::filesystem::is_directory("assets")) {
 assets_path = "assets";
 }
 // A direct map launch is still a complete SIM runtime. When the operator
 // does not select a robot or SIM profile, use the same nominal defaults as
 // the sandbox instead of passing empty required inputs to the invariant
 // digest builder.
 if (!experience_loaded && !map_path.empty()) {
   if (robot_path.empty()) robot_path = "configs/robot/differential.json";
   if (sim_path.empty()) sim_path = "configs/sim/default.json";
 }
 std::optional<robot::platform::RobotConfig> robot_config;
 std::optional<sim::eval::SimConfig> sim_config;
  std::unique_ptr<sim::scenario::ScenarioSession> session;
 std::uint64_t runtime_invariant_digest = 0;
 std::string runtime_code_revision;
 sim::experiment::ExperienceRuntimeConfig runtime_config;
 if (experience_loaded) {
 runtime_config = sim::experiment::experience_runtime_config(
 experience, !assets_path.empty()
 ? (std::filesystem::path(assets_path) / "models/manifest.json").string()
 : std::string{});
 // Explicit CLI paths, when supplied, already replaced the corresponding
 // manifest defaults above. Preserve those visible user selections.
 runtime_config.robot_path = robot_path;
 runtime_config.sim_path = sim_path;
 runtime_config.map_path = map_path;
 runtime_config.scene_path = scene_path;
 } else {
 runtime_config.robot_path = robot_path;
 runtime_config.sim_path = sim_path;
 runtime_config.map_path = map_path;
 runtime_config.scene_path = scene_path;
 if (!assets_path.empty()) {
 runtime_config.asset_manifest_path =
 (std::filesystem::path(assets_path) / "models/manifest.json").string();
 }
 }
  if (!map_path.empty()) {
 sim::experiment::ExperienceRuntime runtime;
 std::string runtime_error;
 if (!sim::experiment::build_experience_runtime(
 runtime_config, runtime, &runtime_error)) {
 std::fprintf(stderr, "cannot build simulation: %s\n",
 runtime_error.c_str());
      return 1;
    }
 robot_config = std::move(runtime.robot_config);
 sim_config = std::move(runtime.sim_config);
 runtime_invariant_digest = runtime.invariant_digest;
 runtime_code_revision = runtime.code_revision;
 if (!runtime.used_robot_path.empty()) {
 std::fprintf(stderr, "robot config: %s\n",
 runtime.used_robot_path.c_str());
 }
 if (!runtime.used_sim_path.empty()) {
 std::fprintf(stderr, "sim config: %s\n", runtime.used_sim_path.c_str());
 }
 session = std::move(runtime.session);
  } else {
 std::string used_config, config_error;
 robot_config = robot::platform::load_default_robot_config(
 robot_path, &used_config, &config_error);
 if (!robot_path.empty() && !robot_config.has_value()) {
 std::fprintf(stderr, "bad --robot file: %s\n", config_error.c_str());
 return 1;
 }
 if (robot_config.has_value()) {
 session = std::make_unique<sim::scenario::ScenarioSession>(
 robot::platform::description_from_config(*robot_config),
 sim::eval::motor_from_config(*robot_config));
 } else {
 session = std::make_unique<sim::scenario::ScenarioSession>();
 }
 sim_config = sim::eval::load_default_sim_config(
 sim_path, nullptr, &config_error);
 if (!sim_path.empty() && !sim_config.has_value()) {
 std::fprintf(stderr, "bad --sim file: %s\n", config_error.c_str());
 return 1;
 }
 if (sim_config.has_value()) session->apply_sim_config(*sim_config);
    setup_demo(*session);
 session->prepare_world();
  }
 // Applied lidar mount for the viz ray fan: read back from the session
 // (the exact extrinsics generation runs — never a separately authored
 // copy, so a misconfigured mount shows up instead of hiding).
 sim::viewer::LidarMount session_lidar_mount;
 session->lidar_mount(session_lidar_mount.xyz_m, session_lidar_mount.rpy_rad);

 // A deque-backed recent window keeps Turbo and 64x sandbox sessions bounded
 // without changing full-run scoring. Zero explicitly requests full history.
 if (!experience_loaded && sandbox_history_seconds > 0.0) {
 constexpr double transition_s =
 static_cast<double>(sim::kScenarioTransitionNs) /
 static_cast<double>(sim::kNanosecondsPerSecond);
 const double record_period_s = sim_config.has_value()
 ? std::max(transition_s, sim_config->record_period_s) : transition_s;
 const double requested_steps = std::ceil(
 sandbox_history_seconds / record_period_s);
 const double size_limit = static_cast<double>(
 std::numeric_limits<std::size_t>::max());
 const std::size_t history_steps = requested_steps >= size_limit
 ? std::numeric_limits<std::size_t>::max()
 : static_cast<std::size_t>(std::max(1.0, requested_steps));
 session->set_episode_history_limit(history_steps);
 std::fprintf(stderr,
 "viewer history: recent %.1f sim seconds (%zu records); full score retained\n",
 sandbox_history_seconds, history_steps);
 }

 // Raylib cannot create a window without a display backend and may fail
 // inside GLFW before returning a useful error. Fail at the application
 // boundary so headless users get an actionable message instead of a crash;
 // deterministic headless simulation remains available through lost_signal_sim.
 if (std::getenv("DISPLAY") == nullptr &&
     std::getenv("WAYLAND_DISPLAY") == nullptr) {
   std::fprintf(stderr,
                "sim_viewer needs DISPLAY or WAYLAND_DISPLAY; use xvfb-run "
                "for a headless 3D smoke test\n");
   return 1;
 }

 unsigned int window_flags = FLAG_WINDOW_RESIZABLE;
 if (initial_maximized) window_flags |= FLAG_WINDOW_MAXIMIZED;
 SetConfigFlags(window_flags);
 InitWindow(kScreenW, kScreenH, experience_loaded
 ? tr("Robot Simu - First Contact", "Robot Simu - 初次接触")
 : tr("Robot Simu - Simulation Lab", "Robot Simu - 仿真实验室"));
 SetWindowMinSize(kMinimumScreenW, kMinimumScreenH);
 if (initial_maximized) MaximizeWindow();
 load_ui_font(experience_loaded ? &experience : nullptr);
 sim::viewer::ViewerAudio viewer_audio(assets_path, !mute_audio);
 VisualAssets visual_assets;
 load_visual_assets(visual_assets, assets_path,
 robot_config.has_value() ? robot_config->robot_id : "ro1",
 robot_config.has_value() ? robot_config->profile.chassis_kind
 : "differential",
  experience_loaded && experience.has_assembly ? &experience.assembly
                                               : nullptr);
 SceneManifest visual_scene;
 bool scene_loaded = false;
 if (scene_path.empty() && !map_path.empty()) {
 std::filesystem::path candidate(map_path);
 candidate.replace_extension(".scene.json");
 if (std::filesystem::is_regular_file(candidate)) {
 scene_path = candidate.string();
	 } else {
	 // Every authored world gets the same deterministic presentation fallback.
	 // A missing decorative scene must not silently drop the 3D camera, floor,
	 // lighting, particles, or physical model bindings.
	 const std::filesystem::path fallback =
	 std::filesystem::path("configs/worlds/lab_default.scene.json");
	 if (std::filesystem::is_regular_file(fallback)) {
	 scene_path = fallback.string();
	 }
 }
 }
 if (!scene_path.empty()) {
 std::string scene_error;
 if (!lost_signal::sim::viewer::load_scene_manifest(
 scene_path, visual_assets.manifest,
 session->setup_world().static_boxes(), visual_scene, &scene_error)) {
 std::fprintf(stderr, "bad scene file: %s\n", scene_error.c_str());
 unload_visual_assets(visual_assets);
 unload_ui_font();
 CloseWindow();
 return 1;
 }
 scene_loaded = true;
 std::fprintf(stderr, "scene: %s (%zu instances)\n", scene_path.c_str(),
 visual_scene.instances.size());
 }
  SetTargetFPS(60);

  sim::viewer::ViewTransform view(50.0f, 2.0, 1.0, kScreenW, kScreenH);
  view.fit(-4.0, -4.0, 8.0, 7.0);
 bool paused = experience_loaded;
 std::unique_ptr<ExperienceFlow> experience_flow;
 HeroSequence hero_sequence;
 if (experience_loaded) {
 experience_flow = std::make_unique<ExperienceFlow>(
 experience_flow_config(experience));
 hero_sequence = hero_sequence_for(experience);
 }
 ExperienceHitRegions experience_hit_regions;
 std::vector<ReviewKeyframe> review_keyframes;
 int selected_review_keyframe = 0;
 ExperimentUiState experiment_ui;
bool show_lidar = initial_show_lidar;
bool show_estimate = initial_show_robot_evidence;
 bool show_actor_routes = false;
 bool show_physics_overlay = initial_physics_overlay;
 ExperienceViewLayer view_layer = experience_loaded
 ? ExperienceViewLayer::kPlayer : ExperienceViewLayer::kEngineering;
 bool show_catalog = false;
  bool follow = true;
 // Manual control is an explicit sandbox mode. It is deliberately kept out
 // of authored experiences so a teaching episode remains reproducible. Keys
 // are sampled before each deterministic step and the resulting body command
 // still passes through ROBOT validation and Safety.
 bool manual_control = false;
 // Drive deterministic 20 ms simulation time from elapsed wall time. The
 // old fixed four-steps-per-frame loop demanded 4.8x real time at 60 FPS and
 // stacked periodic localization/planning work into the window thread.
 sim::viewer::PlaybackController::Config playback_config;
 playback_config.requested_rate = initial_playback_rate;
 sim::viewer::PlaybackController playback(playback_config);
 playback.set_turbo(initial_turbo);
 double previous_wall_s = GetTime();
  Vector2 last_mouse{0, 0};
 bool dragging = false;
 bool middle_dragging = false;
 Vector2 last_middle_mouse{0, 0};
  // Map editor: entering edit mode snapshots the world and pauses the sim;
  // leaving keeps the applied geometry.
  sim::viewer::MapEditor editor;
  bool edit_mode = false;
  std::string edit_msg;
 // 3D orbit camera over the same frame (V toggles 2D/3D, --3d starts in 3D).
 bool view3d = experience_loaded || options.view3d;
 // --shot N path: render N frames, screenshot, exit (visual smoke check).
 int shot_frames = options.shot_frames;
 std::string shot_path = options.shot_path;
 float camera_dist = options.camera_dist;
 float camera_yaw = options.camera_yaw;
 float camera_pitch = options.camera_pitch;
 bool camera_angle_set = options.camera_angle_set;
 bool camera_target_set = options.camera_target_set;
 double camera_target_x = options.camera_target_x;
 double camera_target_y = options.camera_target_y;
  CameraState cam_st;
  if (!map_path.empty()) {
    // Frame the authored map instead of the demo window.
    double bx0, by0, bx1, by1;
    map_bounds(session->setup_world(), bx0, by0, bx1, by1);
    view.fit(bx0 - 2.0, by0 - 2.0, bx1 + 2.0, by1 + 2.0);
    cam_st.target_x = (bx0 + bx1) * 0.5;
    cam_st.target_y = (by0 + by1) * 0.5;
    cam_st.dist_m = view3d ? 4.2f : static_cast<float>(
        std::min(60.0, std::max(10.0, std::max(bx1 - bx0, by1 - by0) * 0.9)));
    cam_st.pitch_deg = view3d ? 34.0f : cam_st.pitch_deg;
    cam_st.target_z = view3d ? 0.18 : 0.0;
    follow = view3d;
 } else {
 cam_st.dist_m = 5.0f;
 cam_st.pitch_deg = 35.0f;
 cam_st.target_z = 0.20;
 }
 if (scene_loaded) {
 cam_st.dist_m = static_cast<float>(visual_scene.camera.distance_m);
 cam_st.yaw_deg = static_cast<float>(visual_scene.camera.yaw_deg);
 cam_st.pitch_deg = static_cast<float>(visual_scene.camera.pitch_deg);
 cam_st.target_z = visual_scene.camera.target_height_m;
 }
 if (camera_dist > 0.0f) cam_st.dist_m = camera_dist;
 if (camera_angle_set) {
  if (camera_yaw != 0.0f) cam_st.yaw_deg = camera_yaw;
  if (camera_pitch != 0.0f) cam_st.pitch_deg = camera_pitch;
 }
 if (camera_target_set) {
 cam_st.target_x = camera_target_x;
 cam_st.target_y = camera_target_y;
 follow = false;
 } else if (follow) {
 // A 3D follow view must start on the robot immediately. Starting at the
 // authored map center and easing toward the robot left it off-screen for
 // several seconds on large maps, even though the HUD already said FOLLOW.
 cam_st.target_x = session->setup_world().truth().body_x_m;
 cam_st.target_y = session->setup_world().truth().body_y_m;
 }
 float follow_base_distance_m = cam_st.dist_m;
 ParticleSystem motion_particles;
 SurfaceTraceSystem surface_traces;
 std::optional<std::size_t> effects_episode_cursor;
 std::string experience_action_error;
 const auto reset_experience_presentation = [&](bool reset_progress = true) {
 if (reset_progress) {
 experience_flow = std::make_unique<ExperienceFlow>(
 experience_flow_config(experience));
 hero_sequence = hero_sequence_for(experience);
 view_layer = ExperienceViewLayer::kPlayer;
 show_catalog = false;
 experiment_ui = ExperimentUiState{};
 }
 review_keyframes.clear();
 selected_review_keyframe = 0;
 experience_hit_regions.clear();
 motion_particles.clear();
 surface_traces.clear();
 effects_episode_cursor.reset();
 viewer_audio.reset();
 const double requested_rate = playback.requested_rate();
 const bool turbo = playback.turbo();
 auto reset_playback_config = playback_config;
 reset_playback_config.requested_rate = requested_rate;
 playback = sim::viewer::PlaybackController(reset_playback_config);
 playback.set_turbo(turbo);
 previous_wall_s = GetTime();
 paused = true;
 show_lidar = false;
 show_estimate = false;
 show_actor_routes = false;
 show_physics_overlay = initial_physics_overlay;
 dragging = false;
 middle_dragging = false;
 follow = view3d;

 cam_st = CameraState{};
 double bx0, by0, bx1, by1;
 map_bounds(session->setup_world(), bx0, by0, bx1, by1);
 view.fit(bx0 - 2.0, by0 - 2.0, bx1 + 2.0, by1 + 2.0);
 cam_st.target_x = (bx0 + bx1) * 0.5;
 cam_st.target_y = (by0 + by1) * 0.5;
 cam_st.dist_m = view3d ? 4.2f : static_cast<float>(
 std::min(60.0, std::max(10.0,
 std::max(bx1 - bx0, by1 - by0) * 0.9)));
 cam_st.pitch_deg = view3d ? 34.0f : cam_st.pitch_deg;
 cam_st.target_z = view3d ? 0.18 : 0.0;
 if (scene_loaded) {
 cam_st.dist_m = static_cast<float>(visual_scene.camera.distance_m);
 cam_st.yaw_deg = static_cast<float>(visual_scene.camera.yaw_deg);
 cam_st.pitch_deg = static_cast<float>(visual_scene.camera.pitch_deg);
 cam_st.target_z = visual_scene.camera.target_height_m;
 }
 if (camera_dist > 0.0f) cam_st.dist_m = camera_dist;
 if (camera_angle_set) {
 if (camera_yaw != 0.0f) cam_st.yaw_deg = camera_yaw;
 if (camera_pitch != 0.0f) cam_st.pitch_deg = camera_pitch;
 }
 if (camera_target_set) {
 cam_st.target_x = camera_target_x;
 cam_st.target_y = camera_target_y;
 follow = false;
 } else {
 cam_st.target_x = session->setup_world().truth().body_x_m;
 cam_st.target_y = session->setup_world().truth().body_y_m;
 }
 follow_base_distance_m = cam_st.dist_m;
 };
 const auto retry_experience = [&]() {
 if (!experience_loaded || runtime_config.map_path.empty() ||
 experience_flow->phase() != ExperiencePhase::kDebrief) {
 return false;
 }
 sim::experiment::ExperienceRuntime rebuilt;
 std::string runtime_error;
 if (!sim::experiment::build_experience_runtime(
 runtime_config, rebuilt, &runtime_error)) {
 experience_action_error = runtime_error;
 std::fprintf(stderr, "cannot retry experience: %s\n",
 runtime_error.c_str());
 return false;
 }
 session = std::move(rebuilt.session);
 robot_config = std::move(rebuilt.robot_config);
 sim_config = std::move(rebuilt.sim_config);
 runtime_invariant_digest = rebuilt.invariant_digest;
 runtime_code_revision = rebuilt.code_revision;
 session->lidar_mount(
 session_lidar_mount.xyz_m, session_lidar_mount.rpy_rad);
 if (!experience_flow->retry_current(0)) return false;
 if (experience_flow->run_kind() == ExperienceRunKind::kBaseline) {
 experiment_ui.baseline = {};
 experiment_ui.episodes.baseline.reset();
 experiment_ui.episodes.variant.reset();
 experiment_ui.baseline_keyframes.clear();
 experiment_ui.variant_keyframes.clear();
 } else {
 experiment_ui.variant = {};
 experiment_ui.episodes.variant.reset();
 experiment_ui.variant_keyframes.clear();
 }
 experiment_ui.standalone.reset();
 experience_action_error.clear();
 reset_experience_presentation(false);
 paused = false;
 return true;
 };

 const auto start_visible_variant = [&]() {
 if (!experience_loaded || experience.experiments.empty() ||
 experience_flow->phase() != ExperiencePhase::kDebrief ||
 experience_flow->run_kind() != ExperienceRunKind::kBaseline ||
 !experiment_ui.baseline.recorded) {
 return false;
 }
 const auto& definition = experience.experiments.front();
 auto variant_config = runtime_config;
 variant_config.variable_override =
     sim::experiment::ExperienceVariableOverride{
         definition.controlled_variable, definition.variant_value,
         definition.variant_category, definition.variant_robot_path};
 sim::experiment::ExperienceRuntime rebuilt;
 std::string runtime_error;
 if (!sim::experiment::build_experience_runtime(
 variant_config, rebuilt, &runtime_error)) {
 experience_action_error = runtime_error;
 return false;
 }
 session = std::move(rebuilt.session);
 robot_config = std::move(rebuilt.robot_config);
 sim_config = std::move(rebuilt.sim_config);
 runtime_invariant_digest = rebuilt.invariant_digest;
 runtime_code_revision = rebuilt.code_revision;
 runtime_config = std::move(variant_config);
 session->lidar_mount(
 session_lidar_mount.xyz_m, session_lidar_mount.rpy_rad);
 if (!experience_flow->start_variant(0)) return false;
 experiment_ui.variant = {};
 experiment_ui.episodes.variant.reset();
 experiment_ui.variant_keyframes.clear();
 experience_action_error.clear();
 reset_experience_presentation(false);
 paused = false;
 return true;
 };

 const auto advance_experience = [&]() {
 if (!experience_loaded || experience.next_experience_path.empty()) {
 return false;
 }
 const sim::experiment::EpisodeArtifact* active_artifact = nullptr;
 if (experience.progression.require_experiment) {
 if (experiment_ui.episodes.variant.has_value()) {
 active_artifact = &*experiment_ui.episodes.variant;
 } else if (experiment_ui.episodes.baseline.has_value()) {
 active_artifact = &*experiment_ui.episodes.baseline;
 }
 } else if (experiment_ui.standalone.has_value()) {
 active_artifact = &*experiment_ui.standalone;
 }
 if (active_artifact == nullptr ||
 !sim::experiment::evaluate_experience_gate(
 experience.progression, *active_artifact,
 experiment_ui.episodes).can_advance) {
 return false;
 }
 ExperienceManifest candidate_experience;
 std::string candidate_error;
 const std::string candidate_path = experience.next_experience_path;
 if (!lost_signal::sim::viewer::load_experience_manifest(
 candidate_path, candidate_experience, &candidate_error)) {
 experience_action_error = candidate_error;
 return false;
 }
 sim::experiment::ExperienceRuntimeConfig candidate_config =
 sim::experiment::experience_runtime_config(
 candidate_experience, runtime_config.asset_manifest_path);
 sim::experiment::ExperienceRuntime candidate_runtime;
 if (!sim::experiment::build_experience_runtime(
 candidate_config, candidate_runtime, &candidate_error)) {
 experience_action_error = candidate_error;
 return false;
 }
 SceneManifest candidate_scene;
 if (!lost_signal::sim::viewer::load_scene_manifest(
 candidate_experience.scene_path, visual_assets.manifest,
 candidate_runtime.session->setup_world().static_boxes(),
 candidate_scene, &candidate_error)) {
 experience_action_error = candidate_error;
 return false;
 }

 experience = std::move(candidate_experience);
 experience_path = candidate_path;
 chapter_catalog = load_chapter_catalog(player_profile, experience);
 runtime_config = std::move(candidate_config);
 robot_path = runtime_config.robot_path;
 sim_path = runtime_config.sim_path;
 map_path = runtime_config.map_path;
 scene_path = experience.scene_path;
 session = std::move(candidate_runtime.session);
 robot_config = std::move(candidate_runtime.robot_config);
 sim_config = std::move(candidate_runtime.sim_config);
 runtime_invariant_digest = candidate_runtime.invariant_digest;
 runtime_code_revision = candidate_runtime.code_revision;
 visual_scene = std::move(candidate_scene);
 scene_loaded = true;
 session->lidar_mount(
 session_lidar_mount.xyz_m, session_lidar_mount.rpy_rad);
 unload_visual_assets(visual_assets);
 load_visual_assets(visual_assets, assets_path,
 robot_config.has_value() ? robot_config->robot_id : "ro1",
 robot_config.has_value() ? robot_config->profile.chassis_kind
 : "differential",
  experience.has_assembly ? &experience.assembly : nullptr);
 load_ui_font(&experience);
 experience_action_error.clear();
 reset_experience_presentation();
 SetWindowTitle(("Robot Simu - " + experience.title).c_str());
 std::fprintf(stderr, "experience: %s\n", experience.experience_id.c_str());
 return true;
 };

 while (!WindowShouldClose()) {
 const double wall_now_s = GetTime();
 // Playback must see the complete wall interval. If the window stalls for
 // longer than one input frame, PlaybackController deliberately drops the
 // excess simulation demand and reports LIMIT/actual RTF against that real
 // interval. Clamping here used to hide long stalls and overstate throughput.
 const double playback_wall_dt_s = std::max(
 0.0, wall_now_s - previous_wall_s);
 // Presentation smoothing is independent of simulation scheduling. Bound it
 // so a debugger pause or window drag cannot snap the follow camera.
 const double presentation_wall_dt_s = std::min(
 playback_wall_dt_s, 0.25);
 previous_wall_s = wall_now_s;
    view.set_screen(GetScreenWidth(), GetScreenHeight());
 if (experience_loaded) hero_sequence.update(playback_wall_dt_s);
 if ((!experience_loaded || !hero_sequence.active()) &&
 IsKeyPressed(KEY_V)) view3d = !view3d;
    // Edit-mode picking: 2D uses the view transform, 3D the ground ray.
    const Vector2 mpos = GetMousePosition();
    double pick_x = 0.0, pick_y = 0.0;
    bool on_map = false;
    if (view3d) {
      const Camera3D pick_cam =
          camera_from_state(cam_st);
      on_map = screen_to_ground_3d(pick_cam, mpos, pick_x,
                                                pick_y);
    } else {
      on_map = view.screen_to_world(mpos.x, mpos.y, pick_x, pick_y);
    }
 bool hero_skip_consumed = false;
 if (experience_loaded) {
 const bool clicked = IsMouseButtonPressed(MOUSE_BUTTON_LEFT);
 const Vector2 pointer = GetMousePosition();
 if (hero_sequence.active()) {
 hero_skip_consumed = clicked || IsKeyPressed(KEY_ENTER) ||
 IsKeyPressed(KEY_SPACE) || IsKeyPressed(KEY_ESCAPE);
 if (hero_skip_consumed) hero_sequence.skip();
 } else if (experience_flow->phase() == ExperiencePhase::kBriefing) {
 const bool hotspot_clicked = clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.world_hotspot);
 if (hotspot_clicked) {
 experience_flow->inspect(experience_flow->walkaround_index());
 } else if (IsKeyPressed(KEY_ENTER) &&
 !experience_flow->walkaround_complete()) {
 // Keyboard/gamepad accessibility path for the same explicit inspection.
 experience_flow->inspect(experience_flow->walkaround_index());
 } else if (IsKeyPressed(KEY_ENTER) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.primary))) {
 experience_flow->continue_to_prediction();
 } else {
 if (clicked) {
 for (std::size_t index = 0;
 index < experience_hit_regions.walkaround.size(); ++index) {
 if (CheckCollisionPointRec(
 pointer, experience_hit_regions.walkaround[index])) {
 experience_flow->focus(index);
 break;
 }
 }
 }
 if (IsKeyPressed(KEY_RIGHT) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.next))) {
 experience_flow->inspect_next();
 }
 if (IsKeyPressed(KEY_LEFT) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.previous))) {
 experience_flow->inspect_previous();
 }
 }
 } else if (experience_flow->phase() == ExperiencePhase::kPrediction) {
 for (std::size_t i = 0; i < experience.choices.size(); ++i) {
 const int key = KEY_ONE + static_cast<int>(i);
 const bool clicked_choice = clicked &&
 i < experience_hit_regions.choices.size() &&
 CheckCollisionPointRec(pointer, experience_hit_regions.choices[i]);
 if (IsKeyPressed(key) || clicked_choice) {
 experience_flow->select_choice(i, session->episode().steps().size());
 paused = false;
 break;
 }
 }
 } else if (experience_flow->phase() == ExperiencePhase::kDebrief) {
 const bool requested_retry = clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.retry);
 if (requested_retry && retry_experience()) {
 continue;
 }
 const bool requested_next = clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.next_chapter);
 if (requested_next && advance_experience()) {
 continue;
 }
 if (clicked) {
 for (const auto& [bounds, keyframe_index] :
 experience_hit_regions.causal_keyframes) {
 if (CheckCollisionPointRec(pointer, bounds)) {
 selected_review_keyframe = static_cast<int>(keyframe_index);
 break;
 }
 }
 }
 if (!review_keyframes.empty()) {
 const int count = static_cast<int>(review_keyframes.size());
 if (IsKeyPressed(KEY_RIGHT) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.next))) {
 selected_review_keyframe = (selected_review_keyframe + 1) % count;
 }
 if (IsKeyPressed(KEY_LEFT) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.previous))) {
 selected_review_keyframe =
 (selected_review_keyframe + count - 1) % count;
 }
 }
 const bool requested_experiment = IsKeyPressed(KEY_A) ||
 (clicked && CheckCollisionPointRec(
 pointer, experience_hit_regions.primary));
 if (requested_experiment && start_visible_variant()) {
 continue;
 }
 }
 }
 // Edit-mode toggle + keys (window thread only).
 if (!experience_loaded && IsKeyPressed(KEY_M)) {
      edit_mode = !edit_mode;
      if (edit_mode) {
        editor.load_from(session->setup_world());
        paused = true;
      } else {
        editor.apply(session->setup_world());
      }
    }
 if (!experience_loaded && IsKeyPressed(KEY_H)) {
 manual_control = !manual_control;
 if (!manual_control) {
   session->clear_operator_command();
   session->clear_operator_arm_effort();
 }
 }
    if (edit_mode) {
      if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT) && on_map) {
        editor.press(pick_x, pick_y);
      }
      if (IsMouseButtonDown(MOUSE_BUTTON_LEFT) && on_map) {
        editor.drag(pick_x, pick_y);
      }
      if (IsMouseButtonReleased(MOUSE_BUTTON_LEFT)) editor.release();
      if (IsKeyPressed(KEY_B) && on_map) editor.add_box(pick_x, pick_y);
      if (IsKeyPressed(KEY_P) && on_map) editor.add_place(pick_x, pick_y);
      if (IsKeyPressed(KEY_DELETE) || IsKeyPressed(KEY_BACKSPACE)) {
        editor.remove_selected();
      }
      if (IsKeyPressed(KEY_S)) {
        editor.save("viewer_map.json");
        edit_msg = editor.message();
      }
      // Live preview: paused sim, so pushing every dirty frame costs only
      // an engine model rebuild per edit, no stepping hazard.
      if (editor.dirty()) editor.apply(session->setup_world());
    }
    // Input (window thread only; sim stays deterministic).
 if (IsKeyPressed(KEY_SPACE) &&
 !hero_skip_consumed &&
 (!experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning)) {
 paused = !paused;
 }
 if (IsKeyPressed(KEY_N) && paused &&
 (!experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning)) {
 session->step_once();
 }
 if (!experience_loaded && manual_control) {
 const double forward =
 (IsKeyDown(KEY_W) || IsKeyDown(KEY_UP) ? 1.0 : 0.0) -
 (IsKeyDown(KEY_S) || IsKeyDown(KEY_DOWN) ? 1.0 : 0.0);
 const double yaw =
 (IsKeyDown(KEY_D) || IsKeyDown(KEY_RIGHT) ? 1.0 : 0.0) -
 (IsKeyDown(KEY_A) || IsKeyDown(KEY_LEFT) ? 1.0 : 0.0);
 const double max_forward = robot_config.has_value()
 ? robot_config->limits.max_forward_mps : 2.0;
 const double max_yaw = robot_config.has_value()
 ? robot_config->limits.max_yaw_rate_radps : 2.0;
 session->set_operator_command(forward * max_forward, yaw * max_yaw);
 if (robot_config.has_value() && robot_config->profile.arm.enabled) {
   // Four arm channels use paired keys. This is an operator refresh, not a
   // direct plant write: RobotApplication applies its encoder freshness and
   // watchdog gates before any effort reaches the SIM actuator boundary.
   std::array<double, 4> arm_effort = {
       (IsKeyDown(KEY_I) ? 1.0 : 0.0) -
           (IsKeyDown(KEY_K) ? 1.0 : 0.0),
       (IsKeyDown(KEY_O) ? 1.0 : 0.0) -
           (IsKeyDown(KEY_L) ? 1.0 : 0.0),
       (IsKeyDown(KEY_P) ? 1.0 : 0.0) -
           (IsKeyDown(KEY_SEMICOLON) ? 1.0 : 0.0),
       (IsKeyDown(KEY_LEFT_BRACKET) ? 1.0 : 0.0) -
           (IsKeyDown(KEY_APOSTROPHE) ? 1.0 : 0.0)};
   session->set_operator_arm_effort(4, arm_effort);
 } else {
   session->clear_operator_arm_effort();
 }
 } else if (!experience_loaded) {
 session->clear_operator_command();
 session->clear_operator_arm_effort();
 }
    if (IsKeyPressed(KEY_L)) show_lidar = !show_lidar;
    if (IsKeyPressed(KEY_E)) show_estimate = !show_estimate;
 if ((!experience_loaded || !hero_sequence.active()) &&
 IsKeyPressed(KEY_U)) {
 view_layer = lost_signal::sim::viewer::next_view_layer(view_layer);
 }
 if (experience_loaded && !hero_sequence.active() && IsKeyPressed(KEY_C)) {
   show_catalog = !show_catalog;
 }
 const bool sim_truth_debug_allowed = !experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kDebrief;
 if (!sim_truth_debug_allowed) {
 show_actor_routes = false;
 show_physics_overlay = false;
 }
 if (IsKeyPressed(KEY_K) && sim_truth_debug_allowed) {
 const bool enabled = !(show_actor_routes || show_physics_overlay);
 show_actor_routes = enabled;
 show_physics_overlay = enabled;
 }
 if (IsKeyPressed(KEY_Q)) {
 render_quality = render_quality == RenderQuality::kHigh
 ? RenderQuality::kFast : RenderQuality::kHigh;
 }
    if (IsKeyPressed(KEY_F)) follow = !follow;
 if (IsKeyPressed(KEY_R)) {
 cam_st.yaw_deg = 35.0f;
 cam_st.pitch_deg = 50.0f;
 cam_st.dist_m = 12.0f;
 follow = false;
 }
 if (IsKeyPressed(KEY_T)) playback.set_turbo(!playback.turbo());
 if (IsKeyPressed(KEY_F10)) {
 if (IsWindowMaximized()) RestoreWindow();
 else MaximizeWindow();
 }
 if (IsKeyPressed(KEY_F11)) ToggleFullscreen();
 const bool playback_controls_visible = !experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning;
 if (playback_controls_visible &&
 IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
 const PlaybackHitRegions controls = playback_hit_regions();
 const Vector2 pointer = GetMousePosition();
 constexpr std::array<double, 4> kRates{1.0, 2.0, 4.0, 8.0};
 for (std::size_t index = 0; index < controls.rates.size(); ++index) {
 if (!CheckCollisionPointRec(pointer, controls.rates[index])) continue;
 playback.set_turbo(false);
 playback.set_requested_rate(kRates[index]);
 break;
 }
 if (CheckCollisionPointRec(pointer, controls.turbo)) {
 playback.set_turbo(true);
 }
 }
 if (IsKeyPressed(KEY_EQUAL) && playback.requested_rate() < 64.0) {
 playback.set_turbo(false);
 playback.set_requested_rate(
 std::min(64.0, playback.requested_rate() * 2.0));
 }
 if (IsKeyPressed(KEY_MINUS) && playback.requested_rate() > 0.125) {
 playback.set_turbo(false);
 playback.set_requested_rate(
 std::max(0.125, playback.requested_rate() * 0.5));
 }
    if (IsKeyPressed(KEY_ZERO)) view.fit(-4.0, -4.0, 8.0, 7.0);
    const float wheel = GetMouseWheelMove();
    if (view3d) {
      if (wheel != 0.0f) {
        cam_st.dist_m *= wheel > 0.0f ? 1.0f / 1.15f : 1.15f;
 if (cam_st.dist_m < 0.6f) cam_st.dist_m = 0.6f;
        if (cam_st.dist_m > 60.0f) cam_st.dist_m = 60.0f;
 follow_base_distance_m = cam_st.dist_m;
      }
    } else if (wheel != 0.0f) {
      view.set_screen(GetScreenWidth(), GetScreenHeight());
      view.zoom_at(GetMouseX(), GetMouseY(),
                   wheel > 0.0f ? 1.15f : 1.0f / 1.15f);
    }
    if (IsMouseButtonDown(MOUSE_BUTTON_RIGHT)) {
      const Vector2 m = GetMousePosition();
      if (dragging) {
        if (view3d) {
          cam_st.yaw_deg += (m.x - last_mouse.x) * 0.3f;
          cam_st.pitch_deg += (last_mouse.y - m.y) * 0.3f;
          if (cam_st.pitch_deg < 10.0f) cam_st.pitch_deg = 10.0f;
          if (cam_st.pitch_deg > 85.0f) cam_st.pitch_deg = 85.0f;
        } else {
          view.set_screen(GetScreenWidth(), GetScreenHeight());
          view.pan_px(last_mouse.x - m.x, last_mouse.y - m.y);
        }
        follow = false;
      }
      last_mouse = m;
      dragging = true;
    } else {
      dragging = false;
    }
 if (view3d && IsMouseButtonDown(MOUSE_BUTTON_MIDDLE)) {
 const Vector2 m = GetMousePosition();
 if (middle_dragging) {
 const float scale = cam_st.dist_m * 0.0025f;
 cam_st.target_x -= (m.x - last_middle_mouse.x) * scale;
 cam_st.target_y += (m.y - last_middle_mouse.y) * scale;
 follow = false;
 }
 last_middle_mouse = m;
 middle_dragging = true;
 } else {
 middle_dragging = false;
 }

 playback.begin_frame(playback_wall_dt_s);
 // An open --map/--sandbox session has no authored objective list, so the
 // ObjectiveDriver quite correctly reports "done" at construction time. That
 // is a terminal state only for an ExperienceRuntime chapter. Direct map
 // viewing must continue to advance the plant until the operator closes the
 // window; otherwise every engineering map appears frozen at its preview.
 if (!paused && (!experience_loaded || !session->objectives_done())) {
 const auto frame_work_begin = std::chrono::steady_clock::now();
 while (true) {
 const double work_ms = std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - frame_work_begin).count();
 if (!playback.step_due(work_ms)) break;
 const auto step_begin = std::chrono::steady_clock::now();
        session->step_once();
 // Advance SIM-owned presentation effects before closing this transition's
 // work sample. Turbo may execute many transitions between draws; consuming
 // each completed record here preserves impacts and single-wheel slip while
 // keeping particle, trace, and audio work inside the same bounded UI budget.
 const auto& live_steps = session->episode().steps();
 if (!live_steps.empty()) {
 const std::size_t live_index = live_steps.size() - 1;
 const sim::viewer::Frame effect_frame = sim::viewer::build_frame(
 session->setup_world(), live_steps.back(), {}, {}, {},
 session_lidar_mount);
 motion_particles.advance(effect_frame);
 surface_traces.advance(effect_frame);
 const bool live_audio = !experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning;
 viewer_audio.update(effect_frame, false, false,
 playback.turbo() || !live_audio);
 effects_episode_cursor = live_index;
 }
 const double step_ms = std::chrono::duration<double, std::milli>(
 std::chrono::steady_clock::now() - step_begin).count();
 playback.on_step(step_ms);
 if (experience_loaded && session->objectives_done()) break;
      }
 } else {
 // Paused wall time is not simulation debt.
 playback.discard_debt();
    }
 playback.end_frame();
 if (experience_loaded &&
 experience_flow->update(session->episode().steps().size(),
 session->objectives_done())) {
 paused = true;
 sim::viewer::KeyframeReviewConfig review_config;
 if (robot_config.has_value()) {
 review_config.maximum_forward_speed_mps =
 robot_config->arrival.maximum_forward_speed_mps;
 review_config.maximum_yaw_rate_radps =
 robot_config->arrival.maximum_yaw_rate_radps;
 review_config.settle_samples = robot_config->arrival.settle_cycles;
 }
 sim::viewer::KeyframeReviewOutcome review_outcome;
 review_outcome.terminal = true;
 review_outcome.timed_out = experience_flow->timed_out();
 const auto terminal_score = session->score();
 review_outcome.sim_accepted = !review_outcome.timed_out &&
 session->objectives_done() && session->objectives_passed() &&
 terminal_score.collision_steps == 0;
 if (review_outcome.timed_out) {
 review_outcome.rejection_reasons.push_back("experiment_time_limit");
 }
 if (!session->objectives_done()) {
 review_outcome.rejection_reasons.push_back("mission_incomplete");
 }
 if (terminal_score.collision_steps > 0) {
 review_outcome.rejection_reasons.push_back("collision");
 }
 for (const auto& result : session->objectives().results()) {
 review_outcome.rejection_reasons.insert(
 review_outcome.rejection_reasons.end(), result.failure_reasons.begin(),
 result.failure_reasons.end());
 }
 review_keyframes = sim::viewer::extract_review_keyframes(
 session->episode(), review_config, std::move(review_outcome));
 selected_review_keyframe = 0;
 for (std::size_t index = 0; index < review_keyframes.size(); ++index) {
 if (review_keyframes[index].kind ==
 sim::viewer::ReviewKeyframeKind::kRobotDone) {
 selected_review_keyframe = static_cast<int>(index);
 break;
 }
 }
 VisibleRunSummary run_summary;
 run_summary.recorded = true;
 run_summary.timed_out = experience_flow->timed_out();
 run_summary.objectives_done = session->objectives_done();
 run_summary.objectives_passed = session->objectives_passed();
 run_summary.scan_period_s = sim_config.has_value()
 ? sim_config->scan_period_s : 0.0;
 run_summary.score = terminal_score;
 const lost_signal::sim::viewer::ExperienceVariableSpec* variable = nullptr;
 double controlled_value = 0.0;
 std::string controlled_value_label;
 if (!experience.experiments.empty() && sim_config.has_value()) {
 variable = &lost_signal::sim::viewer::experience_variable_spec(
 experience.experiments.front().controlled_variable);
 controlled_value = sim::experiment::experience_variable_value(
 experience.experiments.front().controlled_variable, *sim_config);
 if (lost_signal::sim::viewer::experience_variable_kind(
         experience.experiments.front().controlled_variable) ==
     lost_signal::sim::viewer::ExperienceVariableKind::kCategorical &&
     robot_config.has_value()) {
   controlled_value_label = robot_config->profile.chassis_kind;
 }
 }
 auto artifact = sim::experiment::capture_episode_artifact(
 experience_flow->run_kind() == ExperienceRunKind::kBaseline
 ? "baseline" : "variant",
 variable, controlled_value, runtime_invariant_digest,
 runtime_code_revision, review_config,
 experience_flow->run_kind() == ExperienceRunKind::kVariant &&
 !experience.experiments.empty()
 ? static_cast<std::uint64_t>(
 experience.experiments.front().step_budget)
 : experience.time_limit_steps,
 experience_flow->timed_out(),
 *session, controlled_value_label);
 if (experience_flow->run_kind() == ExperienceRunKind::kBaseline) {
 experiment_ui.baseline = run_summary;
 experiment_ui.baseline_keyframes = review_keyframes;
 if (experience.experiments.empty()) {
 experiment_ui.standalone = std::move(artifact);
 } else {
 std::string spec_error;
 experiment_ui.episodes.spec = sim::experiment::make_experiment_spec(
 experience, controlled_value, runtime_invariant_digest,
 runtime_code_revision, &spec_error);
 if (experiment_ui.episodes.spec.has_value() &&
     lost_signal::sim::viewer::experience_variable_kind(
         experience.experiments.front().controlled_variable) ==
         lost_signal::sim::viewer::ExperienceVariableKind::kCategorical) {
   experiment_ui.episodes.spec->baseline_value_label = controlled_value_label;
   experiment_ui.episodes.spec->variant_value_label =
       experience.experiments.front().variant_category;
 }
 if (!experiment_ui.episodes.spec.has_value()) {
 experience_action_error = spec_error;
 }
 experiment_ui.episodes.baseline = std::move(artifact);
 }
 } else {
 experiment_ui.variant = run_summary;
 experiment_ui.variant_keyframes = review_keyframes;
 experiment_ui.episodes.variant = std::move(artifact);
 if (!experiment_output_path.empty()) {
 std::string export_error;
 if (!sim::experiment::write_episode_pair_bundle(
 experiment_output_path, experiment_ui.episodes, &export_error)) {
 experience_action_error = export_error;
 } else {
 std::fprintf(stderr, "experiment bundle: %s\n",
 experiment_output_path.c_str());
 const lost_signal::sim::viewer::TeachingArchive archive{
     experience.experience_id,
     experiment_output_path,
     experiment_output_path + ".baseline.observations.jsonl",
     experiment_output_path + ".variant.observations.jsonl"};
 lost_signal::sim::viewer::record_teaching_archive(player_profile, archive);
 std::string archive_profile_error;
 if (!lost_signal::sim::viewer::save_player_profile(
         profile_path, player_profile, &archive_profile_error)) {
   std::fprintf(stderr, "cannot save teaching archive index: %s\n",
                archive_profile_error.c_str());
 }
 }
 }
 }
 // Publish teaching progress only after the immutable artifact has passed the
 // chapter's independent gate.  In an A/B chapter this waits for both runs;
 // the baseline is never mistaken for a completed lesson.  The profile is a
 // Viewer-side record and is intentionally absent from the ROBOT runtime.
 const sim::experiment::EpisodeArtifact* published_artifact = nullptr;
 if (experience.progression.require_experiment) {
   if (experiment_ui.episodes.variant.has_value() &&
       experiment_ui.episodes.complete()) {
     published_artifact = &*experiment_ui.episodes.variant;
   }
 } else if (experiment_ui.standalone.has_value()) {
   published_artifact = &*experiment_ui.standalone;
 }
 if (published_artifact != nullptr) {
   const auto published_gate = sim::experiment::evaluate_experience_gate(
       experience.progression, *published_artifact, experiment_ui.episodes);
   if (published_gate.can_advance) {
     lost_signal::sim::viewer::ChapterOutcome outcome;
     outcome.experience_id = experience.experience_id;
     outcome.robot_passed = published_gate.robot_passed;
     outcome.safety_passed = published_gate.sim_safety_passed;
     outcome.mission_passed = published_gate.mission_passed;
     outcome.experiment_passed = published_gate.experiment_passed;
     outcome.elapsed_s = published_artifact->score.elapsed_s;
     outcome.minimum_clearance_m = published_artifact->score.min_clearance_m;
     outcome.knowledge_cards.reserve(experience.lessons.size());
     for (std::size_t index = 0; index < experience.lessons.size(); ++index) {
       outcome.knowledge_cards.push_back(
           lost_signal::sim::viewer::knowledge_card_id(
               experience.experience_id, index));
     }
     lost_signal::sim::viewer::record_chapter(player_profile, outcome);
     chapter_catalog = load_chapter_catalog(player_profile, experience);
     std::string profile_save_error;
     if (!lost_signal::sim::viewer::save_player_profile(
             profile_path, player_profile, &profile_save_error)) {
       std::fprintf(stderr, "cannot save player profile: %s\n",
                    profile_save_error.c_str());
     }
   }
 }
 if (!session->episode().steps().empty()) {
 const auto terminal_frame = sim::viewer::build_frame(
 session->setup_world(), session->episode().steps().back(), {}, {}, {},
 session_lidar_mount);
 viewer_audio.update(terminal_frame, true,
 session->objectives_done() && session->objectives_passed() &&
 terminal_score.collision_steps == 0,
 playback.turbo());
 }
 }
    const auto& steps = session->episode().steps();
 if (steps.empty()) {
 if (!experience_loaded) continue;
 sim::eval::EvalStep preview_step;
 preview_step.truth_after = session->setup_world().truth();
 const sim::viewer::Frame preview = sim::viewer::build_frame(
 session->setup_world(), preview_step, {}, {}, {}, session_lidar_mount);
 BeginDrawing();
 ClearBackground(view3d ? scene_background(
 scene_loaded ? &visual_scene : nullptr)
 : Color{18, 31, 34, 255});
 Rectangle projected_world_hotspot{};
 if (view3d) {
 const bool hero_active = hero_sequence.active() && !experience.intro.empty();
 std::string component_id = "chassis";
 if (hero_active) {
 component_id = experience.intro[hero_sequence.shot_index()].component_id;
 } else if (!experience.walkaround.empty()) {
 const int index = std::clamp(
 static_cast<int>(experience_flow->walkaround_index()), 0,
 static_cast<int>(experience.walkaround.size()) - 1);
 component_id = experience.walkaround[static_cast<std::size_t>(index)]
 .component_id;
 }
 const WalkaroundView walkaround = walkaround_view(
 preview, component_id, cam_st);
 Camera3D presentation_camera = walkaround.camera;
 if (hero_active &&
 hero_sequence.shot_index() + 1 < experience.intro.size()) {
 const double transition = std::clamp(
 (hero_sequence.shot_progress() - 0.68) / 0.32, 0.0, 1.0);
 const WalkaroundView next = walkaround_view(
 preview,
 experience.intro[hero_sequence.shot_index() + 1].component_id,
 cam_st);
 presentation_camera = blend_camera(
 walkaround.camera, next.camera, static_cast<float>(transition));
 }
 const RenderQuality active_quality = playback.turbo()
 ? RenderQuality::kFast : render_quality;
 if (active_quality == RenderQuality::kHigh &&
 !draw_frame_3d_postprocessed(
 preview, session->setup_world(), presentation_camera,
 motion_particles, surface_traces, false, false, false, false, false,
 visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount)) {
 draw_frame_3d_direct(
 preview, session->setup_world(), presentation_camera,
 motion_particles, surface_traces, false, false, false, false, false,
 visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount);
 } else if (active_quality == RenderQuality::kFast) {
 draw_frame_3d_direct(
 preview, session->setup_world(), presentation_camera,
 motion_particles, surface_traces, false, false, false, false, false,
 visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount);
 }
 const Vector2 marker = GetWorldToScreen(
 walkaround.focus, presentation_camera);
 if (marker.x >= 0.0f && marker.x < GetScreenWidth() &&
 marker.y >= 0.0f && marker.y < GetScreenHeight()) {
 if (!hero_active) {
 const Rectangle hotspot{marker.x - 28.0f, marker.y - 28.0f,
 56.0f, 56.0f};
 projected_world_hotspot = hotspot;
 }
 const float pulse = hero_active
 ? 4.0f * static_cast<float>(
 0.5 + 0.5 * std::sin(hero_sequence.total_progress() * 38.0))
 : 0.0f;
 DrawCircle(static_cast<int>(marker.x), static_cast<int>(marker.y),
 12.0f, Color{55, 218, 202, 45});
 DrawCircleLines(static_cast<int>(marker.x), static_cast<int>(marker.y),
 19.0f + pulse, Color{55, 218, 202, 230});
 DrawCircleLines(static_cast<int>(marker.x), static_cast<int>(marker.y),
 25.0f + 2.0f * pulse, Color{55, 218, 202, 110});
 if (!hero_active) {
 draw_ui_text(tr("CLICK PART", "点击部件"),
 static_cast<int>(marker.x + 31.0f),
 static_cast<int>(marker.y - 8.0f), 12, Color{55, 218, 202, 255});
 }
 }
 } else {
 draw_frame_2d(preview, view, false, false, false);
 }
 if (hero_sequence.active() && !experience.intro.empty()) {
 draw_hero_overlay(
 experience, experience.intro[hero_sequence.shot_index()], hero_sequence);
 } else {
 draw_experience_overlay(experience, experience_flow->phase(),
 experience_flow->run_kind(), view_layer,
 experience_flow->comparison_complete(),
 sim_config.has_value() ? sim_config->scan_period_s : 0.0,
 experience_flow->selected_choice(),
 static_cast<int>(experience_flow->walkaround_index()),
 experience_flow->walkaround_visited_count(),
 experience_flow->required_walkaround_visited_count(),
 experience_flow->required_walkaround_count(),
 experience_flow->walkaround_complete(), review_keyframes,
 selected_review_keyframe, false, false, {}, &preview, &experiment_ui,
 &player_profile, &chapter_catalog, show_catalog, false,
 PredictionAssessment::kPending, nullptr,
 experience_action_error,
 true, ui_font(), ui_text_scale(), g_zh_cn,
 &experience_hit_regions);

 }

 // draw_experience_overlay clears and rebuilds the panel hit regions. Restore
 // the projected 3D hotspot after it so the next input frame can click the
 // actual part rather than a text tab.
 if (projected_world_hotspot.width > 0.0f) {
 experience_hit_regions.world_hotspot = projected_world_hotspot;
 }

 EndDrawing();
 if (shot_frames > 0 && --shot_frames == 0) {
 TakeScreenshot(shot_path.c_str());
 break;
 }
 continue;
 }
 std::size_t presented_index = steps.size() - 1;
 if (experience_loaded &&
 experience_flow->phase() == ExperiencePhase::kDebrief &&
 !review_keyframes.empty()) {
 const int selected = std::clamp(selected_review_keyframe, 0,
 static_cast<int>(review_keyframes.size()) - 1);
 const std::size_t requested =
 review_keyframes[static_cast<std::size_t>(selected)].episode_index;
 if (requested < steps.size()) presented_index = requested;
 }
 const sim::eval::EvalStep& presented = steps[presented_index];
    std::vector<std::string> reached;
    for (const auto& r : session->objectives().results()) {
 if (r.successful() && r.completed_step <= presented.truth_after.step) {
 reached.push_back(r.place_id);
 }
    }
 const auto historical_frame = sim::viewer::build_episode_frame(
 session->setup_world(), session->episode(), presented_index, reached,
        edit_mode ? sim::viewer::EditorSelection{editor.selected_box(),
                                                 editor.selected_corner(),
                                                 editor.selected_place()}
                  : sim::viewer::EditorSelection{},
        session_lidar_mount);
 if (!historical_frame.has_value()) continue;
 const sim::viewer::Frame& frame = *historical_frame;
 std::optional<sim::viewer::Frame> baseline_comparison_frame;
 bool baseline_event_aligned = false;
 if (experience_loaded &&
 experience_flow->phase() == ExperiencePhase::kDebrief &&
 experiment_ui.episodes.complete()) {
 std::optional<std::size_t> baseline_index;
 if (!review_keyframes.empty()) {
 const int selected = std::clamp(selected_review_keyframe, 0,
 static_cast<int>(review_keyframes.size()) - 1);
 const auto alignment = sim::viewer::align_review_keyframes_by_event(
 experiment_ui.baseline_keyframes,
 experiment_ui.variant_keyframes,
 review_keyframes[static_cast<std::size_t>(selected)].kind);
 if (alignment.has_value()) {
 baseline_index = experiment_ui.baseline_keyframes[
 alignment->baseline_keyframe_index].episode_index;
 baseline_event_aligned = true;
 }
 }
 if (!baseline_index.has_value()) {
 const auto synchronized =
 sim::experiment::synchronize_episode_pair_by_time(
 experiment_ui.episodes, /*reference_is_variant=*/true,
 presented_index);
 if (synchronized.has_value()) {
 baseline_index = synchronized->baseline_index;
 }
 }
 if (baseline_index.has_value()) {
 baseline_comparison_frame = sim::viewer::build_episode_frame(
 session->setup_world(), experiment_ui.episodes.baseline->episode,
 *baseline_index, {}, {}, session_lidar_mount);
 }
 }
 // Presentation consumes every completed physics frame even when Turbo
 // executes several scenario transitions between two draws. Rendering may
 // sample sparsely; physical events (contact traces, spray, dust and impact
 // sparks) must remain deterministic and replayable. A backward review seek
 // rebuilds the bounded pools from episode history instead of carrying
 // effects from the future.
 if (effects_episode_cursor.has_value() &&
 presented_index < *effects_episode_cursor) {
 motion_particles.clear();
 surface_traces.clear();
 effects_episode_cursor.reset();
 }
 const std::size_t effects_begin = effects_episode_cursor.has_value()
 ? *effects_episode_cursor + 1 : 0;
 for (std::size_t index = effects_begin; index < presented_index; ++index) {
 const sim::viewer::Frame event_frame = sim::viewer::build_frame(
 session->setup_world(), steps[index], {}, {}, {}, session_lidar_mount);
 motion_particles.advance(event_frame);
 surface_traces.advance(event_frame);
 const bool live_audio = !experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning;
 viewer_audio.update(event_frame, false, false,
 playback.turbo() || !live_audio);
 }
 if (!effects_episode_cursor.has_value() ||
 presented_index > *effects_episode_cursor) {
 motion_particles.advance(frame);
 surface_traces.advance(frame);
 effects_episode_cursor = presented_index;
 }
 const bool live_audio = !experience_loaded ||
 experience_flow->phase() == ExperiencePhase::kRunning;
 const auto audio_score = session->score();
 const bool mission_terminal = experience_loaded && session->objectives_done();
 viewer_audio.update(frame, mission_terminal,
 mission_terminal && session->objectives_passed() &&
 audio_score.collision_steps == 0,
playback.turbo() || paused || !live_audio);
    if (follow) {
      view.set_screen(GetScreenWidth(), GetScreenHeight());
      // Re-center without touching zoom: pan by the screen-space error.
      const auto cur =
 view.world_to_screen(presented.truth_after.body_x_m,
 presented.truth_after.body_y_m);
      view.pan_px(GetScreenWidth() * 0.5f - cur.x_px,
                  GetScreenHeight() * 0.5f - cur.y_px);
 const auto& camera_config = scene_loaded
 ? visual_scene.camera : lost_signal::sim::viewer::SceneCamera{};
 const double speed_mps = std::hypot(
 presented.truth_after.forward_mps, presented.truth_after.lateral_mps);
 const double lookahead_m =
 std::abs(presented.truth_after.forward_mps) * camera_config.lookahead_s;
 const double direction =
 presented.truth_after.forward_mps < 0.0 ? -1.0 : 1.0;
 const double desired_x = presented.truth_after.body_x_m +
 direction * lookahead_m * std::cos(presented.truth_after.body_yaw_rad);
 const double desired_y = presented.truth_after.body_y_m +
 direction * lookahead_m * std::sin(presented.truth_after.body_yaw_rad);
 const double blend = camera_config.follow_smoothing_s > 0.0
 ? 1.0 - std::exp(
 presentation_wall_dt_s / -camera_config.follow_smoothing_s)
 : 1.0;
 cam_st.target_x += (desired_x - cam_st.target_x) * blend;
 cam_st.target_y += (desired_y - cam_st.target_y) * blend;
 const double desired_distance = std::clamp(
 static_cast<double>(follow_base_distance_m) +
 speed_mps * camera_config.speed_distance_gain_s,
 camera_config.minimum_distance_m, camera_config.maximum_distance_m);
 cam_st.dist_m += static_cast<float>(
 (desired_distance - cam_st.dist_m) * blend);
    }

 const ExperienceViewPolicy view_policy =
 lost_signal::sim::viewer::experience_view_policy(view_layer);
 const bool layered_running = experience_loaded &&
 experience_flow->phase() == ExperiencePhase::kRunning;
 const bool render_lidar = show_lidar ||
 (layered_running && view_policy.show_lidar);
 const bool render_robot_evidence = show_estimate ||
 (layered_running && view_policy.show_robot_evidence);
 const bool engineering_chrome = !experience_loaded ||
 view_layer == ExperienceViewLayer::kEngineering;

    BeginDrawing();
 ClearBackground(view3d ? scene_background(
 scene_loaded ? &visual_scene : nullptr) : RAYWHITE);
    if (view3d) {
      const Camera3D cam = camera_from_state(cam_st);
 const RenderQuality active_quality = playback.turbo()
 ? RenderQuality::kFast : render_quality;
 if (active_quality == RenderQuality::kHigh &&
 !draw_frame_3d_postprocessed(
 frame, session->setup_world(), cam, motion_particles, surface_traces,
 render_lidar, render_robot_evidence, show_actor_routes, true,
 show_physics_overlay, visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount)) {
 draw_frame_3d_direct(
 frame, session->setup_world(), cam, motion_particles, surface_traces,
 render_lidar, render_robot_evidence, show_actor_routes, true,
 show_physics_overlay, visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount);
 } else if (active_quality == RenderQuality::kFast) {
 draw_frame_3d_direct(
 frame, session->setup_world(), cam, motion_particles, surface_traces,
 render_lidar, render_robot_evidence, show_actor_routes, true,
 show_physics_overlay, visual_assets,
 scene_loaded ? &visual_scene : nullptr, &session_lidar_mount);
 }
 if (baseline_comparison_frame.has_value()) {
 draw_baseline_comparison_ghost_3d(
 *baseline_comparison_frame, cam, baseline_event_aligned);
 }
    } else {
      view.set_screen(GetScreenWidth(), GetScreenHeight());
 draw_frame_2d(frame, view, render_lidar, render_robot_evidence,
 show_actor_routes);
 if (baseline_comparison_frame.has_value()) {
 draw_baseline_comparison_ghost_2d(
 *baseline_comparison_frame, view, baseline_event_aligned);
 }
    }
 const auto playback_stats = playback.statistics();
 const RenderQuality active_quality = playback_stats.turbo
 ? RenderQuality::kFast : render_quality;
 char speed[192];
 if (playback_stats.turbo) {
 std::snprintf(speed, sizeof(speed),
 "req=TURBO actual=%.2fx %.0fstep/s p99/max=%.1f/%.1fms%s",
 playback_stats.actual_rtf, playback_stats.steps_per_s,
 playback_stats.tick_p99_ms, playback_stats.tick_max_ms,
 playback_stats.throttled ? " LIMIT" : "");
 } else {
 std::snprintf(speed, sizeof(speed),
 "req=%.2fx actual=%.2fx %.0fstep/s p99/max=%.1f/%.1fms%s",
 playback_stats.requested_rate, playback_stats.actual_rtf,
 playback_stats.steps_per_s, playback_stats.tick_p99_ms,
 playback_stats.tick_max_ms,
 playback_stats.throttled ? " LIMIT" : "");
 }
 char hud[512];
 std::snprintf(hud, sizeof(hud),
 "step %llu x=%.2f y=%.2f yaw=%.2f clr=%.2f obj %d/%zu %s %dfps cam=%s quality=%s view=%s %s",
 static_cast<unsigned long long>(frame.step), frame.body.x_m,
 frame.body.y_m, frame.body.yaw_rad, frame.static_clearance_m,
 static_cast<int>(reached.size()), session->objectives().total(), speed,
 GetFPS(), follow ? "FOLLOW" : "FREE",
 render_quality_name(active_quality), view_layer_name(view_layer),
 paused ? "PAUSED"
 : experience_loaded && session->objectives_done()
 ? (session->objectives_passed() ? "PASSED" : "FAILED")
 : "RUN");
	 if (engineering_chrome) {
	 DrawRectangle(6, 6, std::min(GetScreenWidth() - 12, 1060),
	 engineering_chrome && render_robot_evidence ? 78 : 48,
	 Color{242, 246, 244, 208});
	 draw_ui_text(hud, 12, 8, 14, Color{18, 34, 39, 255});
	 }
	 if (!experience_loaded) {
	 // Sandbox runs use the same ObjectiveDriver as authored chapters, but
	 // historically exposed no active target in the HUD. That made a safe
	 // wait for fresh observations look like a dead simulation. Keep this
	 // summary presentation-only: it is derived from scenario progress and
	 // never enters ROBOT estimation or control.
	 const std::string& active_place = session->objectives().active_place();
	 char target_hud[256];
	 std::snprintf(target_hud, sizeof(target_hud),
	 "TARGET %s  progress=%zu/%zu successful=%zu  %s",
	 active_place.empty() ? "(settling / complete)" : active_place.c_str(),
	 session->objectives().completed(), session->objectives().total(),
	 session->objectives().successful(),
	 session->objectives_done()
	 ? (session->objectives_passed() ? "MISSION PASSED" : "MISSION FAILED")
	 : "ROBOT is planning from measurements");
	 draw_ui_text(target_hud, 12, 32, 14, Color{28, 72, 78, 255});
	 }
 if (engineering_chrome && render_robot_evidence) {
 draw_ui_text("ROBOT ESTIMATE VIEW  scan origin=estimate sensor (green); L shows physical sensor frame",
 12, 50, 13, Color{78, 54, 132, 255});
 draw_ui_text("ROBOT ESTIMATE  map<-odom correction=amber->cyan dynamic forecast=purple",
 12, 66, 13, Color{78, 54, 132, 255});
 const auto& physical_audio = viewer_audio.physical_sample();
 char physical_evidence[256];
 std::snprintf(physical_evidence, sizeof(physical_evidence),
 "SIM PHYSICS/AUDIO  motor %.2f roll %.2f slip %.2f brake %.2f impact %.2f",
 physical_audio.motor_intensity, physical_audio.rolling_intensity,
 physical_audio.slip_intensity, physical_audio.brake_intensity,
 physical_audio.impact_intensity);
 draw_ui_text(physical_evidence, 12, 82, 13, Color{24, 116, 105, 255});
 }
    if (edit_mode) {
      char ebuf[256];
      std::snprintf(ebuf, sizeof(ebuf),
                    "EDIT  click/drag move  B box  P place  Del remove  S save viewer_map.json  M exit  %s",
                    edit_msg.c_str());
 DrawText(ebuf, 10,
 engineering_chrome && render_robot_evidence ? 82 : 32, 14, RED);
    }
 const char* control_help = experience_loaded &&
 view_layer == ExperienceViewLayer::kPlayer
 ? "U explain C catalog space pause F follow F10 maximize F11 fullscreen Q quality +/- speed V 2D/3D"
 : experience_loaded && view_layer == ExperienceViewLayer::kLearning
 ? "U engineering C catalog space pause F follow F10 maximize F11 fullscreen Q quality +/- speed V 2D/3D"
 : sim_truth_debug_allowed
 ? "space pause  N step  L lidar  E ROBOT AI  K SIM truth debug  U view  F follow  Q quality  0 fit  +/- speed  V 2D/3D  M edit  right-drag pan/orbit  wheel zoom"
 : "space pause  N step  L lidar  E ROBOT AI  U view  F follow  Q quality  0 fit  +/- speed  V 2D/3D  right-drag pan/orbit  wheel zoom";
 if (!experience_loaded && manual_control) {
 control_help = "space pause N step H manual ON WASD/arrows drive I/K O/L P/; [/'] arm L lidar E ROBOT AI K SIM truth U view F follow Q quality +/- speed V 2D/3D M edit";
 }
 DrawRectangle(6, GetScreenHeight() - 34, GetScreenWidth() - 12, 28,
 Color{242, 246, 244, 208});
 draw_ui_text(control_help, 12, GetScreenHeight() - 30, 12,
 Color{32, 46, 50, 255});
 draw_ui_text("T turbo", std::max(12, GetScreenWidth() - 72),
 GetScreenHeight() - 30, 12, Color{32, 46, 50, 255});
 if (experience_loaded) {
 const sim::experiment::EpisodeArtifact* active_artifact = nullptr;
 if (experience.progression.require_experiment) {
 if (experiment_ui.episodes.variant.has_value()) {
 active_artifact = &*experiment_ui.episodes.variant;
 } else if (experiment_ui.episodes.baseline.has_value()) {
 active_artifact = &*experiment_ui.episodes.baseline;
 }
 } else if (experiment_ui.standalone.has_value()) {
 active_artifact = &*experiment_ui.standalone;
 }
 PredictionAssessment prediction_assessment =
 PredictionAssessment::kPending;
 const int selected_choice = experience_flow->selected_choice();
 if (selected_choice >= 0 &&
 selected_choice < static_cast<int>(experience.choices.size())) {
 if (active_artifact != nullptr) {
 prediction_assessment = lost_signal::sim::viewer::assess_prediction(
 static_cast<std::size_t>(selected_choice), experience.choices,
 sim::experiment::episode_evidence(
 *active_artifact, experiment_ui.episodes));
 }
 }
 std::optional<sim::experiment::ExperienceGateVerdict> gate_verdict;
 if (active_artifact != nullptr) {
 gate_verdict = sim::experiment::evaluate_experience_gate(
 experience.progression, *active_artifact, experiment_ui.episodes);
 }
 draw_experience_overlay(experience, experience_flow->phase(),
 experience_flow->run_kind(), view_layer,
 experience_flow->comparison_complete(),
 sim_config.has_value() ? sim_config->scan_period_s : 0.0,
 selected_choice,
 static_cast<int>(experience_flow->walkaround_index()),
 experience_flow->walkaround_visited_count(),
 experience_flow->required_walkaround_visited_count(),
 experience_flow->required_walkaround_count(),
 experience_flow->walkaround_complete(), review_keyframes,
 selected_review_keyframe, session->objectives_done(),
 session->objectives_passed(), session->score(), &frame, &experiment_ui,
 &player_profile, &chapter_catalog, show_catalog,
 experience_flow->timed_out(), prediction_assessment,
 gate_verdict.has_value() ? &*gate_verdict : nullptr,
 experience_action_error,
 true, ui_font(), ui_text_scale(), g_zh_cn,
 &experience_hit_regions);
 }
 if (playback_controls_visible) {
 draw_playback_controls(playback_hit_regions(), playback);
 }
    EndDrawing();
    if (shot_frames > 0 && --shot_frames == 0) {
      TakeScreenshot(shot_path.c_str());
      break;
    }
  }
 const auto final_playback_stats = playback.statistics();
 std::fprintf(stderr,
 "viewer performance: requested=%s actual=%.2fx steps=%.0f/s "
 "tick_p99=%.2fms tick_max=%.2fms quality=%s\n",
 final_playback_stats.turbo ? "turbo" : "rate",
 final_playback_stats.actual_rtf, final_playback_stats.steps_per_s,
 final_playback_stats.tick_p99_ms, final_playback_stats.tick_max_ms,
 render_quality_name(final_playback_stats.turbo
 ? RenderQuality::kFast : render_quality));
 unload_visual_assets(visual_assets);
 unload_ui_font();
 CloseWindow();
  return 0;
}
