#include "sim/viewer/experience_overlay.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <sstream>

#include "sim/viewer/keyframe_review.hpp"

namespace lost_signal::sim::viewer {
namespace {

Font g_overlay_font{};
float g_overlay_scale = 1.0f;
bool g_overlay_zh_cn = false;

const char* overlay_tr(const char* english, const char* chinese) {
  return g_overlay_zh_cn ? chinese : english;
}

void draw_overlay_text(const char* text, int x, int y, float size,
                       Color color) {
  DrawTextEx(g_overlay_font, text,
             {static_cast<float>(x), static_cast<float>(y)},
             size * g_overlay_scale, 0.5f, color);
}

float measure_overlay_text(const char* text, float size) {
  return MeasureTextEx(g_overlay_font, text, size * g_overlay_scale, 0.5f).x;
}

float draw_overlay_wrapped(const std::string& value, float x, float y,
                           float width, float size, float gap, Color color) {
  const float font_size = size * g_overlay_scale;
  const float line_gap = gap * g_overlay_scale;
  std::istringstream input(value);
  std::string word;
  std::string line;
  while (input >> word) {
    const std::string candidate = line.empty() ? word : line + " " + word;
    if (!line.empty() &&
        MeasureTextEx(g_overlay_font, candidate.c_str(), font_size, 1.0f).x >
            width) {
      DrawTextEx(g_overlay_font, line.c_str(), {x, y}, font_size, 1.0f,
                 color);
      y += font_size + line_gap;
      line = word;
    } else {
      line = candidate;
    }
  }
  if (!line.empty()) {
    DrawTextEx(g_overlay_font, line.c_str(), {x, y}, font_size, 1.0f, color);
    y += font_size + line_gap;
  }
  return y;
}

void draw_overlay_button(Rectangle bounds, const char* label, bool primary,
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
  const float text_size = 14.0f * g_overlay_scale;
  const float width = measure_overlay_text(label, 14.0f);
  draw_overlay_text(label, static_cast<int>(bounds.x +
      (bounds.width - width) * 0.5f),
      static_cast<int>(bounds.y + (bounds.height - text_size) * 0.5f),
      14, primary ? Color{7, 18, 18, 255} : text);
}

}  // namespace

void draw_experience_overlay(const ExperienceManifest& experience,
 ExperiencePhase phase, ExperienceRunKind run_kind,
 ExperienceViewLayer view_layer,
 bool comparison_complete, double active_scan_period_s,
 int selected_choice, int walkaround_stop,
 std::size_t walkaround_visited_count,
 std::size_t required_walkaround_visited_count,
 std::size_t required_walkaround_count, bool walkaround_complete,
 const std::vector<ReviewKeyframe>& review_keyframes,
 int selected_review_keyframe,
 bool objectives_done,
 bool objectives_passed, const lost_signal::sim::eval::EpisodeScore& score,
 const Frame* frame,
 const ExperimentUiState* experiment_ui, const PlayerProfile* player_profile,
 const std::vector<ChapterCatalogEntry>* chapter_catalog,
 bool show_catalog,
 bool timed_out,
 PredictionAssessment prediction_assessment,
 const lost_signal::sim::experiment::ExperienceGateVerdict* gate_verdict,
 const std::string& action_error, bool actions_enabled,
 Font font, float text_scale, bool zh_cn,
 ExperienceHitRegions* hit_regions) {
 g_overlay_font = font;
 g_overlay_scale = text_scale;
 g_overlay_zh_cn = zh_cn;
 constexpr Color kPanel{13, 21, 22, 242};
 constexpr Color kPanelSoft{24, 37, 38, 238};
 constexpr Color kText{226, 235, 230, 255};
 constexpr Color kMuted{157, 177, 169, 255};
 constexpr Color kCyan{55, 218, 202, 255};
 constexpr Color kAmber{246, 169, 61, 255};
 constexpr Color kRed{236, 82, 72, 255};
 if (hit_regions != nullptr) hit_regions->clear();
 if (phase == ExperiencePhase::kBriefing) {
   if (show_catalog && chapter_catalog != nullptr) {
     draw_chapter_catalog_panel(*chapter_catalog, experience.experience_id,
                                font, text_scale, zh_cn);
   } else {
     draw_player_profile_panel(experience, player_profile, false, font,
                               text_scale, zh_cn);
   }
 }
 if (phase == ExperiencePhase::kDebrief) {
 const float screen_width = static_cast<float>(GetScreenWidth());
 const float screen_height = static_cast<float>(GetScreenHeight());
 const float drawer_width = std::min(448.0f, screen_width - 32.0f);
 const float drawer_height = screen_height - 24.0f;
 const float x = screen_width - drawer_width - 12.0f;
 const float y0 = 12.0f;
 const bool safe = !timed_out && objectives_done && objectives_passed &&
 score.collision_steps == 0;

 DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(),
 Color{5, 10, 12, 38});
 DrawRectangleRounded({x, y0, drawer_width, drawer_height}, 0.025f, 10,
 kPanel);
 DrawRectangleRoundedLinesEx(
 {x, y0, drawer_width, drawer_height}, 0.025f, 10, 2.0f,
 safe ? kCyan : kRed);

 draw_overlay_text(overlay_tr("CAUSAL DEBRIEF", "因果复盘"),
 static_cast<int>(x + 22), 31, 14, kCyan);
 draw_overlay_text((safe ? experience.success_title
 : experience.failure_title).c_str(),
 static_cast<int>(x + 22), 57, 20, safe ? kCyan : kRed);

 char metrics[256];
 std::snprintf(metrics, sizeof(metrics),
 zh_cn
 ? "SIM 离线验收　碰撞 %zu　净空 %.2f m　估计误差 %.2f m"
 : "SIM offline | collisions %zu | clearance %.2f m | estimate error %.2f m",
 score.collision_steps, score.min_clearance_m, score.mean_pos_err_m);
 draw_overlay_wrapped(metrics, x + 22, 89, drawer_width - 44, 13, 3,
 safe ? kText : kRed);

 if (!action_error.empty()) {
 draw_overlay_wrapped(std::string(overlay_tr("ACTION FAILED: ", "操作失败：")) +
 action_error, x + 22, 118, drawer_width - 44, 12, 2, kRed);
 }

 if (gate_verdict != nullptr) {
 struct VerdictChip {
 const char* label;
 bool passed;
 bool pending;
 };
 const std::array<VerdictChip, 4> verdicts{{
 {overlay_tr("ROBOT", "机器人"), gate_verdict->robot_passed, false},
 {overlay_tr("SIM SAFETY", "仿真安全"), gate_verdict->sim_safety_passed, false},
 {overlay_tr("MISSION", "任务"), gate_verdict->mission_passed, false},
 {overlay_tr("EXPERIMENT", "实验"), gate_verdict->experiment_passed,
 !comparison_complete},
 }};
 const float gap = 6.0f;
 const float chip_width =
 (drawer_width - 44.0f - gap * 3.0f) / 4.0f;
 for (std::size_t index = 0; index < verdicts.size(); ++index) {
 const float chip_x = x + 22.0f + index * (chip_width + gap);
 const Color accent = verdicts[index].pending ? kAmber
 : verdicts[index].passed ? kCyan : kRed;
 DrawRectangleRounded({chip_x, 137.0f, chip_width, 29.0f},
 0.16f, 6, Color{25, 43, 43, 255});
 DrawRectangleRoundedLinesEx({chip_x, 137.0f, chip_width, 29.0f},
 0.16f, 6, 1.0f, accent);
 char verdict[48];
 std::snprintf(verdict, sizeof(verdict), "%s %s", verdicts[index].label,
 verdicts[index].pending ? "..." :
 verdicts[index].passed ? "PASS" : "FAIL");
 const float text_width = measure_overlay_text(verdict, 11.0f);
 draw_overlay_text(verdict,
 static_cast<int>(chip_x + (chip_width - text_width) * 0.5f),
 146, 11, accent);
 }
 }

 draw_overlay_text(overlay_tr("WHY DID THIS RUN END HERE?", "为什么本次运行止于这里？"),
 static_cast<int>(x + 22), 184, 14, kAmber);
 draw_overlay_text(overlay_tr("CLICK A STEP / LEFT RIGHT TO REPLAY",
 "点击步骤 / 左右键回放记录"),
 static_cast<int>(x + 22), 205, 11, kMuted);

 const auto chain =
 lost_signal::sim::viewer::build_review_causal_chain(review_keyframes);
 struct CausalRow {
 const char* label;
 std::optional<std::size_t> keyframe_index;
 };
 const std::array<CausalRow, 5> rows{{
 {overlay_tr("FIRST VALID OBSERVATION", "首次有效观测"),
 chain.first_observation},
 {overlay_tr("BRAKING COMMAND", "制动命令"), chain.braking_command},
 {overlay_tr("STABLE STOP", "稳定停车"), chain.stable_stop},
 {overlay_tr("ROBOT DONE", "机器人报告完成"), chain.robot_done},
 {overlay_tr("SIM ACCEPTED / REJECTED", "仿真验收 / 拒绝"),
 chain.sim_verdict},
 }};
 constexpr float kRowHeight = 51.0f;
 constexpr float kRowsY = 225.0f;
 DrawLineEx({x + 37.0f, kRowsY + 19.0f},
 {x + 37.0f, kRowsY + kRowHeight * 4.0f + 19.0f},
 2.0f, Color{64, 96, 90, 255});
 for (std::size_t row = 0; row < rows.size(); ++row) {
 const float row_y = kRowsY + row * kRowHeight;
 const Rectangle bounds{x + 18.0f, row_y, drawer_width - 36.0f,
 kRowHeight - 4.0f};
 const bool recorded = rows[row].keyframe_index.has_value();
 const bool selected = recorded &&
 static_cast<int>(*rows[row].keyframe_index) == selected_review_keyframe;
 bool rejected = false;
 const ReviewKeyframe* keyframe = nullptr;
 if (recorded && *rows[row].keyframe_index < review_keyframes.size()) {
 keyframe = &review_keyframes[*rows[row].keyframe_index];
 rejected = keyframe->kind ==
 lost_signal::sim::viewer::ReviewKeyframeKind::kSimRejected;
 }
 const Color accent = !recorded ? Color{88, 105, 100, 255}
 : rejected ? kRed : selected ? kAmber : kCyan;
 if (selected) {
 DrawRectangleRounded(bounds, 0.08f, 6, Color{45, 48, 35, 255});
 }
 DrawCircle(static_cast<int>(x + 37.0f),
 static_cast<int>(row_y + 19.0f), 9.0f,
 recorded ? accent : Color{35, 51, 48, 255});
 if (recorded) {
 DrawCircle(static_cast<int>(x + 37.0f),
 static_cast<int>(row_y + 19.0f), 3.0f, kPanel);
 }
 draw_overlay_text(rows[row].label, static_cast<int>(x + 57.0f),
 static_cast<int>(row_y + 5.0f), 13, recorded ? kText : kMuted);
 if (keyframe != nullptr) {
 const double time_s = keyframe->stamp_ns ==
 lost_signal::robot::core::kInvalidTime ? 0.0
 : static_cast<double>(keyframe->stamp_ns) * 1e-9;
 char stamp[96];
 std::snprintf(stamp, sizeof(stamp), "t %.2f s / record %llu", time_s,
 static_cast<unsigned long long>(keyframe->step));
 draw_overlay_text(stamp, static_cast<int>(x + 57.0f),
 static_cast<int>(row_y + 25.0f), 11, accent);
 if (hit_regions != nullptr) {
 hit_regions->causal_keyframes.push_back(
 {bounds, *rows[row].keyframe_index});
 }
 } else {
 draw_overlay_text(overlay_tr("NOT RECORDED", "未记录"),
 static_cast<int>(x + 57.0f), static_cast<int>(row_y + 25.0f),
 11, kMuted);
 }
 }

 const ReviewKeyframe* selected_keyframe = nullptr;
 if (selected_review_keyframe >= 0 &&
 selected_review_keyframe < static_cast<int>(review_keyframes.size())) {
 selected_keyframe =
 &review_keyframes[static_cast<std::size_t>(selected_review_keyframe)];
 }
 if (selected_keyframe != nullptr) {
 const bool sim_record = selected_keyframe->kind ==
 lost_signal::sim::viewer::ReviewKeyframeKind::kSimAccepted ||
 selected_keyframe->kind ==
 lost_signal::sim::viewer::ReviewKeyframeKind::kSimRejected ||
 selected_keyframe->kind ==
 lost_signal::sim::viewer::ReviewKeyframeKind::kCollision ||
 selected_keyframe->kind ==
 lost_signal::sim::viewer::ReviewKeyframeKind::kTimeout;
 draw_overlay_text(sim_record ? overlay_tr("SIM OFFLINE EVIDENCE", "仿真离线证据")
 : overlay_tr("ROBOT RECORDED EVIDENCE", "机器人记录证据"),
 static_cast<int>(x + 22), 486, 11,
 sim_record ? kAmber : kCyan);
 draw_overlay_wrapped(selected_keyframe->evidence, x + 22, 506,
 drawer_width - 44, 12, 3, kText);
 }

 const bool has_comparison = !experience.experiments.empty() &&
 experiment_ui != nullptr && experiment_ui->baseline.recorded;
 float summary_y = 558.0f;
 if (has_comparison) {
 const auto& baseline = experiment_ui->baseline;
 char comparison[256];
 if (comparison_complete && experiment_ui->variant.recorded) {
 const auto& variant = experiment_ui->variant;
 std::snprintf(comparison, sizeof(comparison),
 zh_cn
 ? "A/B　基线 %.2fs / %.3fm　变体 %.2fs / %.3fm"
 : "A/B | base %.2fs / %.3fm | variant %.2fs / %.3fm",
 baseline.score.elapsed_s, baseline.score.min_clearance_m,
 variant.score.elapsed_s, variant.score.min_clearance_m);
 } else {
 std::snprintf(comparison, sizeof(comparison),
 zh_cn ? "基线已记录 %.2fs / 净空 %.3fm；下一步只改变扫描周期"
 : "Baseline recorded %.2fs / %.3fm; next run changes scan cadence only",
 baseline.score.elapsed_s, baseline.score.min_clearance_m);
 }
 draw_overlay_wrapped(comparison, x + 22, summary_y,
 drawer_width - 44, 12, 3, kText);
 summary_y += 38.0f;
 }
 if (selected_choice >= 0 &&
 selected_choice < static_cast<int>(experience.choices.size())) {
 const char* prediction = prediction_assessment ==
 PredictionAssessment::kSupported
 ? overlay_tr("PREDICTION SUPPORTED", "预测得到支持")
 : prediction_assessment == PredictionAssessment::kRefuted
 ? overlay_tr("PREDICTION REFUTED", "预测被证据反驳")
 : prediction_assessment == PredictionAssessment::kInsufficientEvidence
 ? overlay_tr("EVIDENCE DOES NOT SUPPORT A DECLARED OUTCOME",
 "证据不足以支持任一声明结果")
 : overlay_tr("PREDICTION LOCKED UNTIL BOTH RUNS FINISH",
 "预测将在两次运行完成后验证");
 draw_overlay_text(prediction, static_cast<int>(x + 22),
 static_cast<int>(summary_y), 12,
 prediction_assessment == PredictionAssessment::kSupported
 ? kCyan : kAmber);
 }

 const bool final_chapter = experience.next_experience_path.empty();
 if (final_chapter && gate_verdict != nullptr &&
 gate_verdict->can_advance) {
 draw_overlay_text(overlay_tr("CAMPAIGN COMPLETE", "全部关卡完成"),
 static_cast<int>(x + 22), static_cast<int>(summary_y + 21.0f),
 13, kCyan);
 summary_y += 28.0f;
 }

 const float actions_y = y0 + drawer_height - 58.0f;
 float primary_width = 0.0f;
 if (!experience.experiments.empty() && experiment_ui != nullptr &&
 experiment_ui->baseline.recorded && !comparison_complete) {
 const Rectangle variant_button{x + 20.0f, actions_y, 184.0f, 34.0f};
 const bool can_run = actions_enabled &&
 run_kind == ExperienceRunKind::kBaseline;
 draw_overlay_button(variant_button,
 overlay_tr("RUN VISIBLE VARIANT", "运行可视变体"),
 true, kAmber, kText, can_run);
 if (hit_regions != nullptr && can_run) {
 hit_regions->primary = variant_button;
 }
 primary_width = variant_button.width + 8.0f;
 } else if (!experience.next_experience_path.empty()) {
 const Rectangle next_button{x + 20.0f, actions_y, 184.0f, 34.0f};
 const bool can_advance = actions_enabled && gate_verdict != nullptr &&
 gate_verdict->can_advance;
 draw_overlay_button(next_button,
 can_advance ? overlay_tr("NEXT CHAPTER", "下一章")
 : overlay_tr("PASS GATES TO CONTINUE", "通过验收后继续"),
 true, kCyan, kText, can_advance);
 if (hit_regions != nullptr && can_advance) {
 hit_regions->next_chapter = next_button;
 }
 primary_width = next_button.width + 8.0f;
 }
 const Rectangle retry_button{x + 20.0f + primary_width, actions_y,
 104.0f, 34.0f};
 draw_overlay_button(retry_button,
 final_chapter && gate_verdict != nullptr && gate_verdict->can_advance
 ? overlay_tr("REPLAY FINAL", "重玩本关") : overlay_tr("RETRY", "重试"), false, kCyan,
 kText, actions_enabled);
 if (hit_regions != nullptr && actions_enabled) {
 hit_regions->retry = retry_button;
 }
 draw_overlay_text(overlay_tr("ROBOT never received SIM truth.",
 "机器人从未收到仿真真值。"),
 static_cast<int>(x + 22), static_cast<int>(y0 + drawer_height - 17.0f),
 11, kAmber);
 draw_player_profile_panel(experience, player_profile, true, font,
                           text_scale, zh_cn);
 return;
 }
 if (phase == ExperiencePhase::kRunning) {
 if (view_layer != ExperienceViewLayer::kEngineering) {
 const bool learning = view_layer == ExperienceViewLayer::kLearning;
 const float width = learning ? 390.0f : 330.0f;
 const float height = learning ? 350.0f : 178.0f;
 const float x = static_cast<float>(GetScreenWidth()) - width - 18.0f;
 DrawRectangleRounded({x, 58.0f, width, height}, 0.04f, 8,
 learning ? kPanelSoft : Color{13, 21, 22, 218});
 draw_overlay_text(learning
 ? overlay_tr("LEARNING VIEW / CAUSAL CHAIN", "教学视图 / 因果链")
 : overlay_tr("MISSION VIEW", "任务视图"),
 static_cast<int>(x + 18), 74, 14, kCyan);
 float y = 104.0f;
 if (frame != nullptr) {
 const auto& observation = frame->observation;
 const auto& estimate = frame->estimate;
 const auto& decision = frame->decision;
 const FramePlace* active_place = nullptr;
 for (const auto& place : frame->places) {
 if (!place.reached) {
 active_place = &place;
 break;
 }
 }
 const double distance_m = estimate.valid && active_place != nullptr
 ? std::hypot(active_place->x_m - estimate.map_x_m,
 active_place->y_m - estimate.map_y_m) : -1.0;
 const bool holding = std::abs(decision.command_forward_mps) <= 0.03 &&
 std::abs(decision.command_yaw_rate_radps) <= 0.03;
 const char* stage = !observation.accepted
 ? overlay_tr("WAITING FOR MEASUREMENT", "等待测量")
 : !estimate.valid ? overlay_tr("BUILDING ESTIMATE", "建立估计")
 : !decision.valid ? overlay_tr("PLANNING", "规划")
 : holding ? overlay_tr("BRAKING / SETTLING", "制动 / 稳定")
 : overlay_tr("MOVING UNDER VALIDATED COMMAND", "按已验证命令移动");
 draw_overlay_text(stage, static_cast<int>(x + 18), static_cast<int>(y),
 learning ? 18 : 20, kAmber);
 y += 34.0f;
 char line[256];
 if (active_place != nullptr && distance_m >= 0.0) {
 std::snprintf(line, sizeof(line), "%s / %.2f m",
 active_place->place_id.c_str(), distance_m);
 } else {
 std::snprintf(line, sizeof(line), "%s", overlay_tr("TASK COMPLETE", "任务完成"));
 }
 draw_overlay_text(line, static_cast<int>(x + 18), static_cast<int>(y),
 15, kText);
 y += 29.0f;
 if (learning) {
 draw_overlay_text(overlay_tr("MEASURE  >  ESTIMATE  >  PLAN  >  VALIDATE  >  SAFETY",
 "测量  >  估计  >  规划  >  验证  >  安全锁"),
 static_cast<int>(x + 18), static_cast<int>(y), 12,
 Color{180, 132, 255, 255});
 y += 31.0f;
 std::snprintf(line, sizeof(line),
 zh_cn ? "测量：LiDAR %zu/%zu 返回，轮速 %s，IMU %s"
 : "MEASURE: LiDAR %zu/%zu returns, wheels %s, IMU %s",
 observation.lidar_returns, observation.lidar_beams,
 observation.odometry_valid ? "OK" : "--",
 observation.imu_valid ? "OK" : "--");
 draw_overlay_wrapped(line, x + 18, y, width - 36, 13, 4, kText);
 y += 35.0f;
 if (estimate.valid) {
 std::snprintf(line, sizeof(line),
 zh_cn ? "估计：位置不确定度 %.2f m；%s"
 : "ESTIMATE: position uncertainty %.2f m; %s",
 estimate.position_sigma_m,
 estimate.localized ? overlay_tr("localized", "已定位") : overlay_tr("tracking", "跟踪中"));
 } else {
 std::snprintf(line, sizeof(line), "%s",
 overlay_tr("ESTIMATE: waiting for enough evidence",
 "估计：等待足够的传感器证据"));
 }
 draw_overlay_wrapped(line, x + 18, y, width - 36, 13, 4, kText);
 y += 35.0f;
 std::snprintf(line, sizeof(line),
 zh_cn ? "决策：%s；独立 Safety %s"
 : "DECIDE: %s; independent Safety %s",
 holding ? overlay_tr("hold and observe", "停车并观察")
 : overlay_tr("track local trajectory", "跟踪局部轨迹"),
 decision.safety_state.empty() ? "--" : decision.safety_state.c_str());
 draw_overlay_wrapped(line, x + 18, y, width - 36, 13, 4, kText);
 y += 40.0f;
 draw_overlay_wrapped(
 overlay_tr("SIM truth is hidden. Press U for engineering evidence.",
 "运行时隐藏仿真真值。按 U 查看工程证据。"),
 x + 18, y, width - 36, 12, 3, kMuted);
 }
 }
 if (!learning) {
 draw_overlay_text(overlay_tr("U  EXPLAIN WHY", "U  解释原因"),
 static_cast<int>(x + 18), static_cast<int>(58.0f + height - 31.0f),
 12, kMuted);
 }
 return;
 }
 const float width = 370.0f;
 const float x = static_cast<float>(GetScreenWidth()) - width - 18.0f;
 DrawRectangleRounded({x, 58.0f, width, 450.0f}, 0.04f, 8, kPanelSoft);
 draw_overlay_text(overlay_tr("LIVE EVIDENCE / AUTHORITY", "实时证据 / 数据权限"),
 static_cast<int>(x + 18), 74, 14, kCyan);
 float y = 101.0f;
 char condition[160];
 std::snprintf(condition, sizeof(condition),
 zh_cn ? "%s条件　LiDAR %.0f ms"
 : "%s CONDITION  /  LiDAR %.0f ms",
 run_kind == ExperienceRunKind::kVariant
 ? overlay_tr("VARIANT", "慢扫描") : overlay_tr("BASELINE", "基线"),
 active_scan_period_s * 1000.0);
 draw_overlay_text(condition, static_cast<int>(x + 18), static_cast<int>(y),
 14, kAmber);
 y += 27.0f;
 if (frame != nullptr) {
 char line[320];
 const auto& observation = frame->observation;
   const auto& estimate = frame->estimate;
   const auto& decision = frame->decision;
 const double task_time_s = std::max({
 0.0, observation.stamp_s, estimate.stamp_s, decision.stamp_s});
 const FramePlace* active_place = nullptr;
 for (const auto& place : frame->places) {
 if (!place.reached) {
 active_place = &place;
 break;
 }
 }
 const double goal_distance_m =
 estimate.valid && active_place != nullptr
 ? std::hypot(active_place->x_m - estimate.map_x_m,
 active_place->y_m - estimate.map_y_m)
 : -1.0;
 const char* stage = overlay_tr("OBSERVE", "观察");
   if (observation.accepted && observation.scan_valid) {
 stage = !estimate.valid ? overlay_tr("ESTIMATE", "估计")
 : !decision.valid || decision.action_code.empty()
 ? overlay_tr("PLAN", "规划")
    : std::abs(decision.command_forward_mps) <= 0.03 &&
      std::abs(decision.command_yaw_rate_radps) <= 0.03
 ? overlay_tr("BRAKE / SETTLE", "制动 / 稳定")
 : overlay_tr("PLAN / MOVE", "规划 / 移动");
   }
 draw_overlay_text(stage, static_cast<int>(x + 18), static_cast<int>(y), 20,
    kAmber);
   y += 31.0f;
 draw_overlay_text(overlay_tr("GAME / TASK", "游戏 / 当前任务"),
 static_cast<int>(x + 18), static_cast<int>(y), 13, kAmber);
 if (goal_distance_m >= 0.0) {
 std::snprintf(line, sizeof(line), "%s %.2f m t %.1f s",
 active_place->place_id.c_str(), goal_distance_m, task_time_s);
 } else {
 std::snprintf(line, sizeof(line), "target -- t %.1f s", task_time_s);
 }
 draw_overlay_text(line, static_cast<int>(x + 18), static_cast<int>(y + 19),
 14, kText);
 y += 48.0f;
 draw_overlay_text(overlay_tr("ROBOT / OBSERVATION", "机器人 / 传感器观测"),
 static_cast<int>(x + 18),
 static_cast<int>(y), 13, kCyan);
 std::snprintf(line, sizeof(line),
 "scan %s  beams %zu/%zu  odom %s  IMU %s",
 observation.accepted && observation.scan_valid ? "ACCEPTED" : "WAIT",
 observation.lidar_returns, observation.lidar_beams,
 observation.odometry_valid ? "OK" : "--",
 observation.imu_valid ? "OK" : "--");
 draw_overlay_text(line, static_cast<int>(x + 18), static_cast<int>(y + 19), 13,
 kText);
 std::snprintf(line, sizeof(line), "%s seq %llu t=%.3fs",
 observation.scan_source.empty() ? "unknown source"
 : observation.scan_source.c_str(),
 static_cast<unsigned long long>(observation.scan_sequence),
 observation.scan_stamp_s);
 draw_overlay_text(line, static_cast<int>(x + 18), static_cast<int>(y + 37), 12,
 kMuted);
 y += 62.0f;

 draw_overlay_text(overlay_tr("ROBOT / ESTIMATE", "机器人 / 状态估计"),
 static_cast<int>(x + 18),
 static_cast<int>(y), 13, kCyan);
 if (estimate.valid) {
 std::snprintf(line, sizeof(line),
 "map (%.2f, %.2f)  sigma %.2f m  %s",
 estimate.map_x_m, estimate.map_y_m, estimate.position_sigma_m,
 estimate.localized ? "LOCALIZED" : "TRACKING");
 } else {
 std::snprintf(line, sizeof(line), "waiting for a valid state estimate");
 }
 draw_overlay_text(line, static_cast<int>(x + 18), static_cast<int>(y + 19), 13,
 kText);
 std::snprintf(line, sizeof(line), "localization %s / SLAM keyframes %zu loops %zu",
 estimate.localization_action.empty() ? "--"
 : estimate.localization_action.c_str(),
 estimate.slam_keyframes, estimate.slam_loops_closed);
 draw_overlay_text(line, static_cast<int>(x + 18), static_cast<int>(y + 37), 12,
 kMuted);
 y += 62.0f;

 draw_overlay_text(overlay_tr("ROBOT / DECISION", "机器人 / 规划决策"),
 static_cast<int>(x + 18),
 static_cast<int>(y), 13, Color{180, 132, 255, 255});
 std::snprintf(line, sizeof(line), "%s / %s",
 decision.action_code.empty() ? "idle" : decision.action_code.c_str(),
 decision.safety_state.empty() ? "UNKNOWN" : decision.safety_state.c_str());
 draw_overlay_text(line, static_cast<int>(x + 18), static_cast<int>(y + 19), 13,
 kText);
 std::snprintf(line, sizeof(line), "cmd %.2f m/s  %.2f rad/s  candidates %zu",
 decision.command_forward_mps, decision.command_yaw_rate_radps,
 decision.moving_candidate_count);
 draw_overlay_text(line, static_cast<int>(x + 18), static_cast<int>(y + 37), 13,
 kMuted);
 y += 66.0f;
 }
 draw_overlay_text(overlay_tr("SIM / PHYSICS TRUTH", "仿真 / 物理真值"),
 static_cast<int>(x + 18),
 static_cast<int>(y), 13, kAmber);
 draw_overlay_text(overlay_tr("HIDDEN DURING RUN / REVEALED IN DEBRIEF",
 "运行时隐藏 / 复盘时显示"),
 static_cast<int>(x + 18), static_cast<int>(y + 19), 13, kMuted);
 y += 47.0f;
 draw_overlay_text(overlay_tr("GAME / OFFLINE EVAL", "游戏 / 离线验收"),
 static_cast<int>(x + 18),
 static_cast<int>(y), 13, kAmber);
 draw_overlay_text(overlay_tr("does not publish evidence back to ROBOT",
 "这些证据不会发布回机器人"),
 static_cast<int>(x + 18), static_cast<int>(y + 19), 13, kMuted);
  if (!experience.running_prompt.empty()) {
   draw_overlay_wrapped(experience.running_prompt, x + 18, y + 48,
    width - 36, 12, 3, kText);
  }
 return;
 }

 const bool briefing = phase == ExperiencePhase::kBriefing;
 const bool compact_briefing = briefing &&
 view_layer == ExperienceViewLayer::kPlayer;
 const float screen_width = static_cast<float>(GetScreenWidth());
 const float screen_height = static_cast<float>(GetScreenHeight());
 const float presentation_scale = text_scale;
 const float panel_width = briefing
 ? std::min(400.0f * presentation_scale, screen_width - 48.0f)
 : std::min(720.0f * presentation_scale, screen_width - 100.0f);
 const float panel_height = briefing
 ? std::min((compact_briefing ? 430.0f : 700.0f) * presentation_scale,
 screen_height - 48.0f)
 : phase == ExperiencePhase::kPrediction
 ? std::min(610.0f * presentation_scale, screen_height - 48.0f)
 : std::min(700.0f * presentation_scale, screen_height - 48.0f);
 const float x = briefing ? 24.0f : (screen_width - panel_width) * 0.5f;
 const float y0 = (static_cast<float>(GetScreenHeight()) - panel_height) * 0.5f;
 DrawRectangle(0, 0, GetScreenWidth(), GetScreenHeight(),
 briefing ? Color{5, 10, 12, 38} : Color{5, 10, 12, 150});
 DrawRectangleRounded({x, y0, panel_width, panel_height}, 0.035f, 10,
 kPanel);
 DrawRectangleRoundedLinesEx({x, y0, panel_width, panel_height}, 0.035f, 10,
 2.0f, phase == ExperiencePhase::kDebrief ? kAmber : kCyan);
 draw_overlay_text(overlay_tr("LOST SIGNAL / FIELD LAB", "失落信号 / 机器人现场实验室"),
 static_cast<int>(x + 34),
 static_cast<int>(y0 + 28), 15, kCyan);
 float y = draw_overlay_wrapped(experience.title, x + 34, y0 + 58,
 panel_width - 68, briefing ? 22.0f : 25.0f, 4.0f, kText);
 y = draw_overlay_wrapped(experience.subtitle, x + 34, y + 7,
 panel_width - 68, 16, 5, kMuted);
 y += 14.0f;

 if (phase == ExperiencePhase::kBriefing) {
 draw_overlay_text(compact_briefing
 ? overlay_tr("PREDICT FIRST / INSPECT WHEN CURIOUS", "先做预测 / 按需探索")
 : overlay_tr("MISSION BRIEF / COMPONENT WALKAROUND",
 "任务简报 / 物理部件检查"),
 static_cast<int>(x + 34), static_cast<int>(y), 15, kAmber);
  y += 27.0f;
 const std::size_t briefing_lines = compact_briefing
 ? std::min<std::size_t>(1, experience.briefing.size())
 : experience.briefing.size();
 for (std::size_t line_index = 0;
 line_index < briefing_lines; ++line_index) {
 const auto& line = experience.briefing[line_index];
   DrawCircle(static_cast<int>(x + 42), static_cast<int>(y + 6), 3.0f,
    kCyan);
   y = draw_overlay_wrapped(line, x + 54, y, panel_width - 92, 13, 4,
    kText) + 4.0f;
  }
  y += 5.0f;
 if (!experience.walkaround.empty() && !compact_briefing) {
 const int count = static_cast<int>(experience.walkaround.size());
 const int index = std::clamp(walkaround_stop, 0, count - 1);
 const auto& stop = experience.walkaround[static_cast<std::size_t>(index)];
 char progress[96];
 if (required_walkaround_count == 0) {
 std::snprintf(progress, sizeof(progress),
 zh_cn ? "物理部件 %02d / %02d　已检查 %02zu / %02d"
 : "PHYSICAL COMPONENT %02d / %02d    INSPECTED %02zu / %02d",
 index + 1, count, walkaround_visited_count, count);
 } else {
 std::snprintf(progress, sizeof(progress),
 zh_cn ? "物理部件 %02d / %02d　必看 %02zu / %02zu"
 : "PHYSICAL COMPONENT %02d / %02d REQUIRED %02zu / %02zu",
 index + 1, count, required_walkaround_visited_count,
 required_walkaround_count);
 }
 draw_overlay_text(progress, static_cast<int>(x + 34), static_cast<int>(y), 13,
 kMuted);
   y += 23.0f;
   y = draw_overlay_wrapped(stop.title, x + 34, y, panel_width - 68, 18, 4,
    kCyan) + 9.0f;
   y = draw_overlay_wrapped(stop.lesson, x + 34, y, panel_width - 68, 14, 5,
    kText) + 10.0f;
 draw_overlay_text(overlay_tr("POSE SOURCE", "姿态来源"), static_cast<int>(x + 34),
 static_cast<int>(y),
 13, kAmber);
 y += 22.0f;
 const std::string source = stop.component_id + overlay_tr(" / MuJoCo body + joint", " / MuJoCo 刚体与关节");
 draw_overlay_wrapped(source, x + 34, y, panel_width - 68, 14, 5, kMuted);
 }
 if (compact_briefing && !experience.walkaround.empty()) {
 draw_overlay_wrapped(
 overlay_tr("Component cards are optional. Press U for the teaching layer, or predict now.",
 "部件卡片为可选教学内容。按 U 进入教学层，或现在直接预测。"),
 x + 34, y, panel_width - 68, 13, 4, kMuted);
 }
 const int tab_count = static_cast<int>(experience.walkaround.size());
 const int selected_tab = std::clamp(walkaround_stop, 0,
 std::max(0, tab_count - 1));
 const float tab_y = y0 + panel_height - 108.0f;
 const float tab_gap = 6.0f;
 const float tab_width = (panel_width - 68.0f -
 tab_gap * static_cast<float>(std::max(0, tab_count - 1))) /
 static_cast<float>(std::max(1, tab_count));
 for (int tab = 0; tab < tab_count; ++tab) {
 const auto& tab_stop =
 experience.walkaround[static_cast<std::size_t>(tab)];
 const char* label = tab_stop.component_id == "drive_wheel"
 ? overlay_tr("WHEEL", "DRIVE")
 : tab_stop.component_id == "caster"
 ? overlay_tr("CASTER", "CASTER")
 : tab_stop.component_id == "lidar"
 ? "LiDAR"
 : overlay_tr("BODY", "BODY");
 const Rectangle tab_bounds{
 x + 34.0f + static_cast<float>(tab) * (tab_width + tab_gap),
 tab_y, tab_width, 32.0f};
 draw_overlay_button(tab_bounds, label, tab == selected_tab,
 tab_stop.required ? kCyan : Color{93, 126, 119, 255}, kText);
 if (hit_regions != nullptr) {
 hit_regions->walkaround.push_back(tab_bounds);
 }
 }
 const float button_y = y0 + panel_height - 64.0f;
 const Rectangle primary{x + 34.0f, button_y,
 panel_width - 68.0f, 36.0f};
  char primary_label[64];
  if (walkaround_complete) {
 std::snprintf(primary_label, sizeof(primary_label), "%s",
 overlay_tr("MAKE PREDICTION", "开始预测"));
  } else {
 std::snprintf(primary_label, sizeof(primary_label),
 zh_cn ? "检查必看部件 %zu/%zu" : "INSPECT REQUIRED %zu/%zu",
 required_walkaround_visited_count, required_walkaround_count);
 }
  draw_overlay_button(primary, primary_label, true, kCyan, kText,
   walkaround_complete);
 if (hit_regions != nullptr) {
   if (walkaround_complete) hit_regions->primary = primary;
 }
 return;
 }

 if (phase == ExperiencePhase::kPrediction) {
 draw_overlay_text(overlay_tr("PREDICT FIRST", "先做预测"), static_cast<int>(x + 34),
 static_cast<int>(y), 15, kAmber);
 y = draw_overlay_wrapped(experience.prediction_question, x + 34, y + 28,
 panel_width - 68, 19, 6, kText) + 10.0f;
 for (std::size_t i = 0; i < experience.choices.size(); ++i) {
 const float card_y = y;
 const Rectangle card{x + 32, card_y, panel_width - 64, 78};
 const bool hovered = CheckCollisionPointRec(GetMousePosition(), card);
 DrawRectangleRounded(card, 0.08f, 8,
 hovered ? Color{42, 67, 66, 255} : Color{29, 48, 48, 255});
 if (hovered) DrawRectangleRoundedLinesEx(card, 0.08f, 8, 1.0f, kCyan);
 if (hit_regions != nullptr) hit_regions->choices.push_back(card);
 const char key[2]{static_cast<char>('1' + i), '\0'};
 DrawCircle(static_cast<int>(x + 58), static_cast<int>(card_y + 39),
 15.0f, kCyan);
 draw_overlay_text(key, static_cast<int>(x + 53), static_cast<int>(card_y + 29),
 19, Color{7, 18, 18, 255});
 draw_overlay_wrapped(experience.choices[i].label, x + 84, card_y + 17,
 panel_width - 132, 16, 4, kText);
 y += 91.0f;
 }
 draw_overlay_text(overlay_tr("CLICK A CHOICE OR PRESS ITS NUMBER TO START",
 "点击一个预测，或按数字键开始实验"),
 static_cast<int>(x + 34), static_cast<int>(y0 + panel_height - 40),
 15, kAmber);
 return;
}

}

}  // namespace lost_signal::sim::viewer
