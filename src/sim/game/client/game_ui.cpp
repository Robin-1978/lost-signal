#include "game_ui.hpp"
#include "sim/game/mission/campaign_progress.hpp"

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iomanip>
#include <memory>
#include <sstream>
#include <vector>

namespace client {
namespace {

constexpr float kPanelWidth = 360.0f;
constexpr float kMargin = 24.0f;
constexpr float kContentWidth = 312.0f;
constexpr double kRadToDeg = 57.29577951308232;

std::size_t utf8_sequence_length(unsigned char lead) {
 if ((lead & 0x80U) == 0)
 return 1;
 if ((lead & 0xE0U) == 0xC0U)
 return 2;
 if ((lead & 0xF0U) == 0xE0U)
 return 3;
 if ((lead & 0xF8U) == 0xF0U)
 return 4;
 return 1;
}

std::uint32_t decode_utf8(const std::string& text, std::size_t offset, std::size_t length) {
 const auto byte = [&](std::size_t index) { return static_cast<unsigned char>(text[offset + index]); };
 if (length == 1)
 return byte(0);
 if (length == 2)
 return static_cast<std::uint32_t>((byte(0) & 0x1FU) << 6U) |
 static_cast<std::uint32_t>(byte(1) & 0x3FU);
 if (length == 3)
 return static_cast<std::uint32_t>((byte(0) & 0x0FU) << 12U) |
 static_cast<std::uint32_t>((byte(1) & 0x3FU) << 6U) |
 static_cast<std::uint32_t>(byte(2) & 0x3FU);
 return static_cast<std::uint32_t>((byte(0) & 0x07U) << 18U) |
 static_cast<std::uint32_t>((byte(1) & 0x3FU) << 12U) |
 static_cast<std::uint32_t>((byte(2) & 0x3FU) << 6U) |
 static_cast<std::uint32_t>(byte(3) & 0x3FU);
}

std::size_t glyph_columns(std::uint32_t codepoint) {
 const bool wide = (codepoint >= 0x1100U && codepoint <= 0x115FU) ||
 (codepoint >= 0x2E80U && codepoint <= 0xA4CFU) ||
 (codepoint >= 0xAC00U && codepoint <= 0xD7A3U) ||
 (codepoint >= 0xF900U && codepoint <= 0xFAFFU) ||
 (codepoint >= 0xFE10U && codepoint <= 0xFE6FU) ||
 (codepoint >= 0xFF00U && codepoint <= 0xFF60U) ||
 (codepoint >= 0xFFE0U && codepoint <= 0xFFE6U) ||
 (codepoint >= 0x1F300U && codepoint <= 0x1FAFFU);
 return wide ? 2 : 1;
}

std::size_t display_columns(const std::string& text) {
 std::size_t columns = 0;
 for (std::size_t offset = 0; offset < text.size();) {
 const std::size_t requested = utf8_sequence_length(static_cast<unsigned char>(text[offset]));
 const std::size_t length = std::min(requested, text.size() - offset);
 columns += glyph_columns(decode_utf8(text, offset, length));
 offset += length;
 }
 return columns;
}

std::vector<std::string> split_token(const std::string& token, std::size_t columns) {
 std::vector<std::string> chunks;
 std::size_t start = 0;
 std::size_t used = 0;
 for (std::size_t offset = 0; offset < token.size();) {
 const std::size_t requested = utf8_sequence_length(static_cast<unsigned char>(token[offset]));
 const std::size_t length = std::min(requested, token.size() - offset);
 const std::size_t glyph_width = glyph_columns(decode_utf8(token, offset, length));
 if (used > 0 && used + glyph_width > columns) {
 chunks.push_back(token.substr(start, offset - start));
 start = offset;
 used = 0;
 }
 used += glyph_width;
 offset += length;
 }
 chunks.push_back(token.substr(start));
 return chunks;
}

std::string wrap_words(const std::string& text, std::size_t columns) {
 if (columns == 0 || display_columns(text) <= columns)
 return text;
 std::istringstream input(text);
 std::ostringstream output;
 std::string word;
 std::size_t line_length = 0;
 while (input >> word) {
 const auto word_columns = display_columns(word);
 if (word_columns > columns) {
 if (line_length != 0)
 output << '\n';
 const auto chunks = split_token(word, columns);
 for (std::size_t index = 0; index < chunks.size(); ++index) {
 if (index != 0)
 output << '\n';
 output << chunks[index];
 }
 line_length = display_columns(chunks.back());
 continue;
 }
 if (line_length == 0) {
 output << word;
 line_length = word_columns;
 } else if (line_length + 1 + word_columns <= columns) {
 output << ' ' << word;
 line_length += 1 + word_columns;
 } else {
 output << '\n' << word;
 line_length = word_columns;
 }
 }
 return output.str();
}

std::size_t text_line_count(const std::string& text) {
 return 1 + static_cast<std::size_t>(std::count(text.begin(), text.end(), '\n'));
}

tgui::Label::Ptr make_tooltip(const std::string& text) {
 auto tooltip = tgui::Label::create(text);
 tooltip->setTextSize(13);
 tooltip->setMaximumTextWidth(280.0f);
 auto renderer = tooltip->getRenderer();
 renderer->setTextColor({232, 235, 229});
 renderer->setBackgroundColor({18, 24, 23, 246});
 renderer->setBorderColor({94, 126, 116, 230});
 renderer->setBorders({1});
 renderer->setPadding({9, 7});
 return tooltip;
}

std::string format_number(double value, int precision = 1) {
 std::ostringstream output;
 output << std::fixed << std::setprecision(precision) << value;
 return output.str();
}

std::string format_delta(double value, int precision = 1) {
 return (value > 0.0 ? "+" : "") + format_number(value, precision);
}

std::string evidence_value(double value, const std::string& unit, int precision = 1) {
 return std::isfinite(value) ? format_number(value, precision) + unit : "—";
}

std::string decision_reason(const mission::ContentCatalog& catalog,
 const mission::DecisionEvidence& decision) {
 if (decision.reason_code == "mission-start")
 return catalog.tr("decision.reason.mission-start");
 if (decision.reason_code == "no-route")
 return catalog.tr("decision.reason.no-route");
 if (decision.reason_code == "strategy-hold")
 return catalog.tr("decision.reason.strategy-hold");
 if (decision.reason_code == "route-clear")
 return catalog.tr("decision.reason.route-clear");
 if (decision.reason_code == "no-dynamic-track")
 return catalog.tr("decision.reason.no-dynamic-track");
 if (decision.reason_code == "dynamic-track-clear")
 return catalog.tr("decision.reason.dynamic-track-clear");
 if (decision.reason_code == "no-safe-local-trajectory")
 return catalog.tr("decision.reason.no-safe-local-trajectory");
 if (decision.reason_code == "entry-window-occupied")
 return catalog.tr("decision.reason.entry-window-occupied");
 if (decision.reason_code == "predicted-path-conflict")
 return catalog.tr("decision.reason.predicted-path-conflict");
 if (decision.reason_code == "safe-local-trajectory")
 return catalog.tr("decision.reason.safe-local-trajectory");
 return catalog.tr("decision.reason.unassessed");
}

std::string decision_action(const mission::ContentCatalog& catalog,
 const mission::DecisionEvidence& decision) {
 if (decision.action_code == "plan-route")
 return catalog.tr("decision.action.plan-route");
 if (decision.action_code == "follow-route")
 return catalog.tr("decision.action.follow-route");
 if (decision.action_code == "avoid")
 return catalog.tr("decision.action.avoid");
 if (decision.action_code == "wait")
 return catalog.tr("decision.action.wait");
 if (decision.action_code == "minimum-risk-escape")
 return catalog.tr("decision.action.minimum-risk-escape");
 return catalog.tr("decision.action.idle");
}

std::vector<std::string> decision_arguments(const mission::ContentCatalog& catalog,
 const mission::DecisionEvidence& decision) {
 return {
 std::to_string(decision.observation_refs.size()),
 evidence_value(decision.observation_time_s, " s"),
 evidence_value(decision.measured_separation_m, " m"),
 evidence_value(decision.prediction_horizon_s, " s"),
 evidence_value(decision.predicted_separation_m, " m"),
 decision_reason(catalog, decision),
 decision_action(catalog, decision),
 };
}

std::string selected_id(const mission::BuildSelection& build, const std::string& slot) {
 return build.part(slot);
}

void assign_id(mission::BuildSelection& build, const std::string& slot, const std::string& id) {
 build.parts[slot] = id;
}

std::string field_kind_label(const mission::ContentCatalog& catalog, const std::string& kind) {
 if (kind == "delivery")
 return catalog.tr("field.kind.delivery");
 if (kind == "assist")
 return catalog.tr("field.kind.assist");
 return catalog.tr("field.kind.survey");
}

std::string field_step_label(const mission::ContentCatalog& catalog,
 const mission::FieldDispatchState& dispatch) {
 if (dispatch.kind == "delivery") {
 if (dispatch.stage == 0)
 return catalog.tr("field.step.pickup");
 return catalog.tr("field.step.deliver");
 }
 if (dispatch.kind == "assist")
 return catalog.tr("field.step.assist");
 return catalog.tr("field.step.survey");
}

std::string field_intent_label(const mission::ContentCatalog& catalog,
 mission::FieldIntentKind kind) {
 switch (kind) {
 case mission::FieldIntentKind::Follow:
 return catalog.tr("field.intent.follow");
 case mission::FieldIntentKind::GoTo:
 return catalog.tr("field.intent.go_to");
 case mission::FieldIntentKind::Search:
 return catalog.tr("field.intent.search");
 case mission::FieldIntentKind::Wait:
 return catalog.tr("field.intent.wait");
 case mission::FieldIntentKind::Return:
 return catalog.tr("field.intent.return");
 }
 return catalog.tr("field.intent.wait");
}

std::string field_ack_label(const mission::ContentCatalog& catalog,
 mission::FieldIntentAcknowledgement acknowledgement) {
 switch (acknowledgement) {
 case mission::FieldIntentAcknowledgement::Pending:
 return catalog.tr("field.ack.pending");
 case mission::FieldIntentAcknowledgement::Accepted:
 return catalog.tr("field.ack.accepted");
 case mission::FieldIntentAcknowledgement::Rejected:
 return catalog.tr("field.ack.rejected");
 case mission::FieldIntentAcknowledgement::Idle:
 break;
 }
 return catalog.tr("field.ack.idle");
}

std::string contract_axis_label(const mission::ContentCatalog& catalog, const std::string& axis) {
 if (axis == "fast")
 return catalog.tr("contract.axis.fast");
 if (axis == "efficient")
 return catalog.tr("contract.axis.efficient");
 return catalog.tr("contract.axis.safe");
}

std::string weather_label(const mission::ContentCatalog& catalog, mission::WeatherKind weather) {
 switch (weather) {
 case mission::WeatherKind::Clear:
 return catalog.tr("weather.clear");
 case mission::WeatherKind::Overcast:
 return catalog.tr("weather.overcast");
 case mission::WeatherKind::Drizzle:
 return catalog.tr("weather.drizzle");
 case mission::WeatherKind::Fog:
 return catalog.tr("weather.fog");
 }
 return catalog.tr("weather.clear");
}

std::string incident_label(const mission::ContentCatalog& catalog,
 mission::WorldIncidentKind incident) {
 switch (incident) {
 case mission::WorldIncidentKind::RoadClosure:
 return catalog.tr("incident.road_closure");
 case mission::WorldIncidentKind::FallenCargo:
 return catalog.tr("incident.fallen_cargo");
 case mission::WorldIncidentKind::PowerFault:
 return catalog.tr("incident.power_fault");
 }
 return catalog.tr("incident.road_closure");
}

std::string field_time(double hours) {
 const int minutes = static_cast<int>(std::floor(hours * 60.0 + 0.5)) % (24 * 60);
 std::ostringstream value;
 value << std::setfill('0') << std::setw(2) << minutes / 60 << ':'
 << std::setw(2) << minutes % 60;
 return value.str();
}

} // namespace

GameUi::GameUi(tgui::BackendGui& gui, const mission::ContentCatalog& catalog,
               const mission::PresentationDefinition& presentation)
    : gui_(gui), catalog_(catalog), presentation_(&presentation), build_(presentation.default_build) {
 tgui::ToolTip::setInitialDelay(std::chrono::milliseconds(350));
 tgui::ToolTip::setDistanceToMouse({12, 18});
 const auto contract = std::find_if(catalog_.contracts().begin(), catalog_.contracts().end(),
 [&](const auto& job) { return job.level == presentation_->level; });
 if (contract != catalog_.contracts().end())
 active_contract_id_ = contract->id;
 panel_ = tgui::Panel::create();
 gui_.add(panel_, "control-panel");
 style_panel();
 resize(width_, height_);
 rebuild(nullptr);
 layout_ready_ = true;
}

void GameUi::style_panel() {
 auto renderer = panel_->getRenderer();
 renderer->setBackgroundColor({12, 15, 18});
 renderer->setBorderColor({63, 65, 61});
 renderer->setBorders({1, 0, 0, 0});
}

void GameUi::resize(int width, int height, const mission::ScenarioSession* mission) {
 width_ = width;
 height_ = height;
 panel_->setPosition(std::max(0.0f, static_cast<float>(width) - kPanelWidth), 0);
 panel_->setSize(std::min(kPanelWidth, static_cast<float>(width)), static_cast<float>(height));
 if (layout_ready_)
 rebuild(mission);
}

bool GameUi::world_pointer_available(float screen_x) const {
 return screen_x < std::max(0.0f, static_cast<float>(width_) - kPanelWidth);
}

tgui::Label::Ptr GameUi::add_label(const std::string& text, float x, float y, float width, unsigned text_size,
                                   tgui::Color color, bool bold) {
 auto label = tgui::Label::create(text);
 label->setPosition(x, y);
 label->setTextSize(text_size);
 label->setMaximumTextWidth(width);
 label->getRenderer()->setTextColor(color);
 if (bold) {
  label->getRenderer()->setTextStyle(tgui::TextStyle::Bold);
 }
 add_parent_->add(label);
 return label;
}

tgui::Button::Ptr GameUi::add_button(const std::string& text, float x, float y, float width, float height,
                                     bool primary) {
 auto button = tgui::Button::create(text);
 button->setPosition(x, y);
 button->setSize(width, height);
 button->setTextSize(19);
 auto renderer = button->getRenderer();
 renderer->setRoundedBorderRadius(9);
 renderer->setBorders({primary ? 0.0f : 1.0f});
 renderer->setTextColor(primary ? tgui::Color{25, 25, 20} : tgui::Color{225, 224, 215});
 renderer->setTextColorHover(primary ? tgui::Color{17, 17, 14} : tgui::Color::White);
 renderer->setBackgroundColor(primary ? tgui::Color{232, 172, 80} : tgui::Color{26, 30, 31});
 renderer->setBackgroundColorHover(primary ? tgui::Color{247, 193, 104} : tgui::Color{38, 44, 43});
 renderer->setBackgroundColorDisabled({19, 22, 22});
 renderer->setBorderColor(primary ? tgui::Color{232, 172, 80} : tgui::Color{64, 69, 67});
 renderer->setBorderColorHover(primary ? tgui::Color{247, 193, 104} : tgui::Color{102, 118, 111});
 renderer->setBorderColorDisabled({42, 47, 45});
 renderer->setTextColorDisabled({91, 98, 94});
 add_parent_->add(button);
 return button;
}

void GameUi::rebuild(const mission::ScenarioSession* mission) {
 panel_->removeAllWidgets();
 add_parent_ = panel_;
 style_panel();
 slot_cards_.clear();
 chassis_cards_.clear();
 task_policy_cards_.clear();
 style_slider_ = nullptr;
 style_value_ = nullptr;
 budget_label_ = nullptr;
 garage_heading_label_ = nullptr;
 garage_next_button_ = nullptr;
 endless_checkbox_ = nullptr;
 running_metrics_ = nullptr;
 running_state_ = nullptr;
 running_explanation_card_ = nullptr;
 pause_button_ = nullptr;
 replay_slider_ = nullptr;
 replay_value_ = nullptr;
 replay_cause_ = nullptr;
 replay_evidence_ = nullptr;
 comparison_label_ = nullptr;
 switch (screen_) {
 case Screen::Chapter:
  build_chapter();
  break;
 case Screen::Garage:
  build_garage();
  break;
 case Screen::Prediction:
  build_prediction();
  break;
 case Screen::Running:
  build_running(mission);
  break;
 case Screen::Debrief:
  build_debrief(mission);
  break;
 }
}

bool GameUi::is_unlocked(const mission::LevelDefinition& level) const {
 return std::all_of(level.prerequisites.begin(), level.prerequisites.end(),
                    [this](const auto& id) { return completed_levels_.contains(id); });
}

std::string GameUi::next_level_id() const {
 const auto& levels = catalog_.campaign().levels;
 const auto found = std::find(levels.begin(), levels.end(), presentation_->level);
 if (found == levels.end() || std::next(found) == levels.end())
  return {};
 const auto& next = *std::next(found);
 if (catalog_.level(next).mode != catalog_.level(*found).mode)
  return {};
 return next;
}

void GameUi::build_chapter() {
 auto scroll = tgui::ScrollablePanel::create({static_cast<float>(kPanelWidth), static_cast<float>(height_)});
 scroll->setPosition(0, 0);
 {
  auto look = scroll->getRenderer();
  look->setBackgroundColor({0, 0, 0, 0});
  look->setBorderColor({0, 0, 0, 0});
  look->setBorders({0});
 }
 scroll->getHorizontalScrollbar()->setPolicy(tgui::Scrollbar::Policy::Never);
 scroll->getVerticalScrollbar()->setPolicy(tgui::Scrollbar::Policy::Automatic);
 panel_->add(scroll);
 add_parent_ = scroll;
 float y = 24.0f;
 const auto place_text = [&](const std::string& text, unsigned size, tgui::Color color,
 bool bold = false, float gap = 8.0f) {
 auto label = add_label(text, kMargin, y, kContentWidth, size, color, bold);
 y += std::max(static_cast<float>(size) * 1.35f, label->getFullSize().y) + gap;
 return label;
 };
 place_text(catalog_.tr("chapter.game_title"), 14, {107, 190, 176}, true, 5);
 place_text(catalog_.campaign().title, 25, {239, 238, 229}, true, 8);
 place_text(catalog_.tr("chapter.subtitle"), 14, {157, 163, 158}, false, 10);
 place_text(catalog_.trf("chapter.contract_balance", {std::to_string(reputation_)}),
 14, {232, 172, 80}, true, 14);

 int shown_chapter = 0;
 for (const auto& level_id : catalog_.campaign().levels) {
  const auto& level = catalog_.level(level_id);
  if (level.chapter != shown_chapter) {
   shown_chapter = level.chapter;
   const mission::ChapterDefinition* chapter = nullptr;
   for (const auto& entry : catalog_.chapters()) {
    if (entry.id == shown_chapter) {
     chapter = &entry;
    }
   }
   const std::string heading =
     chapter != nullptr ? chapter->title : catalog_.trf("chapter.fallback", {std::to_string(shown_chapter)});
 place_text(heading, 17, {107, 190, 176}, true, 7);
  }
  const bool complete = completed_levels_.contains(level_id);
  const bool unlocked = is_unlocked(level);
  const std::string prefix = complete ? "✓ " : unlocked ? "○ " : "— ";
  const auto contract = std::find_if(catalog_.contracts().begin(), catalog_.contracts().end(),
    [&](const auto& job) { return job.level == level_id; });
 std::string button_text = wrap_words(prefix + std::to_string(level.order) + " " + level.title, 34);
 unsigned button_text_size = 17;
  if (contract != catalog_.contracts().end()) {
   const bool claimed = claimed_contracts_.contains(contract->id);
 button_text += "\n" + wrap_words(catalog_.trf(
     claimed ? "chapter.contract_claimed" : "chapter.contract_offer",
 {contract_axis_label(catalog_, contract->axis), std::to_string(contract->rep)}), 38);
 button_text_size = 13;
  }
 const float button_height =
 std::max(46.0f, 14.0f + static_cast<float>(text_line_count(button_text)) * (button_text_size + 4.0f));
  auto button = add_button(button_text, kMargin, y, kContentWidth, button_height,
    level_id == presentation_->level && unlocked);
 button->setTextSize(button_text_size);
 std::string tooltip_text = level.title + "\n" + level.knowledge_card;
 if (contract != catalog_.contracts().end())
 tooltip_text += "\n\n" + contract->blurb;
 button->setToolTip(make_tooltip(tooltip_text));
  button->setEnabled(unlocked);
  button->onClick([this, level_id] { actions_.selected_level = level_id; });
  y += button_height + 10.0f;
 }
 if (!catalog_.campaign().playgrounds.empty()) {
 y += 8.0f;
 place_text(catalog_.tr("chapter.playgrounds_heading"), 18, {232, 172, 80}, true, 5);
 place_text(catalog_.tr("chapter.playgrounds_subtitle"), 13, {157, 163, 158}, false, 10);
 for (const auto& level_id : catalog_.campaign().playgrounds) {
 const auto& level = catalog_.level(level_id);
 const bool complete = completed_levels_.contains(level_id);
 const std::string prefix = complete ? "✓ " : "◇ ";
 const std::string button_text = wrap_words(prefix + level.title, 34);
 const float button_height =
 std::max(46.0f, 14.0f + static_cast<float>(text_line_count(button_text)) * 21.0f);
 auto button = add_button(button_text, kMargin, y, kContentWidth, button_height,
 level_id == presentation_->level);
 button->setTextSize(17);
 button->setToolTip(make_tooltip(level.title + "\n" + level.knowledge_card));
 button->onClick([this, level_id] { actions_.selected_level = level_id; });
 y += button_height + 10.0f;
 }
 }
 y += 4.0f;
 place_text(catalog_.tr("chapter.footer_head") + " " + catalog_.tr("chapter.footer_tail"),
 13, {137, 151, 145}, false, 24);
 scroll->setContentSize({static_cast<float>(kPanelWidth), y});
}

void GameUi::set_screen(Screen screen, const mission::ScenarioSession* mission) {
 if (screen == Screen::Debrief && mission) {
 const auto previous = previous_results_.find(mission->level().id);
 comparison_result_ = previous == previous_results_.end()
 ? std::optional<mission::MissionResult>{}
 : std::optional<mission::MissionResult>{previous->second};
 previous_results_[mission->level().id] = mission->result();
 } else if (screen != Screen::Debrief) {
 comparison_result_.reset();
 }
 screen_ = screen;
 rebuild(mission);
}

void GameUi::build_garage() {
 // Same overflow story as the chapter roster: 5 slots + style + budget never
 // fit 900px. Scroll it; widgets keep working (ptrs retained, parent only).
 auto scroll = tgui::ScrollablePanel::create({static_cast<float>(kPanelWidth), static_cast<float>(height_)});
 scroll->setPosition(0, 0);
 // Transparent like the parent panel: ScrollablePanel defaults to a WHITE
 // background (shipped regression once — chapter screen went white).
 {
  auto look = scroll->getRenderer();
  look->setBackgroundColor({0, 0, 0, 0});
  look->setBorderColor({0, 0, 0, 0});
  look->setBorders({0});
 }
 scroll->getHorizontalScrollbar()->setPolicy(tgui::Scrollbar::Policy::Never);
 scroll->getVerticalScrollbar()->setPolicy(tgui::Scrollbar::Policy::Automatic);
 panel_->add(scroll);
 add_parent_ = scroll;
 float y = 22.0f;
 garage_heading_label_ = add_label(catalog_.trf("garage.heading", {selected_robot_title()}),
 kMargin, y, 184, 14,
 {107, 190, 176}, true);
 auto chapter_button = add_button(catalog_.tr("chapter.garage_button"), kMargin + 194, y - 6, 118, 32);
 chapter_button->setTextSize(14);
 chapter_button->onClick([this] { actions_.open_chapter = true; });
 y += std::max(32.0f, garage_heading_label_->getFullSize().y) + 8.0f;
 const auto place_text = [&](const std::string& text, unsigned size, tgui::Color color,
 bool bold = false, float gap = 8.0f) {
 auto label = add_label(text, kMargin, y, kContentWidth, size, color, bold);
 y += std::max(static_cast<float>(size) * 1.35f, label->getFullSize().y) + gap;
 return label;
 };
 place_text(presentation_->garage_heading, 25, {239, 238, 229}, true, 10);
 place_text(presentation_->garage_brief, 15, {157, 163, 158}, false, 12);

 if (const auto* beat = catalog_.story_beat(presentation_->level, "before")) {
 place_text(beat->speaker + ": " + beat->text, 14, {184, 199, 190}, false, 12);
 }
 if (!active_contract_id_.empty()) {
 const auto contract = std::find_if(catalog_.contracts().begin(), catalog_.contracts().end(),
 [&](const auto& job) { return job.id == active_contract_id_; });
 if (contract != catalog_.contracts().end()) {
 place_text(catalog_.trf("garage.contract_payload",
 {contract->blurb, format_number(contract->mass_kg, 0),
 format_number(contract->cog_height_m, 2)}),
 14, {232, 172, 80}, false, 14);
 }
 }
 constexpr float kCardGap = 8.0f;
 constexpr float kCardHeight = 70.0f;

 if (!presentation_->chassis_options.empty()) {
 place_text(catalog_.tr("garage.chassis"), 16, {132, 139, 135}, true, 4);
 for (const auto& robot_id : presentation_->chassis_options) {
 const auto& robot = catalog_.robot(robot_id);
 auto card = tgui::Button::create(" ");
 card->setPosition(kMargin, y);
 card->setSize(kContentWidth, kCardHeight);
 card->setTextSize(14);
 auto look = card->getRenderer();
 look->setRoundedBorderRadius(8);
 look->setBorders({1.0f});
 add_parent_->add(card);
 card->setToolTip(make_tooltip(robot.title + "\n" + catalog_.tr(robot.drive_label_key)));
 card->onClick([this, robot_id] { select_chassis(robot_id); });
 chassis_cards_.push_back({robot_id, card});
 y += kCardHeight + kCardGap;
 }
 y += 8.0f;
 }

 for (const auto& column : presentation_->component_columns) {
 if (!column.player_choice)
 continue;
 place_text(column.title, 16, {132, 139, 135}, true, 4);
  for (const auto& option : column.options) {
   const auto& item = catalog_.component(option);
   auto card = tgui::Button::create(" ");
 card->setPosition(kMargin, y);
 card->setSize(kContentWidth, kCardHeight);
   card->setTextSize(14);
   card->setText(card_text(column.slot, item, false));
   auto look = card->getRenderer();
   look->setRoundedBorderRadius(8);
   look->setBorders({1.0f});
   add_parent_->add(card);
 card->setToolTip(make_tooltip(item.title + "\n" + item.description + "\n" + card_stat(column.slot, item)));
 card->onClick([this, slot = column.slot, id = option] {
 const bool open_loadout = catalog_.level(presentation_->level).loadout_access == "open";
 if (open_loadout || owned_.empty() || owned_.contains(id))
 select_component(slot, id);
 else
 actions_.purchase_component = id;
 });
   slot_cards_[column.slot].push_back({option, card});
 y += kCardHeight + kCardGap;
  }
 y += 8.0f;
 }
 if (catalog_.level(presentation_->level).objective.goal_order == "autonomous") {
 place_text(catalog_.tr("garage.task_policy"), 16, {132, 139, 135}, true, 4);
 for (const auto& policy_id : presentation_->task_policies) {
 const auto& policy = catalog_.task_policy(policy_id);
 auto card = tgui::Button::create(policy.title);
 card->setPosition(kMargin, y);
 card->setSize(kContentWidth, 48.0f);
 card->setTextSize(14);
 auto look = card->getRenderer();
 look->setRoundedBorderRadius(8);
 look->setBorders({1.0f});
 add_parent_->add(card);
 card->setToolTip(make_tooltip(policy.title + "\n" + policy.description));
 card->onClick([this, choice = policy_id] { select_task_policy(choice); });
 task_policy_cards_.push_back({policy_id, card});
 y += 56.0f;
 }
 y += 8.0f;
 }
 place_text(presentation_->policy_title, 14, {132, 139, 135}, true, 6);
 style_slider_ = tgui::Slider::create(0, 100);
 style_slider_->setPosition(kMargin, y);
 style_slider_->setSize(kContentWidth, 18);
 style_slider_->setValue(static_cast<float>(build_.style * 100.0));
 style_slider_->onValueChange([this](float value) { set_style(static_cast<double>(value) / 100.0); });
 add_parent_->add(style_slider_);
 y += 26.0f;
 add_label(catalog_.tr("garage.style_gentle"), kMargin, y, 80, 14, {132, 139, 135});
 auto style_mid =
 add_label(catalog_.tr("garage.style_normal"), kMargin, y, kContentWidth, 14, {132, 139, 135});
 style_mid->setHorizontalAlignment(tgui::Label::HorizontalAlignment::Center);
 auto style_fast =
 add_label(catalog_.tr("garage.style_fast"), kMargin, y, kContentWidth, 14, {132, 139, 135});
 style_fast->setHorizontalAlignment(tgui::Label::HorizontalAlignment::Right);
 y += 27.0f;
 style_value_ = add_label("", kMargin, y, kContentWidth, 15, {143, 201, 187});
 y += 64.0f;

 endless_checkbox_ = tgui::CheckBox::create(catalog_.tr("garage.endless_mode"));
 endless_checkbox_->setPosition(kMargin, y);
 endless_checkbox_->setTextSize(15);
 endless_checkbox_->setChecked(endless_mode_);
 endless_checkbox_->getRenderer()->setTextColor({184, 199, 190});
 endless_checkbox_->onChange([this](bool checked) {
 endless_mode_ = checked;
 if (garage_next_button_)
 garage_next_button_->setText(catalog_.tr(checked ? "garage.enter_world" : "garage.next"));
 });
 add_parent_->add(endless_checkbox_);
 y += std::max(28.0f, endless_checkbox_->getFullSize().y) + 8.0f;
 auto roles = add_label(catalog_.tr("garage.expedition_control"), kMargin + 22, y,
 kContentWidth - 22, 14, {154, 183, 174});
 y += std::max(20.0f, roles->getFullSize().y) + 12.0f;
 budget_label_ = add_label("", kMargin, y, kContentWidth, 18, {151, 167, 159}, true);
 y += 38.0f;
 auto recommended = add_button(catalog_.tr("garage.use_recommended"), kMargin, y, 162, 42);
 recommended->setTextSize(14);
 recommended->onClick([this] { use_recommended_build(); });
 garage_next_button_ = add_button(catalog_.tr("garage.next"), kMargin + 172, y, 140, 42, true);
 garage_next_button_->setText(
 catalog_.tr(endless_mode_ ? "garage.enter_world" : "garage.next"));
 garage_next_button_->setTextSize(16);
 garage_next_button_->onClick([this] { actions_.open_prediction = true; });
 update_garage_labels();
 scroll->setContentSize({static_cast<float>(kPanelWidth), y + 66.0f});
}

void GameUi::build_prediction() {
 auto scroll = tgui::ScrollablePanel::create(
 {static_cast<float>(kPanelWidth), std::max(1.0f, static_cast<float>(height_) - 88.0f)});
 scroll->setPosition(0, 0);
 scroll->getHorizontalScrollbar()->setPolicy(tgui::Scrollbar::Policy::Never);
 scroll->getVerticalScrollbar()->setPolicy(tgui::Scrollbar::Policy::Automatic);
 auto scroll_renderer = scroll->getRenderer();
 scroll_renderer->setBackgroundColor({0, 0, 0, 0});
 scroll_renderer->setBorderColor({0, 0, 0, 0});
 scroll_renderer->setBorders({0});
 panel_->add(scroll);
 add_parent_ = scroll;
 float y = 24.0f;
 const auto place_text = [&](const std::string& text, unsigned size, tgui::Color color,
 bool bold = false, float gap = 8.0f) {
 auto label = add_label(text, kMargin, y, kContentWidth, size, color, bold);
 y += std::max(static_cast<float>(size) * 1.35f, label->getFullSize().y) + gap;
 return label;
 };
 place_text(catalog_.tr("prediction.departure"), 14, {107, 190, 176}, true, 5);
 place_text(presentation_->prediction_heading, 23, {239, 238, 229}, true, 10);
 place_text(presentation_->prediction_hint, 14, {157, 163, 158}, false, 16);

 for (const auto& choice : presentation_->prediction_choices) {
 const std::string wrapped = wrap_words(choice.text, 38);
 const float height = std::max(58.0f, 20.0f + 19.0f * static_cast<float>(text_line_count(wrapped)));
 auto button = add_button(wrapped, kMargin, y, kContentWidth, height);
 button->setTextSize(15);
 button->setToolTip(make_tooltip(choice.feedback));
  button->onClick([this, id = choice.id] {
   prediction_choice_ = id;
   rebuild(nullptr);
  });
  if (choice.id == prediction_choice_) {
   button->getRenderer()->setBackgroundColor({52, 61, 57});
   button->getRenderer()->setBorderColor({108, 189, 174});
  }
 y += height + 12.0f;
 }
 scroll->setContentSize({static_cast<float>(kPanelWidth), y + 16.0f});
 add_parent_ = panel_;
 auto back = add_button(catalog_.tr("garage.back"), kMargin, height_ - 74.0f, 116, 46);
 back->onClick([this] { actions_.open_garage = true; });
 auto launch = add_button(catalog_.trf("garage.launch", {selected_robot_title()}),
 kMargin + 132, height_ - 74.0f, 180, 46, true);
 launch->setEnabled(!prediction_choice_.empty());
 launch->onClick([this] { actions_.launch = true; });
}

void GameUi::build_running(const mission::ScenarioSession* mission) {
 auto panel_renderer = panel_->getRenderer();
 panel_renderer->setBackgroundColor({0, 0, 0, 0});
 panel_renderer->setBorders({0});

 auto objective_card = tgui::Panel::create({kContentWidth, 100});
 objective_card->setPosition(kMargin, 22);
 objective_card->getRenderer()->setBackgroundColor({12, 17, 17, 218});
 objective_card->getRenderer()->setBorderColor({78, 104, 95, 150});
 objective_card->getRenderer()->setBorders({1});
 objective_card->getRenderer()->setRoundedBorderRadius(12);
 panel_->add(objective_card);
 const auto& level = mission ? mission->level() : catalog_.level(presentation_->level);
 auto chapter = tgui::Label::create(
   catalog_.trf("ui.chapter_row", {std::to_string(level.chapter), std::to_string(level.order), level.title}));
 chapter->setPosition(16, 12);
 chapter->setTextSize(13);
 chapter->setMaximumTextWidth(kContentWidth - 32.0f);
 chapter->getRenderer()->setTextColor({107, 190, 176});
 chapter->getRenderer()->setTextStyle(tgui::TextStyle::Bold);
 objective_card->add(chapter);
 auto objective = tgui::Label::create(
 mission && mission->endless_mode() ? catalog_.tr("running.endless_heading")
 : presentation_->running_heading);
 const float chapter_height = std::max(18.0f, chapter->getFullSize().y);
 objective->setPosition(16, 12.0f + chapter_height + 7.0f);
 objective->setTextSize(18);
 objective->setMaximumTextWidth(kContentWidth - 32.0f);
 objective->getRenderer()->setTextColor({239, 238, 229});
 objective->getRenderer()->setTextStyle(tgui::TextStyle::Bold);
 objective_card->add(objective);
 const float objective_height = std::max(24.0f, objective->getFullSize().y);
 const float objective_card_height = 12.0f + chapter_height + 7.0f + objective_height + 13.0f;
 objective_card->setSize({kContentWidth, objective_card_height});

 auto state_card = tgui::Panel::create({kContentWidth, 128});
 const bool manual = mission && !mission->endless_mode() &&
 mission->level().control_mode == "manual_calibration";
 const bool operator_drive = mission && !mission->endless_mode() &&
 mission->level().control_mode == "operator_drive";
 const bool expedition = mission && mission->endless_mode();
 const float state_y = 22.0f + objective_card_height + 12.0f;
 state_card->setPosition(kMargin, state_y);
 state_card->getRenderer()->setBackgroundColor({19, 27, 25, 225});
 state_card->getRenderer()->setBorderColor({73, 212, 185, 180});
 state_card->getRenderer()->setBorders({0, 0, 0, 3});
 state_card->getRenderer()->setRoundedBorderRadius(12);
 panel_->add(state_card);
 const std::string robot_title = mission ? mission->robot().title : selected_robot_title();
 auto live = tgui::Label::create(expedition ? catalog_.trf("running.live_together", {robot_title})
 : (operator_drive ? catalog_.tr("running.live_operator")
 : (manual ? catalog_.tr("running.live_drive")
 : catalog_.trf("running.live_decide", {robot_title}))));
 live->setPosition(18, 14);
 live->setTextSize(12);
 live->getRenderer()->setTextColor({104, 218, 193});
 live->getRenderer()->setTextStyle(tgui::TextStyle::Bold);
 state_card->add(live);
 running_state_ = tgui::Label::create(presentation_->normal_line);
 running_state_->setPosition(18, 42);
 running_state_->setTextSize(16);
 running_state_->setMaximumTextWidth(276);
 running_state_->getRenderer()->setTextColor({239, 238, 229});
 running_state_->getRenderer()->setTextStyle(tgui::TextStyle::Bold);
 state_card->add(running_state_);

 const float controls_y = std::max(650.0f, static_cast<float>(height_) - 70.0f);
 const float garage_y = controls_y - 48.0f;
 const float explanation_y = state_y + 142.0f;
 const float explanation_height = std::max(180.0f, garage_y - explanation_y - 14.0f);
 auto card = tgui::ScrollablePanel::create({kContentWidth, explanation_height});
 running_explanation_card_ = card;
 card->setPosition(kMargin, explanation_y);
 card->setContentSize({kContentWidth - 12.0f, explanation_height - 4.0f});
 card->getHorizontalScrollbar()->setPolicy(tgui::Scrollbar::Policy::Never);
 card->getVerticalScrollbar()->setPolicy(tgui::Scrollbar::Policy::Automatic);
 card->getRenderer()->setBackgroundColor({10, 14, 15, 210});
 card->getRenderer()->setBorderColor({62, 76, 72, 150});
 card->getRenderer()->setBorders({1});
 card->getRenderer()->setRoundedBorderRadius(12);
 panel_->add(card);
 auto why = tgui::Label::create(expedition ? catalog_.tr("running.partner_title")
 : (manual ? catalog_.tr("running.manual_title")
 : catalog_.tr("running.why")));
 why->setPosition(18, 14);
 why->setTextSize(13);
 why->getRenderer()->setTextColor({150, 177, 168});
 why->getRenderer()->setTextStyle(tgui::TextStyle::Bold);
 card->add(why);
 running_metrics_ = tgui::Label::create(catalog_.tr("running.metrics_idle"));
 running_metrics_->setPosition(18, 42);
 running_metrics_->setTextSize(14);
 running_metrics_->setMaximumTextWidth(264);
 running_metrics_->getRenderer()->setTextColor({194, 199, 194});
 card->add(running_metrics_);

 for (int index = 0; index < 3; ++index) {
  const double speeds[] = {1.0, 2.0, 4.0};
  auto button = add_button(std::to_string(static_cast<int>(speeds[index])) + "×", kMargin + index * 58.0f,
                           controls_y, 50, 38);
  button->onClick([this, speed = speeds[index]] { actions_.speed = speed; });
 }
 pause_button_ = add_button(catalog_.tr("running.pause"), kMargin + 190, controls_y, 122, 38, true);
 pause_button_->onClick([this] { actions_.toggle_pause = true; });
 const bool endless = mission && mission->endless_mode();
 const float garage_x = endless ? kMargin + 160.0f : kMargin;
 const float garage_width = endless ? 152.0f : kContentWidth;
 auto garage = add_button(catalog_.tr("running.to_garage"), garage_x, garage_y,
 garage_width, 36);
 garage->setTextSize(16);
 garage->onClick([this] { actions_.open_garage = true; });
 if (endless) {
 auto reset = add_button(catalog_.tr("running.reset_robot"), kMargin, garage_y, 152, 36);
 reset->onClick([this] { actions_.reset_simulation = true; });
 }
 if (mission) {
  refresh_running(*mission, false, 1.0);
 }
}

void GameUi::build_debrief(const mission::ScenarioSession* mission) {
 if (!mission) {
  return;
 }
 auto scroll = tgui::ScrollablePanel::create(
   {static_cast<float>(kPanelWidth), std::max(1.0f, static_cast<float>(height_) - 88.0f)});
 scroll->setPosition(0, 0);
 scroll->setContentSize({static_cast<float>(kPanelWidth), 900.0f});
 scroll->getHorizontalScrollbar()->setPolicy(tgui::Scrollbar::Policy::Never);
 scroll->getVerticalScrollbar()->setPolicy(tgui::Scrollbar::Policy::Automatic);
 auto scroll_renderer = scroll->getRenderer();
 scroll_renderer->setBackgroundColor({0, 0, 0, 0});
 scroll_renderer->setBorderColor({0, 0, 0, 0});
 scroll_renderer->setBorders({0});
 panel_->add(scroll);
 add_parent_ = scroll;

 const auto& result = mission->result();
 const bool succeeded = result.status == mission::MissionStatus::Succeeded;
 std::string title = succeeded ? presentation_->success_title : catalog_.tr("debrief.interrupted");
 if (!succeeded) {
  const auto found = presentation_->failure_titles.find(result.failure_reason);
  if (found != presentation_->failure_titles.end())
   title = found->second;
 }
 float layout_y = 24.0f;
 const auto advance = [&](float used_height, float gap = 8.0f) {
  layout_y += std::max(1.0f, used_height) + gap;
 };
 const auto placed_text = [&](const std::string& text, unsigned size, tgui::Color color, bool bold = false,
                            float gap = 8.0f) {
  auto label = add_label(text, kMargin, layout_y, kContentWidth, size, color, bold);
  advance(std::max(static_cast<float>(size) * 1.35f, label->getFullSize().y), gap);
  return label;
 };
 placed_text(catalog_.tr("debrief.title"), 14, {107, 190, 176}, true, 4);
 placed_text(title, 24, succeeded ? tgui::Color{105, 235, 193} : tgui::Color{255, 145, 120}, true, 10);
 std::string summary = succeeded ? presentation_->success_summary : presentation_->failure_summary;
 if (succeeded) {
  if (const auto* beat = catalog_.story_beat(presentation_->level, "after")) {
   summary += "\n" + beat->speaker + "：" + beat->text;
  }
 }
 placed_text(summary, 15, {157, 163, 158}, false, 10);

 const auto badge = [&](bool earned, const char* key) {
  return (earned ? catalog_.tr("debrief.badge_on") : catalog_.tr("debrief.badge_off")) + catalog_.tr(key);
 };
 placed_text(badge(result.safe_score, "debrief.badge_safe") + " " +
             badge(result.fast_score, "debrief.badge_fast") + " " +
             badge(result.efficient_score, "debrief.badge_efficient"),
           17, {220, 222, 216}, true, 10);
 std::string metrics =
   catalog_.trf("debrief.metrics", {format_number(result.time_s), format_number(result.path_length_m),
                                    format_number(result.energy_wh, 2)});
 metrics += mission->level().concept_id == "odometry_drift"
              ? catalog_.trf("debrief.metrics_odom", {format_number(result.max_odometry_error_m, 2)})
              : catalog_.trf("debrief.metrics_clear", {format_number(result.scored_clearance_m, 2)});
 placed_text(metrics, 16, {182, 190, 186}, false, 14);

 if (mission->level().objective.goal_order == "autonomous") {
 const auto labels = [&](const std::vector<std::string>& ids) {
 std::string text;
 for (const auto& id : ids) {
 const auto goal = std::find_if(mission->level().objective.goals.begin(),
 mission->level().objective.goals.end(), [&](const auto& candidate) {
 return (candidate.id.empty() ? candidate.marker : candidate.id) == id;
 });
 if (!text.empty())
 text += " → ";
 text += goal == mission->level().objective.goals.end() ? id : goal->label;
 }
 return text.empty() ? catalog_.tr("debrief.none") : text;
 };
 placed_text(catalog_.trf("debrief.task_metrics",
 {labels(mission->snapshot().completed_goal_ids),
 labels(mission->snapshot().abandoned_goal_ids)}),
 14, {166, 187, 178}, false, 10);
 }
 if (result.collected_items > 0 || !mission->level().environment_keyframes.empty()) {
 placed_text(catalog_.trf("debrief.payload_environment_metrics",
 {std::to_string(result.collected_items), format_number(result.final_payload_mass_kg, 1),
 format_number(result.cargo_integrity * 100.0, 1),
 format_number(result.max_rollover_risk * 100.0, 0),
 format_number(result.peak_wetness * 100.0, 0),
 format_number(result.min_visibility * 100.0, 0)}),
 14, {166, 187, 178}, false, 14);
 }

 placed_text(catalog_.tr("debrief.prediction_title"), 15, {107, 190, 176}, true, 6);
 const auto prediction =
 std::find_if(presentation_->prediction_choices.begin(), presentation_->prediction_choices.end(),
              [&](const auto& value) { return value.id == prediction_choice_; });
 if (prediction != presentation_->prediction_choices.end()) {
  placed_text(prediction->text + "\n" + prediction->feedback, 14, {170, 184, 178}, false, 14);
 } else {
  placed_text(catalog_.tr("debrief.prediction_missing"), 14, {132, 145, 139}, false, 14);
 }

 if (comparison_result_) {
  const auto& previous = *comparison_result_;
  comparison_label_ = add_label(
   catalog_.trf("debrief.comparison",
               {format_delta(result.time_s - previous.time_s),
                format_delta(result.energy_wh - previous.energy_wh, 2),
                format_delta(result.scored_clearance_m - previous.scored_clearance_m, 2)}),
   kMargin, layout_y, kContentWidth, 14, {232, 185, 103}, true);
  advance(std::max(19.0f, comparison_label_->getFullSize().y), 14);
 } else {
  comparison_label_ = add_label(catalog_.tr("debrief.comparison_first"), kMargin, layout_y,
                                kContentWidth, 14, {132, 145, 139});
  advance(std::max(19.0f, comparison_label_->getFullSize().y), 14);
 }

 placed_text(catalog_.tr("debrief.replay_title"), 16, {239, 238, 229}, true, 6);
 replay_slider_ = tgui::Slider::create(0.0f, static_cast<float>(std::max(0.05, result.time_s)));
 replay_slider_->setPosition(kMargin, layout_y);
 replay_slider_->setSize(kContentWidth, 18);
 replay_slider_->setValue(static_cast<float>(mission->snapshot().time_s));
 replay_slider_->onValueChange([this](float value) { actions_.replay_time_s = static_cast<double>(value); });
 add_parent_->add(replay_slider_);
 advance(20.0f, 6);
 replay_value_ = add_label("", kMargin, layout_y, kContentWidth, 14, {143, 201, 187}, true);
 advance(std::max(19.0f, replay_value_->getFullSize().y), 4);
 replay_cause_ = add_label("", kMargin, layout_y, kContentWidth, 14, {194, 199, 194});
 advance(std::max(19.0f, replay_cause_->getFullSize().y), 14);

 placed_text(presentation_->evidence_heading, 16, {239, 238, 229}, true, 6);
 replay_evidence_ = add_label("", kMargin, layout_y, kContentWidth, 14, {151, 170, 163});
 advance(std::max(19.0f, replay_evidence_->getFullSize().y), 14);

 placed_text(presentation_->knowledge_title, 16, {107, 190, 176}, true, 4);
 placed_text(mission->level().knowledge_card, 14, {170, 184, 178}, false, 12);
 scroll->setContentSize({static_cast<float>(kPanelWidth), layout_y + 32.0f});


 add_parent_ = panel_;
 if (succeeded) {
  auto chapter = add_button(catalog_.tr("debrief.to_chapter"), kMargin, height_ - 74.0f, 148, 46);
  chapter->onClick([this] { actions_.open_chapter = true; });
  const auto next_id = next_level_id();
  auto next =
    add_button(next_id.empty() ? catalog_.tr("debrief.view_chapter") : catalog_.tr("debrief.next_level"),
               kMargin + 164, height_ - 74.0f, 148, 46, true);
  if (next_id.empty()) {
   next->onClick([this] { actions_.open_chapter = true; });
  } else {
   next->onClick([this, next_id] { actions_.selected_level = next_id; });
  }
 } else {
  auto garage = add_button(catalog_.tr("debrief.to_garage"), kMargin, height_ - 74.0f, 148, 46);
  garage->onClick([this] { actions_.open_garage = true; });
  auto retry = add_button(catalog_.tr("debrief.retry"), kMargin + 164, height_ - 74.0f, 148, 46, true);
  retry->onClick([this] { actions_.retry = true; });
 }
 refresh_debrief(*mission);
}

void GameUi::refresh_debrief(const mission::ScenarioSession& mission) {
 if (!replay_slider_ || !replay_value_ || !replay_cause_ || !replay_evidence_) {
  return;
 }
 const auto& state = mission.snapshot();
 replay_value_->setText(catalog_.trf("debrief.replay_time",
                                     {format_number(state.time_s), format_number(mission.result().time_s)}));

 const double grade_degrees = std::atan(state.terrain_grade) * kRadToDeg;
 const double cross_degrees = std::atan(state.cross_slope) * kRadToDeg;
 const double risk_percent = state.rollover_risk * 100.0;
 if (state.rollover_risk >= 0.75) {
  replay_cause_->setText(catalog_.trf(
    "debrief.cause_rollover", {format_number(std::abs(cross_degrees)), format_number(risk_percent, 0)}));
 } else if (state.traction_saturated) {
  replay_cause_->setText(catalog_.trf(
    "debrief.cause_traction", {format_number(grade_degrees), format_number(std::abs(state.robot.v))}));
 } else if (!mission.level().environment_keyframes.empty() && state.environment.wetness >= 0.2) {
 const auto& stage = mission.level().environment_keyframes[state.environment.keyframe_index];
 replay_cause_->setText(catalog_.trf("debrief.cause_environment",
 {stage.label, format_number(state.environment.wetness * 100.0, 0),
 format_number(state.environment.visibility * 100.0, 0)}));
 } else if (state.decision.sequence > 0 && !state.decision.observation_refs.empty()) {
 replay_cause_->setText(catalog_.trf("debrief.cause_decision",
 decision_arguments(catalog_, state.decision)));
 } else if (std::abs(grade_degrees) >= 1.0 || std::abs(cross_degrees) >= 1.0) {
  replay_cause_->setText(
    catalog_.trf("debrief.cause_slope", {format_number(grade_degrees), format_number(cross_degrees),
                                         format_number(risk_percent, 0)}));
 } else {
  replay_cause_->setText(catalog_.trf("debrief.cause_clear", {format_number(std::abs(state.robot.v)),
                                                              format_number(mission.result().energy_wh, 2)}));
 }

 std::vector<const mission::MissionEvent*> visible_events;
 for (const auto& event : mission.events()) {
  if (event.time_s <= state.time_s + 1e-9) {
   visible_events.push_back(&event);
  }
 }
 const std::size_t first = visible_events.size() > 4 ? visible_events.size() - 4 : 0;
 std::ostringstream evidence;
 if (state.decision.sequence > 0) {
 evidence << catalog_.trf("debrief.decision_record",
 {std::to_string(state.decision.sequence), decision_reason(catalog_, state.decision),
 decision_action(catalog_, state.decision), state.decision.outcome_code})
 << '\n';
 }
 for (std::size_t index = first; index < visible_events.size(); ++index) {
  evidence << std::fixed << std::setprecision(1) << visible_events[index]->time_s << "s "
           << visible_events[index]->message << '\n';
 }
 if (visible_events.empty()) {
  evidence << catalog_.tr("debrief.no_event");
 }
 replay_evidence_->setText(evidence.str());
}

void GameUi::select_component(const std::string& slot, const std::string& id) {
 // Locked options are disabled at the widget level, but this guard keeps
 // programmatic callers honest too. Playgrounds deliberately expose their
 // authored experiment matrix without mutating campaign inventory.
 const bool open_loadout = catalog_.level(presentation_->level).loadout_access == "open";
 if (!open_loadout && !owned_.empty() && !owned_.contains(id))
  return;
 assign_id(build_, slot, id);
 actions_.build_changed = true;
 update_garage_labels();
}

void GameUi::cycle_component(const std::string& slot) {
 const auto column = std::find_if(presentation_->component_columns.begin(),
 presentation_->component_columns.end(),
 [&](const auto& candidate) { return candidate.slot == slot; });
 if (column == presentation_->component_columns.end() || column->options.empty())
 return;
 const std::string current = selected_id(build_, slot);
 const auto selected = std::find(column->options.begin(), column->options.end(), current);
 const std::size_t start = selected == column->options.end()
 ? 0
 : (static_cast<std::size_t>(std::distance(column->options.begin(), selected)) + 1) %
 column->options.size();
 for (std::size_t offset = 0; offset < column->options.size(); ++offset) {
 const auto& candidate = column->options[(start + offset) % column->options.size()];
 if (catalog_.level(presentation_->level).loadout_access == "open" ||
 owned_.empty() || owned_.contains(candidate)) {
 select_component(slot, candidate);
 return;
 }
 }
}

void GameUi::select_chassis(const std::string& robot_id) {
 const auto& robot = catalog_.robot(robot_id);
 if (catalog_.level(presentation_->level).loadout_access != "open" &&
 !robot.unlock_ability.empty() && !owned_abilities_.contains(robot.unlock_ability)) {
 return;
 }
 build_.chassis_robot = robot_id;
 actions_.build_changed = true;
 update_garage_labels();
}

void GameUi::select_task_policy(const std::string& policy) {
 build_.task_policy = policy;
 actions_.build_changed = true;
 update_garage_labels();
}

void GameUi::set_style(double style) {
 build_.style = std::clamp(style, 0.0, 1.0);
 actions_.build_changed = true;
 update_style_label();
}

bool GameUi::build_within_budget() const {
 return catalog_.build_cost(build_) <= catalog_.level(presentation_->level).budget;
}

void GameUi::update_style_label() {
 if (!style_value_)
  return;
 std::vector<const mission::PolicyDefinition*> anchors;
 for (const auto& [id, policy] : catalog_.policies()) {
  (void)id;
  anchors.push_back(&policy);
 }
 std::sort(anchors.begin(), anchors.end(),
           [](const mission::PolicyDefinition* left, const mission::PolicyDefinition* right) {
            return left->style_anchor < right->style_anchor;
           });
 const mission::PolicyDefinition* nearest = anchors.front();
 for (const auto* anchor : anchors) {
  if (std::abs(build_.style - anchor->style_anchor) < std::abs(build_.style - nearest->style_anchor)) {
   nearest = anchor;
  }
 }
 style_value_->setText(nearest->title + "  ·  " + nearest->description);
}

void GameUi::set_inventory(const std::vector<std::string>& component_ids) {
 owned_.clear();
 owned_.insert(component_ids.begin(), component_ids.end());
 sanitize_build();
 if (screen_ == Screen::Garage)
  update_garage_labels();
}

void GameUi::set_abilities(const std::vector<std::string>& ability_ids) {
 owned_abilities_.clear();
 owned_abilities_.insert(ability_ids.begin(), ability_ids.end());
 sanitize_build();
 if (screen_ == Screen::Garage) {
 update_garage_labels();
 }
}

void GameUi::set_contract_status(int reputation,
 const std::vector<std::string>& claimed_contracts) {
 reputation_ = std::max(0, reputation);
 claimed_contracts_.clear();
 claimed_contracts_.insert(claimed_contracts.begin(), claimed_contracts.end());
 if (screen_ == Screen::Chapter)
 rebuild(nullptr);
}

void GameUi::set_build(const mission::BuildSelection& build) {
 build_ = build;
 sanitize_build();
 update_garage_labels();
}

std::string GameUi::card_stat(const std::string& slot, const mission::ComponentDefinition& item) const {
 const auto& behavior = item.behavior;
 const auto get = [&](const char* key) {
  const auto found = behavior.find(key);
  return found == behavior.end() ? 0.0 : found->second;
 };
 // Card line 2: the stat that makes the choice meaningful (content numbers,
 // never invented): grip mu / top speed / range+rate / decision rate / gyro
 // noise. Compact units fit 100px cards at 14px (verified by screenshot).
 if (slot == "tires")
  return catalog_.trf("garage.stat_tire", {format_number(get("friction_mu"), 1)});
 if (slot == "motor")
  return catalog_.trf("garage.stat_motor", {format_number(get("max_wheel_speed_mps"), 1)});
 if (slot == "sensor")
  return catalog_.trf("garage.stat_sensor",
                      {format_number(get("range_m"), 0), format_number(get("scan_hz"), 0)});
 if (slot == "computer")
  return catalog_.trf("garage.stat_computer", {format_number(get("decision_hz"), 0)});
 if (slot == "imu")
 return catalog_.trf("garage.stat_imu",
 {format_number(get("gyro_noise_density_radps_sqrt_hz"), 4)});
 return std::string();
}

std::string GameUi::card_text(const std::string& slot, const mission::ComponentDefinition& item,
                              bool locked) const {
 return catalog_.trf("garage.card", {item.title, card_stat(slot, item), std::to_string(item.cost),
                                     locked ? catalog_.tr("garage.card_locked") : ""});
}

void GameUi::sanitize_build() {
 // Selections the player does not own (stale saves, future content) fall back
 // to the level default, which content validation guarantees to be ownable.
 const bool playground = catalog_.level(presentation_->level).loadout_access == "open";
 if (owned_.empty())
  return;
 if (!playground) {
 for (const auto& slot : {"tires", "sensor", "computer"}) {
 const auto& current = selected_id(build_, slot);
 if (!current.empty() && !owned_.contains(current)) {
 assign_id(build_, slot, selected_id(presentation_->default_build, slot));
 }
  }
 }
 if (!build_.chassis_robot.empty()) {
 const auto offered = std::find(presentation_->chassis_options.begin(),
 presentation_->chassis_options.end(), build_.chassis_robot);
 const auto& robot = catalog_.robot(build_.chassis_robot);
 const bool locked = !playground && !robot.unlock_ability.empty() &&
 !owned_abilities_.contains(robot.unlock_ability);
 if (offered == presentation_->chassis_options.end() || locked) {
 build_.chassis_robot.clear();
 }
 }
 build_.style = std::clamp(build_.style, 0.0, 1.0);
}

void GameUi::update_garage_labels() {
 const bool playground = catalog_.level(presentation_->level).loadout_access == "open";
 const std::string selected_chassis = !build_.chassis_robot.empty()
 ? build_.chassis_robot
 : (!presentation_->robot_id.empty() ? presentation_->robot_id : catalog_.campaign().companion_robot);
 if (garage_heading_label_)
 garage_heading_label_->setText(catalog_.trf("garage.heading", {catalog_.robot(selected_chassis).title}));
 for (const auto& card : chassis_cards_) {
 const auto& robot = catalog_.robot(card.id);
 const bool locked = !playground && !robot.unlock_ability.empty() &&
 !owned_abilities_.contains(robot.unlock_ability);
 const bool active = card.id == selected_chassis;
 card.button->setEnabled(!locked);
 auto look = card.button->getRenderer();
 if (active) {
 look->setBackgroundColor({52, 61, 57});
 look->setBackgroundColorHover({58, 68, 63});
 look->setBorderColor({108, 189, 174});
 look->setTextColor({235, 244, 238});
 } else {
 look->setBackgroundColor({26, 30, 31});
 look->setBackgroundColorHover({38, 44, 43});
 look->setBorderColor({64, 69, 67});
 look->setTextColor({225, 224, 215});
 }
 const std::string drive = catalog_.tr(robot.drive_label_key);
 card.button->setText(catalog_.trf("garage.chassis_card",
 {robot.title, drive, locked ? catalog_.tr("garage.card_locked") : "",
 active ? catalog_.tr("garage.chassis_selected") : ""}));
 }
 for (const auto& [slot, cards] : slot_cards_) {
  const std::string current = selected_id(build_, slot);
  for (const auto& card : cards) {
   const auto& item = catalog_.component(card.id);
 const bool locked = !playground && !owned_.empty() && !owned_.contains(card.id);
   const bool active = card.id == current;
 card.button->setEnabled(!locked || reputation_ >= mission::component_reputation_cost(item));
   auto look = card.button->getRenderer();
   if (active) {
    look->setBackgroundColor({52, 61, 57});
    look->setBackgroundColorHover({58, 68, 63});
    look->setBorderColor({108, 189, 174});
    look->setTextColor({235, 244, 238});
   } else {
    look->setBackgroundColor({26, 30, 31});
    look->setBackgroundColorHover({38, 44, 43});
    look->setBorderColor({64, 69, 67});
    look->setTextColor({225, 224, 215});
   }
 card.button->setText(locked
 ? catalog_.trf("garage.card_reputation",
 {item.title, card_stat(slot, item),
 std::to_string(mission::component_reputation_cost(item))})
 : card_text(slot, item, false));
  }
 }
 for (const auto& card : task_policy_cards_) {
 const bool active = card.id == build_.task_policy ||
 (build_.task_policy.empty() && card.id == catalog_.level(presentation_->level).objective.task_policy);
 auto look = card.button->getRenderer();
 look->setBackgroundColor(active ? tgui::Color{52, 61, 57} : tgui::Color{26, 30, 31});
 look->setBackgroundColorHover(active ? tgui::Color{58, 68, 63} : tgui::Color{38, 44, 43});
 look->setBorderColor(active ? tgui::Color{108, 189, 174} : tgui::Color{64, 69, 67});
 look->setTextColor(active ? tgui::Color{235, 244, 238} : tgui::Color{225, 224, 215});
 }
 if (style_slider_) {
  style_slider_->setValue(static_cast<float>(build_.style * 100.0));
 }
 update_style_label();
 if (budget_label_) {
  const int used = catalog_.build_cost(build_);
  const int budget = catalog_.level(presentation_->level).budget;
  budget_label_->setText(catalog_.trf("garage.budget", {std::to_string(used), std::to_string(budget)}));
  budget_label_->getRenderer()->setTextColor(used <= budget ? tgui::Color{147, 190, 185}
                                                            : tgui::Color{255, 130, 112});
 if (garage_next_button_) {
 garage_next_button_->setEnabled(used <= budget);
 }
 }
}

std::string GameUi::selected_robot_title() const {
 const std::string id = !build_.chassis_robot.empty()
 ? build_.chassis_robot
 : (!presentation_->robot_id.empty()
 ? presentation_->robot_id
 : catalog_.campaign().companion_robot);
 return catalog_.robot(id).title;
}

void GameUi::refresh_running(const mission::ScenarioSession& mission, bool paused, double speed) {
 if (!running_metrics_)
  return;
 const auto& state = mission.snapshot();
 const bool manual = !mission.endless_mode() &&
 mission.level().control_mode == "manual_calibration";
 const bool operator_drive = !mission.endless_mode() &&
 mission.level().control_mode == "operator_drive";
 const double grade_degrees = std::atan(state.terrain_grade) * kRadToDeg;
 const double cross_degrees = std::atan(state.cross_slope) * kRadToDeg;
 const bool slope_active = std::abs(grade_degrees) >= 1.0 || std::abs(cross_degrees) >= 1.0;
 if (running_explanation_card_) {
  running_explanation_card_->setVisible(
 mission.endless_mode() || manual || operator_drive || state.static_obstacle_detected ||
 state.dynamic_object_detected ||
    state.waiting_for_clearance || slope_active || state.traction_saturated || state.rollover_risk >= 0.75);
 }
 std::string explanation;
 if (mission.endless_mode()) {
 const auto& intent = mission.field_intent();
 const auto& dispatch = mission.field_dispatch();
 if (intent.active) {
 const double intent_distance = std::hypot(
 state.robot.x - intent.target.x, state.robot.y - intent.target.y);
 explanation = catalog_.trf("running.partner_controls",
 {field_intent_label(catalog_, intent.kind),
 field_ack_label(catalog_, intent.acknowledgement), format_number(intent_distance)});
 } else if (dispatch.active) {
 explanation = catalog_.tr("running.partner_idle") + "\n\n" + catalog_.trf("running.field_offer",
 {field_kind_label(catalog_, dispatch.kind), field_step_label(catalog_, dispatch),
 format_number(dispatch.distance_m)});
 } else {
 explanation = catalog_.tr("running.partner_idle") + "\n\n" + catalog_.trf("running.partner_record",
 {std::to_string(dispatch.completed_count)});
 }
 const auto& environment = mission.world_activity();
 explanation += "\n\n" + catalog_.trf("running.environment",
 {weather_label(catalog_, environment.weather), field_time(environment.local_time_hours),
 format_number(environment.wetness * 100.0, 0)});
 if (environment.incident.active) {
 const double incident_distance = std::hypot(
 state.robot.x - environment.incident.position.x,
 state.robot.y - environment.incident.position.y);
 explanation += "\n" + catalog_.trf("running.incident",
 {incident_label(catalog_, environment.incident.kind), format_number(incident_distance),
 format_number(environment.incident.resolve_progress * 100.0, 0)});
 }
 } else if (operator_drive) {
 const auto& intent = mission.field_intent();
 explanation = catalog_.tr(mission.manual_control_active()
 ? "running.operator_manual_body"
 : "running.operator_auto_body");
 if (intent.active) {
 const double intent_distance = std::hypot(
 state.robot.x - intent.target.x, state.robot.y - intent.target.y);
 explanation += "\n\n" + catalog_.trf("running.operator_target",
 {field_ack_label(catalog_, intent.acknowledgement), format_number(intent_distance)});
 }
 } else if (manual) {
 explanation = catalog_.tr("running.manual_body");
 } else if (state.rollover_risk >= 0.75) {
  explanation = catalog_.trf("running.risk_body", {format_number(std::abs(cross_degrees)),
                                                   format_number(state.rollover_risk * 100.0, 0)});
 } else if (state.traction_saturated) {
  explanation = catalog_.trf("running.traction_body",
                             {format_number(grade_degrees), format_number(std::abs(state.robot.v))});
 } else if (state.decision.sequence > 0 && !state.decision.observation_refs.empty()) {
 explanation = catalog_.trf("running.decision_body", decision_arguments(catalog_, state.decision));
 } else if (state.static_obstacle_detected) {
  explanation = catalog_.tr("running.static_body");
 } else if (slope_active) {
  explanation =
    catalog_.trf("running.slope_body", {format_number(grade_degrees), format_number(cross_degrees),
                                        format_number(state.rollover_risk * 100.0, 0)});
 } else {
  explanation = catalog_.tr("running.clear_head") + mission.level().objective.goal_label;
 }
 if (mission.level().objective.goals.size() > 1) {
 const auto* active = mission.active_goal();
 explanation += catalog_.trf("running.goal_progress",
 {std::to_string(state.goals_reached),
 std::to_string(mission.level().objective.goals.size()),
 active ? active->label : mission.level().objective.goal_label});
 }
 running_metrics_->setText(explanation);
 if (running_explanation_card_) {
 const float content_height = 42.0f + std::max(20.0f, running_metrics_->getFullSize().y) + 18.0f;
 running_explanation_card_->setContentSize(
 {kContentWidth - 12.0f,
 std::max(running_explanation_card_->getSize().y - 4.0f, content_height)});
 }
 if (running_state_) {
  if (paused)
   running_state_->setText(catalog_.tr("running.paused"));
 else if (mission.endless_mode()) {
 const auto& intent = mission.field_intent();
 const auto& dispatch = mission.field_dispatch();
 running_state_->setText(intent.active
 ? catalog_.trf("running.partner_state",
 {field_intent_label(catalog_, intent.kind),
 field_ack_label(catalog_, intent.acknowledgement)})
 : (dispatch.active
 ? catalog_.trf("running.field_state",
 {field_step_label(catalog_, dispatch), format_number(dispatch.distance_m)})
 : catalog_.tr("running.endless_state_auto")));
 } else if (manual)
 running_state_->setText(catalog_.tr("running.manual_state"));
 else if (state.decision.sequence > 0 && state.decision.action_code != "follow-route" &&
 state.decision.action_code != "plan-route")
 running_state_->setText(catalog_.trf("running.decision_state",
 {decision_action(catalog_, state.decision)}));
 else if (state.dynamic_object_detected || state.static_obstacle_detected) {
   running_state_->setText(presentation_->detected_line);
 } else if (!presentation_->goal_lines.empty()) {
 const std::size_t completed_line = mission.level().objective.goals.size();
 const std::size_t line = state.active_goal_index < mission.level().objective.goals.size()
 ? state.active_goal_index
 : completed_line;
 if (line < presentation_->goal_lines.size())
 running_state_->setText(presentation_->goal_lines[line]);
 else
 running_state_->setText(presentation_->normal_line);
 } else
 running_state_->setText(presentation_->normal_line);
 }
 if (pause_button_)
  pause_button_->setText(paused ? catalog_.tr("running.resume") : catalog_.tr("running.pause"));
 (void)speed;
}

UiActions GameUi::consume_actions() {
 const auto result = actions_;
 actions_ = {};
 return result;
}

void GameUi::use_recommended_build() {
 build_ = presentation_->recommended_build;
 update_garage_labels();
}

void GameUi::select_level(const std::string& level_id) {
 const auto& level = catalog_.level(level_id);
 if (!is_unlocked(level))
  return;
 presentation_ = &catalog_.presentation_for_level(level_id);
 active_contract_id_.clear();
 const auto contract = std::find_if(catalog_.contracts().begin(), catalog_.contracts().end(),
 [&](const auto& job) { return job.level == level_id; });
 if (contract != catalog_.contracts().end())
 active_contract_id_ = contract->id;
 build_ = presentation_->default_build;
 prediction_choice_.clear();
}

void GameUi::mark_completed(const std::string& level_id) {
 completed_levels_.insert(level_id);
}

void GameUi::restore_completed(const std::vector<std::string>& level_ids) {
 completed_levels_.insert(level_ids.begin(), level_ids.end());
 if (screen_ == Screen::Chapter)
  rebuild(nullptr);
}

} // namespace client
