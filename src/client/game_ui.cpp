#include "game_ui.hpp"
#include "mission/campaign_progress.hpp"

#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>

namespace client {
namespace {

constexpr float kPanelWidth = 360.0f;
constexpr float kMargin = 24.0f;
constexpr float kContentWidth = 312.0f;
constexpr double kRadToDeg = 57.29577951308232;

std::string format_number(double value, int precision = 1) {
 std::ostringstream output;
 output << std::fixed << std::setprecision(precision) << value;
 return output.str();
}

std::string format_delta(double value, int precision = 1) {
 return (value > 0.0 ? "+" : "") + format_number(value, precision);
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
 const auto contract = std::find_if(catalog_.contracts().begin(), catalog_.contracts().end(),
 [&](const auto& job) { return job.level == presentation_->level; });
 if (contract != catalog_.contracts().end())
 active_contract_id_ = contract->id;
 panel_ = tgui::Panel::create();
 gui_.add(panel_, "control-panel");
 style_panel();
 resize(width_, height_);
 rebuild(nullptr);
}

void GameUi::style_panel() {
 auto renderer = panel_->getRenderer();
 renderer->setBackgroundColor({12, 15, 18});
 renderer->setBorderColor({63, 65, 61});
 renderer->setBorders({1, 0, 0, 0});
}

void GameUi::resize(int width, int height) {
 width_ = width;
 height_ = height;
 panel_->setPosition(std::max(0.0f, static_cast<float>(width) - kPanelWidth), 0);
 panel_->setSize(std::min(kPanelWidth, static_cast<float>(width)), static_cast<float>(height));
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

void GameUi::rebuild(const mission::MissionSimulation* mission) {
 panel_->removeAllWidgets();
 add_parent_ = panel_;
 style_panel();
 slot_cards_.clear();
 chassis_cards_.clear();
 style_slider_ = nullptr;
 style_value_ = nullptr;
 budget_label_ = nullptr;
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
 // Level roster lives in a scroll panel: 21 buttons × 54px overflow any fixed
 // window (720p fit ~12). Scrollbar appears only when content exceeds space.
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
 add_label(catalog_.tr("chapter.game_title"), kMargin, 25, kContentWidth, 14, {107, 190, 176}, true);
 add_label(catalog_.campaign().title, kMargin, 53, kContentWidth, 25, {239, 238, 229}, true);
 add_label(catalog_.tr("chapter.subtitle"), kMargin, 94, kContentWidth, 14, {157, 163, 158});

 add_label(catalog_.trf("chapter.contract_balance", {std::to_string(reputation_)}),
 kMargin, 124, kContentWidth, 14, {232, 172, 80}, true);
 float y = 166.0f;
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
   add_label(heading, kMargin, y, kContentWidth, 17, {107, 190, 176}, true);
   y += 30.0f;
  }
  const bool complete = completed_levels_.contains(level_id);
  const bool unlocked = is_unlocked(level);
  const std::string prefix = complete ? "✓ " : unlocked ? "○ " : "— ";
  const auto contract = std::find_if(catalog_.contracts().begin(), catalog_.contracts().end(),
    [&](const auto& job) { return job.level == level_id; });
  std::string button_text = prefix + std::to_string(level.order) + " " + level.title;
  float button_height = 44.0f;
  if (contract != catalog_.contracts().end()) {
   const bool claimed = claimed_contracts_.contains(contract->id);
   button_text += "\n" + catalog_.trf(
     claimed ? "chapter.contract_claimed" : "chapter.contract_offer",
 {contract_axis_label(catalog_, contract->axis), std::to_string(contract->rep)});
   button_height = 58.0f;
  }
  auto button = add_button(button_text, kMargin, y, kContentWidth, button_height,
    level_id == presentation_->level && unlocked);
  if (contract != catalog_.contracts().end())
   button->setTextSize(15);
  button->setEnabled(unlocked);
  button->onClick([this, level_id] { actions_.selected_level = level_id; });
  y += button_height + 10.0f;
 }
 add_label(catalog_.tr("chapter.footer_head") + catalog_.tr("chapter.footer_tail"), kMargin, y + 12,
           kContentWidth, 13, {137, 151, 145});
 if (!add_parent_) {
  return;
 }
 if (const auto scrollable = std::dynamic_pointer_cast<tgui::ScrollablePanel>(add_parent_)) {
  scrollable->setContentSize({static_cast<float>(kPanelWidth), y + 60.0f});
 }
}

void GameUi::set_screen(Screen screen, const mission::MissionSimulation* mission) {
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
 add_label(catalog_.tr("garage.heading"), kMargin, 24, kContentWidth, 16, {107, 190, 176}, true);
 auto chapter_button = add_button(catalog_.tr("chapter.garage_button"), kMargin + 216, 16, 96, 30);
 chapter_button->onClick([this] { actions_.open_chapter = true; });
 add_label(presentation_->garage_heading, kMargin, 51, kContentWidth, 25, {239, 238, 229}, true);
 add_label(presentation_->garage_brief, kMargin, 94, kContentWidth, 16, {157, 163, 158});

 float y = 226.0f;
 if (const auto* beat = catalog_.story_beat(presentation_->level, "before")) {
  add_label(beat->speaker + "：" + beat->text, kMargin, 168, kContentWidth, 14, {184, 199, 190});
  y = 248.0f;
 }
 if (!active_contract_id_.empty()) {
 const auto contract = std::find_if(catalog_.contracts().begin(), catalog_.contracts().end(),
 [&](const auto& job) { return job.id == active_contract_id_; });
 if (contract != catalog_.contracts().end()) {
 add_label(catalog_.trf("garage.contract_payload",
 {contract->blurb, format_number(contract->mass_kg, 0),
 format_number(contract->cog_height_m, 2)}),
 kMargin, y, kContentWidth, 14, {232, 172, 80});
 y += 86.0f;
 }
 }
 constexpr float kCardGap = 6.0f;
 constexpr float kCardHeight = 56.0f;

 if (!presentation_->chassis_options.empty()) {
 add_label(catalog_.tr("garage.chassis"), kMargin, y, kContentWidth, 16, {132, 139, 135}, true);
 const float chassis_width = (kContentWidth - kCardGap) / 2.0f;
 std::size_t index = 0;
 for (const auto& robot_id : presentation_->chassis_options) {
 const float row = static_cast<float>(index / 2);
 const float col = static_cast<float>(index % 2);
 auto card = tgui::Button::create(" ");
 card->setPosition(kMargin + col * (chassis_width + kCardGap),
 y + 24 + row * (kCardHeight + kCardGap));
 card->setSize(chassis_width, kCardHeight);
 card->setTextSize(14);
 auto look = card->getRenderer();
 look->setRoundedBorderRadius(8);
 look->setBorders({1.0f});
 add_parent_->add(card);
 card->onClick([this, robot_id] { select_chassis(robot_id); });
 chassis_cards_.push_back({robot_id, card});
 ++index;
 }
 const float rows = static_cast<float>((presentation_->chassis_options.size() + 1) / 2);
 y += 24 + rows * (kCardHeight + kCardGap) + 10;
 }

 const float card_width = (kContentWidth - 2.0f * kCardGap) / 3.0f;
 for (const auto& column : presentation_->component_columns) {
 if (!column.player_choice)
 continue;
  add_label(column.title, kMargin, y, kContentWidth, 16, {132, 139, 135}, true);
  std::size_t index = 0;
  for (const auto& option : column.options) {
   const auto& item = catalog_.component(option);
   const float row = static_cast<float>(index / 3);
   const float col = static_cast<float>(index % 3);
   auto card = tgui::Button::create(" ");
   card->setPosition(kMargin + col * (card_width + kCardGap), y + 24 + row * (kCardHeight + kCardGap));
   card->setSize(card_width, kCardHeight);
   card->setTextSize(14);
   card->setText(card_text(column.slot, item, false));
   auto look = card->getRenderer();
   look->setRoundedBorderRadius(8);
   look->setBorders({1.0f});
   add_parent_->add(card);
 card->onClick([this, slot = column.slot, id = option] {
 if (owned_.empty() || owned_.contains(id))
 select_component(slot, id);
 else
 actions_.purchase_component = id;
 });
   slot_cards_[column.slot].push_back({option, card});
   ++index;
  }
  const float rows = static_cast<float>((column.options.size() + 2) / 3);
  y += 24 + rows * (kCardHeight + kCardGap) + 10;
 }
 add_label(presentation_->policy_title, kMargin, y, kContentWidth, 14, {132, 139, 135}, true);
 style_slider_ = tgui::Slider::create(0, 100);
 style_slider_->setPosition(kMargin, y + 28);
 style_slider_->setSize(kContentWidth, 18);
 style_slider_->setValue(static_cast<float>(build_.style * 100.0));
 style_slider_->onValueChange([this](float value) { set_style(static_cast<double>(value) / 100.0); });
 add_parent_->add(style_slider_);
 add_label(catalog_.tr("garage.style_gentle"), kMargin, y + 50, 80, 14, {132, 139, 135});
 auto style_mid =
   add_label(catalog_.tr("garage.style_normal"), kMargin, y + 50, kContentWidth, 14, {132, 139, 135});
 style_mid->setHorizontalAlignment(tgui::Label::HorizontalAlignment::Center);
 auto style_fast =
   add_label(catalog_.tr("garage.style_fast"), kMargin, y + 50, kContentWidth, 14, {132, 139, 135});
 style_fast->setHorizontalAlignment(tgui::Label::HorizontalAlignment::Right);
 style_value_ = add_label("", kMargin, y + 70, kContentWidth, 15, {143, 201, 187});

 endless_checkbox_ = tgui::CheckBox::create(catalog_.tr("garage.endless_mode"));
 endless_checkbox_->setPosition(kMargin, y + 104);
 endless_checkbox_->setTextSize(15);
 endless_checkbox_->setChecked(endless_mode_);
 endless_checkbox_->getRenderer()->setTextColor({184, 199, 190});
 endless_checkbox_->onChange([this](bool checked) {
 endless_mode_ = checked;
 if (garage_next_button_)
 garage_next_button_->setText(catalog_.tr(checked ? "garage.enter_world" : "garage.next"));
 });
 add_parent_->add(endless_checkbox_);
 add_label(catalog_.tr("garage.expedition_control"), kMargin + 22, y + 134,
 kContentWidth - 22, 14, {154, 183, 174});
 budget_label_ = add_label("", kMargin, y + 170, kContentWidth, 18, {151, 167, 159}, true);
 auto recommended = add_button(catalog_.tr("garage.use_recommended"), kMargin, y + 206, 148, 42);
 recommended->onClick([this] { use_recommended_build(); });
 garage_next_button_ = add_button(catalog_.tr("garage.next"), kMargin + 164, y + 206, 148, 42, true);
 garage_next_button_->setText(
 catalog_.tr(endless_mode_ ? "garage.enter_world" : "garage.next"));
 garage_next_button_->onClick([this] { actions_.open_prediction = true; });
 update_garage_labels();
 if (const auto scrollable = std::dynamic_pointer_cast<tgui::ScrollablePanel>(add_parent_)) {
 scrollable->setContentSize({static_cast<float>(kPanelWidth), y + 262.0f});
 }
}

void GameUi::build_prediction() {
 add_label(catalog_.tr("prediction.departure"), kMargin, 26, kContentWidth, 14, {107, 190, 176}, true);
 add_label(presentation_->prediction_heading, kMargin, 55, kContentWidth, 25, {239, 238, 229}, true);
 add_label(presentation_->prediction_hint, kMargin, 101, kContentWidth, 14, {157, 163, 158});

 float y = 184.0f;
 for (const auto& choice : presentation_->prediction_choices) {
  auto button = add_button(choice.text, kMargin, y, kContentWidth, 58);
  button->onClick([this, id = choice.id] {
   prediction_choice_ = id;
   rebuild(nullptr);
  });
  if (choice.id == prediction_choice_) {
   button->getRenderer()->setBackgroundColor({52, 61, 57});
   button->getRenderer()->setBorderColor({108, 189, 174});
  }
  y += 72;
 }
 auto back = add_button(catalog_.tr("garage.back"), kMargin, height_ - 74.0f, 116, 46);
 back->onClick([this] { actions_.open_garage = true; });
 auto launch = add_button(catalog_.tr("garage.launch"), kMargin + 132, height_ - 74.0f, 180, 46, true);
 launch->setEnabled(!prediction_choice_.empty());
 launch->onClick([this] { actions_.launch = true; });
}

void GameUi::build_running(const mission::MissionSimulation* mission) {
 auto panel_renderer = panel_->getRenderer();
 panel_renderer->setBackgroundColor({0, 0, 0, 0});
 panel_renderer->setBorders({0});

 auto objective_card = tgui::Panel::create({kContentWidth, 86});
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
 chapter->getRenderer()->setTextColor({107, 190, 176});
 chapter->getRenderer()->setTextStyle(tgui::TextStyle::Bold);
 objective_card->add(chapter);
 auto objective = tgui::Label::create(
 mission && mission->endless_mode() ? catalog_.tr("running.endless_heading")
 : presentation_->running_heading);
 objective->setPosition(16, 39);
 objective->setTextSize(18);
 objective->getRenderer()->setTextColor({239, 238, 229});
 objective->getRenderer()->setTextStyle(tgui::TextStyle::Bold);
 objective_card->add(objective);

 auto state_card = tgui::Panel::create({kContentWidth, 104});
 const bool manual = mission && !mission->endless_mode() &&
 mission->level().control_mode == "manual_calibration";
 const bool expedition = mission && mission->endless_mode();
 state_card->setPosition(kMargin, 120);
 state_card->getRenderer()->setBackgroundColor({19, 27, 25, 225});
 state_card->getRenderer()->setBorderColor({73, 212, 185, 180});
 state_card->getRenderer()->setBorders({0, 0, 0, 3});
 state_card->getRenderer()->setRoundedBorderRadius(12);
 panel_->add(state_card);
 auto live = tgui::Label::create(expedition ? catalog_.tr("running.live_together")
 : (manual ? catalog_.tr("running.live_drive")
 : catalog_.tr("running.live_decide")));
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

 auto card = tgui::Panel::create({kContentWidth, 210});
 running_explanation_card_ = card;
 card->setPosition(kMargin, 238);
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
 running_metrics_->setMaximumTextWidth(276);
 running_metrics_->getRenderer()->setTextColor({194, 199, 194});
 card->add(running_metrics_);

 const float controls_y = std::max(468.0f, static_cast<float>(height_) - 70.0f);
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
 auto garage = add_button(catalog_.tr("running.to_garage"), garage_x, controls_y - 48.0f,
 garage_width, 36);
 garage->onClick([this] { actions_.open_garage = true; });
 if (endless) {
 auto reset = add_button(catalog_.tr("running.reset_robot"), kMargin, controls_y - 48.0f, 152, 36);
 reset->onClick([this] { actions_.reset_simulation = true; });
 }
 if (mission) {
  refresh_running(*mission, false, 1.0);
 }
}

void GameUi::build_debrief(const mission::MissionSimulation* mission) {
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
 add_label(catalog_.tr("debrief.title"), kMargin, 24, kContentWidth, 14, {107, 190, 176}, true);
 add_label(title, kMargin, 51, kContentWidth, 24,
           succeeded ? tgui::Color{105, 235, 193} : tgui::Color{255, 145, 120}, true);
 std::string summary = succeeded ? presentation_->success_summary : presentation_->failure_summary;
 if (succeeded) {
  if (const auto* beat = catalog_.story_beat(presentation_->level, "after")) {
   summary += "\n" + beat->speaker + "：" + beat->text;
  }
 }
 add_label(summary, kMargin, 96, kContentWidth, 15, {157, 163, 158});

 const auto badge = [&](bool earned, const char* key) {
  return (earned ? catalog_.tr("debrief.badge_on") : catalog_.tr("debrief.badge_off")) + catalog_.tr(key);
 };
 add_label(badge(result.safe_score, "debrief.badge_safe") + " " +
             badge(result.fast_score, "debrief.badge_fast") + " " +
             badge(result.efficient_score, "debrief.badge_efficient"),
           kMargin, 166, kContentWidth, 17, {220, 222, 216}, true);
 std::string metrics =
   catalog_.trf("debrief.metrics", {format_number(result.time_s), format_number(result.path_length_m),
                                    format_number(result.energy_wh, 2)});
 metrics += mission->level().concept_id == "odometry_drift"
              ? catalog_.trf("debrief.metrics_odom", {format_number(result.max_odometry_error_m, 2)})
              : catalog_.trf("debrief.metrics_clear", {format_number(result.scored_clearance_m, 2)});
 add_label(metrics, kMargin, 207, kContentWidth, 16, {182, 190, 186});

 add_label(catalog_.tr("debrief.prediction_title"), kMargin, 267, kContentWidth, 15,
 {107, 190, 176}, true);
 const auto prediction =
 std::find_if(presentation_->prediction_choices.begin(), presentation_->prediction_choices.end(),
 [&](const auto& value) { return value.id == prediction_choice_; });
 if (prediction != presentation_->prediction_choices.end()) {
 add_label(prediction->text + "\n" + prediction->feedback, kMargin, 293, kContentWidth, 14,
 {170, 184, 178});
 } else {
 add_label(catalog_.tr("debrief.prediction_missing"), kMargin, 293, kContentWidth, 14,
 {132, 145, 139});
 }

 if (comparison_result_) {
 const auto& previous = *comparison_result_;
 comparison_label_ = add_label(
 catalog_.trf("debrief.comparison",
 {format_delta(result.time_s - previous.time_s),
 format_delta(result.energy_wh - previous.energy_wh, 2),
 format_delta(result.scored_clearance_m - previous.scored_clearance_m, 2)}),
 kMargin, 389, kContentWidth, 14, {232, 185, 103}, true);
 } else {
 comparison_label_ = add_label(catalog_.tr("debrief.comparison_first"), kMargin, 389,
 kContentWidth, 14, {132, 145, 139});
 }

 add_label(catalog_.tr("debrief.replay_title"), kMargin, 438, kContentWidth, 16, {239, 238, 229}, true);
 replay_slider_ = tgui::Slider::create(0.0f, static_cast<float>(std::max(0.05, result.time_s)));
 replay_slider_->setPosition(kMargin, 470);
 replay_slider_->setSize(kContentWidth, 18);
 replay_slider_->setValue(static_cast<float>(mission->snapshot().time_s));
 replay_slider_->onValueChange([this](float value) { actions_.replay_time_s = static_cast<double>(value); });
 add_parent_->add(replay_slider_);
 replay_value_ = add_label("", kMargin, 494, kContentWidth, 14, {143, 201, 187}, true);
 replay_cause_ = add_label("", kMargin, 522, kContentWidth, 14, {194, 199, 194});

 add_label(presentation_->evidence_heading, kMargin, 616, kContentWidth, 16, {239, 238, 229}, true);
 replay_evidence_ = add_label("", kMargin, 646, kContentWidth, 14, {151, 170, 163});

 add_label(presentation_->knowledge_title, kMargin, 770, kContentWidth, 16, {107, 190, 176}, true);
 add_label(mission->level().knowledge_card, kMargin, 800, kContentWidth, 14, {170, 184, 178});

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

void GameUi::refresh_debrief(const mission::MissionSimulation& mission) {
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
 } else if (state.waiting_for_clearance) {
  replay_cause_->setText(
    catalog_.trf("debrief.cause_waiting", {std::to_string(state.dynamic_obstacles.size())}));
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
 // programmatic callers honest too. Empty ownership means no filtering.
 if (!owned_.empty() && !owned_.contains(id))
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
 if (owned_.empty() || owned_.contains(candidate)) {
 select_component(slot, candidate);
 return;
 }
 }
}

void GameUi::select_chassis(const std::string& robot_id) {
 const auto& robot = catalog_.robot(robot_id);
 if (!robot.unlock_ability.empty() && !owned_abilities_.contains(robot.unlock_ability)) {
 return;
 }
 build_.chassis_robot = robot_id;
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
  return catalog_.trf("garage.stat_imu", {format_number(get("gyro_noise_radps"), 3)});
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
 if (owned_.empty())
  return;
 for (const auto& slot : {"tires", "sensor", "computer"}) {
  const auto& current = selected_id(build_, slot);
  if (!current.empty() && !owned_.contains(current)) {
   assign_id(build_, slot, selected_id(presentation_->default_build, slot));
  }
 }
 if (!build_.chassis_robot.empty()) {
 const auto offered = std::find(presentation_->chassis_options.begin(),
 presentation_->chassis_options.end(), build_.chassis_robot);
 const auto& robot = catalog_.robot(build_.chassis_robot);
 const bool locked = !robot.unlock_ability.empty() && !owned_abilities_.contains(robot.unlock_ability);
 if (offered == presentation_->chassis_options.end() || locked) {
 build_.chassis_robot.clear();
 }
 }
 build_.style = std::clamp(build_.style, 0.0, 1.0);
}

void GameUi::update_garage_labels() {
 const std::string selected_chassis = !build_.chassis_robot.empty()
 ? build_.chassis_robot
 : (!presentation_->robot_id.empty() ? presentation_->robot_id : catalog_.campaign().companion_robot);
 for (const auto& card : chassis_cards_) {
 const auto& robot = catalog_.robot(card.id);
 const bool locked = !robot.unlock_ability.empty() && !owned_abilities_.contains(robot.unlock_ability);
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
 {robot.title, drive, locked ? catalog_.tr("garage.card_locked") : ""}));
 }
 for (const auto& [slot, cards] : slot_cards_) {
  const std::string current = selected_id(build_, slot);
  for (const auto& card : cards) {
   const auto& item = catalog_.component(card.id);
   const bool locked = !owned_.empty() && !owned_.contains(card.id);
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

void GameUi::refresh_running(const mission::MissionSimulation& mission, bool paused, double speed) {
 if (!running_metrics_)
  return;
 const auto& state = mission.snapshot();
 const bool manual = !mission.endless_mode() &&
 mission.level().control_mode == "manual_calibration";
 const double grade_degrees = std::atan(state.terrain_grade) * kRadToDeg;
 const double cross_degrees = std::atan(state.cross_slope) * kRadToDeg;
 const bool slope_active = std::abs(grade_degrees) >= 1.0 || std::abs(cross_degrees) >= 1.0;
 if (running_explanation_card_) {
  running_explanation_card_->setVisible(
 mission.endless_mode() || manual || state.static_obstacle_detected || state.dynamic_object_detected ||
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
 } else if (manual) {
 explanation = catalog_.tr("running.manual_body");
 } else if (state.rollover_risk >= 0.75) {
  explanation = catalog_.trf("running.risk_body", {format_number(std::abs(cross_degrees)),
                                                   format_number(state.rollover_risk * 100.0, 0)});
 } else if (state.traction_saturated) {
  explanation = catalog_.trf("running.traction_body",
                             {format_number(grade_degrees), format_number(std::abs(state.robot.v))});
 } else if (state.waiting_for_clearance) {
  explanation = catalog_.tr("running.waiting_body");
 } else if (state.dynamic_object_detected) {
  explanation = catalog_.tr("running.dynamic_body");
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
  else if (state.waiting_for_clearance)
   running_state_->setText(catalog_.tr("running.waiting_state"));
  else if (state.dynamic_object_detected || state.static_obstacle_detected) {
   running_state_->setText(presentation_->detected_line);
 } else if (state.goals_reached < presentation_->goal_lines.size()) {
 running_state_->setText(presentation_->goal_lines[state.goals_reached]);
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
