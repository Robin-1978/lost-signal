#include "game_ui.hpp"

#include <algorithm>
#include <iomanip>
#include <memory>
#include <sstream>

namespace client {
namespace {

constexpr float kPanelWidth = 360.0f;
constexpr float kMargin = 24.0f;
constexpr float kContentWidth = 312.0f;

std::string format_number(double value, int precision = 1) {
 std::ostringstream output;
 output << std::fixed << std::setprecision(precision) << value;
 return output.str();
}

std::string selected_id(const game::BuildSelection& build, const std::string& slot) {
 return build.part(slot);
}

void assign_id(game::BuildSelection& build, const std::string& slot, const std::string& id) {
 build.parts[slot] = id;
}

} // namespace

GameUi::GameUi(tgui::BackendGui& gui, const game::ContentCatalog& catalog,
               const game::PresentationDefinition& presentation)
    : gui_(gui), catalog_(catalog), presentation_(&presentation), build_(presentation.default_build) {
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

void GameUi::rebuild(const game::MissionSimulation* mission) {
 panel_->removeAllWidgets();
 add_parent_ = panel_;
 style_panel();
 slot_cards_.clear();
 style_slider_ = nullptr;
 style_value_ = nullptr;
 budget_label_ = nullptr;
 running_metrics_ = nullptr;
 running_state_ = nullptr;
 running_explanation_card_ = nullptr;
 pause_button_ = nullptr;
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

bool GameUi::is_unlocked(const game::LevelDefinition& level) const {
 return std::all_of(level.prerequisites.begin(), level.prerequisites.end(),
                    [this](const auto& id) { return completed_levels_.contains(id); });
}

std::string GameUi::next_level_id() const {
 const auto& levels = catalog_.campaign().levels;
 const auto found = std::find(levels.begin(), levels.end(), presentation_->level);
 if (found == levels.end() || std::next(found) == levels.end())
  return {};
 return *std::next(found);
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

 float y = 160.0f;
 int shown_chapter = 0;
 for (const auto& level_id : catalog_.campaign().levels) {
  const auto& level = catalog_.level(level_id);
  if (level.chapter != shown_chapter) {
   shown_chapter = level.chapter;
   const game::ChapterDefinition* chapter = nullptr;
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
  auto button = add_button(prefix + std::to_string(level.order) + "  " + level.title, kMargin, y,
                           kContentWidth, 44, level_id == presentation_->level && unlocked);
  button->setEnabled(unlocked);
  button->onClick([this, level_id] { actions_.selected_level = level_id; });
  y += 54.0f;
 }
 add_label(catalog_.tr("chapter.footer_head") + catalog_.tr("chapter.footer_tail"),
           kMargin, y + 12, kContentWidth, 13, {137, 151, 145});
 if (!add_parent_) {
  return;
 }
 if (const auto scrollable = std::dynamic_pointer_cast<tgui::ScrollablePanel>(add_parent_)) {
  scrollable->setContentSize({static_cast<float>(kPanelWidth), y + 60.0f});
 }
}

void GameUi::set_screen(Screen screen, const game::MissionSimulation* mission) {
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
 constexpr float kCardGap = 6.0f;
 constexpr float kCardHeight = 56.0f;

 const float card_width = (kContentWidth - 2.0f * kCardGap) / 3.0f;
 for (const auto& column : presentation_->component_columns) {
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
   card->onClick([this, slot = column.slot, id = option] { select_component(slot, id); });
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
 auto style_mid = add_label(catalog_.tr("garage.style_normal"), kMargin, y + 50, kContentWidth, 14, {132, 139, 135});
 style_mid->setHorizontalAlignment(tgui::Label::HorizontalAlignment::Center);
 auto style_fast = add_label(catalog_.tr("garage.style_fast"), kMargin, y + 50, kContentWidth, 14, {132, 139, 135});
 style_fast->setHorizontalAlignment(tgui::Label::HorizontalAlignment::Right);
 style_value_ = add_label("", kMargin, y + 70, kContentWidth, 15, {143, 201, 187});

 budget_label_ = add_label("", kMargin, y + 108, kContentWidth, 18, {151, 167, 159}, true);
 auto recommended = add_button(catalog_.tr("garage.use_recommended"), kMargin, y + 144, 148, 42);
 recommended->onClick([this] { use_recommended_build(); });
 auto next = add_button(catalog_.tr("garage.next"), kMargin + 164, y + 144, 148, 42, true);
 next->onClick([this] { actions_.open_prediction = true; });
 update_garage_labels();
 if (const auto scrollable = std::dynamic_pointer_cast<tgui::ScrollablePanel>(add_parent_)) {
  scrollable->setContentSize({static_cast<float>(kPanelWidth), y + 200.0f});
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
 if (!prediction_choice_.empty()) {
  const auto found =
    std::find_if(presentation_->prediction_choices.begin(), presentation_->prediction_choices.end(),
                 [&](const auto& value) { return value.id == prediction_choice_; });
  if (found != presentation_->prediction_choices.end()) {
   add_label(found->feedback, kMargin, y + 8, kContentWidth, 14, {143, 201, 187});
  }
 }
 auto back = add_button(catalog_.tr("garage.back"), kMargin, height_ - 74.0f, 116, 46);
 back->onClick([this] { actions_.open_garage = true; });
 auto launch = add_button(catalog_.tr("garage.launch"), kMargin + 132, height_ - 74.0f, 180, 46, true);
 launch->setEnabled(!prediction_choice_.empty());
 launch->onClick([this] { actions_.launch = true; });
}

void GameUi::build_running(const game::MissionSimulation* mission) {
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
 auto chapter = tgui::Label::create(catalog_.trf("ui.chapter_row", {std::to_string(level.chapter), std::to_string(level.order), level.title}));
 chapter->setPosition(16, 12);
 chapter->setTextSize(13);
 chapter->getRenderer()->setTextColor({107, 190, 176});
 chapter->getRenderer()->setTextStyle(tgui::TextStyle::Bold);
 objective_card->add(chapter);
 auto objective = tgui::Label::create(presentation_->running_heading);
 objective->setPosition(16, 39);
 objective->setTextSize(18);
 objective->getRenderer()->setTextColor({239, 238, 229});
 objective->getRenderer()->setTextStyle(tgui::TextStyle::Bold);
 objective_card->add(objective);

 auto state_card = tgui::Panel::create({kContentWidth, 104});
 const bool manual = mission && mission->level().control_mode == "manual_calibration";
 state_card->setPosition(kMargin, 120);
 state_card->getRenderer()->setBackgroundColor({19, 27, 25, 225});
 state_card->getRenderer()->setBorderColor({73, 212, 185, 180});
 state_card->getRenderer()->setBorders({0, 0, 0, 3});
 state_card->getRenderer()->setRoundedBorderRadius(12);
 panel_->add(state_card);
 auto live = tgui::Label::create(manual ? catalog_.tr("running.live_drive") : catalog_.tr("running.live_decide"));
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
 auto why = tgui::Label::create(manual ? catalog_.tr("running.manual_title") : catalog_.tr("running.why"));
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
 if (mission) {
  refresh_running(*mission, false, 1.0);
 }
}

void GameUi::build_debrief(const game::MissionSimulation* mission) {
 if (!mission) {
  return;
 }
 const auto& result = mission->result();
 const bool succeeded = result.status == game::MissionStatus::Succeeded;
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
 add_label(badge(result.safe_score, "debrief.badge_safe") + "   " + badge(result.fast_score, "debrief.badge_fast") + "   " +
             badge(result.efficient_score, "debrief.badge_efficient"),
           kMargin, 166, kContentWidth, 17, {220, 222, 216}, true);
 std::string metrics = catalog_.trf("debrief.metrics", {format_number(result.time_s),
                                                             format_number(result.path_length_m),
                                                             format_number(result.energy_wh, 2)});
 metrics += mission->level().concept == "odometry_drift"
              ? catalog_.trf("debrief.metrics_odom", {format_number(result.odometry_error_m, 2)})
              : catalog_.trf("debrief.metrics_clear", {format_number(result.min_clearance_m, 2)});
 add_label(metrics, kMargin, 207, kContentWidth, 16, {182, 190, 186});

 add_label(presentation_->evidence_heading, kMargin, 320, kContentWidth, 16, {239, 238, 229}, true);
 std::ostringstream evidence;
 const auto& events = mission->events();
 const std::size_t first = events.size() > 4 ? events.size() - 4 : 0;
 for (std::size_t index = first; index < events.size(); ++index) {
  evidence << std::fixed << std::setprecision(1) << events[index].time_s << "s  " << events[index].message
           << '\n';
 }
 add_label(evidence.str(), kMargin, 352, kContentWidth, 14, {151, 170, 163});

 add_label(presentation_->knowledge_title, kMargin, 500, kContentWidth, 16, {107, 190, 176}, true);
 add_label(mission->level().knowledge_card, kMargin, 532, kContentWidth, 14, {170, 184, 178});

 if (succeeded) {
  auto chapter = add_button(catalog_.tr("debrief.to_chapter"), kMargin, height_ - 74.0f, 148, 46);
  chapter->onClick([this] { actions_.open_chapter = true; });
  const auto next_id = next_level_id();
  auto next =
    add_button(next_id.empty() ? catalog_.tr("debrief.view_chapter") : catalog_.tr("debrief.next_level"), kMargin + 164, height_ - 74.0f, 148, 46, true);
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

void GameUi::set_style(double style) {
 build_.style = std::clamp(style, 0.0, 1.0);
 actions_.build_changed = true;
 update_style_label();
}

void GameUi::update_style_label() {
 if (!style_value_)
  return;
 std::vector<const game::PolicyDefinition*> anchors;
 for (const auto& [id, policy] : catalog_.policies()) {
  (void)id;
  anchors.push_back(&policy);
 }
 std::sort(anchors.begin(), anchors.end(),
           [](const game::PolicyDefinition* left, const game::PolicyDefinition* right) {
            return left->style_anchor < right->style_anchor;
           });
 const game::PolicyDefinition* nearest = anchors.front();
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

void GameUi::set_build(const game::BuildSelection& build) {
 build_ = build;
 sanitize_build();
 update_garage_labels();
}

std::string GameUi::card_stat(const std::string& slot, const game::ComponentDefinition& item) const {
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

std::string GameUi::card_text(const std::string& slot, const game::ComponentDefinition& item,
                              bool locked) const {
 return catalog_.trf("garage.card",
                     {item.title, card_stat(slot, item), std::to_string(item.cost),
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
 build_.style = std::clamp(build_.style, 0.0, 1.0);
}

void GameUi::update_garage_labels() {
 for (const auto& [slot, cards] : slot_cards_) {
  const std::string current = selected_id(build_, slot);
  for (const auto& card : cards) {
   const auto& item = catalog_.component(card.id);
   const bool locked = !owned_.empty() && !owned_.contains(card.id);
   const bool active = card.id == current;
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
   card.button->setText(card_text(slot, item, locked));
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
 }
}

void GameUi::refresh_running(const game::MissionSimulation& mission, bool paused, double speed) {
 if (!running_metrics_)
  return;
 const auto& state = mission.snapshot();
 const bool manual = mission.level().control_mode == "manual_calibration";
 if (running_explanation_card_) {
  running_explanation_card_->setVisible(manual || state.static_obstacle_detected ||
                                        state.dynamic_object_detected || state.waiting_for_clearance);
 }
 std::string explanation;
 if (manual) {
  explanation = catalog_.tr("running.manual_body");
 } else if (state.waiting_for_clearance) {
  explanation = catalog_.tr("running.waiting_body");
 } else if (state.dynamic_object_detected) {
  explanation = catalog_.tr("running.dynamic_body");
 } else if (state.static_obstacle_detected) {
  explanation = catalog_.tr("running.static_body");
 } else {
  explanation = catalog_.tr("running.clear_head") + mission.level().objective.goal_label;
 }
 if (!mission.level().objective.via.empty()) {
  if (state.via_reached)
   explanation += catalog_.trf("running.via_done", {mission.level().objective.via_label,
                                                   mission.level().objective.goal_label});
  else
   explanation += catalog_.trf("running.via_todo", {mission.level().objective.via_label,
                                                   mission.level().objective.goal_label});
 }
 running_metrics_->setText(explanation);
 if (running_state_) {
  if (paused)
   running_state_->setText(catalog_.tr("running.paused"));
  else if (manual)
   running_state_->setText(catalog_.tr("running.manual_state"));
  else if (state.waiting_for_clearance)
   running_state_->setText(catalog_.tr("running.waiting_state"));
  else if (state.dynamic_object_detected || state.static_obstacle_detected) {
   running_state_->setText(presentation_->detected_line);
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
