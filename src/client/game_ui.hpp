#pragma once

#include "game/content_catalog.hpp"
#include "game/mission_simulation.hpp"

#include <TGUI/TGUI.hpp>

#include <string>
#include <unordered_map>
#include <unordered_set>

namespace client {

enum class Screen { Chapter, Garage, Prediction, Running, Debrief };

struct UiActions {
 bool open_prediction{false};
 bool launch{false};
 bool retry{false};
 bool open_garage{false};
 bool open_chapter{false};
 bool toggle_pause{false};
 double speed{0.0};
 std::string selected_level;
};

class GameUi {
public:
 GameUi(tgui::BackendGui& gui, const game::ContentCatalog& catalog,
 const game::PresentationDefinition& presentation);

 void set_screen(Screen screen, const game::MissionSimulation* mission = nullptr);
 void resize(int width, int height);
 void refresh_running(const game::MissionSimulation& mission, bool paused, double speed);
 UiActions consume_actions();

 Screen screen() const { return screen_; }
 const game::BuildSelection& build() const { return build_; }
 const game::PresentationDefinition& presentation() const { return *presentation_; }
 const std::string& level_id() const { return presentation_->level; }
 void use_recommended_build();
 void select_level(const std::string& level_id);
 void mark_completed(const std::string& level_id);
 void restore_completed(const std::vector<std::string>& level_ids);
 // DIY garage: owned component ids filter the columns; the last attempted
 // build is restored when re-entering a level.
 void set_inventory(const std::vector<std::string>& component_ids);
 void set_build(const game::BuildSelection& build);

private:
 void rebuild(const game::MissionSimulation* mission);
 void build_chapter();
 void build_garage();
 void build_prediction();
 void build_running(const game::MissionSimulation* mission);
 void build_debrief(const game::MissionSimulation* mission);
 void cycle_component(const std::string& slot);
 void set_style(double style);
 void update_style_label();
 void update_garage_labels();
 void sanitize_build();
 bool is_unlocked(const game::LevelDefinition& level) const;
 std::string next_level_id() const;

 tgui::Label::Ptr add_label(const std::string& text, float x, float y,
 float width, unsigned text_size, tgui::Color color, bool bold = false);
 tgui::Button::Ptr add_button(const std::string& text, float x, float y,
 float width, float height, bool primary = false);
 void style_panel();

 tgui::BackendGui& gui_;
 const game::ContentCatalog& catalog_;
 const game::PresentationDefinition* presentation_;
 tgui::Panel::Ptr panel_;
 game::BuildSelection build_;
 Screen screen_{Screen::Chapter};
 UiActions actions_;
 int width_{1440};
 int height_{900};
 std::string prediction_choice_;
 std::unordered_set<std::string> completed_levels_;
 std::unordered_set<std::string> owned_;
 std::unordered_map<std::string, tgui::Button::Ptr> slot_buttons_;
 tgui::Slider::Ptr style_slider_;
 tgui::Label::Ptr style_value_;
 tgui::Label::Ptr budget_label_;
 tgui::Label::Ptr running_metrics_;
 tgui::Label::Ptr running_state_;
 tgui::Panel::Ptr running_explanation_card_;
 tgui::Button::Ptr pause_button_;
};

} // namespace client
