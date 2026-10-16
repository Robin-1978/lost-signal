#pragma once

#include "mission/content_catalog.hpp"
#include "mission/mission_simulation.hpp"

#include <TGUI/TGUI.hpp>

#include <optional>
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
 bool build_changed{false};
 bool reset_simulation{false};
 double speed{0.0};
 std::string selected_level;
 std::optional<double> replay_time_s;
 std::string purchase_component;
};

class GameUi {
 public:
 GameUi(tgui::BackendGui& gui, const mission::ContentCatalog& catalog,
        const mission::PresentationDefinition& presentation);

 void set_screen(Screen screen, const mission::MissionSimulation* mission = nullptr);
 void resize(int width, int height);
 void refresh_running(const mission::MissionSimulation& mission, bool paused, double speed);
 void refresh_debrief(const mission::MissionSimulation& mission);
 UiActions consume_actions();

 Screen screen() const { return screen_; }
 const mission::BuildSelection& build() const { return build_; }
 bool build_within_budget() const;
 bool endless_mode() const { return endless_mode_; }
 const mission::PresentationDefinition& presentation() const { return *presentation_; }
 const std::string& level_id() const { return presentation_->level; }
 const std::string& contract_id() const { return active_contract_id_; }
 void use_recommended_build();
 void cycle_component(const std::string& slot);
 void select_level(const std::string& level_id);
 void mark_completed(const std::string& level_id);
 void restore_completed(const std::vector<std::string>& level_ids);
 // DIY garage: owned component ids filter the columns; the last attempted
 // build is restored when re-entering a level.
 void set_inventory(const std::vector<std::string>& component_ids);
 void set_abilities(const std::vector<std::string>& ability_ids);
 void set_contract_status(int reputation, const std::vector<std::string>& claimed_contracts);
 void set_build(const mission::BuildSelection& build);

 private:
 void rebuild(const mission::MissionSimulation* mission);
 void build_chapter();
 void build_garage();
 void build_prediction();
 void build_running(const mission::MissionSimulation* mission);
 void build_debrief(const mission::MissionSimulation* mission);
 void select_component(const std::string& slot, const std::string& id);
 void select_chassis(const std::string& robot_id);
 void set_style(double style);
 void update_style_label();
 void update_garage_labels();
 void sanitize_build();
 bool is_unlocked(const mission::LevelDefinition& level) const;
 std::string next_level_id() const;
 // Garage card text (single source for create + refresh): title / slot stat
 // from component behavior / cost. Stats are content numbers, never invented.
 std::string card_stat(const std::string& slot, const mission::ComponentDefinition& item) const;
 std::string card_text(const std::string& slot, const mission::ComponentDefinition& item, bool locked) const;

 tgui::Label::Ptr add_label(const std::string& text, float x, float y, float width, unsigned text_size,
                            tgui::Color color, bool bold = false);
 tgui::Button::Ptr add_button(const std::string& text, float x, float y, float width, float height,
                              bool primary = false);
 void style_panel();

 tgui::BackendGui& gui_;
 const mission::ContentCatalog& catalog_;
 const mission::PresentationDefinition* presentation_;
 tgui::Panel::Ptr panel_;
 // Widget parent for add_label/add_button: the plain panel everywhere except
 // the chapter screen, which hosts 21+ level buttons in a ScrollablePanel
 // (fixed panel overflowed past 720p once the roster passed ~12 levels).
 tgui::Panel::Ptr add_parent_;
 mission::BuildSelection build_;
 Screen screen_{Screen::Chapter};
 UiActions actions_;
 int width_{1440};
 int height_{900};
 std::string prediction_choice_;
 std::unordered_set<std::string> completed_levels_;
 std::unordered_set<std::string> owned_;
 std::unordered_set<std::string> owned_abilities_;
 std::unordered_set<std::string> claimed_contracts_;
 int reputation_{0};
 std::string active_contract_id_;
 bool endless_mode_{false};
 struct SlotCard {
  std::string id;
  tgui::Button::Ptr button;
 };
 std::unordered_map<std::string, std::vector<SlotCard>> slot_cards_;
 std::vector<SlotCard> chassis_cards_;
 tgui::Slider::Ptr style_slider_;
 tgui::Label::Ptr style_value_;
 tgui::Label::Ptr budget_label_;
 tgui::Button::Ptr garage_next_button_;
 tgui::CheckBox::Ptr endless_checkbox_;
 tgui::Label::Ptr running_metrics_;
 tgui::Label::Ptr running_state_;
 tgui::Panel::Ptr running_explanation_card_;
 tgui::Button::Ptr pause_button_;
 tgui::Slider::Ptr replay_slider_;
 tgui::Label::Ptr replay_value_;
 tgui::Label::Ptr replay_cause_;
 tgui::Label::Ptr replay_evidence_;
 tgui::Label::Ptr comparison_label_;
 std::unordered_map<std::string, mission::MissionResult> previous_results_;
 std::optional<mission::MissionResult> comparison_result_;
};

} // namespace client
