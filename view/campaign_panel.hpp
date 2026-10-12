#pragma once
#include <SFML/Graphics.hpp>
#include <string>
#include <vector>
#include "../game/campaign.hpp"
// View-only: 只读camp状态做展示, 无任何战役逻辑(选关/结算全在camp::Play).
class CampaignPanel {
 public:
  bool init();
  void setVisible(bool v) { visible_ = v; }
  bool visible() const { return visible_; }
  void toggle() { visible_ = !visible_; }
  void setLevels(const std::vector<camp::Level>& ls, const camp::CampaignState& st, int sel) {
    ls_ = &ls;
    st_ = &st;
    sel_ = sel;
  }
  void setDetail(const std::string& story, const std::string& brief, const std::string& clue,
                 int fails) {
    story_ = story;
    brief_ = brief;
    clue_ = clue;
    fails_ = fails;
  }
  void setRunning(bool r, const std::string& label = "") {
    running_ = r;
    run_label_ = label;
  }
  void setToast(const std::string& s) { toast_ = s; }
  void draw(sf::RenderTarget& win);

 private:
  std::vector<std::string> wrap(const std::string& s, unsigned size, float max_w);
  void text(sf::RenderTarget& win, const std::string& s, float x, float y, int size,
            sf::Color c);
  sf::Font font_;
  bool has_font_{false}, visible_{false}, running_{false};
  const std::vector<camp::Level>* ls_{nullptr};
  const camp::CampaignState* st_{nullptr};
  int sel_{1}, fails_{0};
  std::string story_, brief_, clue_, run_label_, toast_;
};
