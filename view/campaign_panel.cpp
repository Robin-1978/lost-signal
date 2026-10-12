#include "campaign_panel.hpp"
#include <cstdint>

bool CampaignPanel::init() {
  // 中文剧情需要CJK字体(DejaVu无CJK字形会显示方框)
  if (initFromFile("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc")) return true;
  return initFromFile("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf");
}

bool CampaignPanel::initFromFile(const std::string& path) {
  has_font_ = font_.loadFromFile(path);
  return has_font_;
}

void CampaignPanel::text(sf::RenderTarget& win, const std::string& s, float x, float y, int size,
                         sf::Color c) {
  if (!has_font_) return;
  // SFML2的std::string隐转按ANSI解码,C locale下中文变方框,必须显式fromUtf8
  sf::Text t(sf::String::fromUtf8(s.begin(), s.end()), font_, (unsigned)size);
  t.setFillColor(c);
  t.setPosition(x, y);
  win.draw(t);
}

// 按像素宽贪心换行(中文无空格, 按UTF-8码点逐字累积测量).
std::vector<std::string> CampaignPanel::wrap(const std::string& s, unsigned size, float max_w) {
  std::vector<std::string> out;
  std::string cur;
  size_t i = 0;
  auto push = [&]() {
    if (!cur.empty()) out.push_back(cur);
    cur.clear();
  };
  while (i < s.size()) {
    size_t n = 1;
    uint8_t c = (uint8_t)s[i];
    if ((c & 0x80) == 0) n = 1;
    else if ((c & 0xE0) == 0xC0) n = 2;
    else if ((c & 0xF0) == 0xE0) n = 3;
    else n = 4;
    std::string piece = s.substr(i, n);
    sf::String cur_u = sf::String::fromUtf8(cur.begin(), cur.end());
    sf::String piece_u = sf::String::fromUtf8(piece.begin(), piece.end());
    sf::Text tm(cur_u + piece_u, font_, size);
    if (tm.getLocalBounds().width > max_w && !cur.empty()) {
      push();
      cur = piece;
    } else {
      cur += piece;
    }
    i += n;
  }
  push();
  return out;
}

void CampaignPanel::draw(sf::RenderTarget& win) {
  if (!visible_ || !ls_ || !st_) return;
  float x0 = 8, y0 = 8, w = 330;
  sf::RectangleShape bg(sf::Vector2f(w, 700));
  bg.setPosition(x0, y0);
  bg.setFillColor(sf::Color(10, 10, 16, 220));
  bg.setOutlineColor(sf::Color(80, 80, 100));
  bg.setOutlineThickness(1);
  win.draw(bg);

  float y = y0 + 8;
  char hd[64];
  snprintf(hd, sizeof hd, "CAMPAIGN (N hide)  %d/%zu", st_->progress(), ls_->size());
  text(win, hd, x0 + 10, y, 13, sf::Color(255, 220, 80));
  y += 24;
  for (auto& l : *ls_) {
    bool done = st_->completed(l.task);
    bool open = st_->unlocked(l.task);
    char row[160];
    snprintf(row, sizeof row, "%s %d %s %s", (l.task == sel_ ? ">" : " "), l.task, l.name,
             (done ? "[DONE]" : (open ? "[OPEN]" : "[LOCK]")));
    sf::Color c = done ? sf::Color(120, 220, 120)
                       : (open ? (l.task == sel_ ? sf::Color(255, 255, 255)
                                                : sf::Color(200, 200, 200))
                               : sf::Color(100, 100, 110));
    text(win, row, x0 + 10, y, 12, c);
    y += 19;
  }
  y += 6;
  char f[64];
  snprintf(f, sizeof f, "-- briefing (fail %d) --", fails_);
  text(win, f, x0 + 10, y, 12, sf::Color(120, 220, 120));
  y += 20;
  for (auto& ln : wrap(story_, 11, w - 20)) {
    text(win, ln, x0 + 10, y, 11, sf::Color(200, 200, 200));
    y += 16;
  }
  y += 4;
  for (auto& ln : wrap(std::string("目标: ") + brief_, 11, w - 20)) {
    text(win, ln, x0 + 10, y, 11, sf::Color(180, 220, 255));
    y += 16;
  }
  y += 4;
  for (auto& ln : wrap(std::string("线索: ") + clue_, 11, w - 20)) {
    text(win, ln, x0 + 10, y, 11, sf::Color(255, 200, 120));
    y += 16;
  }
  if (!fail_why_.empty()) {
    y += 4;
    for (auto& ln : wrap(std::string("上次失败: ") + fail_why_, 11, w - 20)) {
      text(win, ln, x0 + 10, y, 11, sf::Color(255, 140, 140));
      y += 16;
    }
    if (!fail_hint_.empty())
      for (auto& ln : wrap(std::string("诊断: ") + fail_hint_, 11, w - 20)) {
        text(win, ln, x0 + 10, y, 11, sf::Color(255, 170, 120));
        y += 16;
      }
  }
  y += 8;
  if (running_)
    text(win, std::string("RUNNING ") + run_label_ + " ...", x0 + 10, y, 12,
         sf::Color(255, 220, 80));
  else
    text(win, "[/]选关  [Enter]跑本关", x0 + 10, y, 12, sf::Color(150, 150, 150));
  y += 22;
  if (!toast_.empty())
    for (auto& ln : wrap(toast_, 11, w - 20)) {
      text(win, ln, x0 + 10, y, 11, sf::Color(140, 255, 140));
      y += 16;
      if (y > y0 + 690) break;
    }
}
