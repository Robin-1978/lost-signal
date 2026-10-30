#include "sim/viewer/player_progress_ui.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <sstream>

namespace lost_signal::sim::viewer {
namespace {

const ChapterProgress* chapter_progress(const PlayerProfile* profile,
                                        const std::string& experience_id) {
  if (profile == nullptr) return nullptr;
  const auto it = std::find_if(
      profile->chapters.begin(), profile->chapters.end(),
      [&](const ChapterProgress& chapter) {
        return chapter.experience_id == experience_id;
      });
  return it == profile->chapters.end() ? nullptr : &*it;
}

bool card_unlocked(const ExperienceManifest& experience,
                   const ChapterProgress* progress, std::size_t index) {
  if (progress == nullptr || index >= experience.lessons.size()) return false;
  const std::string id = knowledge_card_id(experience.experience_id, index);
  return std::find(progress->knowledge_cards.begin(),
                   progress->knowledge_cards.end(), id) !=
         progress->knowledge_cards.end();
}

const char* text(bool zh_cn, const char* english, const char* chinese) {
  return zh_cn ? chinese : english;
}

float measure(const Font& font, const char* value, float size, float scale) {
  return MeasureTextEx(font, value, size * scale, 0.5f).x;
}

void draw_text(const Font& font, const char* value, int x, int y, float size,
               float scale, Color color) {
  DrawTextEx(font, value, {static_cast<float>(x), static_cast<float>(y)},
             size * scale, 0.5f, color);
}

float draw_wrapped(const Font& font, const std::string& value, float x, float y,
                   float width, float size, float gap, float scale,
                   Color color) {
  const float font_size = size * scale;
  const float line_gap = gap * scale;
  const float spacing = 1.0f;
  std::istringstream input(value);
  std::string word;
  std::string line;
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

void draw_stars(const Font& font, std::uint8_t stars, int x, int y,
                float scale, Color color) {
  std::string value = "[";
  for (int index = 0; index < 3; ++index) {
    value += index < static_cast<int>(stars) ? '*' : '-';
  }
  value += "]";
  draw_text(font, value.c_str(), x, y, 11, scale, color);
}

}  // namespace

std::vector<ChapterCatalogEntry> load_chapter_catalog(
    const PlayerProfile& profile, const ExperienceManifest& current,
    const std::string& root_path) {
  std::vector<ChapterCatalogEntry> entries;
  const std::filesystem::path root(root_path);
  if (std::filesystem::is_directory(root)) {
    for (const auto& file : std::filesystem::directory_iterator(root)) {
      if (!file.is_regular_file() || file.path().extension() != ".json") {
        continue;
      }
      ExperienceManifest manifest;
      std::string error;
      if (!load_experience_manifest(file.path().string(), manifest, &error)) {
        continue;
      }
      const ChapterProgress* progress =
          chapter_progress(&profile, manifest.experience_id);
      entries.push_back({manifest.experience_id, manifest.title,
                         progress == nullptr ? std::uint8_t{0} : progress->stars,
                         progress == nullptr ? 0 : progress->completions});
    }
  }
  if (std::none_of(entries.begin(), entries.end(), [&](const auto& entry) {
        return entry.experience_id == current.experience_id;
      })) {
    const ChapterProgress* progress =
        chapter_progress(&profile, current.experience_id);
    entries.push_back({current.experience_id, current.title,
                       progress == nullptr ? std::uint8_t{0} : progress->stars,
                       progress == nullptr ? 0 : progress->completions});
  }
  std::sort(entries.begin(), entries.end(),
            [](const auto& left, const auto& right) {
              return left.experience_id < right.experience_id;
            });
  return entries;
}

void draw_player_profile_panel(const ExperienceManifest& experience,
                               const PlayerProfile* profile, bool detailed,
                               Font font, float scale, bool zh_cn) {
  if (profile == nullptr) return;
  constexpr Color kPanel{13, 21, 22, 238};
  constexpr Color kText{226, 235, 230, 255};
  constexpr Color kMuted{157, 177, 169, 255};
  constexpr Color kCyan{55, 218, 202, 255};
  constexpr Color kAmber{246, 169, 61, 255};
  const ChapterProgress* progress =
      chapter_progress(profile, experience.experience_id);
  const float screen_width = static_cast<float>(GetScreenWidth());
  const float width = std::min(292.0f, screen_width - 36.0f);
  const float height = detailed ? 242.0f : 128.0f;
  const float x = detailed ? 18.0f : screen_width - width - 18.0f;
  const float y = detailed ? 18.0f : 24.0f;
  DrawRectangleRounded({x, y, width, height}, 0.04f, 8, kPanel);
  DrawRectangleRoundedLinesEx({x, y, width, height}, 0.04f, 8, 1.5f,
                              detailed ? kAmber : kCyan);
  draw_text(font, text(zh_cn, "CHAPTER PROGRESS", "章节进度"), x + 16,
            y + 14, 13, scale, detailed ? kAmber : kCyan);
  draw_text(font, experience.title.c_str(), x + 16, y + 37, 14, scale,
            kText);

  const std::uint8_t stars = progress == nullptr ? 0 : progress->stars;
  std::string star_line = "STARS ";
  for (int index = 0; index < 3; ++index) {
    star_line += index < static_cast<int>(stars) ? '*' : '-';
  }
  char stats[128];
  std::snprintf(stats, sizeof(stats), "%s   RUNS %u", star_line.c_str(),
                progress == nullptr ? 0U : progress->completions);
  draw_text(font, stats, x + 16, y + 60, 12, scale,
            stars > 0 ? kAmber : kMuted);
  char best[160];
  if (progress != nullptr && progress->best_elapsed_s > 0.0) {
    std::snprintf(best, sizeof(best),
                  zh_cn ? "最佳耗时 %.2f s　最佳净空 %.2f m"
                        : "BEST %.2f s   CLEARANCE %.2f m",
                  progress->best_elapsed_s, progress->best_clearance_m);
  } else {
    std::snprintf(best, sizeof(best), "%s",
                  text(zh_cn, "NO PUBLISHED RUN YET", "尚无已发布成绩"));
  }
  draw_text(font, best, x + 16, y + 82, 11, scale, kMuted);
  std::size_t unlocked = 0;
  for (std::size_t index = 0; index < experience.lessons.size(); ++index) {
    if (card_unlocked(experience, progress, index)) ++unlocked;
  }
  char cards[96];
  std::snprintf(cards, sizeof(cards),
                zh_cn ? "知识卡 %zu / %zu　按 U 查看教学"
                      : "KNOWLEDGE CARDS %zu / %zu   U: LEARNING VIEW",
                unlocked, experience.lessons.size());
  draw_text(font, cards, x + 16, y + 104, 11, scale, kCyan);
  if (!detailed || experience.lessons.empty()) return;
  float card_y = y + 128.0f;
  const float card_width = width - 24.0f;
  for (std::size_t index = 0; index < experience.lessons.size(); ++index) {
    if (!card_unlocked(experience, progress, index)) continue;
    const Rectangle card{x + 12.0f, card_y, card_width, 32.0f};
    DrawRectangleRounded(card, 0.06f, 6, Color{25, 43, 43, 255});
    draw_text(font, text(zh_cn, "CARD", "知识卡"), card.x + 8,
              card.y + 8, 10, scale, kCyan);
    draw_wrapped(font, experience.lessons[index], card.x + 58.0f,
                 card.y + 7.0f, card.width - 68.0f, 10.0f, 1.0f, scale,
                 kText);
    card_y += 37.0f;
    if (card_y > y + height - 26.0f) break;
  }
}

void draw_chapter_catalog_panel(
    const std::vector<ChapterCatalogEntry>& entries,
    const std::string& current_experience_id, Font font, float scale,
    bool zh_cn) {
  constexpr Color kPanel{13, 21, 22, 246};
  constexpr Color kText{226, 235, 230, 255};
  constexpr Color kMuted{157, 177, 169, 255};
  constexpr Color kCyan{55, 218, 202, 255};
  constexpr Color kAmber{246, 169, 61, 255};
  const float screen_width = static_cast<float>(GetScreenWidth());
  const float screen_height = static_cast<float>(GetScreenHeight());
  const float width = std::min(430.0f, screen_width - 36.0f);
  const float height = std::min(screen_height - 36.0f,
                                58.0f + 27.0f * entries.size());
  const float x = screen_width - width - 18.0f;
  const float y = 18.0f;
  DrawRectangleRounded({x, y, width, height}, 0.04f, 8, kPanel);
  DrawRectangleRoundedLinesEx({x, y, width, height}, 0.04f, 8, 1.5f,
                              kAmber);
  draw_text(font, text(zh_cn, "CHAPTER CATALOG", "章节目录"), x + 16,
            y + 14, 14, scale, kAmber);
  draw_text(font, text(zh_cn, "PUBLISHED PROGRESS / READ ONLY",
                       "已发布进度 / 只读目录"),
            x + 16, y + 35, 10, scale, kMuted);
  const float row_y = y + 54.0f;
  const float row_height = 25.0f;
  for (std::size_t index = 0; index < entries.size(); ++index) {
    const auto& entry = entries[index];
    const bool current = entry.experience_id == current_experience_id;
    const float item_y = row_y + static_cast<float>(index) * row_height;
    if (current) {
      DrawRectangleRounded({x + 10.0f, item_y - 2.0f, width - 20.0f,
                            row_height - 2.0f},
                           0.08f, 5, Color{34, 58, 56, 255});
    }
    draw_text(font, current ? ">" : " ", x + 16, item_y + 3, 12, scale,
              current ? kCyan : kMuted);
    const std::string label = entry.title.empty() ? entry.experience_id
                                                   : entry.title;
    draw_text(font, label.c_str(), x + 32, item_y + 3, 11, scale,
              current ? kText : kMuted);
    draw_stars(font, entry.stars, static_cast<int>(x + width - 84),
               static_cast<int>(item_y + 3), scale,
               entry.stars > 0 ? kAmber : kMuted);
  }
}

}  // namespace lost_signal::sim::viewer
