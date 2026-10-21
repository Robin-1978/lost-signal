#include "robot/autonomy/mapping/occupancy_log_odds.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

namespace sim::autonomy {
namespace {

bool valid_config(const OccupancyLogOddsConfig& config) {
 return std::isfinite(config.free_delta) && config.free_delta < 0.0F &&
 std::isfinite(config.occupied_delta) && config.occupied_delta > 0.0F &&
 std::isfinite(config.minimum_log_odds) && config.minimum_log_odds < 0.0F &&
 std::isfinite(config.maximum_log_odds) && config.maximum_log_odds > 0.0F &&
 config.free_probability > 0.0F && config.free_probability < 0.5F &&
 config.occupied_probability > 0.5F && config.occupied_probability < 1.0F;
}

float probability(float log_odds) {
 return 1.0F / (1.0F + std::exp(-log_odds));
}

} // namespace

OccupancyLogOdds::OccupancyLogOdds(OccupancyLogOddsConfig config)
 : config_(std::move(config)) {}

void OccupancyLogOdds::reset(std::size_t cells) { cells_.assign(cells, Cell{}); }

float OccupancyLogOdds::clamp(float value) const {
 if (!std::isfinite(value))
 return 0.0F;
 return std::clamp(value, config_.minimum_log_odds, config_.maximum_log_odds);
}

void OccupancyLogOdds::update_publication(Cell& cell) const {
 const float p = probability(cell.log_odds);
 if (cell.published == 100) {
 // Clearing a confirmed obstacle requires sustained contrary evidence.
 if (p <= config_.free_probability)
 cell.published = 0;
 return;
 }
 if (cell.published == 0) {
 if (p >= config_.occupied_probability)
 cell.published = 100;
 return;
 }
 // Never-published cells resolve on the first decisive evidence. Marking an
 // unseen cell free from one passing ray keeps the map usable near the
 // sensor, while an unseen cell needs confirmation to become an obstacle.
 if (p >= config_.occupied_probability)
 cell.published = 100;
 else if (p < 0.5F)
 cell.published = 0;
}

void OccupancyLogOdds::assign_values(const std::vector<std::int8_t>& values) {
 cells_.assign(values.size(), {});
 const auto log_odds_of = [](float probability) {
 return std::log(probability / (1.0F - probability));
 };
 const float occupied = log_odds_of(config_.occupied_probability);
 const float free = log_odds_of(config_.free_probability);
 for (std::size_t index = 0; index < values.size(); ++index) {
 const std::int8_t value = values[index];
 if (value < 0)
 continue;
 auto& cell = cells_[index];
 cell.observed = true;
 cell.published = value;
 cell.log_odds = value >= 50 ? occupied : free;
 }
}

void OccupancyLogOdds::observe_free(std::size_t cell, int repeats) {
 if (!valid_config(config_) || cell >= cells_.size() || repeats <= 0)
 return;
 auto& stored = cells_[cell];
 stored.log_odds = clamp(stored.log_odds +
 config_.free_delta * static_cast<float>(std::min(repeats, 64)));
 stored.observed = true;
 update_publication(stored);
}

void OccupancyLogOdds::observe_occupied(std::size_t cell) {
 if (!valid_config(config_) || cell >= cells_.size())
 return;
 auto& stored = cells_[cell];
 stored.log_odds = clamp(stored.log_odds + config_.occupied_delta);
 stored.observed = true;
 update_publication(stored);
}

float OccupancyLogOdds::log_odds(std::size_t cell) const {
 if (cell >= cells_.size() || !cells_[cell].observed)
 return 0.0F;
 return cells_[cell].log_odds;
}

std::int8_t OccupancyLogOdds::value(std::size_t cell) const {
 if (cell >= cells_.size())
 return -1;
 return cells_[cell].published;
}

std::vector<std::int8_t> OccupancyLogOdds::values() const {
 std::vector<std::int8_t> output;
 output.reserve(cells_.size());
 for (const auto& cell : cells_)
 output.push_back(cell.published);
 return output;
}

std::size_t OccupancyLogOdds::observed_cells() const {
 return static_cast<std::size_t>(std::count_if(
 cells_.begin(), cells_.end(), [](const auto& cell) { return cell.observed; }));
}

} // namespace sim::autonomy
