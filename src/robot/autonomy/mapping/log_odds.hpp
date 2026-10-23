#pragma once

// ROBOT-owned log-odds occupancy fusion (pure belief update over
// measurement evidence, never SIM truth). It replaces last-writer-wins
// cell assignment so repeated free observations erase a transient return
// (a person, a noisy flicker) while a static wall accumulates enough
// evidence to survive an occasional spurious clearing ray.
//
// Inverse-sensor-model increments, not tuned probabilities: one confirmed
// passage clears an unseen cell; a confirmed obstacle needs sustained
// contrary evidence to clear (the hysteresis dead band). Publication uses
// thresholds with memory: inside the band the previous decision is
// retained, so one noisy frame cannot flip a wall on and off.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace lost_signal::robot::autonomy {

struct LogOddsConfig {
  float free_delta = -0.40f;      // one passing ray
  float occupied_delta = 1.60f;   // one confirmed endpoint
  float min_log_odds = -2.5f;
  float max_log_odds = 3.5f;
  float occupied_probability = 0.65f;  // publish occupied at/above
  float free_probability = 0.35f;      // publish free at/below
};

class OccupancyLogOdds {
 public:
  explicit OccupancyLogOdds(LogOddsConfig config = {}) : config_(config) {}

  void reset(std::size_t cells) { cells_.assign(cells, Cell{}); }
  std::size_t cell_count() const { return cells_.size(); }

  // Out-of-range cells and non-positive repeats are ignored. Repeats apply
  // in one bounded step so a long ray cannot overflow the clamp.
  void observe_free(std::size_t cell, int repeats = 1) {
    if (!valid() || cell >= cells_.size() || repeats <= 0) return;
    Cell& c = cells_[cell];
    c.log_odds = clamp(c.log_odds +
                       config_.free_delta *
                           static_cast<float>(std::min(repeats, 64)));
    c.observed = true;
    publish(c);
  }
  void observe_occupied(std::size_t cell) {
    if (!valid() || cell >= cells_.size()) return;
    Cell& c = cells_[cell];
    c.log_odds = clamp(c.log_odds + config_.occupied_delta);
    c.observed = true;
    publish(c);
  }

  float log_odds(std::size_t cell) const {
    if (cell >= cells_.size() || !cells_[cell].observed) return 0.0f;
    return cells_[cell].log_odds;
  }
  // -1 unseen, 0 free, 100 occupied (hysteresis on the last decision).
  std::int8_t value(std::size_t cell) const {
    if (cell >= cells_.size()) return -1;
    return cells_[cell].published;
  }
  std::vector<std::int8_t> values() const {
    std::vector<std::int8_t> out;
    out.reserve(cells_.size());
    for (const auto& c : cells_) out.push_back(c.published);
    return out;
  }

  const LogOddsConfig& config() const { return config_; }

 private:
  struct Cell {
    float log_odds = 0.0f;
    bool observed = false;
    std::int8_t published = -1;
  };
  bool valid() const {
    return std::isfinite(config_.free_delta) && config_.free_delta < 0.0f &&
           std::isfinite(config_.occupied_delta) &&
           config_.occupied_delta > 0.0f &&
           std::isfinite(config_.min_log_odds) &&
           config_.min_log_odds < 0.0f &&
           std::isfinite(config_.max_log_odds) &&
           config_.max_log_odds > 0.0f && config_.free_probability > 0.0f &&
           config_.free_probability < 0.5f &&
           config_.occupied_probability > 0.5f &&
           config_.occupied_probability < 1.0f;
  }
  float clamp(float v) const {
    if (!std::isfinite(v)) return 0.0f;
    return std::clamp(v, config_.min_log_odds, config_.max_log_odds);
  }
  static float probability(float l) { return 1.0f / (1.0f + std::exp(-l)); }
  void publish(Cell& c) const {
    const float p = probability(c.log_odds);
    if (c.published == 100) {
      if (p <= config_.free_probability) c.published = 0;
      return;
    }
    if (c.published == 0) {
      if (p >= config_.occupied_probability) c.published = 100;
      return;
    }
    // Unseen cells: one passing ray frees (keeps the map usable near the
    // sensor); occupation needs confirmation above the upper threshold —
    // satisfied by a single endpoint (+1.6 -> p = 0.83), so first-sight
    // walls still paint promptly.
    if (p >= config_.occupied_probability) {
      c.published = 100;
    } else if (p < 0.5f) {
      c.published = 0;
    }
  }
  LogOddsConfig config_;
  std::vector<Cell> cells_;
};

}  // namespace lost_signal::robot::autonomy
