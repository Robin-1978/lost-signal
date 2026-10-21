#pragma once

#include <cstddef>
#include <cstdint>
#include <vector>

namespace sim::autonomy {

struct OccupancyLogOddsConfig {
    // Evidence per measurement. A passing ray clears a cell; an endpoint
    // fills it. These are inverse-sensor-model increments, not tuned
    // probabilities.
 // One confirmed passage clears an unseen cell; a cell that already holds
 // occupied evidence needs a second contradictory observation, which is the
 // dead band doing its job.
 float free_delta{-0.40F};
    // A confirmed endpoint outweighs a single passage: obstacles are recorded
 // promptly, while clearing them needs sustained free evidence.
 float occupied_delta{1.60F};
    float minimum_log_odds{-2.5F};
    float maximum_log_odds{3.5F};
    // Publication thresholds with a dead band. Inside the band the previous
    // decision is retained, so one noisy frame cannot flip a wall on and off
    // in the navigation map.
    float occupied_probability{0.65F};
    float free_probability{0.35F};
};

// Deterministic log-odds occupancy fusion. It replaces last-writer-wins cell
// assignment so repeated free observations can erase a transient return (a
// person or a parked cart), while a genuinely static wall accumulates enough
// evidence to survive an occasional spurious clearing ray.
class OccupancyLogOdds final {
public:
    explicit OccupancyLogOdds(OccupancyLogOddsConfig config = {});

    void reset(std::size_t cells);
 // Restores a previously published raster after a rigid submap correction.
 // The hysteresis history is not preserved across the resample, but the
 // published decision is, so a correction cannot erase confirmed obstacles.
 void assign_values(const std::vector<std::int8_t>& values);
    std::size_t cell_count() const { return cells_.size(); }

    // Out-of-range cells and non-positive repeat counts are ignored. Repeats
    // are applied in one bounded step so a long ray cannot overflow the clamp.
    void observe_free(std::size_t cell, int repeats = 1);
    void observe_occupied(std::size_t cell);

    float log_odds(std::size_t cell) const;
    // -1 unseen, 0 free, 100 occupied, using the configured thresholds with
    // hysteresis on the previously published value.
    std::int8_t value(std::size_t cell) const;
    std::vector<std::int8_t> values() const;
    std::size_t observed_cells() const;

    const OccupancyLogOddsConfig& config() const { return config_; }

private:
    struct Cell {
        float log_odds{0.0F};
        bool observed{false};
        std::int8_t published{-1};
    };

    float clamp(float value) const;
 void update_publication(Cell& cell) const;
    OccupancyLogOddsConfig config_;
    std::vector<Cell> cells_;
};

} // namespace sim::autonomy
