#pragma once

#include "robot/autonomy/mapping/map_builder.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace sim::autonomy {

// A bounded, deterministic online SLAM frontend for small maps. It maintains
// one immutable occupancy submap and estimates SE(2) pose from the wheel/IMU
// prior plus correlative LiDAR alignment. It deliberately does not expose
// simulator truth, actor IDs, or a physics model.
struct OnlineSlamConfig {
 std::uint32_t width{240};
 std::uint32_t height{240};
 double resolution_m{0.05};
 double maximum_range_m{12.0};
 double sensor_x_m{0.0};
 double sensor_y_m{0.0};
 double sensor_yaw_rad{0.0};
 double translation_search_m{0.30};
 double translation_step_m{0.05};
 double heading_search_rad{0.20};
 double heading_step_rad{0.05};
 std::size_t maximum_matching_returns{64};
 double minimum_match_score{0.30};
 double minimum_match_margin{0.02};
 std::string map_id{"online-slam"};
};

class OnlineSlamMapBuilder final : public IMapBuilder {
 public:
 explicit OnlineSlamMapBuilder(OnlineSlamConfig config = {});

 void reset() override;
 MapBuilderUpdate update(const ObservationFrame& observations,
 const EstimatedState& prior, runtime::TimePoint sample_time) override;

 private:
 OnlineSlamConfig config_;
 runtime::MapGrid map_;
 bool initialized_{false};
 double last_stamp_s_{-1.0};
 Vec2 anchor_world_position_{};
 double anchor_world_heading_rad_{0.0};
 Vec2 pose_{};
 double heading_rad_{0.0};
 std::uint64_t revision_{0};

 void initialize_map();
 Vec2 world_to_local(Vec2 world) const;
 Vec2 sensor_position(Vec2 pose, double heading) const;
 bool valid_config() const;
 std::size_t known_cells() const;
 double match_score(const Scan& scan, Vec2 pose, double heading,
 std::size_t* matched_returns) const;
 void integrate_scan(const Scan& scan);
 MapBuilderUpdate make_update(MappingState state, double stamp_s,
 std::string diagnostic) const;
};

} // namespace sim::autonomy
