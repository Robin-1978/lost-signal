#pragma once

#include "robot/autonomy/mapping/map_builder.hpp"
#include "robot/autonomy/mapping/elevation_fusion.hpp"
#include "robot/autonomy/mapping/occupancy_log_odds.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

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
 double sensor_height_m{0.30};
 double ground_rejection_band_m{0.06};
 double translation_search_m{0.30};
 double translation_step_m{0.05};
 double heading_search_rad{0.20};
 double heading_step_rad{0.05};
 std::size_t maximum_matching_returns{64};
 double minimum_match_score{0.30};
 double minimum_match_margin{0.02};
 bool publish_elevation_layer{true};
 ElevationFusionConfig elevation_fusion{};
 // The live submap fuses inverse-sensor-model evidence instead of
 // overwriting cells. A passing person or cart must decay out of the map
 // once rays pass through it again, while a real wall keeps accumulating
 // endpoint evidence and survives an occasional spurious clearing ray.
 // A fresh endpoint earns confirmation immediately (2.4 > 0.65); clearing
 // that confirmation needs about 1.5 s of 10 Hz free-carving, which is the
 // cost of not letting a single bad ray delete a real obstacle.
 OccupancyLogOddsConfig occupancy{-0.20F, 2.40F, -2.5F, 3.5F, 0.65F, 0.35F};
 std::string map_id{"online-slam"};
};

class OnlineSlamMapBuilder final : public IMapBuilder {
 public:
 explicit OnlineSlamMapBuilder(OnlineSlamConfig config = {});

 void reset() override;
 // Rigid correction of the local map frame produced by a pose-graph solve.
 // The submap raster, elevation and pose are all shifted so that later scans
 // stay in the corrected frame; without this feedback a published loop
 // closure would be followed by poses in the old frame.
 void apply_map_correction(Vec2 translation_m, double heading_rad);
 MapBuilderUpdate update(const ObservationFrame& observations,
 const EstimatedState& prior, runtime::TimePoint sample_time) override;

 private:
 OnlineSlamConfig config_;
 runtime::MapGrid map_;
 bool initialized_{false};
 double last_stamp_s_{-1.0};
 Vec2 anchor_world_position_{};
 double anchor_world_heading_rad_{0.0};
 Vec2 last_prior_position_{};
 double last_prior_heading_rad_{0.0};
 Vec2 pose_{};
 double heading_rad_{0.0};
 std::uint64_t revision_{0};
 ElevationFusion elevation_;
 OccupancyLogOdds occupancy_;

 void initialize_map();
 Vec2 world_to_local(Vec2 world) const;
 Vec2 sensor_position(Vec2 pose, double heading) const;
 bool valid_config() const;
 std::size_t known_cells() const;
 double match_score(const Scan& scan, const EstimatedState& motion,
 Vec2 pose, double heading,
 std::size_t* matched_returns) const;
 void integrate_scan(const Scan& scan, const EstimatedState& motion);
 double ray_time_offset_s(const Scan& scan, std::size_t index) const;
 std::optional<Scan> project_cloud(const LidarPointCloud& cloud,
 const EstimatedState& state) const;
 void update_elevation(std::size_t cell_index, double elevation_m);
 MapBuilderUpdate make_update(MappingState state, double stamp_s,
 std::string diagnostic) const;
};

} // namespace sim::autonomy
