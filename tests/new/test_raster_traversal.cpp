#include "robot/autonomy/mapping/raster_traversal.hpp"

#include <cmath>
#include <iostream>
#include <memory>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

namespace {

std::shared_ptr<sim::runtime::MapSnapshot> map() {
 auto output = std::make_shared<sim::runtime::MapSnapshot>();
 output->map_id = "terrain/test";
 output->revision = 1;
 output->frame_id = "map";
 output->grid.emplace();
 output->grid->width = 4;
 output->grid->height = 4;
 output->grid->resolution_m = 1.0;
 output->grid->occupancy.assign(16, 0);
 return output;
}

} // namespace

int main() {
 sim::RasterTraversalModel model;
 model.bind(map());
 // Unknown terrain is not silently treated as perfectly smooth.
 const auto unknown = model.sample({1.5, 1.5});
 CHECK(unknown.roughness > 0.0);

 sim::EstimatedState estimate;
 estimate.position = {1.5, 1.5};
 estimate.linear_velocity_mps = 1.0;
 estimate.timestamp_s = 1.0;
 sim::ImuObservation smooth;
 smooth.up_acceleration_mps2 = 9.81;
 smooth.acceleration_noise_sigma_mps2 = 0.01;
 smooth.header.stamp_s = 1.0;
 model.observe_motion(estimate, smooth, 0.1);
 const auto after_smooth = model.sample(estimate.position);
 CHECK(after_smooth.roughness < 0.03);

 sim::ImuObservation rough = smooth;
 rough.up_acceleration_mps2 = 30.0;
 rough.header.stamp_s = 2.0;
 estimate.timestamp_s = 2.0;
 for (int i = 0; i < 30; ++i)
 model.observe_motion(estimate, rough, 0.1);
 const auto after_rough = model.sample(estimate.position);
 CHECK(after_rough.roughness > after_smooth.roughness);
 CHECK(after_rough.roughness <= 0.20 + 1e-9);
 estimate.linear_velocity_mps = 0.0;
 for (int i = 0; i < 120; ++i) {
 rough.header.stamp_s = 12.0 + i * 0.1;
 model.observe_motion(estimate, rough, 0.1);
 }
 const auto stale = model.sample(estimate.position);
 CHECK(stale.roughness < after_rough.roughness);
 CHECK(stale.roughness >= 0.0);
 // Roughness is a planning input as well as a diagnostic: it contributes to
 // traversal cost, and the scenario's ride-speed cap consumes the same value.
 auto rough_map = map();
 rough_map->grid->roughness_m.assign(16, 0.10F);
 rough_map->grid->roughness_confidence.assign(16, 1.0F);
 sim::RasterTraversalModel rough_model;
 rough_model.bind(rough_map);
 sim::TerrainCapability capability;
 capability.mass_kg = 10.0;
 capability.available_drive_force_N = 1000.0;
 capability.roughness_cost_weight = 2.0;
 CHECK(rough_model.traversal_cost({1.5, 1.5}, {2.5, 1.5}, capability) > 1.15);
 return 0;
}
