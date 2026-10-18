#include "sim/game/mission/component_registry.hpp"
#include "sim/game/mission/content_catalog.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#define CHECK(condition)                                                                                     \
 do {                                                                                                        \
  if (!(condition)) {                                                                                        \
   std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n";                           \
   return 1;                                                                                                 \
  }                                                                                                          \
 } while (false)

namespace fs = std::filesystem;

namespace {

struct PayloadSpec {
 double capacity_kg{0.0};
};

const char kBrokenTire[] = R"({
 "schema_version": 2,
 "slot": "tires",
 "model_profile": {"fidelity":"typical","basis":"test","source_refs":["test"]},
 "components": [{
 "id": "tires.broken_registry_test",
 "title": "Broken tire",
 "description": "Missing slip_scale on purpose.",
 "cost": 1,
 "behavior": {
 "brake_decel_mps2": 3.0,
 "power_scale": 1.0,
 "friction_mu": 0.7,
 "rolling_resistance": 0.01
 },
 "visual": {
 "primitive_2d": "wheel",
 "size_m": [0.2, 0.1, 0.12],
 "color": "#202D32",
 "model_3d": "models/robots/ro1/wheel_standard.glb"
 }
 }]
})";

const char kPayload[] = R"({
 "schema_version": 2,
 "slot": "payload",
 "model_profile": {"fidelity":"typical","basis":"test","source_refs":["test"]},
 "components": [{
 "id": "payload.cooler",
 "title": "Cold box",
 "description": "Registry extension fixture.",
 "cost": 1,
 "behavior": {"capacity_kg": 12.0},
 "visual": {
 "primitive_2d": "box",
 "size_m": [0.2, 0.2, 0.2],
 "color": "#FFFFFF",
 "model_3d": "models/robots/ro1/compute_light.glb"
 }
 }]
})";

void write_file(const fs::path& path, const char* contents) {
 std::ofstream output(path, std::ios::trunc);
 if (!output) {
  throw std::runtime_error("cannot create registry test fixture");
 }
 output << contents;
}

} // namespace

int main() {
 const auto catalog = mission::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 mission::BuildSelection all;
 all.parts = {
   {"tires", "tires.standard"},
   {"sensor", "sensor.standard_lidar"},
   {"computer", "computer.performance"},
   {"motor", "motor.standard"},
   {"imu", "imu.basic"},
 };
 const auto resolved = mission::ComponentRegistry::builtins().resolve(catalog, all);
 CHECK(resolved.has("tires"));
 CHECK(resolved.definition("sensor").model_profile.fidelity == "typical");
 CHECK(!resolved.definition("sensor").model_profile.source_refs.empty());
 CHECK(resolved.traction().brake_decel_mps2 == 3.0);
 CHECK(resolved.traction().normal_stiffness_Npm == 40000.0);
 CHECK(resolved.traction().normal_damping_Nspm == 1000.0);
CHECK(resolved.lidar() && resolved.lidar()->rays == 800);
 CHECK(resolved.lidar()->min_range_m == 0.2);
 CHECK(resolved.lidar()->dropout_probability == 0.0);
 CHECK(resolved.computer() && resolved.computer()->decision_hz == 10.0);
 CHECK(resolved.motor() && resolved.motor()->stall_force_N == 54.0);
 CHECK(resolved.motor()->controller_bandwidth_radps == 6.0);
CHECK(resolved.imu() && resolved.imu()->gyro_bias_stddev_radps == 0.005);
 CHECK(std::abs(resolved.imu()->gyro_noise_radps -
 0.0002443461 * std::sqrt(20.0)) < 1e-12);
 CHECK(std::abs(resolved.imu()->accel_noise_mps2 -
 0.00171616375 * std::sqrt(20.0)) < 1e-12);
 CHECK(mission::ComponentRegistry::builtins().supported_slots().size() == 6);

 mission::BuildSelection tracked;
 tracked.parts["tracks"] = "tracks.grip";
 const auto resolved_tracks = mission::ComponentRegistry::builtins().resolve(catalog, tracked);
 CHECK(resolved_tracks.has("tracks"));
 CHECK(resolved_tracks.traction().friction_mu == 1.2);

 const fs::path overlay = fs::temp_directory_path() / "lost_signal_component_registry";
 std::error_code error;
 fs::remove_all(overlay, error);
 error.clear();
 fs::copy(LOST_SIGNAL_TEST_ASSETS, overlay,
          fs::copy_options::recursive | fs::copy_options::overwrite_existing, error);
 CHECK(!error);
 write_file(overlay / "components" / "broken_registry_test.json", kBrokenTire);
 write_file(overlay / "components" / "payload_registry_test.json", kPayload);
 const auto extended_catalog = mission::ContentCatalog::load(overlay.string());

 mission::BuildSelection broken;
 broken.parts["tires"] = "tires.broken_registry_test";
 bool threw = false;
 try {
  (void)mission::ComponentRegistry::builtins().resolve(extended_catalog, broken);
 } catch (const mission::ContentError&) {
  threw = true;
 }
 CHECK(threw);

 mission::BuildSelection payload;
 payload.parts["payload"] = "payload.cooler";
 threw = false;
 try {
  (void)mission::ComponentRegistry::builtins().resolve(extended_catalog, payload);
 } catch (const mission::ContentError&) {
  threw = true;
 }
 CHECK(threw);

 mission::ComponentRegistry extension_registry = mission::ComponentRegistry::with_builtins();
 extension_registry.register_slot("payload", [](const mission::ComponentDefinition& definition) {
  return PayloadSpec{definition.behavior.at("capacity_kg")};
 });
 const auto custom = extension_registry.resolve(extended_catalog, payload);
 CHECK(custom.has("payload"));
 CHECK(custom.definition("payload").id == "payload.cooler");
 CHECK(custom.get<PayloadSpec>("payload"));
 CHECK(custom.get<PayloadSpec>("payload")->capacity_kg == 12.0);
 CHECK(extension_registry.supported_slots().size() == 7);

 threw = false;
 try {
  extension_registry.register_slot("payload", [](const mission::ComponentDefinition& definition) {
   return PayloadSpec{definition.behavior.at("capacity_kg")};
  });
 } catch (const std::runtime_error&) {
  threw = true;
 }
 CHECK(threw);
 return 0;
}
