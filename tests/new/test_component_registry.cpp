#include "game/component_registry.hpp"
#include "game/content_catalog.hpp"

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
 "schema_version": 1,
 "slot": "tires",
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
 "schema_version": 1,
 "slot": "payload",
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
 const auto catalog = game::ContentCatalog::load(LOST_SIGNAL_TEST_ASSETS);
 game::BuildSelection all;
 all.parts = {
   {"tires", "tires.standard"},
   {"sensor", "sensor.standard_lidar"},
   {"computer", "computer.performance"},
   {"motor", "motor.standard"},
   {"imu", "imu.basic"},
 };
 const auto resolved = game::ComponentRegistry::builtins().resolve(catalog, all);
 CHECK(resolved.has("tires"));
 CHECK(resolved.tires().brake_decel_mps2 == 3.0);
 CHECK(resolved.lidar() && resolved.lidar()->rays == 72);
 CHECK(resolved.lidar()->dropout_probability == 0.0);
 CHECK(resolved.computer() && resolved.computer()->decision_hz == 10.0);
 CHECK(resolved.motor() && resolved.motor()->stall_force_N == 54.0);
 CHECK(resolved.imu() && resolved.imu()->gyro_bias_radps == 0.02);
 CHECK(resolved.imu()->accel_noise_mps2 == 0.15);
 CHECK(game::ComponentRegistry::builtins().supported_slots().size() == 5);

 const fs::path overlay = "/tmp/lost_signal_component_registry";
 std::error_code error;
 fs::remove_all(overlay, error);
 error.clear();
 fs::copy(LOST_SIGNAL_TEST_ASSETS, overlay,
          fs::copy_options::recursive | fs::copy_options::overwrite_existing, error);
 CHECK(!error);
 write_file(overlay / "components" / "broken_registry_test.json", kBrokenTire);
 write_file(overlay / "components" / "payload_registry_test.json", kPayload);
 const auto extended_catalog = game::ContentCatalog::load(overlay.string());

 game::BuildSelection broken;
 broken.parts["tires"] = "tires.broken_registry_test";
 bool threw = false;
 try {
  (void)game::ComponentRegistry::builtins().resolve(extended_catalog, broken);
 } catch (const game::ContentError&) {
  threw = true;
 }
 CHECK(threw);

 game::BuildSelection payload;
 payload.parts["payload"] = "payload.cooler";
 threw = false;
 try {
  (void)game::ComponentRegistry::builtins().resolve(extended_catalog, payload);
 } catch (const game::ContentError&) {
  threw = true;
 }
 CHECK(threw);

 game::ComponentRegistry extension_registry = game::ComponentRegistry::with_builtins();
 extension_registry.register_slot("payload", [](const game::ComponentDefinition& definition) {
  return PayloadSpec{definition.behavior.at("capacity_kg")};
 });
 const auto custom = extension_registry.resolve(extended_catalog, payload);
 CHECK(custom.has("payload"));
 CHECK(custom.definition("payload").id == "payload.cooler");
 CHECK(custom.get<PayloadSpec>("payload"));
 CHECK(custom.get<PayloadSpec>("payload")->capacity_kg == 12.0);
 CHECK(extension_registry.supported_slots().size() == 6);

 threw = false;
 try {
  extension_registry.register_slot("payload", [](const game::ComponentDefinition& definition) {
   return PayloadSpec{definition.behavior.at("capacity_kg")};
  });
 } catch (const std::runtime_error&) {
  threw = true;
 }
 CHECK(threw);
 return 0;
}
