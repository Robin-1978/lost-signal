#include "component_registry.hpp"

#include <cmath>
#include <stdexcept>
#include <utility>

namespace game {
namespace {

double required_number(const ComponentDefinition& component, const char* name, double minimum,
                       bool allow_minimum = true) {
 const auto found = component.behavior.find(name);
 if (found == component.behavior.end()) {
  throw ContentError("component '" + component.id + "' is missing behavior property '" + name + "'");
 }
 const double value = found->second;
 const bool in_range = allow_minimum ? value >= minimum : value > minimum;
 if (!std::isfinite(value) || !in_range) {
  throw ContentError("component '" + component.id + "' has invalid behavior property '" + name + "'");
 }
 return value;
}

double optional_number(const ComponentDefinition& component, const char* name, double fallback,
 double minimum, double maximum) {
 const auto found = component.behavior.find(name);
 if (found == component.behavior.end()) {
 return fallback;
 }
 const double value = found->second;
 if (!std::isfinite(value) || value < minimum || value > maximum) {
 throw ContentError("component '" + component.id + "' has invalid behavior property '" + name + "'");
 }
 return value;
}

ComponentSpec resolve_tires(const ComponentDefinition& component) {
 return TireSpec{
   required_number(component, "slip_scale", 0.0),
   required_number(component, "brake_decel_mps2", 0.0, false),
   required_number(component, "power_scale", 0.0),
   required_number(component, "friction_mu", 0.0, false),
   required_number(component, "rolling_resistance", 0.0),
 };
}

ComponentSpec resolve_lidar(const ComponentDefinition& component) {
 const double rays = required_number(component, "rays", 1.0);
 if (rays > 4096.0 || std::floor(rays) != rays) {
  throw ContentError("component '" + component.id + "' has invalid behavior property 'rays'");
 }
 return LidarSpec{
   required_number(component, "range_m", 0.0, false), static_cast<int>(rays),
   required_number(component, "scan_hz", 0.0, false), required_number(component, "noise_sigma_m", 0.0),
 optional_number(component, "range_bias_m", 0.0, -10.0, 10.0),
 optional_number(component, "range_resolution_m", 0.0, 0.0, 10.0),
 optional_number(component, "dropout_probability", 0.0, 0.0, 1.0),
   required_number(component, "power_w", 0.0),
 };
}

ComponentSpec resolve_computer(const ComponentDefinition& component) {
 return ComputeSpec{
   required_number(component, "decision_hz", 0.0, false),
   required_number(component, "prediction_horizon_s", 0.0),
   required_number(component, "power_w", 0.0),
   required_number(component, "track_max_age_s", 0.0, false),
 };
}

ComponentSpec resolve_motor(const ComponentDefinition& component) {
 return MotorSpec{
   required_number(component, "max_wheel_speed_mps", 0.0, false),
   required_number(component, "stall_force_N", 0.0, false),
   required_number(component, "free_speed_mps", 0.0, false),
   required_number(component, "power_scale", 0.0, false),
 };
}

ComponentSpec resolve_imu(const ComponentDefinition& component) {
 return ImuSpec{
   required_number(component, "gyro_noise_radps", 0.0),
   required_number(component, "gyro_bias_radps", 0.0),
   required_number(component, "power_w", 0.0),
   required_number(component, "accel_noise_mps2", 0.0),
 optional_number(component, "gyro_bias_random_walk_radps_sqrt_s", 0.0, 0.0, 10.0),
 };
}

} // namespace

bool ResolvedBuild::has(const std::string& slot) const {
 return components_.contains(slot);
}

const ComponentDefinition& ResolvedBuild::definition(const std::string& slot) const {
 const auto found = components_.find(slot);
 if (found == components_.end() || !found->second.definition) {
  throw ContentError("build has no component in slot '" + slot + "'");
 }
 return *found->second.definition;
}

const TireSpec& ResolvedBuild::tires() const {
 const auto* value = get<TireSpec>("tires");
 if (!value) {
  throw ContentError("build has no component in required slot 'tires'");
 }
 return *value;
}

const LidarSpec* ResolvedBuild::lidar() const {
 return get<LidarSpec>("sensor");
}

const ComputeSpec* ResolvedBuild::computer() const {
 return get<ComputeSpec>("computer");
}

const MotorSpec* ResolvedBuild::motor() const {
 return get<MotorSpec>("motor");
}

const ImuSpec* ResolvedBuild::imu() const {
 return get<ImuSpec>("imu");
}

const ComponentRegistry& ComponentRegistry::builtins() {
 static const ComponentRegistry registry = with_builtins();
 return registry;
}

ComponentRegistry ComponentRegistry::with_builtins() {
 ComponentRegistry result;
 result.register_slot("tires", resolve_tires);
 result.register_slot("sensor", resolve_lidar);
 result.register_slot("computer", resolve_computer);
 result.register_slot("motor", resolve_motor);
 result.register_slot("imu", resolve_imu);
 return result;
}

void ComponentRegistry::register_slot(std::string slot, Resolver resolver) {
 if (slot.empty() || !resolver) {
  throw std::invalid_argument("component slot and resolver must be non-empty");
 }
 if (!resolvers_.emplace(slot, std::move(resolver)).second) {
  throw std::runtime_error("component slot '" + slot + "' is already registered");
 }
}

bool ComponentRegistry::supports(const std::string& slot) const {
 return resolvers_.contains(slot);
}

std::vector<std::string> ComponentRegistry::supported_slots() const {
 std::vector<std::string> result;
 result.reserve(resolvers_.size());
 for (const auto& [slot, resolver] : resolvers_) {
  (void)resolver;
  result.push_back(slot);
 }
 return result;
}

ResolvedBuild ComponentRegistry::resolve(const ContentCatalog& catalog, const BuildSelection& build) const {
 ResolvedBuild result;
 for (const auto& [slot, id] : build.parts) {
  if (id.empty()) {
   continue;
  }
  const auto resolver = resolvers_.find(slot);
  if (resolver == resolvers_.end()) {
   throw ContentError("component slot '" + slot + "' has no registered runtime resolver");
  }
  const auto& definition = catalog.component(id);
  if (definition.slot != slot) {
   throw ContentError("component '" + id + "' does not belong in slot '" + slot + "'");
  }
  result.components_.emplace(slot, ResolvedComponent{&definition, resolver->second(definition)});
 }
 return result;
}

} // namespace game
