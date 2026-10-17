#include "task_ordering.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>

namespace autonomy {
namespace {

double parameter(const TaskOrderingRegistry::Parameters& values, const std::string& name,
 double fallback) {
 const auto found = values.find(name);
 return found == values.end() ? fallback : found->second;
}

} // namespace

TaskOrderingRegistry TaskOrderingRegistry::with_builtins() {
 TaskOrderingRegistry registry;
 registry.add("route-cost", [](const Parameters&, const TaskCandidateEvidence& evidence) {
 return evidence.traversal_cost;
 });
 registry.add("minimax-risk", [](const Parameters& parameters,
 const TaskCandidateEvidence& evidence) {
 const double risk_scale = parameter(parameters, "risk_scale", 1000.0);
 return std::max(0.0, evidence.worst_traversal_multiplier - 1.0) * risk_scale +
 evidence.traversal_cost;
 });
 registry.add("value-density", [](const Parameters& parameters,
 const TaskCandidateEvidence& evidence) {
 const double exponent = parameter(parameters, "value_exponent", 1.0);
 return evidence.traversal_cost /
 std::pow(std::max(1.0, evidence.item_value), exponent);
 });
 registry.add("balanced-density", [](const Parameters& parameters,
 const TaskCandidateEvidence& evidence) {
 const double exponent = parameter(parameters, "value_exponent", 0.5);
 const double distance_mix = std::clamp(parameter(parameters, "distance_mix", 0.15), 0.0, 1.0);
 const double route_cost = evidence.traversal_cost * (1.0 - distance_mix) +
 evidence.route_distance_m * distance_mix;
 return route_cost / std::pow(std::max(1.0, evidence.item_value), exponent);
 });
 return registry;
}

void TaskOrderingRegistry::add(std::string id, Scorer scorer) {
 if (id.empty() || !scorer || !scorers_.emplace(id, std::move(scorer)).second) {
 throw std::invalid_argument("invalid or duplicate task-ordering algorithm");
 }
}

bool TaskOrderingRegistry::contains(const std::string& id) const {
 return scorers_.contains(id);
}

double TaskOrderingRegistry::score(const std::string& id,
 const Parameters& parameters, const TaskCandidateEvidence& evidence) const {
 const auto found = scorers_.find(id);
 if (found == scorers_.end()) {
 throw std::invalid_argument("unknown task-ordering algorithm '" + id + "'");
 }
 const double result = found->second(parameters, evidence);
 if (!std::isfinite(result)) {
 throw std::runtime_error("task-ordering algorithm produced a non-finite score");
 }
 return result;
}

} // namespace autonomy
