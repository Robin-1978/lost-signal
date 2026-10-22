#include "robot/autonomy/planning/task/task_ordering.hpp"

#include <stdexcept>

namespace lost_signal::robot::autonomy {

TaskOrderingRegistry TaskOrderingRegistry::with_builtins() {
  TaskOrderingRegistry registry;
  // Shortest route first; traversal cost breaks ties.
  registry.add("shortest", [](const Parameters&,
                              const TaskCandidateEvidence& e) {
    return e.route_distance_m + 0.5 * e.traversal_cost;
  });
  // Highest value per unit traversal cost.
  registry.add("value_density", [](const Parameters&,
                                   const TaskCandidateEvidence& e) {
    return -e.item_value / (1.0 + e.traversal_cost);
  });
  return registry;
}

void TaskOrderingRegistry::add(std::string id, Scorer scorer) {
  scorers_[std::move(id)] = std::move(scorer);
}

bool TaskOrderingRegistry::contains(const std::string& id) const {
  return scorers_.find(id) != scorers_.end();
}

double TaskOrderingRegistry::score(const std::string& id,
                                   const Parameters& parameters,
                                   const TaskCandidateEvidence& evidence) const {
  const auto it = scorers_.find(id);
  if (it == scorers_.end()) {
    throw std::invalid_argument("unknown task ordering: " + id);
  }
  return it->second(parameters, evidence);
}

}  // namespace lost_signal::robot::autonomy
