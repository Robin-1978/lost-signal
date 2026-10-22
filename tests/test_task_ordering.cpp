#include <cassert>
#include <cstdio>
#include <stdexcept>

#include "robot/autonomy/planning/task/task_ordering.hpp"

int main() {
  using namespace lost_signal::robot::autonomy;

  auto registry = TaskOrderingRegistry::with_builtins();
  assert(registry.contains("shortest"));
  assert(registry.contains("value_density"));

  TaskCandidateEvidence near{.route_distance_m = 10.0, .traversal_cost = 2.0};
  TaskCandidateEvidence far{.route_distance_m = 50.0, .traversal_cost = 2.0};
  TaskOrderingRegistry::Parameters params;
  assert(registry.score("shortest", params, near) <
         registry.score("shortest", params, far));

  TaskCandidateEvidence rich{.route_distance_m = 50.0,
                             .traversal_cost = 2.0,
                             .item_value = 100.0};
  TaskCandidateEvidence poor{.route_distance_m = 10.0,
                             .traversal_cost = 2.0,
                             .item_value = 1.0};
  assert(registry.score("value_density", params, rich) <
         registry.score("value_density", params, poor));

  registry.add("constant", [](const TaskOrderingRegistry::Parameters&,
                              const TaskCandidateEvidence&) { return 1.0; });
  assert(registry.score("constant", params, near) == 1.0);

  bool threw = false;
  try {
    registry.score("nope", params, near);
  } catch (const std::invalid_argument&) {
    threw = true;
  }
  assert(threw);

  std::printf("task_ordering ok\n");
  return 0;
}
