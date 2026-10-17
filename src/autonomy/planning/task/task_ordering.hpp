#pragma once

#include <functional>
#include <string>
#include <unordered_map>

namespace autonomy {

struct TaskCandidateEvidence {
 double route_distance_m{0.0};
 double traversal_cost{0.0};
 double worst_traversal_multiplier{1.0};
 double item_value{0.0};
};

// Registry boundary for task ordering. Content chooses a registered algorithm
// and supplies its numeric parameters; missions never branch on policy names.
class TaskOrderingRegistry {
 public:
 using Parameters = std::unordered_map<std::string, double>;
 using Scorer = std::function<double(const Parameters&, const TaskCandidateEvidence&)>;

 static TaskOrderingRegistry with_builtins();
 void add(std::string id, Scorer scorer);
 bool contains(const std::string& id) const;
 double score(const std::string& id, const Parameters& parameters,
 const TaskCandidateEvidence& evidence) const;

 private:
 std::unordered_map<std::string, Scorer> scorers_;
};

} // namespace autonomy
