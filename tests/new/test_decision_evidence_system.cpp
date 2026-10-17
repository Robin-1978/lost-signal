#include "mission/systems/decision_evidence_system.hpp"

#include <iostream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 mission::DecisionEvidenceSystem system;
 mission::DecisionEvidence moving;
 moving.constraint_code = "none";
 moving.reason_code = "safe-local-trajectory";
 moving.action_code = "avoid";
 moving.target_speed_mps = 0.5;
 const auto& first = system.publish(moving, 1.0);
 CHECK(first.sequence == 1);
 CHECK(first.decision_time_s == 1.0);

 moving.target_speed_mps = 0.75;
 const auto& updated = system.publish(moving, 1.1);
 CHECK(updated.sequence == 1);
 CHECK(updated.decision_time_s == 1.0);
 CHECK(updated.target_speed_mps == 0.75);
 CHECK(system.records().size() == 1);

 mission::DecisionEvidence waiting;
 waiting.constraint_code = "dynamic-clearance";
 waiting.reason_code = "predicted-path-conflict";
 waiting.action_code = "wait";
 const auto& second = system.publish(waiting, 1.2);
 CHECK(second.sequence == 2);
 CHECK(system.records().size() == 2);
 CHECK(system.records().front().outcome_code == "superseded");
 CHECK(system.records().front().end_time_s == 1.2);
 CHECK(system.records().front().superseded_by == 2);

 const auto* closed = system.close("mission-succeeded", 2.0);
 CHECK(closed);
 CHECK(closed->outcome_code == "mission-succeeded");
 CHECK(closed->end_time_s == 2.0);
 CHECK(closed->superseded_by == 0);

 system.reset();
 CHECK(system.records().empty());
 CHECK(system.current() == nullptr);
 CHECK(system.close("unused", 3.0) == nullptr);
 return 0;
}
