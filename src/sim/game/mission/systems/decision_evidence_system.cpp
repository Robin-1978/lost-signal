#include "sim/game/mission/systems/decision_evidence_system.hpp"

#include <utility>

namespace mission {

void DecisionEvidenceSystem::reset() {
 records_.clear();
 next_sequence_ = 0;
}

const DecisionEvidence& DecisionEvidenceSystem::publish(
 DecisionEvidence decision, double now_s) {
 const bool same_decision = !records_.empty() &&
 records_.back().constraint_code == decision.constraint_code &&
 records_.back().reason_code == decision.reason_code &&
 records_.back().action_code == decision.action_code &&
 records_.back().outcome_code == "active";

 decision.outcome_code = "active";
 decision.end_time_s = -1.0;
 decision.superseded_by = 0;
 if (same_decision) {
 decision.sequence = records_.back().sequence;
 decision.decision_time_s = records_.back().decision_time_s;
 records_.back() = std::move(decision);
 return records_.back();
 }

 const std::uint64_t sequence = ++next_sequence_;
 if (!records_.empty() && records_.back().outcome_code == "active") {
 records_.back().end_time_s = now_s;
 records_.back().outcome_code = "superseded";
 records_.back().superseded_by = sequence;
 }
 decision.sequence = sequence;
 decision.decision_time_s = now_s;
 records_.push_back(std::move(decision));
 return records_.back();
}

const DecisionEvidence* DecisionEvidenceSystem::close(
 std::string outcome_code, double now_s) {
 if (records_.empty())
 return nullptr;
 records_.back().end_time_s = now_s;
 records_.back().outcome_code = std::move(outcome_code);
 records_.back().superseded_by = 0;
 return &records_.back();
}

} // namespace mission
