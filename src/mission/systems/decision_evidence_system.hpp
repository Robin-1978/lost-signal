#pragma once

#include "mission/decision_evidence.hpp"

#include <string>
#include <vector>

namespace mission {

// Owns the stable decision-transition log. It deduplicates controller ticks,
// closes superseded records, and exposes the one current immutable value used
// by snapshot, replay, headless output, and UI presentation.
class DecisionEvidenceSystem {
 public:
 void reset();
 const DecisionEvidence& publish(DecisionEvidence decision, double now_s);
 const DecisionEvidence* close(std::string outcome_code, double now_s);

 const std::vector<DecisionEvidence>& records() const { return records_; }
 const DecisionEvidence* current() const {
 return records_.empty() ? nullptr : &records_.back();
 }

 private:
 std::vector<DecisionEvidence> records_;
 std::uint64_t next_sequence_{0};
};

} // namespace mission
