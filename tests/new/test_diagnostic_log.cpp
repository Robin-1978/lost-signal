#include "robot/runtime/diagnostic_log.hpp"

#include <iostream>
#include <sstream>

#define CHECK(condition) \
 do { \
 if (!(condition)) { \
 std::cerr << __FILE__ << ':' << __LINE__ \
 << ": check failed: " #condition "\n"; \
 return 1; \
 } \
 } while (false)

int main() {
 sim::runtime::DiagnosticLog log({3, sim::runtime::DiagnosticSeverity::Debug});
 log.write(sim::runtime::TimePoint::from_seconds(1.0),
 sim::runtime::DiagnosticSeverity::Info, "localizer", "accepted",
 {{"score", "0.82"}, {"reason", "map\\\"match"}});
 log.write(sim::runtime::TimePoint::from_seconds(1.1),
 sim::runtime::DiagnosticSeverity::Debug, "planner", "candidate",
 {{"state", "waiting\\nfor\\tclearance"}});
 log.write(sim::runtime::TimePoint::from_seconds(1.2),
 sim::runtime::DiagnosticSeverity::Warn, "provider", "stale",
 {{"age_s", "0.42"}});
 log.write(sim::runtime::TimePoint::from_seconds(1.3),
 sim::runtime::DiagnosticSeverity::Error, "provider", "dropped",
 {{"count", "3"}});
 CHECK(log.size() == 3);
 const auto records = log.snapshot();
 CHECK(records.front().event == "candidate");
 CHECK(records.back().severity == sim::runtime::DiagnosticSeverity::Error);
 std::ostringstream output;
 log.write_jsonl(output);
 const auto serialized = output.str();
 CHECK(serialized.find("\"event\":\"candidate\"") != std::string::npos);
 CHECK(serialized.find("waiting\\\\nfor\\\\tclearance") != std::string::npos);
 CHECK(serialized.find("\\n") != std::string::npos);
 const auto drained = log.drain();
 CHECK(drained.size() == 3);
 CHECK(log.size() == 0);

 sim::runtime::DiagnosticLog filtered(
 {10, sim::runtime::DiagnosticSeverity::Warn});
 filtered.write(sim::runtime::TimePoint::from_seconds(0.0),
 sim::runtime::DiagnosticSeverity::Info, "test", "ignored");
 filtered.write(sim::runtime::TimePoint::from_seconds(0.0),
 sim::runtime::DiagnosticSeverity::Fault, "test", "kept");
 CHECK(filtered.size() == 1);
 std::cout << "diagnostic_log ok\n";
 return 0;
}
