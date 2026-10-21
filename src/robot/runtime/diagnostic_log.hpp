#pragma once

#include "robot/runtime/runtime_channel.hpp"

#include <cstddef>
#include <cstdint>
#include <map>
#include <ostream>
#include <string>
#include <vector>

namespace sim::runtime {

enum class DiagnosticSeverity { Trace, Debug, Info, Warn, Error, Fault };

const char* diagnostic_severity_name(DiagnosticSeverity severity);

struct DiagnosticRecord {
 TimePoint time;
 DiagnosticSeverity severity{DiagnosticSeverity::Info};
 std::string component;
 std::string event;
 std::map<std::string, std::string> fields;
};

struct DiagnosticLogConfig {
 // A bounded log is safe to keep enabled on a robot. The oldest records are
 // discarded only after this limit; the caller can drain them periodically.
 std::size_t maximum_records{4096};
 DiagnosticSeverity minimum_severity{DiagnosticSeverity::Info};
};

class DiagnosticLog {
 public:
 explicit DiagnosticLog(DiagnosticLogConfig config = {});

 void write(TimePoint time, DiagnosticSeverity severity,
 const std::string& component, const std::string& event,
 std::map<std::string, std::string> fields = {});

 std::vector<DiagnosticRecord> snapshot() const;
 std::vector<DiagnosticRecord> drain();
 std::size_t size() const;

 // JSON Lines is deliberately the interchange format: it can be tailed while
 // a robot runs, ingested by jq/ELK/Python, and attached to a deterministic
 // replay artifact without making the runtime depend on a logging framework.
 void write_jsonl(std::ostream& output) const;

 private:
 static std::string json_escape(const std::string& value);

 DiagnosticLogConfig config_;
 std::vector<DiagnosticRecord> records_;
};

} // namespace sim::runtime
