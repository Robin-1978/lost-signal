#include "robot/runtime/diagnostic_log.hpp"

#include <algorithm>

namespace sim::runtime {

const char* diagnostic_severity_name(DiagnosticSeverity severity) {
 switch (severity) {
 case DiagnosticSeverity::Trace: return "trace";
 case DiagnosticSeverity::Debug: return "debug";
 case DiagnosticSeverity::Info: return "info";
 case DiagnosticSeverity::Warn: return "warn";
 case DiagnosticSeverity::Error: return "error";
 case DiagnosticSeverity::Fault: return "fault";
 }
 return "unknown";
}

DiagnosticLog::DiagnosticLog(DiagnosticLogConfig config) : config_(config) {
 if (config_.maximum_records == 0)
 config_.maximum_records = 1;
}

void DiagnosticLog::write(TimePoint time, DiagnosticSeverity severity,
 const std::string& component, const std::string& event,
 std::map<std::string, std::string> fields) {
 if (severity < config_.minimum_severity)
 return;
 records_.push_back({time, severity, component, event, std::move(fields)});
 if (records_.size() > config_.maximum_records) {
 records_.erase(records_.begin(),
 records_.begin() + static_cast<std::ptrdiff_t>(
 records_.size() - config_.maximum_records));
 }
}

std::vector<DiagnosticRecord> DiagnosticLog::snapshot() const {
 return records_;
}

std::vector<DiagnosticRecord> DiagnosticLog::drain() {
 std::vector<DiagnosticRecord> drained;
 drained.swap(records_);
 return drained;
}

std::size_t DiagnosticLog::size() const {
 return records_.size();
}

std::string DiagnosticLog::json_escape(const std::string& value) {
 std::string escaped;
 escaped.reserve(value.size() + 8);
 for (const char character : value) {
 switch (character) {
 case '\"': escaped += "\\\""; break;
 case '\\': escaped += "\\\\"; break;
 case '\n': escaped += "\\n"; break;
 case '\r': escaped += "\\r"; break;
 case '\t': escaped += "\\t"; break;
 default:
 if (static_cast<unsigned char>(character) < 0x20) {
 escaped += ' ';
 } else {
 escaped += character;
 }
 }
 }
 return escaped;
}

void DiagnosticLog::write_jsonl(std::ostream& output) const {
 for (const auto& record : records_) {
 output << "{\"time_s\":" << record.time.seconds()
 << ",\"severity\":\"" << diagnostic_severity_name(record.severity)
 << "\",\"component\":\"" << json_escape(record.component)
 << "\",\"event\":\"" << json_escape(record.event) << "\",\"fields\":{";
 bool first = true;
 for (const auto& [key, value] : record.fields) {
 if (!first) output << ',';
 first = false;
 output << '\"' << json_escape(key) << "\":\""
 << json_escape(value) << '\"';
 }
 output << "}}\n";
 }
}

} // namespace sim::runtime
