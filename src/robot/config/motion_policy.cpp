#include "robot/config/motion_policy.hpp"

#include <nlohmann/json.hpp>

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <initializer_list>
#include <stdexcept>
#include <unordered_set>

namespace sim {
namespace {

using json = nlohmann::json;

struct Field {
 const char* key;
 double* value;
};

struct CountField {
	const char* key;
	std::size_t* value;
};

// Applies one policy section. Unknown and missing keys are rejected so that a
// renamed or misspelled parameter cannot quietly fall back to a compiled-in
// default in a deployment that believes it is configured.
void apply_section(const json& object, const std::string& section,
	std::initializer_list<Field> fields,
	std::initializer_list<CountField> counts = {}) {
 if (!object.is_object()) {
 throw std::runtime_error("motion policy: section '" + section + "' must be an object");
 }
 std::unordered_set<std::string> known;
 for (const auto& field : fields) {
 known.emplace(field.key);
 }
	for (const auto& field : counts) {
		known.emplace(field.key);
	}
 for (const auto& item : object.items()) {
 if (!known.count(item.key())) {
 throw std::runtime_error("motion policy: unknown key '" + section + "." + item.key() + "'");
 }
 if (!item.value().is_number()) {
 throw std::runtime_error("motion policy: '" + section + "." + item.key() +
 "' must be a number");
 }
 }
 for (const auto& field : fields) {
 const auto found = object.find(field.key);
 if (found == object.end()) {
 throw std::runtime_error("motion policy: missing key '" + section + "." + field.key + "'");
 }
 *field.value = found->get<double>();
 }
	for (const auto& field : counts) {
		const auto found = object.find(field.key);
		if (found == object.end()) {
			throw std::runtime_error("motion policy: missing key '" + section + "." + field.key + "'");
		}
		if (!found->is_number_integer()) {
			throw std::runtime_error("motion policy: '" + section + "." + field.key +
			                             "' must be an integer");
		}
		const auto parsed = found->get<long long>();
		if (parsed < 0) {
			throw std::runtime_error("motion policy: '" + section + "." + field.key +
			                             "' must not be negative");
		}
		*field.value = static_cast<std::size_t>(parsed);
	}
}

bool finite_positive(double value) { return std::isfinite(value) && value > 0.0; }

bool unit_interval(double value) { return std::isfinite(value) && value >= 0.0 && value <= 1.0; }

} // namespace

MotionPolicy MotionPolicy::defaults() {
 MotionPolicy policy;
 policy.validate();
 return policy;
}

std::filesystem::path MotionPolicy::resolve_path(const std::filesystem::path& content_root) {
 if (const char* configured = std::getenv("LOST_SIGNAL_ROBOT_DATA"); configured != nullptr) {
 if (*configured != '\0') {
 return std::filesystem::path(configured) / "policy" / "motion_policy.json";
 }
 }
 return content_root.parent_path() / "robot_data" / "policy" / "motion_policy.json";
}

MotionPolicy MotionPolicy::load(const std::filesystem::path& path) {
 std::ifstream input(path);
 if (!input) {
 throw std::runtime_error("motion policy: cannot open '" + path.string() + "'");
 }
 json document;
 try {
 input >> document;
 } catch (const std::exception& error) {
 throw std::runtime_error("motion policy: invalid JSON in '" + path.string() + "': " +
 error.what());
 }
 if (!document.is_object()) {
 throw std::runtime_error("motion policy: root must be an object");
 }

 static const std::unordered_set<std::string> top_level{
 "schema_version", "provenance", "localization_quality", "localization_supervisor",
 "localization_motion_authority"};
 for (const auto& item : document.items()) {
 if (!top_level.count(item.key())) {
 throw std::runtime_error("motion policy: unknown top-level key '" + item.key() + "'");
 }
 }
 const auto version = document.at("schema_version");
 if (!version.is_number_integer() || version.get<int>() != 1) {
 throw std::runtime_error("motion policy: unsupported schema_version");
 }
 if (!document.contains("provenance") || !document.at("provenance").is_string() ||
 document.at("provenance").get<std::string>().empty()) {
 throw std::runtime_error("motion policy: 'provenance' must name the tuning record");
 }
 for (const char* section : {"localization_quality", "localization_supervisor",
 "localization_motion_authority"}) {
 if (!document.contains(section)) {
 throw std::runtime_error("motion policy: missing section '" + std::string(section) + "'");
 }
 }

 MotionPolicy policy;
 apply_section(document.at("localization_quality"), "localization_quality",
 {{"position_half_quality_sigma_m", &policy.localization_quality.position_half_quality_sigma_m},
 {"heading_half_quality_sigma_rad", &policy.localization_quality.heading_half_quality_sigma_rad},
 {"covariance_weight", &policy.localization_quality.covariance_weight},
 {"evidence_weight", &policy.localization_quality.evidence_weight},
 {"confidence_decay_time_constant_s",
 &policy.localization_quality.confidence_decay_time_constant_s},
 {"untrusted_position_sigma_m", &policy.localization_quality.untrusted_position_sigma_m},
 {"untrusted_heading_sigma_rad", &policy.localization_quality.untrusted_heading_sigma_rad}});

 auto& supervisor = policy.localization_supervisor;
 apply_section(document.at("localization_supervisor"), "localization_supervisor",
 {{"correction_interval_s", &supervisor.correction_interval_s},
 {"global_retry_interval_s", &supervisor.global_retry_interval_s},
 {"minimum_translation_m", &supervisor.minimum_translation_m},
 {"minimum_heading_rad", &supervisor.minimum_heading_rad},
 {"maximum_idle_interval_s", &supervisor.maximum_idle_interval_s},
 {"local_window_half_extent_m", &supervisor.local_window_half_extent_m},
		 {"lost_position_sigma_m", &supervisor.lost_position_sigma_m},
		 {"lost_heading_sigma_rad", &supervisor.lost_heading_sigma_rad}},
		{{"local_failure_recovery_threshold", &supervisor.local_failure_recovery_threshold}});

 auto& motion = policy.localization_motion;
 apply_section(document.at("localization_motion_authority"), "localization_motion_authority",
 {{"hold_confidence", &motion.hold_confidence},
 {"hold_max_age_s", &motion.hold_max_age_s},
 {"reduced_confidence", &motion.reduced_confidence},
 {"reduced_speed_scale", &motion.reduced_speed_scale},
 {"cautious_confidence", &motion.cautious_confidence},
 {"cautious_speed_scale", &motion.cautious_speed_scale}});

 policy.validate();
 return policy;
}

void MotionPolicy::validate() const {
 const auto& quality = localization_quality;
 if (!finite_positive(quality.position_half_quality_sigma_m) ||
 !finite_positive(quality.heading_half_quality_sigma_rad)) {
 throw std::runtime_error("motion policy: quality sigma scales must be positive");
 }
 if (!unit_interval(quality.covariance_weight) || !unit_interval(quality.evidence_weight) ||
 std::abs(quality.covariance_weight + quality.evidence_weight - 1.0) > 1e-9) {
 throw std::runtime_error("motion policy: quality weights must be a unit-interval partition");
 }
 if (!finite_positive(quality.confidence_decay_time_constant_s)) {
 throw std::runtime_error("motion policy: confidence decay time constant must be positive");
 }
 if (!finite_positive(quality.untrusted_position_sigma_m) ||
 !finite_positive(quality.untrusted_heading_sigma_rad)) {
 throw std::runtime_error("motion policy: untrusted sigmas must be positive");
 }

 const auto& supervisor = localization_supervisor;
 if (!finite_positive(supervisor.correction_interval_s) ||
 !finite_positive(supervisor.global_retry_interval_s) ||
 !finite_positive(supervisor.maximum_idle_interval_s) ||
 !finite_positive(supervisor.local_window_half_extent_m) ||
 !finite_positive(supervisor.lost_position_sigma_m) ||
 !finite_positive(supervisor.lost_heading_sigma_rad)) {
 throw std::runtime_error("motion policy: localization supervisor intervals and sigmas must be positive");
 }
 if (!std::isfinite(supervisor.minimum_translation_m) || supervisor.minimum_translation_m < 0.0 ||
 !std::isfinite(supervisor.minimum_heading_rad) || supervisor.minimum_heading_rad < 0.0) {
 throw std::runtime_error("motion policy: localization supervisor motion gates must be non-negative");
 }
 if (supervisor.local_failure_recovery_threshold < 1.0) {
 throw std::runtime_error("motion policy: local failure recovery threshold must be at least one");
 }

 const auto& motion = localization_motion;
 for (const double confidence : {motion.hold_confidence, motion.reduced_confidence,
 motion.cautious_confidence}) {
 if (!unit_interval(confidence)) {
 throw std::runtime_error("motion policy: localization confidence gates must lie in [0, 1]");
 }
 }
 for (const double scale : {motion.reduced_speed_scale, motion.cautious_speed_scale}) {
 if (!(std::isfinite(scale) && scale > 0.0 && scale <= 1.0)) {
 throw std::runtime_error("motion policy: speed scales must lie in (0, 1]");
 }
 }
 if (!(motion.hold_confidence <= motion.reduced_confidence &&
 motion.reduced_confidence <= motion.cautious_confidence)) {
 throw std::runtime_error("motion policy: localization confidence gates must increase");
 }
 if (!(motion.reduced_speed_scale <= motion.cautious_speed_scale &&
 motion.cautious_speed_scale <= 1.0)) {
 throw std::runtime_error("motion policy: speed scales must not decrease with better localization");
 }
 if (!finite_positive(motion.hold_max_age_s)) {
 throw std::runtime_error("motion policy: localization hold age must be positive");
 }
}

} // namespace sim
