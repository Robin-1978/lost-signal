#include "campaign_progress.hpp"

#include "content_catalog.hpp"
#include "sim/game/scenario/scenario_session.hpp"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <system_error>

namespace mission {
namespace {

using json = nlohmann::json;

bool known_level(const ContentCatalog& catalog, const std::string& id) {
 return catalog.levels().contains(id);
}

WeatherKind parse_weather(const std::string& id) {
 if (id == "clear") return WeatherKind::Clear;
 if (id == "overcast") return WeatherKind::Overcast;
 if (id == "drizzle") return WeatherKind::Drizzle;
 if (id == "fog") return WeatherKind::Fog;
 throw ContentError("progress: unknown regional weather '" + id + "'");
}

WorldIncidentKind parse_incident(const std::string& id) {
 if (id == "road-closure") return WorldIncidentKind::RoadClosure;
 if (id == "fallen-cargo") return WorldIncidentKind::FallenCargo;
 if (id == "power-fault") return WorldIncidentKind::PowerFault;
 throw ContentError("progress: unknown regional incident '" + id + "'");
}

FieldIntentKind parse_field_intent(const std::string& id) {
 if (id == "follow") return FieldIntentKind::Follow;
 if (id == "go_to") return FieldIntentKind::GoTo;
 if (id == "search") return FieldIntentKind::Search;
 if (id == "wait") return FieldIntentKind::Wait;
 if (id == "return") return FieldIntentKind::Return;
 throw ContentError("progress: unknown field intent '" + id + "'");
}

} // namespace

int component_reputation_cost(const ComponentDefinition& component) {
 return std::max(1, (component.cost + 1) / 2);
}

CampaignProgress CampaignProgress::load(const std::filesystem::path& path, const ContentCatalog& catalog) {
 CampaignProgress progress;
 progress.inventory_ = catalog.campaign().starting_inventory;
 if (path.empty() || !std::filesystem::exists(path))
  return progress;

 std::ifstream input(path);
 if (!input)
  throw ContentError("cannot open progress file: " + path.string());
 json value;
 try {
  input >> value;
 } catch (const json::exception& error) {
  throw ContentError(path.string() + ": invalid progress JSON: " + error.what());
 }
 if (value.value("schema_version", 0) != 2) {
  throw ContentError(path.string() + ": unsupported progress schema_version");
 }

 for (const auto& id : value.value("completed_levels", std::vector<std::string>{})) {
  if (known_level(catalog, id) &&
      std::find(progress.completed_levels_.begin(), progress.completed_levels_.end(), id) ==
        progress.completed_levels_.end()) {
   progress.completed_levels_.push_back(id);
  }
 }
 progress.inventory_ = catalog.campaign().starting_inventory;
 for (const auto& id : value.value("inventory", std::vector<std::string>{})) {
  if (catalog.components().contains(id) &&
      std::find(progress.inventory_.begin(), progress.inventory_.end(), id) == progress.inventory_.end()) {
   progress.inventory_.push_back(id);
  }
 }
 for (const auto& id : value.value("abilities", std::vector<std::string>{})) {
  if (catalog.abilities().contains(id) &&
      std::find(progress.abilities_.begin(), progress.abilities_.end(), id) == progress.abilities_.end()) {
   progress.abilities_.push_back(id);
  }
 }
 // Additive ledger: old saves without these keys load at zero/empty.
 progress.reputation_ = std::max(0, value.value("reputation", 0));
 for (const auto& id : value.value("claimed_contracts", std::vector<std::string>{})) {
  if (!id.empty() && std::find(progress.claimed_contracts_.begin(), progress.claimed_contracts_.end(),
                               id) == progress.claimed_contracts_.end()) {
   progress.claimed_contracts_.push_back(id);
  }
 }
 if (const auto region = value.find("regional_world"); region != value.end() && region->is_object()) {
 const auto incident = region->value("incident", json::object());
 WorldActivityCheckpoint checkpoint;
 checkpoint.state.local_time_hours = region->value("local_time_hours", 7.5);
 checkpoint.state.daylight = region->value("daylight", 0.35);
 checkpoint.state.weather = parse_weather(region->value("weather", std::string("clear")));
 checkpoint.state.wetness = region->value("wetness", 0.0);
 checkpoint.state.visibility = region->value("visibility", 1.0);
 checkpoint.state.wind_mps = region->value("wind_mps", 0.0);
 checkpoint.state.weather_revision = region->value("weather_revision", 0);
 checkpoint.state.incident.active = incident.value("active", false);
 checkpoint.state.incident.kind =
 parse_incident(incident.value("kind", std::string("road-closure")));
 checkpoint.state.incident.marker_id = incident.value("marker_id", std::string{});
 checkpoint.state.incident.position = {
 incident.value("x", 0.0), incident.value("y", 0.0)};
 checkpoint.state.incident.radius_m = incident.value("radius_m", 0.65);
 checkpoint.state.incident.opened_at_s = 0.0;
 checkpoint.state.incident.resolve_progress = incident.value("resolve_progress", 0.0);
 checkpoint.state.incident.completed_count = incident.value("completed_count", 0);
 checkpoint.next_weather_delay_s = region->value("next_weather_delay_s", 38.0);
 checkpoint.next_incident_delay_s = region->value("next_incident_delay_s", 18.0);
 checkpoint.issued_incidents = region->value("issued_incidents", 0);
 checkpoint.rng_state = region->value("rng_state", std::string{});
 try {
 WorldActivitySystem validator;
 validator.restore(checkpoint);
 progress.regional_world_ = std::move(checkpoint);
 } catch (const std::exception& error) {
 throw ContentError(path.string() + ": invalid regional_world: " + error.what());
 }
 }
 const auto builds = value.value("last_builds", json::object());
 for (auto it = builds.begin(); it != builds.end(); ++it) {
  if (!known_level(catalog, it.key()) || !it.value().is_object())
   continue;
  const auto& entry = it.value();
  BuildSelection build;
  for (auto field = entry.begin(); field != entry.end(); ++field) {
 if (field.key() == "style" || field.key() == "chassis" || field.key() == "task_policy") {
    continue;
   }
   build.parts[field.key()] = field.value().get<std::string>();
  }
  build.style = std::clamp(entry.value("style", 0.5), 0.0, 1.0);
  build.chassis_robot = entry.value("chassis", std::string{});
 build.task_policy = entry.value("task_policy", std::string{});
  bool known = true;
  for (const auto& [slot, id] : build.parts) {
   (void)slot;
   if (!id.empty() && !catalog.components().contains(id))
    known = false;
  }
  if (known)
   progress.last_builds_.emplace(it.key(), build);
 }
 const auto scores = value.value("best_scores", json::object());
 for (auto it = scores.begin(); it != scores.end(); ++it) {
  if (!known_level(catalog, it.key()) || !it.value().is_object())
   continue;
  progress.best_scores_.emplace(it.key(), BestScore{
                                            it.value().value("safe", false),
                                            it.value().value("fast", false),
                                            it.value().value("efficient", false),
                                            it.value().value("best_time_s", 0.0),
                                            it.value().value("best_energy_wh", 0.0),
                                          });
 }
 if (const auto expedition = value.find("field_expedition");
 expedition != value.end() && expedition->is_object()) {
 FieldExpeditionCheckpoint checkpoint;
 checkpoint.level_id = expedition->value("level_id", std::string{});
 checkpoint.field_operator.position = {
 expedition->value("operator_x", 0.0), expedition->value("operator_y", 0.0)};
 checkpoint.field_operator.home_position = {
 expedition->value("home_x", 0.0), expedition->value("home_y", 0.0)};
 checkpoint.field_operator.heading_rad = expedition->value("heading_rad", 0.0);
 if (const auto intent = expedition->find("intent"); intent != expedition->end() && intent->is_object()) {
 checkpoint.intent.active = intent->value("active", false);
 checkpoint.intent.kind = parse_field_intent(intent->value("kind", std::string("wait")));
 checkpoint.intent.target = {intent->value("target_x", 0.0), intent->value("target_y", 0.0)};
 checkpoint.intent.revision = intent->value("revision", std::uint64_t{0});
 checkpoint.intent.acknowledged_revision = intent->value("acknowledged_revision", std::uint64_t{0});
 checkpoint.intent.acknowledgement = checkpoint.intent.active
 ? FieldIntentAcknowledgement::Pending
 : FieldIntentAcknowledgement::Idle;
 }
 try {
 if (!known_level(catalog, checkpoint.level_id))
 throw std::invalid_argument("unknown expedition level");
 FieldOperatorSystem validator;
 const auto& level = catalog.level(checkpoint.level_id);
 validator.restore(checkpoint, catalog.map(level.map), checkpoint.level_id);
 progress.field_expedition_ = std::move(checkpoint);
 } catch (const std::exception& error) {
 throw ContentError(path.string() + ": invalid field_expedition: " + error.what());
 }
 }
 return progress;
}

void CampaignProgress::record_completion(const std::string& level_id, const MissionResult& result,
                                         const ContentCatalog& catalog) {
 (void)catalog.level(level_id);
 if (std::find(completed_levels_.begin(), completed_levels_.end(), level_id) == completed_levels_.end()) {
  completed_levels_.push_back(level_id);
 }
 for (const auto& reward : catalog.level(level_id).rewards) {
  if (catalog.components().contains(reward)) {
   if (std::find(inventory_.begin(), inventory_.end(), reward) == inventory_.end()) {
    inventory_.push_back(reward);
   }
  } else if (reward.rfind("ability.", 0) == 0) {
   catalog.ability(reward); // loud on unknown: abilities never vanish silently.
   if (std::find(abilities_.begin(), abilities_.end(), reward) == abilities_.end()) {
    abilities_.push_back(reward);
   }
  }
 }
 std::vector<std::string> order = catalog.campaign().levels;
 order.insert(order.end(), catalog.campaign().playgrounds.begin(), catalog.campaign().playgrounds.end());
 std::sort(completed_levels_.begin(), completed_levels_.end(), [&](const auto& left, const auto& right) {
  return std::find(order.begin(), order.end(), left) < std::find(order.begin(), order.end(), right);
 });

 auto& score = best_scores_[level_id];
 score.safe = score.safe || result.safe_score;
 score.fast = score.fast || result.fast_score;
 score.efficient = score.efficient || result.efficient_score; if (score.best_time_s <= 0.0 || result.time_s < score.best_time_s) {
  score.best_time_s = result.time_s;
 }
 if (score.best_energy_wh <= 0.0 || result.energy_wh < score.best_energy_wh) {
  score.best_energy_wh = result.energy_wh;
 }
 // Contract earn: a job on this level pays once when its axis scores.
 // Idempotent via the claim ledger (replays and re-rides never double-pay).
 for (const auto& job : catalog.contracts()) {
  if (job.level != level_id) {
   continue;
  }
  const bool axis_hit = (job.axis == "fast" && result.fast_score) ||
                        (job.axis == "efficient" && result.efficient_score) ||
                        (job.axis == "safe" && result.safe_score);
  if (axis_hit) {
   claim_contract(job.id, job.rep);
  }
 }
}

void CampaignProgress::record_build(const std::string& level_id, const BuildSelection& build) {
 last_builds_[level_id] = build;
}

bool CampaignProgress::claim_contract(const std::string& job_id, int rep) {
 if (job_id.empty() || rep <= 0) return false;
 if (std::find(claimed_contracts_.begin(), claimed_contracts_.end(), job_id) !=
     claimed_contracts_.end()) {
  return false;
 }
 claimed_contracts_.push_back(job_id);
 reputation_ += rep;
 return true;
}

bool CampaignProgress::spend_reputation(int amount) {
 if (amount <= 0 || amount > reputation_) return false;
 reputation_ -= amount;
 return true;
}

bool CampaignProgress::purchase_component(const std::string& component_id,
 const ContentCatalog& catalog) {
 if (std::find(inventory_.begin(), inventory_.end(), component_id) != inventory_.end())
 return false;
 const auto& component = catalog.component(component_id);
 if (!spend_reputation(component_reputation_cost(component)))
 return false;
 inventory_.push_back(component_id);
 return true;
}

const BuildSelection* CampaignProgress::last_build(const std::string& level_id) const {
 const auto found = last_builds_.find(level_id);
 return found == last_builds_.end() ? nullptr : &found->second;
}

void CampaignProgress::save(const std::filesystem::path& path, const ContentCatalog& catalog) const {
 if (path.empty())
  return;
 if (!path.parent_path().empty())
  std::filesystem::create_directories(path.parent_path());
 json builds = json::object();
 for (const auto& [level_id, build] : last_builds_) {
  json fields = json::object();
  for (const auto& [slot, id] : build.parts) {
   fields[slot] = id;
  }
  fields["style"] = build.style;
 if (!build.task_policy.empty()) {
 fields["task_policy"] = build.task_policy;
 }
  if (!build.chassis_robot.empty()) {
   fields["chassis"] = build.chassis_robot;
  }
  builds[level_id] = std::move(fields);
 }
 json scores = json::object();
 for (const auto& [level_id, score] : best_scores_) {
  scores[level_id] = {
    {"safe", score.safe},
    {"fast", score.fast},
    {"efficient", score.efficient},
    {"best_time_s", score.best_time_s},
    {"best_energy_wh", score.best_energy_wh},
  };
 }
 const json value = {
   {"schema_version", 2},
   {"content_version", catalog.campaign().content_version},
   {"completed_levels", completed_levels_},
   {"inventory", inventory_},
   {"abilities", abilities_},
   {"last_builds", std::move(builds)},
   {"best_scores", std::move(scores)},
   {"reputation", reputation_},
   {"claimed_contracts", claimed_contracts_},
 };
 json document = value;
 if (regional_world_) {
 const auto& checkpoint = *regional_world_;
 const auto& state = checkpoint.state;
 const auto& incident = state.incident;
 document["regional_world"] = {
 {"local_time_hours", state.local_time_hours},
 {"daylight", state.daylight},
 {"weather", weather_kind_id(state.weather)},
 {"wetness", state.wetness},
 {"visibility", state.visibility},
 {"wind_mps", state.wind_mps},
 {"weather_revision", state.weather_revision},
 {"next_weather_delay_s", checkpoint.next_weather_delay_s},
 {"next_incident_delay_s", checkpoint.next_incident_delay_s},
 {"issued_incidents", checkpoint.issued_incidents},
 {"rng_state", checkpoint.rng_state},
 {"incident", {
 {"active", incident.active},
 {"kind", world_incident_kind_id(incident.kind)},
 {"marker_id", incident.marker_id},
 {"x", incident.position.x},
 {"y", incident.position.y},
 {"radius_m", incident.radius_m},
 {"resolve_progress", incident.resolve_progress},
 {"completed_count", incident.completed_count},
 }},
 };
 }
 if (field_expedition_) {
 const auto& checkpoint = *field_expedition_;
 const auto& field_operator = checkpoint.field_operator;
 const auto& intent = checkpoint.intent;
 document["field_expedition"] = {
 {"level_id", checkpoint.level_id},
 {"operator_x", field_operator.position.x},
 {"operator_y", field_operator.position.y},
 {"home_x", field_operator.home_position.x},
 {"home_y", field_operator.home_position.y},
 {"heading_rad", field_operator.heading_rad},
 {"intent", {
 {"active", intent.active},
 {"kind", field_intent_kind_id(intent.kind)},
 {"target_x", intent.target.x},
 {"target_y", intent.target.y},
 {"revision", intent.revision},
 {"acknowledged_revision", intent.acknowledged_revision},
 }},
 };
 }

 auto temporary = path;
 temporary += ".tmp";
 {
 std::ofstream output_file(temporary, std::ios::trunc);
 if (!output_file)
   throw ContentError("cannot write progress file: " + temporary.string());
 output_file << document.dump(2) << '\n';
 }
 std::error_code error;
 std::filesystem::rename(temporary, path, error);
 if (error) {
  std::filesystem::remove(path, error);
  error.clear();
  std::filesystem::rename(temporary, path, error);
 }
 if (error) {
  throw ContentError("cannot replace progress file: " + path.string() + ": " + error.message());
 }
}

} // namespace mission
