#include "sim/providers/replay.hpp"

#include <fstream>
#include <sstream>

#include "sim/eval/json.hpp"

namespace lost_signal::sim::providers {
namespace {

bool write_all(const std::string& path, const std::string& content,
               std::string* error) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    if (error) *error = "cannot open file for writing: " + path;
    return false;
  }
  out.write(content.data(), static_cast<std::streamsize>(content.size()));
  out.close();
  if (!out) {
    if (error) *error = "failed writing file: " + path;
    return false;
  }
  return true;
}

}  // namespace

bool export_episode(const std::string& path, const std::string& map_id,
                    std::uint64_t map_revision,
                    const eval::EpisodeRecorder& episode, std::string* error) {
  eval::Json::Object header;
  header.emplace_back("v", eval::Json::Num(2));
  header.emplace_back("kind", eval::Json::Str("episode"));
  header.emplace_back("map", eval::Json::Str(map_id));
  header.emplace_back("rev", eval::Json::Num(static_cast<double>(map_revision)));
  std::string content = eval::json_dump(eval::Json::Obj(std::move(header)));
  content += "\n";
  for (const auto& step : episode.steps()) {
    content +=
        eval::step_to_json_line(eval::record_step(step.obs)) + "\n";
  }
  return write_all(path, content, error);
}

bool ReplayProvider::load(const std::string& path, std::string* error) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    if (error) *error = "cannot open file: " + path;
    return false;
  }
  std::string line;
  if (!std::getline(in, line)) {
    if (error) *error = "empty episode file: " + path;
    return false;
  }
  std::string header_error;
  auto header = eval::json_parse(line, &header_error);
  if (header == nullptr || !header->is_object()) {
    if (error) *error = "invalid header: " + header_error;
    return false;
  }
  const eval::Json* kind = header->find("kind");
  const eval::Json* map = header->find("map");
  const eval::Json* rev = header->find("rev");
  const eval::Json* v = header->find("v");
  if (kind == nullptr || !kind->is_str() || kind->as_str() != "episode" ||
      map == nullptr || !map->is_str() || rev == nullptr || !rev->is_num() ||
      v == nullptr || !v->is_num() || v->as_num() != 2.0) {
    if (error) *error = "unsupported episode header";
    return false;
  }
  map_id_ = map->as_str();
  map_revision_ = static_cast<std::uint64_t>(rev->as_num());
  steps_.clear();
  cursor_ = 0;
  std::size_t line_no = 1;
  while (std::getline(in, line)) {
    ++line_no;
    if (line.empty()) continue;
    eval::RecordedStep step;
    std::string step_error;
    if (!eval::step_from_json_line(line, step, &step_error)) {
      if (error) {
        *error = "line " + std::to_string(line_no) + ": " + step_error;
      }
      steps_.clear();
      cursor_ = 0;
      return false;
    }
    steps_.push_back(std::move(step));
  }
  if (steps_.empty()) {
    if (error) *error = "episode has no steps";
    return false;
  }
  return true;
}

bool ReplayProvider::poll(robot::iface::ObservationFrame& out) {
  if (cursor_ >= steps_.size()) return false;
  out = eval::step_to_observation(steps_[cursor_++]);
  return robot::iface::valid_observation_frame(out);
}

}  // namespace lost_signal::sim::providers
