#include "sim/providers/replay.hpp"

#include <fstream>
#include <sstream>

#include "robot/core/json.hpp"

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
  robot::core::Json::Object header;
 header.emplace_back("v", robot::core::Json::Num(3));
  header.emplace_back("kind", robot::core::Json::Str("episode"));
  header.emplace_back("map", robot::core::Json::Str(map_id));
  header.emplace_back("rev", robot::core::Json::Num(static_cast<double>(map_revision)));
  std::string content = robot::core::json_dump(robot::core::Json::Obj(std::move(header)));
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
  auto header = robot::core::json_parse(line, &header_error);
  if (header == nullptr || !header->is_object()) {
    if (error) *error = "invalid header: " + header_error;
    return false;
  }
  const robot::core::Json* kind = header->find("kind");
  const robot::core::Json* map = header->find("map");
  const robot::core::Json* rev = header->find("rev");
  const robot::core::Json* v = header->find("v");
  if (kind == nullptr || !kind->is_str() || kind->as_str() != "episode" ||
      map == nullptr || !map->is_str() || rev == nullptr || !rev->is_num() ||
 v == nullptr || !v->is_num() ||
 (v->as_num() != 2.0 && v->as_num() != 3.0)) {
    if (error) *error = "unsupported episode header";
    return false;
  }
  map_id_ = map->as_str();
  map_revision_ = static_cast<std::uint64_t>(rev->as_num());
  steps_.clear();
 descriptors_.clear();
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
 auto frame_id = [](robot::core::Frame frame) {
 switch (frame) {
 case robot::core::Frame::kBaseLink:
 return std::string("base_link");
 case robot::core::Frame::kLidarLink:
 return std::string("lidar_link");
 case robot::core::Frame::kImuLink:
 return std::string("imu_link");
 case robot::core::Frame::kMap:
 return std::string("map");
 case robot::core::Frame::kOdom:
 return std::string("odom");
 case robot::core::Frame::kUnknown:
 return std::string("unknown");
 }
 return std::string("unknown");
 };
 auto add_descriptor = [&](const robot::iface::MeasurementHeader& sample,
 robot::iface::EndpointRole role) {
 if (!sample.valid()) return;
 for (const auto& existing : descriptors_) {
 if (existing.endpoint_id == sample.source && existing.role == role) return;
 }
 descriptors_.push_back({sample.source, role,
 robot::iface::ProviderBackend::kReplay, frame_id(sample.frame)});
 };
 for (const auto& step : steps_) {
 if (step.scan_valid) {
 add_descriptor(step.scan_header, robot::iface::EndpointRole::kLidar);
 }
 if (step.odom_valid || !step.drives_radps.empty()) {
 add_descriptor(step.motion_header, robot::iface::EndpointRole::kEncoder);
 }
 if (step.imu_valid) {
 add_descriptor(step.imu_header, robot::iface::EndpointRole::kImu);
 }
 }
  return true;
}

bool ReplayProvider::poll(robot::iface::ObservationFrame& out) {
  if (cursor_ >= steps_.size()) return false;
  out = eval::step_to_observation(steps_[cursor_++]);
  return robot::iface::valid_observation_frame(out);
}

}  // namespace lost_signal::sim::providers
