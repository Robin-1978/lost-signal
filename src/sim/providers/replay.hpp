#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "robot/iface/motion.hpp"
#include "robot/iface/observation.hpp"
#include "sim/eval/episode.hpp"
#include "sim/eval/recording.hpp"
#include "sim/world/world.hpp"

namespace lost_signal::sim::providers {

// Closed-loop replay: publishes recorded observations verbatim and drives a
// fresh plant with whatever the ROBOT under test commands. Given a
// deterministic stack, the replayed decision sequence matches the recording
// bit-for-bit; any divergence is a nondeterminism bug, not a tolerance.
class ReplayProvider {
 public:
  // Loads a JSONL episode file (header line + step lines). Returns false
  // with a reason; never throws.
  bool load(const std::string& path, std::string* error);
  bool empty() const { return steps_.empty(); }
  std::size_t size() const { return steps_.size(); }
  std::size_t cursor() const { return cursor_; }
 const std::vector<robot::iface::EndpointDescriptor>& descriptors() const {
 return descriptors_;
 }

  // Next recorded observation, converted back to a live-equivalent frame.
  // Returns false at end of recording.
  bool poll(robot::iface::ObservationFrame& out);

  void rewind() { cursor_ = 0; }

 private:
  std::string map_id_;
  std::uint64_t map_revision_ = 0;
  std::vector<eval::RecordedStep> steps_;
 std::vector<robot::iface::EndpointDescriptor> descriptors_;
  std::size_t cursor_ = 0;
};

// Exports a recorded episode to a JSONL file: one header line, then one
// step object per line. Returns false with a reason; never throws.
bool export_episode(const std::string& path, const std::string& map_id,
                    std::uint64_t map_revision,
                    const eval::EpisodeRecorder& episode, std::string* error);

}  // namespace lost_signal::sim::providers
