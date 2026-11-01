#pragma once

#include <cstddef>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace lost_signal::robot::autonomy {

// A lightweight 2-D place descriptor used only to retrieve historical scan
// candidates. It never publishes a pose: every returned id still needs the
// bidirectional geometric matcher and loop-closure gates.
struct PlaceRecognitionConfig {
  std::size_t sector_count = 60;
  double max_range_m = 10.0;
  std::size_t minimum_returns = 8;
  std::size_t maximum_candidates = 32;
  double minimum_similarity = 0.35;
};

struct PlaceRecognitionCandidate {
  std::uint64_t keyframe_id = 0;
  double similarity = 0.0;
  // Best circular descriptor shift. This is a retrieval hint for the
  // geometric matcher, never an accepted heading measurement.
  double heading_shift_rad = 0.0;
};

class PlaceRecognizer {
 public:
  explicit PlaceRecognizer(PlaceRecognitionConfig config = {});

  void reset();
  // Static ranges are used when available so dynamic returns do not become a
  // persistent place signature. A scan with too few finite hits is ignored.
  bool add(std::uint64_t keyframe_id, const std::vector<float>& ranges_m,
           const std::vector<float>& static_ranges_m = {});
  void remove(std::uint64_t keyframe_id);

  std::vector<PlaceRecognitionCandidate> query(
      const std::vector<float>& ranges_m,
      const std::vector<float>& static_ranges_m = {},
      std::size_t maximum_candidates = 0) const;

  std::size_t size() const { return entries_.size(); }

 private:
  struct Entry {
    std::vector<float> descriptor;
    std::size_t returns = 0;
  };

  std::vector<float> describe(const std::vector<float>& ranges_m,
                              const std::vector<float>& static_ranges_m,
                              std::size_t* returns) const;
  PlaceRecognitionCandidate compare(const Entry& entry,
                                    std::uint64_t keyframe_id,
                                    const std::vector<float>& descriptor) const;

  PlaceRecognitionConfig config_;
  std::unordered_map<std::uint64_t, Entry> entries_;
};

}  // namespace lost_signal::robot::autonomy
