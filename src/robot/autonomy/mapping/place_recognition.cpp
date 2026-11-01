#include "robot/autonomy/mapping/place_recognition.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include "robot/core/geometry.hpp"

namespace lost_signal::robot::autonomy {

PlaceRecognizer::PlaceRecognizer(PlaceRecognitionConfig config)
    : config_(std::move(config)) {}

void PlaceRecognizer::reset() { entries_.clear(); }

std::vector<float> PlaceRecognizer::describe(
    const std::vector<float>& ranges_m,
    const std::vector<float>& static_ranges_m,
    std::size_t* returns) const {
  if (returns != nullptr) *returns = 0;
  if (ranges_m.empty() || config_.sector_count == 0 ||
      !(config_.max_range_m > 0.0) || !std::isfinite(config_.max_range_m)) {
    return {};
  }
  std::vector<float> descriptor(config_.sector_count, 0.0f);
  std::vector<bool> occupied(config_.sector_count, false);
  const bool have_static = static_ranges_m.size() == ranges_m.size();
  std::size_t valid_returns = 0;
  for (std::size_t i = 0; i < ranges_m.size(); ++i) {
    float range = ranges_m[i];
    if (have_static) {
      const float static_range = static_ranges_m[i];
      // A zero/invalid static sample means the static pipeline rejected this
      // return (usually a dynamic object), so leave that beam unknown.
      if (!(static_range > 0.0f) || !std::isfinite(static_range)) continue;
      range = static_range;
    }
    if (!(range > 0.0f) || !std::isfinite(range) ||
        range >= static_cast<float>(config_.max_range_m)) {
      continue;
    }
    const std::size_t sector = std::min(
        config_.sector_count - 1,
        (i * config_.sector_count) / ranges_m.size());
    const float normalized = static_cast<float>(std::clamp(
        1.0 - static_cast<double>(range) / config_.max_range_m, 0.0, 1.0));
    if (!occupied[sector] || normalized > descriptor[sector]) {
      descriptor[sector] = normalized;
      occupied[sector] = true;
    }
    ++valid_returns;
  }
  if (returns != nullptr) *returns = valid_returns;
  return descriptor;
}

bool PlaceRecognizer::add(std::uint64_t keyframe_id,
                          const std::vector<float>& ranges_m,
                          const std::vector<float>& static_ranges_m) {
  if (keyframe_id == 0) return false;
  std::size_t returns = 0;
  std::vector<float> descriptor =
      describe(ranges_m, static_ranges_m, &returns);
  if (descriptor.empty() || returns < config_.minimum_returns) return false;
  entries_[keyframe_id] = Entry{std::move(descriptor), returns};
  return true;
}

void PlaceRecognizer::remove(std::uint64_t keyframe_id) {
  entries_.erase(keyframe_id);
}

PlaceRecognitionCandidate PlaceRecognizer::compare(
    const Entry& entry, std::uint64_t keyframe_id,
    const std::vector<float>& descriptor) const {
  PlaceRecognitionCandidate result;
  result.keyframe_id = keyframe_id;
  if (entry.descriptor.size() != descriptor.size() || descriptor.empty()) {
    return result;
  }
  const std::size_t count = descriptor.size();
  double best_error = std::numeric_limits<double>::infinity();
  std::size_t best_shift = 0;
  for (std::size_t shift = 0; shift < count; ++shift) {
    double error = 0.0;
    for (std::size_t i = 0; i < count; ++i) {
      error += std::abs(static_cast<double>(descriptor[i]) -
                        static_cast<double>(entry.descriptor[(i + shift) % count]));
    }
    error /= static_cast<double>(count);
    if (error < best_error ||
        (error == best_error && shift < best_shift)) {
      best_error = error;
      best_shift = shift;
    }
  }
  result.similarity = std::clamp(1.0 - best_error, 0.0, 1.0);
  result.heading_shift_rad =
      core::wrap_angle(2.0 * core::kPi * static_cast<double>(best_shift) /
                       static_cast<double>(count));
  return result;
}

std::vector<PlaceRecognitionCandidate> PlaceRecognizer::query(
    const std::vector<float>& ranges_m,
    const std::vector<float>& static_ranges_m,
    std::size_t maximum_candidates) const {
  std::size_t returns = 0;
  const std::vector<float> descriptor =
      describe(ranges_m, static_ranges_m, &returns);
  if (descriptor.empty() || returns < config_.minimum_returns) return {};
  const std::size_t limit = maximum_candidates == 0
                                ? config_.maximum_candidates
                                : maximum_candidates;
  if (limit == 0) return {};
  std::vector<PlaceRecognitionCandidate> candidates;
  candidates.reserve(entries_.size());
  for (const auto& [keyframe_id, entry] : entries_) {
    PlaceRecognitionCandidate candidate =
        compare(entry, keyframe_id, descriptor);
    if (candidate.similarity >= config_.minimum_similarity) {
      candidates.push_back(candidate);
    }
  }
  std::sort(candidates.begin(), candidates.end(),
            [](const PlaceRecognitionCandidate& lhs,
               const PlaceRecognitionCandidate& rhs) {
              if (lhs.similarity != rhs.similarity) {
                return lhs.similarity > rhs.similarity;
              }
              return lhs.keyframe_id < rhs.keyframe_id;
            });
  if (candidates.size() > limit) candidates.resize(limit);
  return candidates;
}

}  // namespace lost_signal::robot::autonomy
