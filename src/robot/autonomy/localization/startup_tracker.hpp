#pragma once

// Deferred startup disambiguation (ROBOT-side belief over measurements +
// survey prior, never SIM truth). A whole-map search that rejects as
// AMBIGUOUS still produced refined runner-ups; guessing among them would
// kidnap the estimate, retrying from scratch wastes the evidence. The
// tracker keeps the top hypotheses alive across scans instead: each gets
// a bounded local match per scan (the window absorbs inter-scan motion,
// poses follow accepted matches), losers drop on repeated rejects,
// convergences merge (same doctrine as the refine post-merge), and a
// winner promotes only on a sustained margin — or as the sole survivor.
// Timeout returns lost; the supervisor's retry cadence backstops (it
// re-runs the whole-map search). Deterministic: fixed rules, sorted
// order, no sampling.

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <vector>

#include "robot/autonomy/localization/global_search.hpp"
#include "robot/autonomy/localization/scan_matcher.hpp"
#include "robot/core/geometry.hpp"
#include "robot/core/time.hpp"
#include "robot/iface/map.hpp"

namespace lost_signal::robot::autonomy {

struct StartupTrackerConfig {
  // Promotion needs this margin (best minus second, matched scores) held
  // for this many consecutive scans (mirrors the matcher twin doctrine).
  double promotion_margin = 0.05;
  int promotion_confirms = 2;
  // A hypothesis failing the local match this often in a row is dropped
  // (one transient miss is tolerated, never two).
  int drop_after_rejects = 2;
  // Give up after this many scans without promotion (genuinely
  // unobservable: twins that never separate). Supervisor retries after.
  int max_scans = 20;
};

enum class StartupTrackStatus {
  kTracking = 0,
  kPromoted = 1,  // exactly one winner: x/y/yaw carry the pose
  kLost = 2,      // timeout or contradictory evidence: retry from scratch
};

struct StartupTrackResult {
  StartupTrackStatus status = StartupTrackStatus::kTracking;
  double x_m = 0.0;
  double y_m = 0.0;
  double yaw_rad = 0.0;
  double score = 0.0;
  std::size_t hypotheses = 0;
};

class StartupHypotheses {
 public:
  explicit StartupHypotheses(StartupTrackerConfig config = {})
      : config_(config) {}

  void reset() { hyps_.clear(); }
  bool empty() const { return hyps_.empty(); }
  std::size_t size() const { return hyps_.size(); }

  void set_lidar_mount(const core::RigidTransform& base_from_lidar) {
    matcher_.set_lidar_mount(base_from_lidar);
  }

  // Seed from an ambiguous whole-map result: the published pose plus its
  // runner-ups, best-first. Empty runner-ups (decisive single or no
  // evidence) seed nothing — the caller only tracks genuine ambiguity.
  void seed(const GlobalSearchResult& ambiguous) {
    reset();
    hyps_.push_back(
        Hyp{ambiguous.x_m, ambiguous.y_m, ambiguous.yaw_rad,
            ambiguous.score, 0, 0});
    for (const auto& ru : ambiguous.runner_ups) {
      hyps_.push_back(Hyp{ru.x_m, ru.y_m, ru.yaw_rad, ru.score, 0, 0});
    }
    if (hyps_.size() <= 1) reset();  // nothing to disambiguate
  }

  // One scan: local-match every hypothesis, drop losers, promote winner.
  // The match window (default 0.75 m / 0.35 rad) absorbs inter-scan motion
  // at decision rate; poses follow accepted matches, so the track coasts
  // through driving without an odometry input.
  StartupTrackResult update(const std::vector<float>& ranges,
                           double angle_min_rad, double angle_increment_rad,
                           const iface::MapSnapshot& map,
                           core::MonotonicNs stamp_ns) {
    StartupTrackResult out;
    if (hyps_.empty() || !map.has_raster() || ranges.empty()) {
      out.status = StartupTrackStatus::kLost;
      reset();
      return out;
    }
    for (auto& h : hyps_) {
      ++h.age;
      ScanMatchResult m = matcher_.localize(
          ranges, angle_min_rad, angle_increment_rad, h.x, h.y, h.yaw, map,
          stamp_ns);
      if (m.accepted) {
        h.x = m.x_m;
        h.y = m.y_m;
        h.yaw = m.yaw_rad;
        h.score = m.score;
        ++h.confirms;
        h.rejects = 0;
      } else {
        ++h.rejects;
        h.confirms = 0;
      }
    }
    // Convergences merge (same physical hypothesis now): keep the best,
    // confirmed count rides along so a merged winner promotes on time.
    std::sort(hyps_.begin(), hyps_.end(), [](const Hyp& a, const Hyp& b) {
      if (a.score != b.score) return a.score > b.score;
      if (a.x != b.x) return a.x < b.x;
      return a.y < b.y;
    });
    {
      std::vector<Hyp> kept;
      for (auto& h : hyps_) {
        bool merged = false;
        for (auto& k : kept) {
          if (std::hypot(k.x - h.x, k.y - h.y) < 0.25 &&
              std::abs(core::wrap_angle(k.yaw - h.yaw)) < 0.15) {
            k.confirms = std::max(k.confirms, h.confirms);
            merged = true;
            break;
          }
        }
        if (!merged) kept.push_back(h);
      }
      hyps_.swap(kept);
    }
    // Stale hypotheses drop (repeated rejects only — one miss tolerated).
    hyps_.erase(std::remove_if(hyps_.begin(), hyps_.end(),
                               [&](const Hyp& h) {
                                 return h.rejects >= config_.drop_after_rejects;
                               }),
                hyps_.end());
    out.hypotheses = hyps_.size();
    if (hyps_.empty()) {
      out.status = StartupTrackStatus::kLost;
      return out;
    }
    // Promotion: sole survivor with confirmations, or a sustained margin
    // over a live runner-up (stale seconds don't count: the runner-up
    // must have matched this round... tracked via confirms > 0).
    const Hyp& best = hyps_.front();
    if (best.confirms >= config_.promotion_confirms) {
      if (hyps_.size() == 1) {
        return promoted(best);
      }
      const Hyp& second = hyps_[1];
      if (second.confirms > 0 &&
          best.score - second.score >= config_.promotion_margin) {
        return promoted(best);
      }
    }
    if (best.age >= config_.max_scans) {
      out.status = StartupTrackStatus::kLost;
      reset();
      return out;
    }
    out.status = StartupTrackStatus::kTracking;
    return out;
  }

 private:
  struct Hyp {
    double x = 0.0, y = 0.0, yaw = 0.0, score = 0.0;
    int confirms = 0, rejects = 0, age = 0;
  };
  StartupTrackResult promoted(const Hyp& best) {
    StartupTrackResult out;
    out.status = StartupTrackStatus::kPromoted;
    out.x_m = best.x;
    out.y_m = best.y;
    out.yaw_rad = best.yaw;
    out.score = best.score;
    out.hypotheses = hyps_.size();
    reset();
    return out;
  }
  StartupTrackerConfig config_;
  ScanMatcher matcher_;
  std::vector<Hyp> hyps_;
};

}  // namespace lost_signal::robot::autonomy
