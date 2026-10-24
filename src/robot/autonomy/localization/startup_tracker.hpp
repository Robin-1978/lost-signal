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
 // A distant single-scan winner remains a hypothesis until measured wheel
 // motion supplies a useful temporal baseline. Repeated stationary copies
 // of one sparse scan add no observability.
 double promotion_translation_m = 0.30;
 double promotion_rotation_rad = 0.20;
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

 void reset() {
 hyps_.clear();
 require_motion_ = false;
 last_stamp_ns_ = core::kInvalidTime;
 accumulated_translation_m_ = 0.0;
 accumulated_rotation_rad_ = 0.0;
 }
  bool empty() const { return hyps_.empty(); }
  std::size_t size() const { return hyps_.size(); }

  void set_lidar_mount(const core::RigidTransform& base_from_lidar) {
    matcher_.set_lidar_mount(base_from_lidar);
  }

 // Seed from a whole-map result: the published pose plus runner-ups,
 // best-first. Ambiguous tracking requires runner-ups; a decisive but
 // distant candidate can be retained when motion confirmation is required.
 void seed(const GlobalSearchResult& ambiguous,
 core::MonotonicNs stamp_ns = core::kInvalidTime,
 bool require_motion = false) {
    reset();
    hyps_.push_back(
        Hyp{ambiguous.x_m, ambiguous.y_m, ambiguous.yaw_rad,
            ambiguous.score, 0, 0});
    for (const auto& ru : ambiguous.runner_ups) {
      hyps_.push_back(Hyp{ru.x_m, ru.y_m, ru.yaw_rad, ru.score, 0, 0});
    }
 require_motion_ = require_motion;
 last_stamp_ns_ = stamp_ns;
 if (hyps_.size() <= 1 && !require_motion_) reset();
  }

  // One scan: local-match every hypothesis, drop losers, promote winner.
 // Measured odometry first propagates every candidate by the same relative
 // motion, then the bounded match window corrects each prediction.
  StartupTrackResult update(const std::vector<float>& ranges,
                           double angle_min_rad, double angle_increment_rad,
                           const iface::MapSnapshot& map,
 core::MonotonicNs stamp_ns,
 double forward_mps = 0.0, double yaw_rate_radps = 0.0,
 bool odometry_valid = false) {
    StartupTrackResult out;
    if (hyps_.empty() || !map.has_raster() || ranges.empty()) {
      out.status = StartupTrackStatus::kLost;
      reset();
      return out;
    }
 double motion_dt_s = 0.0;
 if (odometry_valid && last_stamp_ns_ != core::kInvalidTime &&
 stamp_ns > last_stamp_ns_) {
 motion_dt_s = static_cast<double>(stamp_ns - last_stamp_ns_) * 1e-9;
 if (!(motion_dt_s > 0.0) || motion_dt_s > 1.0 ||
 !std::isfinite(forward_mps) || !std::isfinite(yaw_rate_radps)) {
 motion_dt_s = 0.0;
 }
 }
 if (motion_dt_s > 0.0) {
 const double distance = forward_mps * motion_dt_s;
 const double delta_yaw = yaw_rate_radps * motion_dt_s;
 accumulated_translation_m_ += std::abs(distance);
 accumulated_rotation_rad_ += std::abs(delta_yaw);
 for (auto& h : hyps_) {
 const double yaw_mid = h.yaw + 0.5 * delta_yaw;
 h.x += distance * std::cos(yaw_mid);
 h.y += distance * std::sin(yaw_mid);
 h.yaw = core::wrap_angle(h.yaw + delta_yaw);
 }
 }
 last_stamp_ns_ = stamp_ns;
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
 const bool motion_confirmed =
 !require_motion_ ||
 accumulated_translation_m_ >= config_.promotion_translation_m ||
 accumulated_rotation_rad_ >= config_.promotion_rotation_rad;
 if (best.confirms >= config_.promotion_confirms && motion_confirmed) {
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
 bool require_motion_ = false;
 core::MonotonicNs last_stamp_ns_ = core::kInvalidTime;
 double accumulated_translation_m_ = 0.0;
 double accumulated_rotation_rad_ = 0.0;
};

}  // namespace lost_signal::robot::autonomy
