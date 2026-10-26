#pragma once

// Deferred startup disambiguation (ROBOT-side belief over measurements +
// survey prior, never SIM truth). A whole-map search that rejects as
// AMBIGUOUS still produced refined runner-ups; guessing among them would
// kidnap the estimate, retrying from scratch wastes the evidence. The
// tracker keeps the top hypotheses alive across scans instead: each gets
// a bounded local match per scan (the window absorbs inter-scan motion,
// poses follow accepted matches), losers drop on repeated rejects,
// convergences merge (same doctrine as the refine post-merge), and a
// candidate promotes only as the sole repeatedly supported survivor after
// useful measured motion. Persistent aliases remain a bounded belief instead
// of being collapsed by uncalibrated score margins or repeatedly re-searched.
// Deterministic: fixed rules, sorted order, no sampling.

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
 // A sole surviving hypothesis must remain supported across this many scans.
 // Multiple separated hypotheses are never collapsed from their uncalibrated
 // local response scores; useful motion must make the alternatives contradict
 // later measurements or converge to the same physical pose.
  int promotion_confirms = 2;
 // A distant single-scan winner remains a hypothesis until measured wheel
 // motion supplies a useful temporal baseline. Repeated stationary copies
 // of one sparse scan add no observability. Zero derives the baseline from
 // the local match window below, so one spatial doctrine owns both values.
 double promotion_translation_m = 0.0;
 double promotion_rotation_rad = 0.0;
  // A hypothesis failing the local match this often in a row is dropped
 // (short transient misses are tolerated before evidence is retired).
 int drop_after_rejects = 3;
 // A supported multi-modal belief should survive repetitive geometry rather
 // than restart the same global search. A positive value is an explicit
 // deployment/test deadline; zero keeps the bounded belief until evidence
 // resolves it, all modes contradict, the map changes, or the session resets.
 int max_scans = 0;
 // The whole-map pass already snapped every retained hypothesis. Per-scan
 // tracking only needs to absorb residual odometry error around the
 // propagated pose; a global-sized local window would repeat millions of
 // scores for each runner-up in one control tick.
 double match_translation_m = 0.30;
 double match_translation_step_m = 0.05;
 double match_heading_rad = 0.20;
 double match_heading_step_rad = 0.05;
 std::size_t match_max_returns = 64;
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
 double position_sigma_m = 0.05;
 double heading_sigma_rad = 0.02;
  double score = 0.0;
  std::size_t hypotheses = 0;
};

class StartupHypotheses {
 public:
  explicit StartupHypotheses(StartupTrackerConfig config = {})
 : config_(config), matcher_(matcher_config(config)) {}

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
 ambiguous.position_sigma_m, ambiguous.heading_sigma_rad,
 ambiguous.score, 0, 0});
    for (const auto& ru : ambiguous.runner_ups) {
 hyps_.push_back(Hyp{ru.x_m, ru.y_m, ru.yaw_rad,
 ambiguous.position_sigma_m, ambiguous.heading_sigma_rad,
 ru.score, 0, 0});
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
 const bool partial_observation =
 !m.accepted && m.partial_observation_valid &&
 m.rejection_reason == "map match is geometrically ambiguous";
 if (m.accepted || partial_observation) {
 if (partial_observation) {
 apply_partial_observation(h, m);
 } else {
        h.x = m.x_m;
        h.y = m.y_m;
        h.yaw = m.yaw_rad;
 }
 h.position_sigma_m = m.position_sigma_m;
 h.heading_sigma_rad = m.heading_sigma_rad;
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
 // Stale hypotheses drop only after the configured run of contradictions;
 // the default tolerates two transient misses.
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
 // Promotion requires one sole, repeatedly supported survivor. Local score
 // margins are deliberately excluded because they are not calibrated
 // probabilities and cannot collapse a still-supported repeated-map mode.
    const Hyp& best = hyps_.front();
 const double required_translation_m =
 config_.promotion_translation_m > 0.0
 ? config_.promotion_translation_m
 : std::max(0.0, config_.match_translation_m);
 const double required_rotation_rad =
 config_.promotion_rotation_rad > 0.0
 ? config_.promotion_rotation_rad
 : std::max(0.0, config_.match_heading_rad);
 const bool motion_confirmed =
 !require_motion_ ||
 accumulated_translation_m_ >= required_translation_m ||
 accumulated_rotation_rad_ >= required_rotation_rad;
 if (hyps_.size() == 1 &&
 best.confirms >= config_.promotion_confirms && motion_confirmed) {
 return promoted(best);
    }
 if (config_.max_scans > 0 && best.age >= config_.max_scans) {
      out.status = StartupTrackStatus::kLost;
      reset();
      return out;
    }
    out.status = StartupTrackStatus::kTracking;
    return out;
  }

 private:
 static ScanMatcherConfig matcher_config(
 const StartupTrackerConfig& config) {
 ScanMatcherConfig matcher;
 matcher.search_translation_m = config.match_translation_m;
 matcher.translation_step_m = config.match_translation_step_m;
 matcher.search_heading_rad = config.match_heading_rad;
 matcher.heading_step_rad = config.match_heading_step_rad;
 matcher.maximum_returns = config.match_max_returns;
 return matcher;
 }
  struct Hyp {
 double x = 0.0, y = 0.0, yaw = 0.0;
 double position_sigma_m = 0.05, heading_sigma_rad = 0.02;
 double score = 0.0;
    int confirms = 0, rejects = 0, age = 0;
  };
 static void apply_partial_observation(Hyp& hypothesis,
 const ScanMatchResult& match) {
 // Project the innovation onto the observed translation subspace. For a
 // straight wall the 2x2 information block has rank one, so this moves the
 // hypothesis toward the wall normal while leaving its along-wall coordinate
 // exactly under odometry propagation. Dividing by the largest eigenvalue
 // produces the orthogonal projector for that rank-one case; a weak second
 // direction remains proportionally weighted instead of becoming a complete
 // pose correction.
 const double ixx = std::max(0.0, match.pose_information[0]);
 const double ixy = 0.5 * (
 match.pose_information[1] + match.pose_information[3]);
 const double iyy = std::max(0.0, match.pose_information[4]);
 const double trace = ixx + iyy;
 const double discriminant = std::sqrt(std::max(
 0.0, (ixx - iyy) * (ixx - iyy) + 4.0 * ixy * ixy));
 const double maximum_information = 0.5 * (trace + discriminant);
 const double dx = match.x_m - hypothesis.x;
 const double dy = match.y_m - hypothesis.y;
 if (maximum_information > 1e-12 && std::isfinite(maximum_information)) {
 hypothesis.x += (ixx * dx + ixy * dy) / maximum_information;
 hypothesis.y += (ixy * dx + iyy * dy) / maximum_information;
 }
 if (match.pose_information[8] > 1e-12) {
 hypothesis.yaw = core::wrap_angle(
 hypothesis.yaw + core::wrap_angle(match.yaw_rad - hypothesis.yaw));
 }
 }
  StartupTrackResult promoted(const Hyp& best) {
    StartupTrackResult out;
    out.status = StartupTrackStatus::kPromoted;
    out.x_m = best.x;
    out.y_m = best.y;
    out.yaw_rad = best.yaw;
 out.position_sigma_m = best.position_sigma_m;
 out.heading_sigma_rad = best.heading_sigma_rad;
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
