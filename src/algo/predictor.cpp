#include "predictor.hpp"

#include "algo/astar.hpp"
#include "algo/math.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <set>

namespace sim {
namespace {

// Arc-fillet smoothing of a polyline corner with radius R: tangent points at
// distance t = R*tan(phi/2) from the vertex along both legs, joined by a
// subdivided arc (chords <= 0.1 m: sagitta <= 0.01/(8R) < 0.5 mm for R >= 1 m,
// stated not hidden). fit=false when any corner needs more tangent than half
// of either adjacent leg (adjacent fillets never overlap — conservative: a
// refused corner is a capability verdict upstream, never silently
// straightened) or the turn is a hairpin. Where a fillet does not fit
// (tangent longer than half either leg: grid-snap endpoint jogs, short
// detour legs), the raw vertex is kept with kappa 0 and pursuit plus the
// swept/static rechecks decide — a geometric pre-refusal there would conflate
// "corner needs swinging wide" with "corner is untraversable". fit=false
// only on degenerate input. Sharp corners never get a free pass: deviation
// is always swept and static-rechecked downstream.
struct FilletResult {
  std::vector<Vec2> points;
  // Signed curvature per point from construction (not estimated): 0 on
  // straights, +/-1/radius_m on arcs. Exact, so steer demand never drifts
  // past the cap on floating-point noise.
  std::vector<double> kappa;
  bool fits{false};
};

FilletResult smooth_fillet(const std::vector<Vec2>& route, double radius_m) {
  FilletResult out;
  std::vector<Vec2> pts;
  for (const auto& p : route) {
    if (pts.empty() || std::hypot(p.x - pts.back().x, p.y - pts.back().y) >= 1e-9) {
      pts.push_back(p);
    }
  }
  if (pts.size() < 3 || !(radius_m > 1e-9)) {
    out.points = pts;
    out.kappa.assign(pts.size(), 0.0);
    out.fits = pts.size() >= 1;
    return out;
  }
  std::vector<double> legs;
  for (std::size_t i = 0; i + 1 < pts.size(); ++i) {
    legs.push_back(std::hypot(pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y));
  }
  out.points.push_back(pts.front());
  out.kappa.push_back(0.0);
  for (std::size_t i = 1; i + 1 < pts.size(); ++i) {
    Vec2 d_in{pts[i].x - pts[i - 1].x, pts[i].y - pts[i - 1].y};
    Vec2 d_out{pts[i + 1].x - pts[i].x, pts[i + 1].y - pts[i].y};
    const double lin = std::hypot(d_in.x, d_in.y);
    const double lout = std::hypot(d_out.x, d_out.y);
    d_in = {d_in.x / lin, d_in.y / lin};
    d_out = {d_out.x / lout, d_out.y / lout};
    const double cos_phi = std::clamp(d_in.x * d_out.x + d_in.y * d_out.y, -1.0, 1.0);
    const double phi = std::acos(cos_phi);
    if (phi < 1e-6) {
      out.points.push_back(pts[i]);
      out.kappa.push_back(0.0);
      continue;
    }
    const double tangent = radius_m * std::tan(phi / 2.0);
    if (!(tangent <= 0.5 * legs[i - 1]) || !(tangent <= 0.5 * legs[i])) {
      // No fillet room: keep the raw vertex (kappa 0), let pursuit swing and
      // the rechecks judge. The vertex stays exactly where A* put it.
      out.points.push_back(pts[i]);
      out.kappa.push_back(0.0);
      continue;
    }
    const Vec2 u{-d_in.x, -d_in.y};
    const Vec2 w = d_out;
    Vec2 bis{u.x + w.x, u.y + w.y};
    const double bis_len = std::hypot(bis.x, bis.y);
    const double sin_half = std::sin(phi / 2.0);
    if (bis_len < 1e-9 || sin_half < 1e-9) {
      out.points.clear();
      out.kappa.clear();
      out.fits = false;
      return out;
    }
    bis = {bis.x / bis_len, bis.y / bis_len};
    const Vec2 center{pts[i].x + bis.x * radius_m / sin_half,
                      pts[i].y + bis.y * radius_m / sin_half};
    const Vec2 t1{pts[i].x + u.x * tangent, pts[i].y + u.y * tangent};
    const Vec2 t2{pts[i].x + w.x * tangent, pts[i].y + w.y * tangent};
    const double cross = d_in.x * d_out.y - d_in.y * d_out.x;
    const double a0 = std::atan2(t1.y - center.y, t1.x - center.x);
    double sweep = std::atan2(t2.y - center.y, t2.x - center.x) - a0;
    while (cross > 0.0 && sweep <= 0.0) {
      sweep += 2.0 * kPi;
    }
    while (cross < 0.0 && sweep >= 0.0) {
      sweep -= 2.0 * kPi;
    }
    const double arc_len = radius_m * std::abs(sweep);
    const int steps = std::max(2, static_cast<int>(std::ceil(arc_len / 0.1)));
    const double turn_sign = (cross > 0.0) ? 1.0 : -1.0;
    out.points.push_back(t1);
    out.kappa.push_back(0.0);
    for (int k = 1; k < steps; ++k) {
      const double a = a0 + sweep * static_cast<double>(k) / static_cast<double>(steps);
      out.points.push_back({center.x + radius_m * std::cos(a), center.y + radius_m * std::sin(a)});
      out.kappa.push_back(turn_sign / radius_m);
    }
    out.points.push_back(t2);
    out.kappa.push_back(turn_sign / radius_m);
  }
  out.points.push_back(pts.back());
  out.kappa.push_back(0.0);
  if (out.points.size() != out.kappa.size()) {
    out.points.clear();
    out.kappa.clear();
    out.fits = false;
    return out;
  }
  out.fits = true;
  return out;
}

// Static recheck of an executed trajectory on the caller-inflated grid.
// The A* contract covers the grid polyline; smoothing and servo lag move the
// body off it, so the CHECKED samples are rechecked here, not inherited.
bool path_hits_obstacle(const std::vector<Vec2>& points, const GridMap& inflated_map) {
  for (const auto& p : points) {
    if (!inflated_map.isFree(p.x, p.y)) {
      return true;
    }
  }
  return false;
}

bool traj_hits_obstacle(const std::vector<TimedPose>& traj, const GridMap& inflated_map) {
  for (const auto& s : traj) {
    if (!inflated_map.isFree(s.p.x, s.p.y)) {
      return true;
    }
  }
  return false;
}

} // namespace

namespace {

// Reflection inside the observed corridor. Unbounded axes (no direction flip
// seen yet) extrapolate linearly; bounded axes bounce off the observed
// min/max. The 0.05 m corridor floor and 8-reflection cap are method
// constants: narrow corridors carry no direction information, and the cap
// bounds work for pathological velocity/corridor ratios.
double reflected_position(double position, double velocity, double time_s, double minimum, double maximum,
                          bool bounded) {
 if (!bounded || maximum - minimum < 0.05)
  return position + velocity * time_s;
 double predicted = position + velocity * time_s;
 for (int reflection = 0; reflection < 8; ++reflection) {
  if (predicted < minimum) {
   predicted = minimum + (minimum - predicted);
  } else if (predicted > maximum) {
   predicted = maximum - (predicted - maximum);
  } else {
   break;
  }
 }
 return std::clamp(predicted, minimum, maximum);
}

// Point-to-segment distance: the workhorse. interval_min_dist below reduces
// the time-parameterized closest approach to this exact call.
double point_seg_dist(Vec2 p, Vec2 s0, Vec2 s1) {
 const double dx = s1.x - s0.x;
 const double dy = s1.y - s0.y;
 const double len2 = dx * dx + dy * dy;
 constexpr double kEps = 1e-12;
 if (len2 < kEps) {
  return std::hypot(p.x - s0.x, p.y - s0.y);
 }
 const double t = std::clamp(((p.x - s0.x) * dx + (p.y - s0.y) * dy) / len2, 0.0, 1.0);
 return std::hypot(p.x - (s0.x + dx * t), p.y - (s0.y + dy * t));
}

// Time-parameterized closest approach over one frame interval: both bodies
// move with the SAME time parameter (relative motion r + w*t), so this is a
// point-to-segment distance, not a segment-segment one. An independent-(s,t)
// segment distance would flag two bodies using the same space at different
// times as a collision — exactly what waiting separates. Reviewer gate:
// discrete frame sampling alone can miss a fast crosser slipping between
// frames; this sweeps every interval exactly (linear motion per interval is
// exact for our predictors) and is conservative nowhere.
double interval_min_dist(Vec2 a0, Vec2 a1, Vec2 b0, Vec2 b1) {
 const Vec2 r{a0.x - b0.x, a0.y - b0.y};
 const Vec2 w{(a1.x - a0.x) - (b1.x - b0.x), (a1.y - a0.y) - (b1.y - b0.y)};
 return point_seg_dist({0.0, 0.0}, r, {r.x + w.x, r.y + w.y});
}

} // namespace

std::vector<std::vector<DynState>> predict_frames(const std::vector<PredictInput>& inputs, double time_s,
                                                  double horizon_s, double frame_dt_s, double max_age_s) {
 std::vector<std::vector<DynState>> frames;
 for (double future_s = 0.0; future_s <= horizon_s + 1e-9; future_s += frame_dt_s) {
  auto& frame = frames.emplace_back();
  for (const auto& input : inputs) {
   const double age = time_s - input.last_observed_s;
   if (age < -1e-9 || age > max_age_s)
    continue;
   auto predicted = input.state;
   const double prediction_time = age + future_s;
   predicted.p.x = reflected_position(predicted.p.x, predicted.v.x, prediction_time, input.min_observed.x,
                                      input.max_observed.x, input.bounded_x);
   predicted.p.y = reflected_position(predicted.p.y, predicted.v.y, prediction_time, input.min_observed.y,
                                      input.max_observed.y, input.bounded_y);
   frame.push_back(predicted);
  }
 }
 return frames;
}

double timed_gap(const std::vector<TimedPose>& ego,
                 const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                 double ego_radius_m) {
 if (ego.empty() || frames.empty()) {
  return std::numeric_limits<double>::infinity();
 }
 const double dt = std::max(1e-6, frame_dt_s);
 auto ego_at = [&](double t) {
  if (t <= ego.front().t_s) {
   return ego.front().p;
  }
  for (std::size_t i = 0; i + 1 < ego.size(); ++i) {
   if (t <= ego[i + 1].t_s) {
    const double span = ego[i + 1].t_s - ego[i].t_s;
    const double a = span < 1e-9 ? 0.0 : std::clamp((t - ego[i].t_s) / span, 0.0, 1.0);
    return Vec2{ego[i].p.x + (ego[i + 1].p.x - ego[i].p.x) * a,
                ego[i].p.y + (ego[i + 1].p.y - ego[i].p.y) * a};
   }
  }
  return ego.back().p;
 };
 double gap = std::numeric_limits<double>::infinity();
 if (frames.size() == 1) {
  for (const auto& threat : frames.front()) {
   const Vec2 e = ego_at(0.0);
   gap = std::min(gap, std::hypot(threat.p.x - e.x, threat.p.y - e.y) - threat.radius -
                           ego_radius_m);
  }
  return gap;
 }
 for (std::size_t k = 0; k + 1 < frames.size(); ++k) {
  const Vec2 ego0 = ego_at(static_cast<double>(k) * dt);
  const Vec2 ego1 = ego_at(static_cast<double>(k + 1) * dt);
  const auto& fa = frames[k];
  const auto& fb = frames[k + 1];
  for (std::size_t i = 0; i < fa.size() && i < fb.size(); ++i) {
   gap = std::min(gap, interval_min_dist(ego0, ego1, fa[i].p, fb[i].p) - fa[i].radius - ego_radius_m);
  }
 }
 return gap;
}

double predicted_gap(const std::vector<Vec2>& path, std::size_t start_idx, double speed_mps,
                     const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                     double ego_radius_m, double depart_delay_s) {
 if (path.empty() || frames.empty()) {
  return std::numeric_limits<double>::infinity();
 }
 // Ego distance at frame k (absolute time k*dt), departing after the delay.
// Frame 0 is now: ego sits at start. Fixed from an off-by-one ((k+1)*dt)
// that compared the threat-now against the ego-in-one-step.
 const double dt = std::max(1e-6, frame_dt_s);
 const double speed = std::max(0.0, speed_mps);
 const double delay = std::max(0.0, depart_delay_s);
 auto ego_at = [&](double distance) {
  double walked = 0.0;
  Vec2 ego = path[std::min(start_idx, path.size() - 1)];
  for (std::size_t seg = start_idx; seg + 1 < path.size(); ++seg) {
   const double leg = std::hypot(path[seg + 1].x - path[seg].x, path[seg + 1].y - path[seg].y);
   if (leg < 1e-9) {
    continue;
   }
   if (walked + leg >= distance) {
    const double t = std::clamp((distance - walked) / leg, 0.0, 1.0);
    return Vec2{path[seg].x + (path[seg + 1].x - path[seg].x) * t,
                path[seg].y + (path[seg + 1].y - path[seg].y) * t};
   }
   walked += leg;
   ego = path[seg + 1];
  }
  return ego;
 };
 const auto ego_frame = [&](std::size_t k) {
  return ego_at(speed * std::max(0.0, static_cast<double>(k) * dt - delay));
 };
 double gap = std::numeric_limits<double>::infinity();
 if (frames.size() == 1) {
  // Degenerate single frame: point check (segment helper covers it, but be
  // explicit so the intent reads).
  for (const auto& threat : frames.front()) {
   gap = std::min(gap, std::hypot(threat.p.x - ego_frame(0).x, threat.p.y - ego_frame(0).y) -
                           threat.radius - ego_radius_m);
  }
  return gap;
 }
 // Swept intervals: both ego and threat move linearly between frames, so the
 // closed-form segment distance cannot miss an inter-frame crossing.
 for (std::size_t k = 0; k + 1 < frames.size(); ++k) {
  const Vec2 ego0 = ego_frame(k);
  const Vec2 ego1 = ego_frame(k + 1);
  const auto& fa = frames[k];
  const auto& fb = frames[k + 1];
  for (std::size_t i = 0; i < fa.size() && i < fb.size(); ++i) {
   gap = std::min(gap, interval_min_dist(ego0, ego1, fa[i].p, fb[i].p) - fa[i].radius - ego_radius_m);
  }
 }
 return gap;
}

double earliest_departure(const std::vector<Vec2>& path, std::size_t start_idx, double cruise_mps,
                           const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                           double ego_radius_m, double margin_m) {
 return assess_departure(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, margin_m)
   .wait_s;
}

const char* depart_reason_name(DepartReason reason) {
 switch (reason) {
  case DepartReason::kGoNow:
   return "GoNow";
  case DepartReason::kWait:
   return "Wait";
  case DepartReason::kBlockedInWindow:
   return "BlockedInWindow";
  case DepartReason::kWaitUnsafe:
   return "WaitUnsafe";
  case DepartReason::kHorizonShort:
   return "HorizonShort";
  case DepartReason::kInvalid:
   return "Invalid";
  case DepartReason::kUnassessed:
   return "Unassessed";
  case DepartReason::kClear:
   return "Clear";
  case DepartReason::kStaticBlocked:
   return "StaticBlocked";
  case DepartReason::kCapsViolated:
   return "CapsViolated";
 }
 return "Invalid";
}

Feasibility check_candidate(LocalCandidate type, const std::vector<Vec2>& path, std::size_t start_idx,
                            double cruise_mps, const std::vector<std::vector<DynState>>& frames,
                            double frame_dt_s, double ego_radius_m, double margin_m) {
 if (type == LocalCandidate::kWaitAtPocket || type == LocalCandidate::kRejoin ||
     type == LocalCandidate::kDetour) {
  // No spatial planner exists yet: Unknown with its own reason, never
  // borrowed. Phase-2 implements these without touching this contract.
  return {Feasibility::Verdict::kUnknown, DepartReason::kUnassessed,
          std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
 }
 const DepartureAssessment assessed =
   assess_departure(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, margin_m);
 if (type == LocalCandidate::kProceed) {
  // Proceed means now: any required wait refuses it, but the refusal keeps
  // its cause (too-early vs blocked vs undecidable).
  if (assessed.reason == DepartReason::kGoNow) {
   return {Feasibility::Verdict::kFeasible, DepartReason::kGoNow, 0.0,
           predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, 0.0)};
  }
  if (assessed.reason == DepartReason::kHorizonShort || assessed.reason == DepartReason::kInvalid) {
   return {Feasibility::Verdict::kUnknown, assessed.reason,
           std::numeric_limits<double>::infinity(),
           predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, 0.0)};
  }
  return {Feasibility::Verdict::kInfeasible, assessed.reason,
          std::numeric_limits<double>::infinity(),
          predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, 0.0)};
 }
 // kWaitInPlace: feasible exactly when the time assessment names a departure
 // (now or later); the waiting spot was already proven inside assess.
 if (assessed.reason == DepartReason::kGoNow || assessed.reason == DepartReason::kWait) {
  const double delay = assessed.reason == DepartReason::kGoNow ? 0.0 : assessed.wait_s;
  return {Feasibility::Verdict::kFeasible, assessed.reason, assessed.wait_s,
          predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, delay)};
 }
 if (assessed.reason == DepartReason::kHorizonShort || assessed.reason == DepartReason::kInvalid) {
  return {Feasibility::Verdict::kUnknown, assessed.reason,
          std::numeric_limits<double>::infinity(),
          predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, 0.0)};
 }
 return {Feasibility::Verdict::kInfeasible, assessed.reason,
         std::numeric_limits<double>::infinity(),
         predicted_gap(path, start_idx, cruise_mps, frames, frame_dt_s, ego_radius_m, 0.0)};
}

namespace {

// Remaining polyline length from start_idx (chaining schedule, not proof).
double remaining_length(const std::vector<Vec2>& path, std::size_t start_idx) {
 double length = 0.0;
 for (std::size_t seg = start_idx; seg + 1 < path.size(); ++seg) {
  length += std::hypot(path[seg + 1].x - path[seg].x, path[seg + 1].y - path[seg].y);
 }
 return length;
}

} // namespace

// Turn-drive-turn execution of a geometric polyline: rotate in place at
// yaw_rate, drive legs at cruise. The checked trajectory is exactly this,
// never the raw grid polyline by proxy (smoothing, when the Ackermann slice
// needs it, will likewise rebuild this trajectory before any check).
std::vector<TimedPose> execute_turn_drive_turn(const std::vector<Vec2>& route, double start_theta,
                                               double cruise_mps, double yaw_rate_rads) {
 auto wrap = [](double angle) { return std::atan2(std::sin(angle), std::cos(angle)); };
 std::vector<TimedPose> exec{{0.0, route.front()}};
 double time_s = 0.0;
 double heading = start_theta;
 for (std::size_t i = 0; i + 1 < route.size(); ++i) {
  const double leg = std::hypot(route[i + 1].x - route[i].x, route[i + 1].y - route[i].y);
  if (leg < 1e-9) {
   continue;
  }
  const double desired = std::atan2(route[i + 1].y - route[i].y, route[i + 1].x - route[i].x);
  time_s += std::abs(wrap(desired - heading)) / std::max(1e-6, yaw_rate_rads);
  exec.push_back({time_s, route[i]});
  heading = desired;
  time_s += leg / std::max(1e-6, cruise_mps);
  exec.push_back({time_s, route[i + 1]});
 }
 return exec;
}

// Ackermann execution: follow the smoothed (filleted) path at cruise through
// a servo-lag bicycle sim. Curvature feedforward comes from EXACT construction
// curvature (kappa parallel to the fillet points), plus identity pursuit of the
// path tangent (gain 1 = point wheels where the path goes, capped by authority;
// only the feedforward part can trip the caps verdict). Servo mirrors
// production structure (ackermann.cpp): P loop on angle error scaled by rate,
// first-order lag on the rate state, clamps on both.
// Lag/rate shape the CHECKED trajectory — a slow servo cuts corners wide and
// the swept + static rechecks see it. Instant-cruise like the spin branch
// (same approximation level, documented); the 0.5 m/s steer-authority floor
// never binds because execution never drives slow.
struct AckermannExec {
  std::vector<TimedPose> traj;
  double max_delta{0.0};
  bool demand_clipped{false};
};

AckermannExec execute_ackermann(const std::vector<Vec2>& points, const std::vector<double>& kappa,
                                double start_theta, double cruise_mps, const ChassisExecCaps& caps) {
  AckermannExec out;
  constexpr double kSimDtS = 0.02;  // servo-lag resolution (matches physics step; stated).
  if (points.empty() || points.size() != kappa.size() || !(cruise_mps > 0.0)) {
    return out;
  }
  std::vector<double> arc(points.size(), 0.0);
  for (std::size_t i = 1; i < points.size(); ++i) {
    arc[i] = arc[i - 1] + std::hypot(points[i].x - points[i - 1].x, points[i].y - points[i - 1].y);
  }
  const double total = arc.back();
  // Per-sample tangent (segment direction; last sample reuses its segment).
  // Demand = curvature feedforward + identity pursuit of the tangent, capped
  // by steer authority. Gain 1 is the identity map (point wheels where the
  // path goes), not a tuned gain; only the feedforward part can trip the
  // caps verdict below. Lag/deviation are CHECKED downstream either way.
  std::vector<double> tangent(points.size(), 0.0);
  for (std::size_t i = 0; i + 1 < points.size(); ++i) {
    tangent[i] = std::atan2(points[i + 1].y - points[i].y, points[i + 1].x - points[i].x);
  }
  tangent.back() = points.size() > 1 ? tangent[points.size() - 2] : start_theta;
  const double kp = 1.0 / std::max(0.05, caps.servo_response_s * caps.max_steer_rate_radps);
  double theta = start_theta;
  double delta = 0.0;
  double rate = 0.0;
  double t = 0.0;
  Vec2 pos = points.front();
  out.traj.push_back({0.0, pos});
  std::size_t seg = 0;
  while (t * cruise_mps < total) {
    const double s = t * cruise_mps;
    while (seg + 1 < arc.size() && arc[seg + 1] < s) {
      ++seg;
    }
    const std::size_t k0 = std::min(seg, kappa.size() - 1);
    const double feed = std::atan(caps.wheel_base_m * kappa[k0]);
    if (std::abs(feed) > caps.max_steer_rad + 1e-9) {
      out.demand_clipped = true;
      out.traj.clear();
      return out;
    }
    const double demand =
      std::clamp(feed + wrap_angle(tangent[k0] - theta), -caps.max_steer_rad, caps.max_steer_rad);
    const double pwm = std::clamp(kp * (demand - delta), -1.0, 1.0);
    const double rate_demand =
      std::clamp(pwm * caps.max_steer_rate_radps, -caps.max_steer_rate_radps, caps.max_steer_rate_radps);
    const double lag = caps.servo_response_s > 0.0 ? 1.0 - std::exp(-kSimDtS / caps.servo_response_s) : 1.0;
    rate += (rate_demand - rate) * lag;
    delta = std::clamp(delta + rate * kSimDtS, -caps.max_steer_rad, caps.max_steer_rad);
    out.max_delta = std::max(out.max_delta, std::abs(delta));
    theta += cruise_mps * std::tan(delta) / std::max(0.05, caps.wheel_base_m) * kSimDtS;
    pos = {pos.x + cruise_mps * std::cos(theta) * kSimDtS,
           pos.y + cruise_mps * std::sin(theta) * kSimDtS};
    t += kSimDtS;
    out.traj.push_back({t, pos});
  }
  return out;
}

LegAssessment assess_transit(const GridMap& inflated_map, Vec2 start, double start_theta, Vec2 pocket,
                             double cruise_mps, const ChassisExecCaps& exec,
                             const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                             double ego_radius_m, double margin_m) {
 const LegAssessment invalid{Feasibility::Verdict::kUnknown, DepartReason::kInvalid,
                             std::numeric_limits<double>::infinity(),
                             std::numeric_limits<double>::infinity(), {}};
 if (frames.empty() || cruise_mps <= 0.0 || (exec.can_spin && exec.yaw_rate_rads <= 0.0)) {
  return invalid;
 }
 const double dt = std::max(1e-6, frame_dt_s);
 const double horizon =
   static_cast<double>(frames.size() > 0 ? frames.size() - 1 : 0) * dt;
 // Geometric path first (A* on the caller-inflated grid); no path is proven
 // static infeasibility, not traffic, not unknown.
 std::vector<Vec2> route;
 if (std::hypot(pocket.x - start.x, pocket.y - start.y) < 1e-9) {
  route = {start};
 } else {
  const auto plan = astar_plan(inflated_map, start, pocket, 1.0);
  if (!plan.ok || plan.path.empty()) {
   return {Feasibility::Verdict::kInfeasible, DepartReason::kStaticBlocked,
           std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), {}};
  }
  route.push_back(start);
  route.insert(route.end(), plan.path.begin(), plan.path.end());
  route.push_back(pocket);
 }
 if (!exec.can_spin) {
  // Ackermann execution: fillet at Rmin, static-recheck the smoothed path,
  // servo-lag bicycle follow, static-recheck + sweep the SIMULATED path.
  if (!(exec.wheel_base_m > 0.0) || !(exec.max_steer_rad > 0.0) ||
      !(exec.max_steer_rate_radps > 0.0) || !(exec.servo_response_s >= 0.0)) {
   double nominal = 0.0;
   for (std::size_t i = 0; i + 1 < route.size(); ++i) {
    nominal += std::hypot(route[i + 1].x - route[i].x, route[i + 1].y - route[i].y);
   }
   return {Feasibility::Verdict::kUnknown, DepartReason::kUnassessed,
           nominal / std::max(1e-6, cruise_mps), std::numeric_limits<double>::infinity(), {}};
  }
  const double r_min = exec.wheel_base_m / std::tan(exec.max_steer_rad);
  const auto smooth = smooth_fillet(route, r_min);
  if (!smooth.fits) {
   return {Feasibility::Verdict::kInfeasible, DepartReason::kCapsViolated,
           std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), {}};
  }
  if (path_hits_obstacle(smooth.points, inflated_map)) {
   return {Feasibility::Verdict::kInfeasible, DepartReason::kCapsViolated,
           std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), {}};
  }
  const auto ack = execute_ackermann(smooth.points, smooth.kappa, start_theta, cruise_mps, exec);
  if (ack.demand_clipped || ack.traj.empty()) {
   return {Feasibility::Verdict::kInfeasible, DepartReason::kCapsViolated,
           std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), {}};
  }
  if (traj_hits_obstacle(ack.traj, inflated_map)) {
   return {Feasibility::Verdict::kInfeasible, DepartReason::kCapsViolated,
           std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
           ack.traj};
  }
  const double ack_end = ack.traj.back().t_s;
  if (ack_end > horizon + 1e-9) {
   return {Feasibility::Verdict::kUnknown, DepartReason::kHorizonShort,
           std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), ack.traj};
  }
  const double ack_gap = timed_gap(ack.traj, frames, dt, ego_radius_m);
  if (ack_gap >= margin_m) {
   return {Feasibility::Verdict::kFeasible, DepartReason::kClear, ack_end, ack_gap, ack.traj};
  }
  const double ack_spot = timed_gap({{0.0, start}, {horizon, start}}, frames, dt, ego_radius_m);
  if (ack_spot < margin_m) {
   return {Feasibility::Verdict::kInfeasible, DepartReason::kWaitUnsafe, ack_end, ack_gap, ack.traj};
  }
  return {Feasibility::Verdict::kInfeasible, DepartReason::kBlockedInWindow, ack_end, ack_gap,
          ack.traj};
 }
 const auto spin_exec = execute_turn_drive_turn(route, start_theta, cruise_mps, exec.yaw_rate_rads);
 const double end_s = spin_exec.back().t_s;
 if (end_s > horizon + 1e-9) {
  return {Feasibility::Verdict::kUnknown, DepartReason::kHorizonShort,
          std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(), spin_exec};
 }
 const double gap = timed_gap(spin_exec, frames, dt, ego_radius_m);
 if (gap >= margin_m) {
  return {Feasibility::Verdict::kFeasible, DepartReason::kClear, end_s, gap, spin_exec};
 }
 const double spot_gap = timed_gap({{0.0, start}, {horizon, start}}, frames, dt, ego_radius_m);
 if (spot_gap < margin_m) {
  return {Feasibility::Verdict::kInfeasible, DepartReason::kWaitUnsafe, end_s, gap, spin_exec};
 }
 return {Feasibility::Verdict::kInfeasible, DepartReason::kBlockedInWindow, end_s, gap, spin_exec};
}

LegAssessment assess_hold(Vec2 spot, double hold_from_s, double hold_until_s,
                          const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                          double ego_radius_m, double margin_m) {
 if (frames.empty() || hold_from_s < 0.0 || hold_until_s < hold_from_s) {
  return {Feasibility::Verdict::kUnknown, DepartReason::kInvalid,
          std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
 }
 const double dt = std::max(1e-6, frame_dt_s);
 const double horizon =
   static_cast<double>(frames.size() > 0 ? frames.size() - 1 : 0) * dt;
 if (hold_until_s > horizon + 1e-9) {
  return {Feasibility::Verdict::kUnknown, DepartReason::kHorizonShort,
          std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
 }
 const double gap = timed_gap({{hold_from_s, spot}, {hold_until_s, spot}}, frames, dt, ego_radius_m);
 if (gap >= margin_m) {
  return {Feasibility::Verdict::kFeasible, DepartReason::kClear, hold_until_s, gap};
 }
 return {Feasibility::Verdict::kInfeasible, DepartReason::kWaitUnsafe, hold_until_s, gap};
}

LegAssessment assess_rejoin(const std::vector<Vec2>& path, std::size_t start_idx, double cruise_mps,
                            double depart_delay_s,
                            const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                            double ego_radius_m, double margin_m) {
 if (path.empty() || frames.empty() || cruise_mps <= 0.0 || depart_delay_s < 0.0) {
  return {Feasibility::Verdict::kUnknown, DepartReason::kInvalid,
          std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
 }
 const double dt = std::max(1e-6, frame_dt_s);
 const double horizon =
   static_cast<double>(frames.size() > 0 ? frames.size() - 1 : 0) * dt;
 const double length = remaining_length(path, start_idx);
 if (depart_delay_s + length / std::max(1e-6, cruise_mps) > horizon + 1e-9) {
  return {Feasibility::Verdict::kUnknown, DepartReason::kHorizonShort,
          std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity()};
 }
 const double gap =
   predicted_gap(path, start_idx, cruise_mps, frames, dt, ego_radius_m, depart_delay_s);
 if (gap >= margin_m) {
  return {Feasibility::Verdict::kFeasible, DepartReason::kClear,
          depart_delay_s + length / std::max(1e-6, cruise_mps), gap};
 }
 // Hold-prefix classification: a violation while still parked at the start
 // means the wait itself kills (WaitUnsafe); otherwise the corridor downstream
 // is what blocks (BlockedInWindow). The hold interval only, not the whole
 // window past departure.
 const Vec2 start = path[std::min(start_idx, path.size() - 1)];
 const double hold_gap =
   timed_gap({{0.0, start}, {depart_delay_s, start}}, frames, dt, ego_radius_m);
 if (hold_gap < margin_m) {
  return {Feasibility::Verdict::kInfeasible, DepartReason::kWaitUnsafe,
          std::numeric_limits<double>::infinity(), gap};
 }
 return {Feasibility::Verdict::kInfeasible, DepartReason::kBlockedInWindow,
         std::numeric_limits<double>::infinity(), gap};
}

PocketAssessment assess_pocket_wait(const GridMap& inflated_map, Vec2 start, double start_theta,
                                    Vec2 pocket, Vec2 merge,
                                    const std::vector<Vec2>& route_suffix, double cruise_mps,
                                    const ChassisExecCaps& exec, double hold_s,
                                    const std::vector<std::vector<DynState>>& frames,
                                    double frame_dt_s, double ego_radius_m, double margin_m) {
 PocketAssessment out;
 out.transit = assess_transit(inflated_map, start, start_theta, pocket, cruise_mps, exec,
                              frames, frame_dt_s, ego_radius_m, margin_m);
 // Rejoin geometry: A* pocket to merge, then the given route suffix (whose
 // first point the caller aligns with merge). A failed rejoin plan is proven
 // static infeasibility of that leg.
 std::vector<Vec2> rejoin_pts;
 bool rejoin_planned = false;
 if (std::hypot(merge.x - pocket.x, merge.y - pocket.y) < 1e-9) {
  rejoin_pts = {pocket};
  rejoin_pts.insert(rejoin_pts.end(), route_suffix.begin(), route_suffix.end());
  rejoin_planned = true;
 } else {
  const auto plan = astar_plan(inflated_map, pocket, merge, 1.0);
  if (plan.ok && !plan.path.empty()) {
   rejoin_pts.push_back(pocket);
   rejoin_pts.insert(rejoin_pts.end(), plan.path.begin(), plan.path.end());
   rejoin_pts.push_back(merge);
   rejoin_pts.insert(rejoin_pts.end(), route_suffix.begin(), route_suffix.end());
   rejoin_planned = true;
  }
 }
 // Chained schedule (frozen convention): hold starts at the transit nominal
 // end, rejoin departs at hold end. Legs without a schedule (inf end) cannot
 // chain: hold/rejoin inherit the transit cause as Unknown.
 if (out.transit.end_s < std::numeric_limits<double>::infinity()) {
  const double hold_end = out.transit.end_s + std::max(0.0, hold_s);
  out.hold = assess_hold(pocket, out.transit.end_s, hold_end, frames, frame_dt_s, ego_radius_m,
                         margin_m);
  if (!rejoin_planned) {
   out.rejoin = {Feasibility::Verdict::kInfeasible, DepartReason::kStaticBlocked,
                 std::numeric_limits<double>::infinity(), std::numeric_limits<double>::infinity(),
                 {}};
  } else {
   out.rejoin = assess_rejoin(rejoin_pts, 0, cruise_mps, hold_end, frames, frame_dt_s,
                              ego_radius_m, margin_m);
  }
 } else {
  const LegAssessment unchained{Feasibility::Verdict::kUnknown, out.transit.reason,
                                std::numeric_limits<double>::infinity(),
                                std::numeric_limits<double>::infinity(), {}};
  out.hold = unchained;
  out.rejoin = unchained;
 }
 if (out.transit.verdict == Feasibility::Verdict::kInfeasible ||
     out.hold.verdict == Feasibility::Verdict::kInfeasible ||
     out.rejoin.verdict == Feasibility::Verdict::kInfeasible) {
  out.overall = Feasibility::Verdict::kInfeasible;
 } else if (out.transit.verdict == Feasibility::Verdict::kUnknown ||
            out.hold.verdict == Feasibility::Verdict::kUnknown ||
            out.rejoin.verdict == Feasibility::Verdict::kUnknown) {
  out.overall = Feasibility::Verdict::kUnknown;
 } else {
  out.overall = Feasibility::Verdict::kFeasible;
 }
 // Chain minimum over legs that produced evidence (nominal model only).
 out.chain_min_gap_m = std::min({out.transit.min_gap_m, out.hold.min_gap_m, out.rejoin.min_gap_m});
 return out;
}

PocketSearch sample_pockets(const GridMap& inflated_map, const std::vector<Vec2>& route,
                            Vec2 start, double start_theta, double cruise_mps, const ChassisExecCaps& exec,
                            double hold_s, const PocketSampleParams& params,
                            const std::vector<std::vector<DynState>>& frames, double frame_dt_s,
                            double ego_radius_m, double margin_m) {
 PocketSearch out;
 if (route.size() < 2 || params.max_candidates <= 0 || params.lateral_offsets_m.empty()) {
  return out;
 }
 // Arc-length table for downstream merge selection.
 std::vector<double> arc(route.size(), 0.0);
 for (std::size_t i = 1; i < route.size(); ++i) {
  arc[i] = arc[i - 1] + std::hypot(route[i].x - route[i - 1].x, route[i].y - route[i - 1].y);
 }
 std::set<std::pair<int, int>> tried;
 double next_vertex_s = 0.0;
 for (std::size_t i = 0; i < route.size() && !out.truncated; ++i) {
  if (arc[i] > params.window_m + 1e-9) {
   break;
  }
  if (arc[i] < next_vertex_s - 1e-9) {
   continue;
  }
  next_vertex_s = arc[i] + std::max(0.0, params.vertex_stride_m);
  const Vec2 prev = route[i == 0 ? 0 : i - 1];
  const Vec2 next = route[i + 1 >= route.size() ? route.size() - 1 : i + 1];
  Vec2 dir{next.x - prev.x, next.y - prev.y};
  const double len = std::hypot(dir.x, dir.y);
  if (len < 1e-9) {
   continue;
  }
  dir = {dir.x / len, dir.y / len};
  const Vec2 normal{-dir.y, dir.x};
  for (const double offset : params.lateral_offsets_m) {
   if (out.truncated) {
    break;
   }
   const Vec2 raw{route[i].x + normal.x * offset, route[i].y + normal.y * offset};
   const auto cell = inflated_map.worldToGrid(raw.x, raw.y);
   if (!tried.insert(cell).second || !inflated_map.isFreeCell(cell.first, cell.second)) {
    continue;
   }
   const Vec2 pocket = inflated_map.gridToWorld(cell.first, cell.second);
   if (std::hypot(pocket.x - start.x, pocket.y - start.y) < params.min_pocket_dist_m) {
    continue;
   }
   std::size_t merge_idx = route.size();
   for (std::size_t j = i + 1; j < route.size(); ++j) {
    if (arc[j] - arc[i] >= params.min_downstream_m - 1e-9) {
     merge_idx = j;
     break;
    }
   }
   if (merge_idx >= route.size()) {
    continue;
   }
   PocketCandidate candidate;
   candidate.pocket = pocket;
   candidate.merge = route[merge_idx];
   candidate.assessment = assess_pocket_wait(
     inflated_map, start, start_theta, pocket, route[merge_idx],
     std::vector<Vec2>(route.begin() + static_cast<std::ptrdiff_t>(merge_idx), route.end()),
     cruise_mps, exec, hold_s, frames, frame_dt_s, ego_radius_m, margin_m);
   out.candidates.push_back(std::move(candidate));
   if (static_cast<int>(out.candidates.size()) >= params.max_candidates) {
    out.truncated = true;
   }
  }
 }
 bool any_feasible = false;
 bool any_unknown = false;
 for (const auto& candidate : out.candidates) {
  if (candidate.assessment.overall == Feasibility::Verdict::kFeasible) {
   any_feasible = true;
  } else if (candidate.assessment.overall == Feasibility::Verdict::kUnknown) {
   any_unknown = true;
  }
 }
 if (any_feasible) {
  out.verdict = SearchVerdict::kFoundFeasible;
 } else if (out.candidates.empty() || out.truncated || any_unknown) {
  out.verdict = SearchVerdict::kIncomplete;
 } else {
  out.verdict = SearchVerdict::kAllRefused;
 }
 return out;
}

DepartureAssessment assess_departure(const std::vector<Vec2>& path, std::size_t start_idx,
                                     double cruise_mps,
                                     const std::vector<std::vector<DynState>>& frames,
                                     double frame_dt_s, double ego_radius_m, double margin_m) {
 if (path.empty() || frames.empty() || cruise_mps <= 0.0) {
  return {std::numeric_limits<double>::infinity(), DepartReason::kInvalid};
 }
 const double dt = std::max(1e-6, frame_dt_s);
 // Last frame stands at (N-1)*dt: predict_frames emits future_s = 0..H
 // inclusive, so N frames cover H = (N-1)*dt seconds of absolute time.
 const double horizon =
   static_cast<double>(frames.size() > 0 ? frames.size() - 1 : 0) * dt;
 // Full-trajectory gate (reviewer requirement): a wait is only feasible if
 // the ego completes its whole remaining path inside the predicted horizon
 // AND the swept gap passes. A wait that merely pushes the collision past
 // the last frame is undecidable, not safe — skip it.
 double path_length = 0.0;
 for (std::size_t seg = start_idx; seg + 1 < path.size(); ++seg) {
  path_length += std::hypot(path[seg + 1].x - path[seg].x, path[seg + 1].y - path[seg].y);
 }
 constexpr double kSearchStepS = 0.2;
 for (double wait = 0.0; wait <= horizon + 1e-9; wait += kSearchStepS) {
  if (cruise_mps * (horizon - wait) < path_length - 1e-9) {
   continue;
  }
  const double gap = predicted_gap(path, start_idx, cruise_mps, frames, dt, ego_radius_m, wait);
  if (gap >= margin_m) {
   return {wait, wait <= 1e-9 ? DepartReason::kGoNow : DepartReason::kWait};
  }
 }
 // No wait is provably safe. Diagnose why, most urgent first: a swept
 // waiting spot means standing still is what kills (observable inside the
 // window regardless of traversal coverage); an uncovered traversal is a
 // window problem (unknown, not unsafe); otherwise the corridor is genuinely
 // blocked inside a sufficient window.
 const Vec2 start = path[std::min(start_idx, path.size() - 1)];
 const double spot_gap = predicted_gap({start}, 0, 0.0, frames, dt, ego_radius_m, 0.0);
 if (spot_gap < margin_m) {
  return {std::numeric_limits<double>::infinity(), DepartReason::kWaitUnsafe};
 }
 if (path_length > cruise_mps * horizon + 1e-9) {
  return {std::numeric_limits<double>::infinity(), DepartReason::kHorizonShort};
 }
 return {std::numeric_limits<double>::infinity(), DepartReason::kBlockedInWindow};
}

} // namespace sim
