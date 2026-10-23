#!/usr/bin/env python3
"""Convert master map JSON (assets/maps/*.json, schema v2) into the clean-tree
SIM interchange (lost-signal-world/2) + an actors sidecar.

Usage:
  python3 tools/convert_master_map.py --all
  python3 tools/convert_master_map.py signal_town_big slope_lab
  python3 tools/convert_master_map.py --ref origin/main --variant slope_launch slope_lab

Sources default to `git show <ref>:assets/...` so no master checkout is
needed; --src-dir reads from a directory instead.

Mapping (documented assumptions, all warned on stdout):
  obstacles (min/max, no height in master schema) -> boxes, --obstacle-height
    default 1.2 m (matches master's wall_panel collision height).
  world_objects collisions (box, position offset + size + object yaw/scale)
    -> boxes as AABBs; top = z-center + sz/2 (floating props stay visible).
  surfaces -> materials (params from assets/surfaces.json) + zones.
    Unknown material id -> keeps the id, substitutes gravel numbers, warns.
  terrain height_patch -> TerrainPatch (grade_xy defaults 0).
  markers [x, y, yaw] -> places, sorted by key (deterministic mission order).
  actors -> SIDEcar <id>.actors.json (world/2 carries no actors):
    patrol -> waypoints loop, dwell -> waypoints + waits, seeded-wander ->
    wander (seed = fnv1a(id), start = path[0]). dormant_until_summoned
    entries are skipped. Patrol loop=true is an assumption (master loops
    patrols); dwell loops only on closed paths.
  conflict_zones (game authority logic) -> dropped, warned.
  active_in filtering: --variant keeps entries with no active_in or with the
    variant listed; default keeps everything.
"""
import argparse
import hashlib
import json
import math
import subprocess
import sys
from pathlib import Path

WORLD_FORMAT = "lost-signal-world/2"
DEFAULT_OBSTACLE_H = 1.2
FALLBACK_MATERIAL = {
    "friction_mu": 0.7,
    "rolling_resist": 0.015,
    "contact_time_constant_s": 0.01,
    "contact_damping_ratio": 1.15,
    "lidar_reflectivity": 0.28,
}


def git_show(ref, path):
    r = subprocess.run(["git", "show", f"{ref}:{path}"],
                       capture_output=True, text=True)
    if r.returncode != 0:
        raise SystemExit(f"git show {ref}:{path} failed")
    return r.stdout


def load_json(args, rel):
    if args.src_dir:
        return json.loads(Path(args.src_dir, rel).read_text())
    return json.loads(git_show(args.ref, rel))


def keep(entry, variant):
    if variant is None:
        return True
    active = entry.get("active_in")
    return not active or variant in active


# Goal geometry assertions (localization-aware content gate). A converted
# marker is only a coordinate; the robot must arrive at it with a drifting
# estimate and verify arrival with a scan. These checks flag the markers
# most likely to wedge a mission, without failing the conversion (warned
# on stdout; the mission scores truth-arrival separately):
# - clearance: goal closer than CLEARANCE_M to a box leaves no room for
#   estimate error inside the arrival radius (dock_door class);
# - flat-wall adjacency: goal beside a long flat face in an otherwise open
#   area is unobservable along the wall (scan matching ties), so the last
#   half meter cannot be verified from any single viewpoint;
# - grade: goal on a graded patch needs traction the arrival hold may not
#   have (informational; slope missions keep such goals deliberately).
def check_places(boxes, places, patches):
    found = []
    for p in places:
        px, py = p["x_m"], p["y_m"]
        nearest, face_len = None, 0.0
        for b in boxes:
            dx = max(b["x0"] - px, 0.0, px - b["x1"])
            dy = max(b["y0"] - py, 0.0, py - b["y1"])
            d = math.hypot(dx, dy)
            if nearest is None or d < nearest:
                nearest = d
                # Tangent extent of the closest face (ambiguity lever arm).
                if dx >= dy:
                    face_len = b["x1"] - b["x0"]
                else:
                    face_len = b["y1"] - b["y0"]
        if nearest is not None and nearest < 1.0:
            found.append(
                f"place {p['place_id']}: {nearest:.2f} m from nearest box "
                f"(< 1.0 m clearance; arrival needs estimate room)")
            if face_len > 3.0:
                found.append(
                    f"place {p['place_id']}: beside a {face_len:.1f} m flat "
                    f"face (along-wall ambiguity: the final approach cannot "
                    f"be verified from one viewpoint)")
        for t in patches:
            if (t["x0"] <= px <= t["x1"] and t["y0"] <= py <= t["y1"]
                    and math.hypot(t.get("grade_x", 0.0),
                                    t.get("grade_y", 0.0)) > 0.05):
                found.append(
                    f"place {p['place_id']}: on graded patch "
                    f"{t['patch_id']} (arrival hold needs traction)")
    return found


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("maps", nargs="*", help="map ids (default with --all)")
    ap.add_argument("--all", action="store_true")
    ap.add_argument("--ref", default="origin/main")
    ap.add_argument("--src-dir", default=None)
    ap.add_argument("--variant", default=None)
    ap.add_argument("--obstacle-height", type=float, default=DEFAULT_OBSTACLE_H)
    ap.add_argument("--out-dir", default="configs/worlds")
    ap.add_argument("--list", action="store_true",
                    help="list available map ids and exit")
    args = ap.parse_args()

    if args.list:
        out = subprocess.run(
            ["git", "ls-tree", "-r", "--name-only", args.ref,
             "assets/maps/"], capture_output=True, text=True)
        for line in sorted(out.stdout.splitlines()):
            if line.endswith(".json"):
                print(Path(line).stem)
        return

    ids = list(args.maps)
    if args.all:
        out = subprocess.run(
            ["git", "ls-tree", "-r", "--name-only", args.ref,
             "assets/maps/"], capture_output=True, text=True)
        ids = sorted(Path(l).stem for l in out.stdout.splitlines()
                     if l.endswith(".json"))
    if not ids:
        raise SystemExit("give map ids or --all")
    # English locale mirrors carry the same ids; prefer the canonical dir.
    # (Both are read from the same ref; duplicates collapse by id.)

    surfaces = load_json(args, "assets/surfaces.json")
    matlib = {m["id"]: m for m in surfaces.get("materials", [])}

    out_dir = Path(args.out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    for mid in ids:
        try:
            m = load_json(args, f"assets/maps/{mid}.json")
        except SystemExit as e:
            print(f"{mid}: SKIP ({e})")
            continue
        convert_one(mid, m, matlib, args, out_dir)


def convert_one(mid, m, matlib, args, out_dir):
    warn = []
    boxes, places, materials, zones, patches = [], [], {}, [], []
    actors = []

    for o in m.get("obstacles", []):
        if not keep(o, args.variant):
            continue
        (x0, y0), (x1, y1) = o["min"], o["max"]
        boxes.append({"x0": min(x0, x1), "y0": min(y0, y1),
                      "x1": max(x0, x1), "y1": max(y0, y1),
                      "height_m": args.obstacle_height})
    if m.get("obstacles"):
        warn.append(f"obstacle height defaulted to {args.obstacle_height} m "
                    "(master schema carries no height)")

    import math
    for o in m.get("world_objects", []):
        if not keep(o, args.variant):
            continue
        px, py, pz = o["position_m"]
        yaw = math.radians(o.get("yaw_degrees", 0.0))
        sx, sy, sz = o.get("scale", [1.0, 1.0, 1.0])
        for c in o.get("collisions", []):
            if c.get("type") != "box":
                warn.append(f"{o['id']}: non-box collision dropped")
                continue
            ox, oy, oz = c["position_m"]
            dx, dy, dz = (c["size_m"][0] * sx, c["size_m"][1] * sy,
                          c["size_m"][2] * sz)
            # rotate offset + extents into world, take the AABB
            cx = px + ox * math.cos(yaw) - oy * math.sin(yaw)
            cy = py + ox * math.sin(yaw) + oy * math.cos(yaw)
            ex = abs(dx * math.cos(yaw)) + abs(dy * math.sin(yaw))
            ey = abs(dx * math.sin(yaw)) + abs(dy * math.cos(yaw))
            top = pz + oz + dz / 2.0
            boxes.append({"x0": cx - ex / 2.0, "y0": cy - ey / 2.0,
                          "x1": cx + ex / 2.0, "y1": cy + ey / 2.0,
                          "height_m": max(0.05, top)})

    for s in m.get("surfaces", []):
        if not keep(s, args.variant):
            continue
        mat_id = s["material"]
        if mat_id not in materials:
            src = matlib.get(mat_id)
            if src is None:
                warn.append(f"surface {s['id']}: unknown material "
                            f"'{mat_id}', gravel numbers substituted")
                src = {"friction_mu": FALLBACK_MATERIAL["friction_mu"],
                       "rolling_drag": FALLBACK_MATERIAL["rolling_resist"],
                       "contact_time_constant_s":
                           FALLBACK_MATERIAL["contact_time_constant_s"],
                       "contact_damping_ratio":
                           FALLBACK_MATERIAL["contact_damping_ratio"],
                       "lidar_reflectivity":
                           FALLBACK_MATERIAL["lidar_reflectivity"]}
            materials[mat_id] = {
                "material_id": mat_id,
                "friction_mu": src["friction_mu"],
                "rolling_resist": src.get("rolling_drag", 0.0),
                "contact_time_constant_s": src["contact_time_constant_s"],
                "contact_damping_ratio": src["contact_damping_ratio"],
                "lidar_reflectivity": src["lidar_reflectivity"],
            }
        (x0, y0), (x1, y1) = s["min"], s["max"]
        zones.append({"zone_id": s["id"], "x0": min(x0, x1),
                      "y0": min(y0, y1), "x1": max(x0, x1),
                      "y1": max(y0, y1), "material": mat_id})

    for t in m.get("terrain", []):
        if not keep(t, args.variant):
            continue
        (x0, y0), (x1, y1) = t["min"], t["max"]
        patches.append({
            "patch_id": t["id"], "x0": min(x0, x1), "y0": min(y0, y1),
            "x1": max(x0, x1), "y1": max(y0, y1),
            "base_height_m": t.get("base_height_m", 0.0),
            "grade_x": t.get("grade_x", 0.0),
            "grade_y": t.get("grade_y", 0.0),
            "grade_xy_per_m": t.get("grade_xy_per_m", 0.0),
            "roughness": t.get("roughness", 0.0)})

    for key in sorted(m.get("markers", {}).keys()):
        v = m["markers"][key]
        places.append({"place_id": key, "x_m": v[0], "y_m": v[1],
                       "yaw_rad": v[2] if len(v) > 2 else 0.0})

    for a in m.get("actors", []):
        if a.get("dormant_until_summoned"):
            warn.append(f"actor {a['id']}: dormant, skipped")
            continue
        if not keep(a, args.variant):
            continue
        kind = a.get("type", "patrol")
        path = a.get("path", [])
        closed = (len(path) >= 2 and path[0] == path[-1])
        behavior = "waypoints"
        if kind == "seeded-wander":
            behavior = "wander"
        seed = int(hashlib.sha256(a["id"].encode()).hexdigest()[:8], 16)
        actors.append({
            "id": a["id"], "x": path[0][0], "y": path[0][1],
            "radius_m": a.get("radius_m", 0.4),
            "height_m": 1.8 if (a.get("visual", {}) or {}).get(
                "kind") == "pedestrian" else 1.5,
            "speed_mps": a.get("speed_mps", 0.5),
            "behavior": behavior,
            "waypoints": [{"x": p[0], "y": p[1]} for p in path]
            if behavior == "waypoints" else [],
            "loop": True if kind == "patrol" else closed,
            "waits": a.get("waits", []),
            "seed": seed,
            "visual_kind": (a.get("visual", {}) or {}).get("kind", "vehicle"),
        })
    if m.get("conflict_zones"):
        warn.append(f"{len(m['conflict_zones'])} conflict_zones dropped "
                    "(game authority logic, no SIM counterpart)")

    world = {"format": WORLD_FORMAT, "boxes": boxes, "places": places,
             "materials": list(materials.values()), "zones": zones,
             "terrain": patches, "slabs": [], "environment": []}
    for w in check_places(boxes, places, patches):
        warn.append(w)
    suffix = f".{args.variant}" if args.variant else ""
    wpath = out_dir / f"{mid}{suffix}.json"
    wpath.write_text(json.dumps(world, indent=1) + "\n")
    apath = out_dir / f"{mid}{suffix}.actors.json"
    apath.write_text(json.dumps({"actors": actors}, indent=1) + "\n")

    print(f"{mid}: boxes={len(boxes)} places={len(places)} "
          f"materials={len(materials)} zones={len(zones)} "
          f"patches={len(patches)} actors={len(actors)} -> {wpath.name}")
    for w in warn:
        print(f"  warn: {w}")


if __name__ == "__main__":
    main()
