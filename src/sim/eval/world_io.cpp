#include "sim/eval/world_io.hpp"

#include <cmath>
#include <fstream>
#include <sstream>

#include "robot/core/json.hpp"
#include "sim/world/terrain.hpp"

namespace lost_signal::sim::eval {
using robot::core::Json;
using robot::core::json_dump;
using robot::core::json_parse;

namespace {

bool read_text(const std::string& path, std::string& text, std::string* error) {
  std::ifstream in(path, std::ios::binary);
  if (!in.is_open()) {
    if (error) *error = "cannot open file: " + path;
    return false;
  }
  std::ostringstream ss;
  ss << in.rdbuf();
  text = ss.str();
  return true;
}

bool write_text(const std::string& path, const std::string& text,
                std::string* error) {
  std::ofstream out(path, std::ios::binary | std::ios::trunc);
  if (!out.is_open()) {
    if (error) *error = "cannot write file: " + path;
    return false;
  }
  out << text;
  out.flush();
  if (!out.good()) {
    if (error) *error = "write failed: " + path;
    return false;
  }
  return true;
}

bool num_field(const Json& j, const char* key, double& out, std::string& err) {
  const Json* n = j.find(key);
  if (n == nullptr || !n->is_num() || !std::isfinite(n->as_num())) {
    err = std::string("key '") + key + "' missing or not a finite number";
    return false;
  }
  out = n->as_num();
  return true;
}

bool str_field(const Json& j, const char* key, std::string& out,
               std::string& err) {
  const Json* n = j.find(key);
  if (n == nullptr || !n->is_str() || n->as_str().empty()) {
    err = std::string("key '") + key + "' missing or empty";
    return false;
  }
  out = n->as_str();
  return true;
}

bool rect_fields(const Json& j, double& x0, double& y0, double& x1,
                 double& y1, std::string& err) {
  return num_field(j, "x0", x0, err) && num_field(j, "y0", y0, err) &&
         num_field(j, "x1", x1, err) && num_field(j, "y1", y1, err);
}

const char* impact_material_name(world::ImpactMaterial material) {
 switch (material) {
 case world::ImpactMaterial::kMetal: return "metal";
 case world::ImpactMaterial::kConcrete: return "concrete";
 case world::ImpactMaterial::kGlass: return "glass";
 case world::ImpactMaterial::kWood: return "wood";
 case world::ImpactMaterial::kPolymer: return "polymer";
 case world::ImpactMaterial::kRubber: return "rubber";
 case world::ImpactMaterial::kUnknown: return "unknown";
 }
 return "unknown";
}

bool parse_static_impact_material(const std::string& name,
 world::ImpactMaterial& material) {
 if (name == "metal") material = world::ImpactMaterial::kMetal;
 else if (name == "concrete") material = world::ImpactMaterial::kConcrete;
 else if (name == "glass") material = world::ImpactMaterial::kGlass;
 else if (name == "wood") material = world::ImpactMaterial::kWood;
 else if (name == "polymer") material = world::ImpactMaterial::kPolymer;
 else if (name == "rubber") material = world::ImpactMaterial::kRubber;
 else return false;
 return true;
}

Json box_to_json(const world::StaticBox& b) {
 Json::Object object;
 if (!b.box_id.empty()) object.emplace_back("box_id", Json::Str(b.box_id));
 object.emplace_back("x0", Json::Num(b.x0));
 object.emplace_back("y0", Json::Num(b.y0));
 object.emplace_back("x1", Json::Num(b.x1));
 object.emplace_back("y1", Json::Num(b.y1));
 object.emplace_back("height_m", Json::Num(b.height_m));
 object.emplace_back("impact_material",
 Json::Str(impact_material_name(b.impact_material)));
 object.emplace_back("prior_map_visible", Json::Bool(b.prior_map_visible));
 return Json::Obj(std::move(object));
}

bool box_from_json(const Json& j, world::StaticBox& b, std::string& err) {
  if (!j.is_object()) {
    err = "box not an object";
    return false;
  }
 const Json* id = j.find("box_id");
 if (id != nullptr) {
 if (!id->is_str() || id->as_str().empty()) {
 err = "box_id must be a non-empty string";
 return false;
 }
 b.box_id = id->as_str();
 }
 const Json* impact_material = j.find("impact_material");
 if (impact_material != nullptr) {
 if (!impact_material->is_str() || !parse_static_impact_material(
 impact_material->as_str(), b.impact_material)) {
 err = "impact_material must be concrete, metal, glass, wood, polymer, "
 "or rubber";
 return false;
 }
 }
 const Json* prior_map_visible = j.find("prior_map_visible");
 if (prior_map_visible != nullptr) {
 if (!prior_map_visible->is_bool()) {
 err = "prior_map_visible must be boolean";
 return false;
 }
 b.prior_map_visible = prior_map_visible->as_bool();
 }
 return num_field(j, "x0", b.x0, err) && num_field(j, "y0", b.y0, err) &&
         num_field(j, "x1", b.x1, err) && num_field(j, "y1", b.y1, err) &&
         num_field(j, "height_m", b.height_m, err);
}

Json place_to_json(const world::NamedPlace& p) {
  return Json::Obj({{"place_id", Json::Str(p.place_id)},
                    {"x_m", Json::Num(p.x_m)},
                    {"y_m", Json::Num(p.y_m)},
                    {"yaw_rad", Json::Num(p.yaw_rad)}});
}

bool place_from_json(const Json& j, world::NamedPlace& p, std::string& err) {
  if (!j.is_object()) {
    err = "place not an object";
    return false;
  }
  return str_field(j, "place_id", p.place_id, err) &&
         num_field(j, "x_m", p.x_m, err) &&
         num_field(j, "y_m", p.y_m, err) &&
         num_field(j, "yaw_rad", p.yaw_rad, err);
}

Json initial_pose_to_json(const world::InitialPosePrior& p) {
 return Json::Obj({{"x_m", Json::Num(p.x_m)},
 {"y_m", Json::Num(p.y_m)},
 {"yaw_rad", Json::Num(p.yaw_rad)},
 {"position_sigma_m", Json::Num(p.position_sigma_m)},
 {"heading_sigma_rad", Json::Num(p.heading_sigma_rad)}});
}

bool initial_pose_from_json(const Json& j, world::InitialPosePrior& p,
 std::string& err) {
 if (!j.is_object()) {
 err = "initial_pose not an object";
 return false;
 }
 if (!num_field(j, "x_m", p.x_m, err) ||
 !num_field(j, "y_m", p.y_m, err) ||
 !num_field(j, "yaw_rad", p.yaw_rad, err) ||
 !num_field(j, "position_sigma_m", p.position_sigma_m, err) ||
 !num_field(j, "heading_sigma_rad", p.heading_sigma_rad, err)) {
 return false;
 }
 if (!(p.position_sigma_m > 0.0) || !(p.heading_sigma_rad > 0.0)) {
 err = "initial_pose uncertainty must be positive";
 return false;
 }
 return true;
}

Json material_to_json(const world::MaterialEntry& m) {
  return Json::Obj({{"material_id", Json::Str(m.material_id)},
                    {"friction_mu", Json::Num(m.friction_mu)},
                    {"rolling_resist", Json::Num(m.rolling_resist)},
                    {"contact_time_constant_s",
                     Json::Num(m.contact_time_constant_s)},
                    {"contact_damping_ratio",
                     Json::Num(m.contact_damping_ratio)},
 {"lidar_reflectivity",
 Json::Num(m.lidar_reflectivity)},
 {"surface_effect", Json::Str(m.surface_effect)},
 {"fluid_depth_m", Json::Num(m.fluid_depth_m)}});
}

bool material_from_json(const Json& j, world::MaterialEntry& m,
                        std::string& err) {
  if (!j.is_object()) {
    err = "material not an object";
    return false;
  }
  if (!str_field(j, "material_id", m.material_id, err)) return false;
  if (!num_field(j, "friction_mu", m.friction_mu, err) ||
      !num_field(j, "rolling_resist", m.rolling_resist, err) ||
      !num_field(j, "contact_time_constant_s",
                 m.contact_time_constant_s, err) ||
      !num_field(j, "contact_damping_ratio", m.contact_damping_ratio,
                 err) ||
      !num_field(j, "lidar_reflectivity", m.lidar_reflectivity, err)) {
    return false;
  }
  if (m.friction_mu < 0.0 || m.rolling_resist < 0.0 ||
      m.contact_time_constant_s <= 0.0 || m.contact_damping_ratio < 0.0 ||
      m.lidar_reflectivity < 0.0) {
    err = "material '" + m.material_id + "' has a negative/nonphysical value";
    return false;
  }
 const Json* surface_effect = j.find("surface_effect");
 if (surface_effect != nullptr) {
 if (!surface_effect->is_str()) {
 err = "material '" + m.material_id + "' surface_effect must be a string";
 return false;
 }
 m.surface_effect = surface_effect->as_str();
 }
 const Json* fluid_depth = j.find("fluid_depth_m");
 if (fluid_depth != nullptr) {
 if (!fluid_depth->is_num() || !std::isfinite(fluid_depth->as_num())) {
 err = "material '" + m.material_id + "' fluid_depth_m must be finite";
 return false;
 }
 m.fluid_depth_m = fluid_depth->as_num();
 }
 if (m.surface_effect != "none" && m.surface_effect != "dust" &&
 m.surface_effect != "mud" && m.surface_effect != "water") {
 err = "material '" + m.material_id + "' has unknown surface_effect";
 return false;
 }
 if (m.fluid_depth_m < 0.0 || m.fluid_depth_m > 0.25 ||
 (m.surface_effect == "water" && !(m.fluid_depth_m > 0.0)) ||
 (m.surface_effect != "water" && m.fluid_depth_m != 0.0)) {
 err = "material '" + m.material_id + "' has invalid fluid_depth_m";
 return false;
 }
  return true;
}

Json zone_to_json(const world::MaterialZone& z) {
  return Json::Obj({{"zone_id", Json::Str(z.zone_id)},
                    {"x0", Json::Num(z.x0)},
                    {"y0", Json::Num(z.y0)},
                    {"x1", Json::Num(z.x1)},
                    {"y1", Json::Num(z.y1)},
                    {"material", Json::Str(z.material_id)}});
}

bool zone_from_json(const Json& j, world::MaterialZone& z, std::string& err) {
  if (!j.is_object()) {
    err = "zone not an object";
    return false;
  }
  return str_field(j, "zone_id", z.zone_id, err) &&
         rect_fields(j, z.x0, z.y0, z.x1, z.y1, err) &&
         str_field(j, "material", z.material_id, err);
}

Json patch_to_json(const world::TerrainPatch& p) {
  return Json::Obj({{"patch_id", Json::Str(p.patch_id)},
                    {"x0", Json::Num(p.x0)},
                    {"y0", Json::Num(p.y0)},
                    {"x1", Json::Num(p.x1)},
                    {"y1", Json::Num(p.y1)},
                    {"base_height_m", Json::Num(p.base_height_m)},
                    {"grade_x", Json::Num(p.grade_x)},
                    {"grade_y", Json::Num(p.grade_y)},
                    {"grade_xy_per_m", Json::Num(p.grade_xy_per_m)},
                    {"roughness", Json::Num(p.roughness)}});
}

bool patch_from_json(const Json& j, world::TerrainPatch& p,
                     std::string& err) {
  if (!j.is_object()) {
    err = "terrain patch not an object";
    return false;
  }
  return str_field(j, "patch_id", p.patch_id, err) &&
         rect_fields(j, p.x0, p.y0, p.x1, p.y1, err) &&
         num_field(j, "base_height_m", p.base_height_m, err) &&
         num_field(j, "grade_x", p.grade_x, err) &&
         num_field(j, "grade_y", p.grade_y, err) &&
         num_field(j, "grade_xy_per_m", p.grade_xy_per_m, err) &&
         num_field(j, "roughness", p.roughness, err);
}

Json slab_to_json(const world::SupportSlab& s) {
  return Json::Obj({{"slab_id", Json::Str(s.slab_id)},
                    {"center_x_m", Json::Num(s.center_x_m)},
                    {"center_y_m", Json::Num(s.center_y_m)},
                    {"half_length_m", Json::Num(s.half_length_m)},
                    {"half_width_m", Json::Num(s.half_width_m)},
                    {"yaw_rad", Json::Num(s.yaw_rad)},
                    {"center_height_m", Json::Num(s.center_height_m)},
                    {"grade_along_local_x",
                     Json::Num(s.grade_along_local_x)}});
}

bool slab_from_json(const Json& j, world::SupportSlab& s, std::string& err) {
  if (!j.is_object()) {
    err = "slab not an object";
    return false;
  }
  return str_field(j, "slab_id", s.slab_id, err) &&
         num_field(j, "center_x_m", s.center_x_m, err) &&
         num_field(j, "center_y_m", s.center_y_m, err) &&
         num_field(j, "half_length_m", s.half_length_m, err) &&
         num_field(j, "half_width_m", s.half_width_m, err) &&
         num_field(j, "yaw_rad", s.yaw_rad, err) &&
         num_field(j, "center_height_m", s.center_height_m, err) &&
         num_field(j, "grade_along_local_x", s.grade_along_local_x, err);
}

Json environment_to_json(const world::EnvironmentKeyframe& k) {
  Json::Object grip;
  for (const auto& [id, mult] : k.material_grip) {
    grip.emplace_back(id, Json::Num(mult));
  }
  return Json::Obj({{"time_s", Json::Num(k.time_s)},
                    {"material_grip", Json::Obj(std::move(grip))}});
}

bool environment_from_json(const Json& j, world::EnvironmentKeyframe& k,
                           std::string& err) {
  if (!j.is_object()) {
    err = "environment keyframe not an object";
    return false;
  }
  if (!num_field(j, "time_s", k.time_s, err)) return false;
  if (k.time_s < 0.0) {
    err = "environment time_s negative";
    return false;
  }
  const Json* g = j.find("material_grip");
  if (g == nullptr || !g->is_object()) {
    err = "environment material_grip missing or not an object";
    return false;
  }
  for (const auto& [id, v] : g->as_object()) {
    if (id.empty() || !v.is_num() || !std::isfinite(v.as_num()) ||
        v.as_num() < 0.0) {
      err = "environment material_grip has a bad entry";
      return false;
    }
    k.material_grip.emplace_back(id, v.as_num());
  }
  return true;
}

}  // namespace

bool save_world_file(const std::string& path, const WorldFileContent& content,
                      std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  Json::Array box_arr, place_arr, mat_arr, zone_arr, patch_arr, slab_arr,
      env_arr;
  for (const auto& b : content.boxes) box_arr.push_back(box_to_json(b));
  for (const auto& p : content.places) place_arr.push_back(place_to_json(p));
  for (const auto& m : content.materials)
    mat_arr.push_back(material_to_json(m));
  for (const auto& z : content.zones) zone_arr.push_back(zone_to_json(z));
  for (const auto& p : content.patches)
    patch_arr.push_back(patch_to_json(p));
  for (const auto& s : content.slabs) slab_arr.push_back(slab_to_json(s));
  for (const auto& k : content.environment)
    env_arr.push_back(environment_to_json(k));
 Json::Object root;
 root.emplace_back("format", Json::Str(kWorldFileFormat));
 if (content.initial_pose.has_value()) {
 root.emplace_back(
 "initial_pose", initial_pose_to_json(*content.initial_pose));
 }
 root.emplace_back("boxes", Json::Arr(std::move(box_arr)));
 root.emplace_back("places", Json::Arr(std::move(place_arr)));
 root.emplace_back("materials", Json::Arr(std::move(mat_arr)));
 root.emplace_back("zones", Json::Arr(std::move(zone_arr)));
 root.emplace_back("terrain", Json::Arr(std::move(patch_arr)));
 root.emplace_back("slabs", Json::Arr(std::move(slab_arr)));
 root.emplace_back("environment", Json::Arr(std::move(env_arr)));
 if (!write_text(path, json_dump(Json::Obj(std::move(root))), error)) {
 return false;
 }
  err.clear();
  return true;
}

bool load_world_file(const std::string& path, WorldFileContent& content,
                      std::string* error) {
  std::string err_local;
  std::string& err = error ? *error : err_local;
  std::string text;
  if (!read_text(path, text, error)) return false;
  std::string parse_error;
  auto root = json_parse(text, &parse_error);
  if (root == nullptr || !root->is_object()) {
    err = "invalid world file: " + parse_error;
    return false;
  }
  const Json* fmt = root->find("format");
  if (fmt == nullptr || !fmt->is_str() ||
      fmt->as_str() != kWorldFileFormat) {
    err = "unsupported world format (want " + std::string(kWorldFileFormat) +
          ")";
    return false;
  }
  const Json* bj = root->find("boxes");
  const Json* pj = root->find("places");
  const Json* mj = root->find("materials");
  const Json* zj = root->find("zones");
  const Json* tj = root->find("terrain");
  const Json* sj = root->find("slabs");
  const Json* ej = root->find("environment");
  if (bj == nullptr || !bj->is_array() || pj == nullptr ||
      !pj->is_array() || mj == nullptr || !mj->is_array() ||
      zj == nullptr || !zj->is_array() || tj == nullptr ||
      !tj->is_array() || sj == nullptr || !sj->is_array() ||
      ej == nullptr || !ej->is_array()) {
    err = "world file needs boxes/places/materials/zones/terrain/slabs/"
          "environment arrays";
    return false;
  }
  WorldFileContent out;
 if (const Json* initial = root->find("initial_pose")) {
 world::InitialPosePrior prior;
 if (!initial_pose_from_json(*initial, prior, err)) return false;
 out.initial_pose = prior;
 }
  for (const auto& e : bj->as_array()) {
    world::StaticBox b;
    if (!box_from_json(e, b, err)) return false;
 if (!b.box_id.empty()) {
 for (const auto& previous : out.boxes) {
 if (previous.box_id == b.box_id) {
 err = "duplicate box_id '" + b.box_id + "'";
 return false;
 }
 }
 }
    out.boxes.push_back(b);
  }
  for (const auto& e : pj->as_array()) {
    world::NamedPlace p;
    if (!place_from_json(e, p, err)) return false;
    out.places.push_back(p);
  }
  for (const auto& e : mj->as_array()) {
    world::MaterialEntry m;
    if (!material_from_json(e, m, err)) return false;
    for (const auto& prev : out.materials) {
      if (prev.material_id == m.material_id) {
        err = "duplicate material '" + m.material_id + "'";
        return false;
      }
    }
    out.materials.push_back(m);
  }
  for (const auto& e : zj->as_array()) {
    world::MaterialZone z;
    if (!zone_from_json(e, z, err)) return false;
    if (world::find_material(out.materials, z.material_id) == nullptr) {
      err = "zone '" + z.zone_id + "' references unknown material '" +
            z.material_id + "'";
      return false;
    }
    out.zones.push_back(z);
  }
  for (const auto& e : tj->as_array()) {
    world::TerrainPatch p;
    if (!patch_from_json(e, p, err)) return false;
    out.patches.push_back(p);
  }
  for (const auto& e : sj->as_array()) {
    world::SupportSlab s;
    if (!slab_from_json(e, s, err)) return false;
    out.slabs.push_back(s);
  }
  for (const auto& e : ej->as_array()) {
    world::EnvironmentKeyframe k;
    if (!environment_from_json(e, k, err)) return false;
    out.environment.push_back(k);
  }
  content = std::move(out);
  err.clear();
  return true;
}

}  // namespace lost_signal::sim::eval
