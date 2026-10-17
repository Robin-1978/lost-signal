# Lost Signal Robot Map (LSRM)

Lost Signal uses a provider-neutral, layered robot-map contract for ground navigation. A global raster is optional:
a compact indoor map may contain occupancy and 2.5D elevation layers, while a campus or road map may contain only
vector geometry, topology, semantic places, and rules. The simulated world remains fully 3D, and the minimap remains
a 2D presentation. The Robot Map is the stable value between map producers and deployable task, localization,
planning, and visualization consumers.

## Sources and products

A robot map is a published navigation product, not the native project file of every mapping system:

```text
scenario editor ─┐
2D/3D SLAM ──────┤
survey / GIS ────┼─> map compiler or provider ─> immutable MapSnapshot
HD vector map ───┤
operator labels ─┘
```

The authored scenario file retains actors, scripts, rendering materials, weather, scoring, and story. A SLAM
session retains its pose graph, constraints, keyframes, calibration, and raw-data references. An automotive source
may retain Lanelet2, OpenDRIVE, surveyed point clouds, and regulatory elements. None of these source artifacts is
silently reduced and presented as lossless Robot Map storage. Their providers publish the subset and derived layers
needed by the running Lost Signal stack and record source provenance.

The current scenario adapter is the first map compiler. The in-process `MapManager` is the first provider. A future
SLAM, file, remote-service, or automotive-map provider implements the same read-only interface.

## Global map and local world model

The map used by autonomy has three distinct products:

```text
global authority       bounded static view       live local world model
vector/topology ─┐                            ┌─ sensor obstacles + clearing
raster/tile ─────┼─> spatial query/rasterizer ├─> rolling occupancy/costmap
elevation/mesh ──┘                            └─ footprint/inflation/prediction costs
```

The global authority does not need a raster. A provider can return vector features and topology directly or serve
only the tiles intersecting a bounded request. Global routing can operate on its route graph. Grid A*, Hybrid A*,
and local trajectory validation receive a bounded planning view produced near the robot or route corridor; they do
not require a city-wide array.

Perception owns short-lived observations in the live local world model. Ray clearing, expiry, tracking, prediction,
and inflation update that model as the robot moves. These observations do not silently become persistent map
content. A SLAM backend may use sensor observations to produce a new global revision, but that is an explicit map
publication with provenance, validation, and revision semantics. Drawing the minimap is presentation of these
products, not map generation.

LSRM v1 implements complete immutable snapshots, an optional compact raster, vector-only packages, and a bounded
`query_region()` contract. The in-process provider crops its compact raster and filters semantic features while
preserving the source revision. A future tiled or remote provider can satisfy the same request without allocating a
global array. The current file provider still loads a complete package into memory, and the current small simulation
compiler intentionally emits one compact raster; true tile-backed I/O remains future work.

## Runtime contract

`MapSnapshot` contains:

- map identity, content revision, frame ID, monotonic publication time, and source provenance;
- grid width, height, resolution, and the full planar pose of cell `(0,0)` including yaw;
- an optional compact raster grid whose occupancy percentages use `-1` for unknown and `0..100` for observed
 probability/occupancy;
- optional elevation and elevation-uncertainty layers in meters, with NaN for unknown;
- an optional surface-class layer and its stable string table;
- named semantic places with position and heading;
- typed point, polyline, and polygon features with stable IDs and string attributes;
- explicit typed relations among places/features for route connectivity and regulatory/topological meaning.

At least a grid or semantic content is required. A small SLAM map commonly has a global raster. A large road,
campus, or regional map may be vector/topology only and serve raster/elevation/point-cloud tiles on demand. A local
map adapter derives the bounded rolling costmap needed by raster planners; a monolithic global raster is never a
mandatory authority.

Cells are row-major with x changing fastest: `index = y * width + x`. The grid-local square for cell `(x,y)` is
`[x*r,(x+1)*r) × [y*r,(y+1)*r)`. The grid origin is the lower-left corner of cell `(0,0)` in the declared map frame.
The origin SE(2) transform maps grid-local coordinates into that frame. `world_to_grid()` and
`grid_cell_center()` are the only reference conversions and have rotated-origin and boundary tests.

SI units and right-handed axes follow [REP-103](https://www.ros.org/reps/rep-0103.html). The separation among
`map`, `odom`, and the robot body follows [REP-105](https://www.ros.org/reps/rep-0105.html). `map → odom` is
localization state and is never persisted as grid origin or map content.

## Disk package v1

The `.lsmap` form is a directory so metadata, large numerical layers, and editable semantic features remain
separate:

```text
factory-a.lsmap/
  manifest.json
  features.json
  layers/
 occupancy.i8 # optional; present for raster-map packages
    elevation_m.f32          # optional
    elevation_sigma_m.f32    # optional
    surface_class.u16         # optional
```

`manifest.json` declares the format and schema version, map identity, content revision, frame, source provenance,
optional grid geometry, exact layout, layer data types, byte order, unknown values, filenames, surface-class table, and a
SHA-256 digest for every payload. Numerical layers are uncompressed little-endian arrays in canonical row-major
order. Version 1 favors a small auditable implementation and exact round trips; compression can be added as an
explicit encoding without changing layer semantics.

The writer builds a temporary sibling directory, writes the manifest last, then renames the completed package.
The reader validates version, paths, byte lengths, checksums, layer values, semantic geometry, and the complete
runtime snapshot before publishing it. Missing, corrupt, unsupported, or half-written packages fail as a unit.

Format schema version and map content revision are independent. The schema changes when encoding or meaning
changes. The revision increases when the content of one map identity changes. Publication time is assigned by the
provider's monotonic clock when a package or SLAM result enters the running system; a persisted wall-clock value
must never masquerade as that time.

## SLAM and live updates

A SLAM provider may first publish occupancy only. A multi-line LiDAR, RGB-D, or surveyed source can additionally
publish elevation and uncertainty. Missing optional layers stay absent; a 2D mapper must not invent zero-height
measurements. Dynamic-object filtering belongs to mapping/perception policy, and moving actors never become
simulation truth inside the persistent static layer.

Snapshots are immutable. A loop closure or continuing-map update publishes a greater revision. Consumers pin one
revision for a planning operation; accepting a new revision invalidates affected routes and triggers re-evaluation.
Version 1 replaces a complete snapshot. A future patch stream may optimize large maps, but it must assemble and
validate a complete new revision atomically before consumers see it.

The deployable map and the mapping session remain separate:

- Robot Map: navigation product, exchange package, minimap input, regression artifact;
- Mapping Session: backend-native pose graph, submaps, keyframes, constraints, raw-data references, and resume state.

This mirrors the distinction in SLAM Toolbox between exported navigation maps and lossless serialized pose graphs.

## Derived runtime layers

Sensor obstacle observations, clearing, inflation, footprint clearance, social costs, prediction occupancy, and DWA
candidate costs are runtime products. They are recreated from the bounded static view, current observations, robot
pose and footprint, and policy. They are not saved into the base package. The existing observed-obstacle layer
retains ray clearing and time-based expiry, so a temporary box cannot permanently contaminate the map.

The minimap reads the same immutable base revision and adds dynamic overlays. It may show elevation with contours,
slope color, hill shade, or layer toggles, but those pixels never become the authoritative map values.

## 3D and automotive extensions

A single 2.5D layer cannot represent two traversable surfaces at the same `(x,y)`, such as a bridge over a road.
Such content uses multiple named submaps/levels or an explicit 3D voxel/mesh provider. Multi-line LiDAR observations
use a separate timestamped 3D point-cloud message with per-point/ring/intensity information; perception derives
the 2D obstacle, elevation, traversability, or short-lived voxel layers needed by each consumer.

Ackermann road scenarios add a lane/route graph through semantic features and explicit relations, plus regulatory
semantics. An adapter may compile it from Lanelet2 or OpenDRIVE while retaining the external source as the authority.
Large providers serve vector/tile windows rather than allocating a global fine raster. Free-space Hybrid A* continues
to work when no road graph is present.

## Verification gates

- exact package round trip, including unknown occupancy, NaN elevation, surface IDs, and semantic feature IDs;
- checksum, truncated-file, unsupported-version, and path-escape rejection;
- world/cell conversion at rotated and negative origins and all map boundaries;
- monotonic revision and publication-time rejection;
- authored ramp and rough-relief export use the same authoritative height function as the Bullet mesh at every
 tested cell centre;
- SLAM 2D snapshots remain valid with all 2.5D layers absent;
- new static revisions trigger replan while local dynamic clearing never changes the static revision;
- documented bridge tests state whether ROS image/YAML, OccupancyGrid, point cloud, or HD-map conversions are
  lossless or lossy.

## Public references

- [ROS `OccupancyGrid`](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/OccupancyGrid.msg)
  defines row-major x-fastest exchange and an explicit unknown value.
- [ROS `MapMetaData`](https://github.com/ros2/common_interfaces/blob/rolling/nav_msgs/msg/MapMetaData.msg)
  defines resolution, cell dimensions, and the pose of the bottom-left corner of cell `(0,0)`.
- [Nav2 Map Server](https://github.com/ros-navigation/navigation2/tree/main/nav2_map_server) separates map server,
  saver, and replaceable map I/O responsibilities.
- [Nav2 Costmap 2D](https://github.com/ros-navigation/navigation2/tree/main/nav2_costmap_2d) composes static,
  observation, filter, and inflation layers at runtime.
- [ANYbotics grid_map](https://github.com/ANYbotics/grid_map) provides a multi-layer 2.5D model for elevation,
  variance, friction, surface normals, and traversability with bridges to occupancy grids and point clouds.
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox) distinguishes exported map images from serialized,
  resumable pose-graph data and supports continuing/lifelong map revisions.
- [Autoware map architecture](https://autowarefoundation.github.io/autoware-documentation/main/design/autoware-architecture/map/)
  separates point-cloud and vector semantic maps for road autonomy.
