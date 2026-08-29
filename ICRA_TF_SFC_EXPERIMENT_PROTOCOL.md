# ICRA TF-SFC GCOPTER experiment protocol

GCOPTER is used as the common corridor/optimizer benchmark. It is not the
primary real-vehicle platform.

## Build

```bash
cd ~/ICRA2027/GCOPTER
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## Methods

All corridor methods feed the same `GCOPTER_PolytopeSFC` optimizer.

| `corridor_method` | Meaning |
|---|---|
| `firi` | native GCOPTER FIRI |
| `tf_firi` | native FIRI geometry with a trajectory-directional MVIE term and hard face budget |
| `ellipsoid_decomp` | Liu et al. / DecompUtil |
| `obb` | six-face trajectory-aligned OBB baseline |
| `tf_sfc` | proposed trajectory-favorable, face-bounded obstacle-plane corridor |

Proposed example:

```bash
roslaunch gcopter global_planning.launch \
  map_seed:=42 corridor_method:=tf_sfc \
  tf_sfc_direction_mode:=1 \
  tf_sfc_max_faces:=12 tf_sfc_max_obs_faces:=6 \
  allow_corridor_fallback:=false \
  experiment_tag:=proposed_pca_f12_seed42
```

TF-FIRI core-stage example (paired directly with native FIRI):

```bash
roslaunch gcopter global_planning.launch \
  map_seed:=42 corridor_method:=tf_firi \
  tf_firi_max_faces:=12 tf_firi_directional_width_weight:=1.0 \
  tf_firi_face_count_weight:=0.25 \
  allow_corridor_fallback:=false \
  experiment_tag:=tf_firi_route_direction_f12_w1_seed42
```

OBB and Liu use the same seed and optimizer:

```bash
roslaunch gcopter global_planning.launch \
  map_seed:=42 corridor_method:=obb allow_corridor_fallback:=false \
  experiment_tag:=obb_pca_seed42

roslaunch gcopter global_planning.launch \
  map_seed:=42 corridor_method:=ellipsoid_decomp \
  allow_corridor_fallback:=false experiment_tag:=liu_seed42
```

Schema v8 writes `gcopter_runs_v8.csv` and
`gcopter_corridors_v8.csv`. It records obstacle-plane count, obstacle points
considered, face-budget saturation and anchor clearance in addition to timing,
face count, overlap and optimizer diagnostics. For `tf_firi`, it additionally
records the achieved directional ellipsoid radius, its configured weight, and
the face-count/coverage weight used by the obstacle-plane selector.

The first TF-FIRI stage uses each shared RRT route segment as a velocity-direction
proxy. It directly addresses FIRI's high-velocity/perpendicular-inflation failure
mode, but it must not be described as MINCO sensitivity. A later bounded second
pass will replace this proxy with directions computed from the initialized MINCO
trajectory while retaining the same route and optimizer.

For paired runs, enable the fixed start/goal interface and use the same map and
OMPL route seeds. Do not copy EGO endpoints into GCOPTER: first select and verify
two free points in the GCOPTER map, then replay those exact logged coordinates:

```bash
fixed_start_goal_enabled:=true map_seed:=42 route_seed:=42 \
fixed_start_x:=<verified_x> fixed_start_y:=<verified_y> fixed_start_z:=<verified_z> \
fixed_goal_x:=<verified_x> fixed_goal_y:=<verified_y> fixed_goal_z:=<verified_z>
```

Schema v8 enforces `MaxFaces` during construction. The obstacle selector only
scores a fixed-size nearest-candidate pool, so work remains bounded by the face
budget instead of generating a full native-FIRI polytope first. On failure,
`unresolved_constraint_count` reports how many local boundary or obstacle
separation constraints remained when the face budget was exhausted.

Use the same 30 fixed seeds, start/goal pairs, map resolution, dilation,
dynamics and timeout for every method. Run the 6/0, 8/2, 10/4 and 12/6
face-budget ablation. A trial is valid only if fallback is false, every corridor
is valid, the route is fully covered, the final corridor violation is within
tolerance and the optimizer returns a finite trajectory.

Do not directly compare EGO and GCOPTER success rates while they use different
map generators or start/goal sets. Cross-planner tables require a shared
point-cloud/route replay dataset; until that replay path is implemented, report
within-planner results and use GCOPTER only for corridor/optimizer ablations.


## v6 logging correction

The first v5 run proved the geometry and optimization path succeeded, but its
corridor CSV retained the old column layout. Schema v6 writes the already
computed `obstacle_face_count`, `obstacle_point_count`,
`face_budget_saturated` and `anchor_clearance_radius` fields explicitly.
