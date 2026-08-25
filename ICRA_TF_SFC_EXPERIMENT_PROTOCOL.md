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

OBB and Liu use the same seed and optimizer:

```bash
roslaunch gcopter global_planning.launch \
  map_seed:=42 corridor_method:=obb allow_corridor_fallback:=false \
  experiment_tag:=obb_pca_seed42

roslaunch gcopter global_planning.launch \
  map_seed:=42 corridor_method:=ellipsoid_decomp \
  allow_corridor_fallback:=false experiment_tag:=liu_seed42
```

Schema v5 writes `gcopter_runs_v5.csv` and
`gcopter_corridors_v5.csv`. It records obstacle-plane count, obstacle points
considered, face-budget saturation and anchor clearance in addition to timing,
face count, overlap and optimizer diagnostics.

Use the same 30 fixed seeds, start/goal pairs, map resolution, dilation,
dynamics and timeout for every method. Run the 6/0, 8/2, 10/4 and 12/6
face-budget ablation. A trial is valid only if fallback is false, every corridor
is valid, the route is fully covered, the final corridor violation is within
tolerance and the optimizer returns a finite trajectory.

Do not directly compare EGO and GCOPTER success rates while they use different
map generators or start/goal sets. Cross-planner tables require a shared
point-cloud/route replay dataset; until that replay path is implemented, report
within-planner results and use GCOPTER only for corridor/optimizer ablations.
