# GCOPTER 编译、运行与实验数据记录指南

本文档对应 `feat/tf-sfc-mvp` 分支。`global_planning.launch` 已支持 `corridor_method:=firi|tf_firi|tf_sfc|obb|ellipsoid_decomp`；所有方法生成的 H-polytope 都会进入同一个 GCOPTER 优化器和 RViz 可视化流程。

## 1. 建立 catkin 工作空间

`catkin_make` 应在包含 `src` 目录的 catkin 工作空间根目录执行，不是在只有 `gcopter/` 和 `map_gen/` 的仓库根目录直接执行。

首次安装：

```bash
sudo apt update
sudo apt install libompl-dev

mkdir -p $HOME/gcopter_ws/src
cd $HOME/gcopter_ws/src
git clone https://github.com/wangyuxuan125/GCOPTER.git
cd $HOME/gcopter_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

如需 Liu et al. (ICRA 2017) 基线，还要先安装 DecompUtil。推荐通过 DecompROS 的递归子模块在独立工作空间编译：

```bash
sudo apt install python3-catkin-tools ros-noetic-catkin-simple
mkdir -p $HOME/decomp_ws/src
cd $HOME/decomp_ws/src
git clone --recursive https://github.com/sikang/DecompROS.git
cd $HOME/decomp_ws
catkin config --cmake-args -DCMAKE_BUILD_TYPE=Release
catkin build
source devel/setup.bash

cd $HOME/gcopter_ws
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

编译输出出现 `gcopter: Liu/DecompUtil corridor baseline enabled` 才能运行 `ellipsoid_decomp`。以后新终端要先 source `$HOME/decomp_ws/devel/setup.bash`，再 source GCOPTER 工作空间。

如果仓库已经下载到其他目录，可以把整个仓库复制或链接到 `$HOME/gcopter_ws/src/GCOPTER`，然后回到 `$HOME/gcopter_ws` 执行 `catkin_make`。

每次新开终端：

```bash
cd $HOME/gcopter_ws
source devel/setup.bash
```

## 2. 启动 FIRI 基线并保存数据

```bash
roslaunch gcopter global_planning.launch \
  map_seed:=42 \
  corridor_method:=firi \
  experiment_tag:=gcopter_firi \
  experiment_log_directory:=$HOME/tf_sfc_results/gcopter
```

启动后，在 RViz 中连续两次使用 `2D Nav Goal` 选择起点和终点，第二次选择后触发规划。箭头方向决定相对高度，行为与原 GCOPTER 示例一致。

TF-FIRI 核心第一阶段（严格禁止回退）：

先在 GCOPTER 当前地图中交互选择并确认一组无障碍起终点，从 run CSV 复制精确坐标，再将下列占位符替换为该坐标。不要直接复用 EGO 地图的端点。

```bash
roslaunch gcopter global_planning.launch \
  map_seed:=42 \
  route_seed:=42 \
  fixed_start_goal_enabled:=true \
  fixed_start_x:=<verified_x> fixed_start_y:=<verified_y> fixed_start_z:=<verified_z> \
  fixed_goal_x:=<verified_x> fixed_goal_y:=<verified_y> fixed_goal_z:=<verified_z> \
  corridor_method:=tf_firi \
  allow_corridor_fallback:=false \
  tf_firi_max_faces:=24 \
  tf_firi_directional_width_weight:=1.0 \
  tf_firi_face_count_weight:=0.25 \
  tf_firi_candidate_pool_size:=8 \
  experiment_tag:=gcopter_tf_firi_f24_w1_fw025 \
  experiment_log_directory:=$HOME/tf_sfc_results/gcopter
```

这一阶段在原生 FIRI 内部加入沿共享 RRT 路段方向的宽度目标，并以“候选面距离（体积代理）—覆盖点数（面数代理）”选择障碍面，同时施加总面数硬上限。v10 强制为尚未加入的局部边界面保留槽位，再以剩余障碍所需的最低平均覆盖量筛选障碍面；达到硬上限时最多执行一次有完整障碍分离复核的一换一候选面交换。候选选择仍只检查固定大小的最近候选池，不会先生成完整 FIRI 再裁剪，也不会无界回溯。它针对高速方向不友好的 FIRI 失效模式，但还不是 MINCO sensitivity 版本。

TF-SFC 六面 OBB（严格禁止回退）：

```bash
roslaunch gcopter global_planning.launch \
  map_seed:=42 \
  corridor_method:=tf_sfc \
  allow_corridor_fallback:=false \
  tf_sfc_direction_mode:=1 \
  tf_sfc_max_segment_length:=1.0 \
  tf_sfc_safety_margin:=0.05 \
  tf_sfc_min_overlap_radius:=0.04 \
  experiment_tag:=gcopter_tf_sfc \
  experiment_log_directory:=$HOME/tf_sfc_results/gcopter
```

TF-SFC 会把过长 RRT 路段细分后逐段生成 OBB。如果生成失败且禁止回退，本次请求以 `tf_sfc_generation_failure` 写入 CSV，不会运行 FIRI 冒充成功。

Liu et al. / DecompUtil 椭球分解（严格禁止回退）：

```bash
roslaunch gcopter global_planning.launch \
  map_seed:=42 \
  corridor_method:=ellipsoid_decomp \
  allow_corridor_fallback:=false \
  decomp_local_bbox_forward:=0.5 \
  decomp_local_bbox_lateral:=3.0 \
  decomp_local_bbox_vertical:=3.0 \
  decomp_max_segment_length:=3.0 \
  decomp_min_overlap_radius:=0.01 \
  experiment_tag:=gcopter_liu_decomp \
  experiment_log_directory:=$HOME/tf_sfc_results/gcopter
```

该方法直接对同一条 RRT* 无碰撞路线调用 DecompUtil；若依赖未编译或分解认证失败，会记录 `ellipsoid_decomp_generation_failure`。

关闭日志：

```bash
roslaunch gcopter global_planning.launch experiment_log_enabled:=false
```

## 3. 输出文件与字段

默认输出：

```text
$HOME/tf_sfc_results/gcopter/gcopter_runs_v10.csv
$HOME/tf_sfc_results/gcopter/gcopter_corridors_v10.csv
```

每次有效的起终点规划请求写入一行，包含：

```text
experiment_tag, requested_method, method, fallback_used, status, success,
map_seed, start_x/start_y/start_z, goal_x/goal_y/goal_z,
voxel_width_m, dilate_radius_m, route_timeout_s,
max_velocity_mps, max_body_rate_radps, max_tilt_rad, min_thrust, max_thrust,
map_point_count, route_point_count,
corridor_count, total_faces, mean_faces,
path_search_ms, corridor_generation_ms,
optimizer_setup_ms, optimizer_ms, total_planning_ms,
final_cost, trajectory_piece_count,
trajectory_duration_s, trajectory_length_m,
corridor_constrained_piece_count,
corridor_penalty_cost_initial, corridor_penalty_cost_final,
max_corridor_violation_initial_m, max_corridor_violation_final_m
```

`requested_method`、`method` 和 `fallback_used` 分别记录请求方法、实际执行方法和是否回退；分段文件额外记录 TF-SFC 宽度、余量、重叠和失败原因。v10 还记录 `route_seed`、`fixed_start_goal`、`directional_radius_m`、`directional_width_weight`、`face_count_weight`、边界/障碍剩余约束数以及面交换尝试/接受标志。预算失败时可区分面数不足来自边界还是障碍分离。失败运行也保留已完成的部分走廊记录。文件使用追加模式。

快速查看：

```bash
head -n 5 $HOME/tf_sfc_results/gcopter/gcopter_runs_v10.csv
```

## 4. 与 EGO/TF-SFC 对比时的口径

- 同一规划器内部使用相同地图、障碍 seed、起终点和动力学上限；EGO 与 GCOPTER 地图不同时不得直接比较跨规划器成功率。
- 正式实验使用 Release 编译，并先完成预热运行。
- 重点比较 `corridor_generation_ms`、`total_faces/mean_faces`、`optimizer_ms`、`total_planning_ms`、`success`、轨迹时长和轨迹长度。
- mean/p95/max 必须由逐次原始记录计算，不要只保存终端平均值。
- 正式对比必须使用 `allow_corridor_fallback:=false`，并分别筛选 `method=firi`、`method=tf_firi`、`method=tf_sfc` 和 `method=ellipsoid_decomp`；失败请求也必须保留在成功率分母中。
- v10 日志保留走廊约束 piece 数、优化前后走廊惩罚和最大走廊越界量，并记录地图/路径 seed、精确起终点、地图分辨率/膨胀、关键动力学参数及 TF-FIRI 质量权重。它们用于证明 H-polytope 实际参与 GCOPTER 优化，并检查跨方法样本是否使用相同实验条件。
- GCOPTER 当前的 `success=1` 表示优化器返回有限目标值和非空轨迹；它不是飞行器实际到达目标的 mission success，也不是连续时间无碰撞证书。

论文最终实验前还需要完成的项目见 [ICRA_EXPERIMENT_READINESS.md](ICRA_EXPERIMENT_READINESS.md)。

## 5. 原始 FIRI 与 Liu et al. 的关系

两者都使用椭球几何，但不是同一算法：

- GCOPTER `convexCover`：按 `progress` 切分 RRT* 路径，用 `range` 裁剪局部点云，建立地图边界盒；`firi()` 反复生成障碍分离面并求当前多面体的最大体积内接椭球，默认 4 次；最后 `shortCut()` 根据多面体交叠删除冗余走廊。
- Liu/DecompUtil：对无碰撞折线的每条线段构造椭球，依照椭球度量选择最近障碍并添加切平面，再附加局部边界；没有 GCOPTER FIRI 的 MVIE 交替优化和 `shortCut`。

因此论文中可以将其作为独立的 `GCOPTER + EllipsoidDecomp SFC (Liu et al., ICRA 2017)` 基线。三组实验必须固定 RRT* 路线搜索参数、地图、起终点、动力学约束和失败统计口径。

## 6. 快速统计

```bash
python3 - $HOME/tf_sfc_results/gcopter/gcopter_runs_v10.csv <<'PY'
import csv, math, statistics, sys

rows = list(csv.DictReader(open(sys.argv[1], newline='')))
for tag in sorted({r['experiment_tag'] for r in rows}):
    group = [r for r in rows if r['experiment_tag'] == tag]
    values = sorted(float(r['total_planning_ms']) for r in group
                    if math.isfinite(float(r['total_planning_ms'])))
    if not values:
        continue
    p95 = values[math.ceil(0.95 * len(values)) - 1]
    success = sum(int(r['success']) for r in group) / len(group)
    print(tag, 'n=', len(group), 'success=', success,
          'mean_ms=', statistics.fmean(values), 'p95_ms=', p95,
          'max_ms=', max(values))
PY
```
