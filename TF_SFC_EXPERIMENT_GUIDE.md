# GCOPTER 编译、运行与实验数据记录指南

本文档对应 `feat/tf-sfc-mvp` 分支。`global_planning.launch` 已支持 `corridor_method:=firi|tf_sfc`；两种方法生成的 H-polytope 都会进入同一个 GCOPTER 优化器和 RViz 可视化流程。

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

关闭日志：

```bash
roslaunch gcopter global_planning.launch experiment_log_enabled:=false
```

## 3. 输出文件与字段

默认输出：

```text
$HOME/tf_sfc_results/gcopter/gcopter_runs_v2.csv
$HOME/tf_sfc_results/gcopter/gcopter_corridors_v2.csv
```

每次有效的起终点规划请求写入一行，包含：

```text
experiment_tag, requested_method, method, fallback_used, status, success,
map_point_count, route_point_count,
corridor_count, total_faces, mean_faces,
path_search_ms, corridor_generation_ms,
optimizer_setup_ms, optimizer_ms, total_planning_ms,
final_cost, trajectory_piece_count,
trajectory_duration_s, trajectory_length_m
```

`requested_method`、`method` 和 `fallback_used` 分别记录请求方法、实际执行方法和是否回退；分段文件额外记录 TF-SFC 宽度、余量、重叠和失败原因。文件使用追加模式。

快速查看：

```bash
head -n 5 $HOME/tf_sfc_results/gcopter/gcopter_runs_v2.csv
```

## 4. 与 EGO/TF-SFC 对比时的口径

- 使用相同地图边界、障碍 seed、起终点和动力学上限。
- 正式实验使用 Release 编译，并先完成预热运行。
- 重点比较 `corridor_generation_ms`、`total_faces/mean_faces`、`optimizer_ms`、`total_planning_ms`、`success`、轨迹时长和轨迹长度。
- mean/p95/max 必须由逐次原始记录计算，不要只保存终端平均值。
- 正式对比必须使用 `allow_corridor_fallback:=false`，并分别筛选 `method=firi` 和 `method=tf_sfc`；失败请求也必须保留在成功率分母中。

## 5. 快速统计

```bash
python3 - $HOME/tf_sfc_results/gcopter/gcopter_runs_v2.csv <<'PY'
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
