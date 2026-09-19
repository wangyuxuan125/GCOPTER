# DAC-SFC 论文视频：真实计算数据的 RViz 回放

视频分为两个镜头：在 `dac_sfc_video.launch` 的固定 Orbit 视角展示方法，然后用原有
`global_planning.launch` 的 ThirdPersonFollower 录飞行镜头。屏幕固定字幕在剪辑时叠加。
回放节点只绘图，不调用规划器或优化器。

## 相对原方案的修正

- 固定坐标系是本工程实际使用的 `odom`，镜头目标 `dac_sfc_focus` 由回放节点发布。
- Eigen 的特征值按**升序**排列。原始 `U.col(2)` 是 Easy、`U.col(1)` 是 Middle、
  `U.col(0)` 是 Hard；这里的 **Easy 是指 MINCO 沿此轴相对容易进行有效变形**，
  因此代码给它更大的构造域扩展预算。它不表示该轴障碍更少、运行更快或最终优化
  代价一定更低。方向预算直接取算法算出的 `extra_radii`，不使用虚构长度。
- construction domain 是按真实半宽画的旋转盒；全局地图裁切由实际 `domain_planes`
  定义。可能发生裁切，因此不要把盒子称为最终安全多面体。左手系特征向量转 RViz
  四元数前须翻转一轴，几何不变。
- 生长动画从 `0.5*|u_j·(b-a)| + overlap_radius` 开始，到此值加上
  `extra_radii[j]` 结束。**Easy 分配的额外半径更大，不意味着最终盒子的该轴
  总半宽一定更大**：长路段沿 Hard 轴的投影也算在总半宽中。回放节点在日志中
  分别打印每轴的 `extra`、基础半宽和最终半宽。
- Active-Witness 显示**真实**局部障碍、选中见证、度量投影、生成的平面及其当轮排除的
  障碍索引；后续逆序删除按真实 `removed_candidate_ids`，只有实际删面才会显示删面。
  场景中没有见证或删面时，直接略过相应动画，不虚构步骤。
- 平面采用存储的 `n·x+d=0`，从路段中点沿法向投影求绘制中心；不能直接把 `d*n`
  当作世界坐标。
- 单个路段的真实原始多面体 (`selected_raw_polytope`) 与整条路径**最终保留**的走廊
  (`retained_corridors`) 分开绘制：全局路径捷径可能略过这一原始路段，不能混称。
- 自动挑选特征值比值最大的有效路段仅是默认选取策略，并不保证那段有障碍或删面。
  想挑更有代表性的段，录好一次后检查日志中的 `segment=` 与 `witnesses=`，再传
  `video_segment_id:=整数` 重新录取。
- 视频记录在 benchmark 写入完成后调用同一确定性路段构造函数**额外重建一个路段**，
  只读取已有地图、路线、CSGN 和优化轨迹。不会重跑优化器，也不计入 benchmark
  时间；录像模式多花的墙钟时间不应用作性能指标。

## 时间线

| 时间（默认速度） | 画面 |
| --- | --- |
| 0–4 s | 地图和碰撞自由路线 |
| 4–8 s | 真实 direct-MINCO probe |
| 8–12 s | 高亮所选路段 |
| 12–18 s | Easy / Middle / Hard 双向特征向量；隐藏早期 probe |
| 18–23 s | 箭头按真实额外预算长度显示 |
| 23–30 s | 保留预算箭头，盒子只按额外半径从路段基础宽度生长 |
| 30–34 s | 域内障碍点 |
| 随后，每轮 3 s | 最多三个真实 Active-Witness 轮次：选点、投影、平面与排除集合 |
| 随后，每面 3 s | 最多两次真实 reverse-delete，未逐个播放的面直接展示结果 |
| 最后 | 原始路段多面体、最终保留走廊与所选最终轨迹，画面保持 |

`speed:=1.5` 可缩短回放；图中 `stage_label` 是三维参考文字，正式字幕可在后期添加。
probe 仅在 4–12 秒可见，使用青色；它可能是一条高高拱起的平滑曲线，不能当成
Easy 方向的绿色箭头。预算及盒子生长阶段始终显示绿色 Easy 与紫色 Hard 的
真实 `extra` 数字，例如 `Easy=3.00 m`、`Hard=0.75 m`。
视频 RViz 的 `GlobalMap` 使用与原 `global_planning.rviz` 完全相同的 PointCloud2
参数（不透明、0.25 m 盒体、Z 轴彩虹着色）。三个方向的三维标签放在地图上界之上，
用对应颜色的细引线指向实际方向；`map_top_z:=5.0` 与当前 mockamap 的上界一致。
引线只是定位标签的视觉辅助，不是算法产生的额外方向或约束。
标签会按与**双向箭头端点的距离**动态排列，避免固定把 Easy 放左边、Hard 放右边时
引线交叉造成“对调”的错觉。方向阶段直接显示各轴真实的 `mu`，预算阶段显示其
`extra` 半径；二者均和同色箭头来自同一特征向量索引。各向同性时没有实际
Easy/Hard 排序，视频会显示 Axis 1/2/3。
若更换地图，请传入新的上界，例如 `map_top_z:=8.0`。方向阶段不显示额外的三维
阶段标题，避免两排文字重叠。RViz 的三维文字无法像屏幕固定字幕一样在所有视角
始终可见；随意大幅旋转或缩放镜头仍可能将地图上方的标签移出画面。
关键标题和方向解释若需要从所有视角都可读，应在剪辑中叠加二维字幕。

## 录制数据（例：固定路线和地图）

先在 `~/ICRA2027/GCOPTER` 完成编译并 `source devel/setup.bash`，执行：

```bash
roslaunch gcopter global_planning.launch \
  map_seed:=42 route_seed:=3 \
  fixed_start_goal_enabled:=true \
  fixed_start_x:=6.4372711181640625 \
  fixed_start_y:=23.116613388061523 fixed_start_z:=0.5 \
  fixed_goal_x:=-4.582256317138672 \
  fixed_goal_y:=-17.233434677124023 \
  fixed_goal_z:=3.494147776291897 \
  benchmark_enabled:=true benchmark_method:=proposed \
  benchmark_variant:=csgn_active_exact \
  benchmark_route_replay_enabled:=true \
  benchmark_route_replay_file:=/home/wyx/tf_route_bank/development/mockamap/dev_mockamap_m42_r3.route \
  benchmark_route_save_enabled:=false \
  benchmark_trajectory_visualization_method:=proposed \
  video_trace_enabled:=true video_segment_id:=-1 \
  video_trace_file:=/tmp/dac_sfc_video.json \
  experiment_log_enabled:=false enable_rviz:=false enable_rqt_plot:=false
```

看到 `DAC_SFC_VIDEO_TRACE file=... segment=... witnesses=... pruned=...` 后，结束第一条
`roslaunch`，再启动以下**独立回放**：

```bash
roslaunch gcopter dac_sfc_video.launch \
  map_seed:=42 trace_file:=/tmp/dac_sfc_video.json \
  start_delay_s:=5 speed:=1.0
```

回放启动会校验 trace 的 map seed。一定要用同一次录制对应的 mockamap 配置；
该文件记录路径、probe、走廊、选中轨迹和每轮算法数据，不包含点云本体。
若需要飞行跟随镜头，仍按原先的 `global_planning.launch` 命令单独运行录制。
