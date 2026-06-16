# M3DGR Corridor02 第一轮对照实验

## 实验目的

本轮用于确认：

1. Adaptive FAST-LIO2 在关闭 `adaptive_map` 后是否与官方 FAST-LIO2 趋势接近；
2. 开启 `adaptive_map` 后，退化检测和地图点筛选是否正常工作；
3. 策略是否造成地图停止增长、点云过度稀疏或轨迹发散。

三组实验必须使用同一份 `Corridor02_ros2`，播放倍率统一为 `1.5`。

## 实验前构建

由工作空间根目录执行：

```bash
cd ~/ros2_ws
colcon build --packages-select fast_lio adaptive_fast_lio2
source install/setup.bash
```

## 每组实验的通用记录方式

每组使用三个终端。启动算法后再启动记录，最后播放 bag。

记录 `/Odometry`：

```bash
mkdir -p ~/ros2_ws/experiments/m3dgr_corridor02/results
ros2 bag record /Odometry \
  -o ~/ros2_ws/experiments/m3dgr_corridor02/results/<实验组>_odometry
```

算法终端日志建议同时保存：

```bash
<算法启动命令> 2>&1 | tee \
  ~/ros2_ws/experiments/m3dgr_corridor02/results/<实验组>.log
```

播放命令：

```bash
ros2 bag play ~/ros2_ws/bag/Corridor02_ros2 --rate 1.5
```

bag 播放结束后，使用 `Ctrl+C` 依次停止轨迹记录和算法节点。

## 实验 A：官方 FAST-LIO2

```bash
ros2 launch fast_lio mapping.launch.py \
  config_file:=/home/romi/ros2_ws/src/FAST_LIO/config/m3dgr_mid360.yaml \
  rviz:=false 2>&1 | tee \
  ~/ros2_ws/experiments/m3dgr_corridor02/results/official_fastlio2.log
```

轨迹记录目录：

```text
results/official_fastlio2_odometry
```

## 实验 B：Adaptive 代码，策略关闭

```bash
ros2 run adaptive_fast_lio2 adaptive_fastlio_mapping \
  --ros-args \
  -r __node:=adaptive_fastlio_mapping \
  --params-file ~/ros2_ws/src/adaptive_fast_lio2/config/m3dgr_mid360.yaml \
  -p adaptive_map.enable:=false 2>&1 | tee \
  ~/ros2_ws/experiments/m3dgr_corridor02/results/adaptive_off.log
```

轨迹记录目录：

```text
results/adaptive_off_odometry
```

## 实验 C：Adaptive 代码，策略开启

```bash
ros2 launch adaptive_fast_lio2 adaptive_fast_lio2.launch.py \
  config_file:=/home/romi/ros2_ws/src/adaptive_fast_lio2/config/m3dgr_mid360.yaml \
  rviz:=false 2>&1 | tee \
  ~/ros2_ws/experiments/m3dgr_corridor02/results/adaptive_on.log
```

轨迹记录目录：

```text
results/adaptive_on_odometry
```

## 第一轮观察项

每组运行完成后记录：

| 项目 | 官方 | Adaptive 关闭 | Adaptive 开启 |
|---|---:|---:|---:|
| 是否完整跑完 |  |  |  |
| 最终地图是否明显重影 |  |  |  |
| 轨迹是否发散 |  |  |  |
| 最终地图点数 `map` | 不适用 |  |  |
| 累计入图点 `total add` | 不适用 |  |  |
| 累计质量拒绝 `total q` | 不适用 |  |  |
| 累计方向拒绝 `total d` | 不适用 |  |  |
| 退化状态是否频繁切换 | 不适用 |  |  |

Adaptive 日志中重点观察：

```text
[Degeneracy]
[Runtime]
[Warning][ScanToMap]
```

如果 Adaptive 关闭组与官方结果差异很大，应先检查基础复现逻辑，不进入策略效果评价。
