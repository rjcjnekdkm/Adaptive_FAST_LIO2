# 关闭 Adaptive 后的基础路径审计

日期：2026-09-08。只读核查当前源码与 validation/*/ours_no_adaptive/run01/runtime.csv；未修改算法、未编译。缺少运行时二进制快照，当前源码不能完全证明历史运行的构建版本。

## 结论

关闭 Adaptive Map 不等于与 FAST-LIO2 基础实现完全等价。当前不能把两者的 ATE 差异归因于 Adaptive Map。已经找到值得优先做单变量验证的稀疏扫描处理差异，但尚未证明唯一根因。

## 已确认的差异

| 项目 | FAST-LIO2 当前源码 | Adaptive 当前源码 |
|---|---|---|
| 稀疏扫描保护 | laserMapping.cpp:1020，下采样点数 <5 时返回，不执行后续匹配和入图 | adaptive_laserMapping.cpp:2848 起，地图建立后的路径没有对应 <5 返回；关闭 Adaptive 后继续入图 |
| 初始化 | IMU_Processing.hpp:26、156、357，MAX_INI_COUNT=10，N 在 IMU 遍历中递增 | adaptive_imu_process.cpp:186 起按 IMU 样本累计，本轮配置200 |
| IMU 订阅 | laserMapping.cpp:929，默认可靠队列深度10 | adaptive_laserMapping.cpp:2749，SensorDataQoS，深度2000 |
| 主定时器 | laserMapping.cpp:940，ROS clock，10ms | adaptive_laserMapping.cpp:2769，wall timer，10ms |
| IMU 积分异常间隔 | IMU_Processing.hpp 的传播路径没有同样的0.2s截断 | adaptive_imu_process.cpp:457起，dt超0.2s跳过该段，帧尾外推也有限制 |

两者主线程均使用 rclcpp::spin；不能把上述差异描述为“双线程接收”。未证明队列丢包或0.2s积分保护在本轮实际触发。

Adaptive 的 allow_map_insert_point 在 adaptive_map_enable=false 时直接返回 true（adaptive_laserMapping.cpp:1364）。关闭模式不会执行后面的质量筛选或方向配额；CSV 中这两条隧道 adaptive_map 均为0、quality_rejected总和为0。

## 与数据对应的异常

以下时间均相对各自 CSV 首条 lidar_end_time，不是 bag 起点，也不是精确的失稳起点。

| 序列 | 下采样点<5的记录数 | 其中仍有入图的记录数 | 这些稀疏记录的 map_added 合计 |
|---|---:|---:|---:|
| Tunnel2 | 8 | 5 | 11 |
| Tunnel3 | 8 | 7 | 11 |

map_added 是 CSV 的候选入图计数，不等于地图净增长或最终保留点数。

- Tunnel2：约138～143s有效点多次为0；141.990s时z=-1.036m、有效点1；148.102s时z=-5.032m。部分零约束扫描仍有入图记录。
- Tunnel3：约131.9～134.4s出现极少点/零有效点；136.200s时z=-1.049m；137.300s时z=-5.123m；143.299s首次|z|>100m。最终z约-55741.794m，原始轨迹已严重发散，不是评分坐标转换导致。
- Tunnel3在z越过-5m时有效点已经恢复至141，但这不证明匹配到了正确地图；约束数量恢复不等于全局轨迹恢复。

注意：FAST-LIO2 对“下采样点<5”有保护，但其 No Effective Points 分支并不必然阻止外层入图。不能宣称 FAST-LIO2 会拒绝所有零有效点扫描。

## 下一项受控验证

建议仅在 Adaptive 中补齐 FAST-LIO2 的下采样点<5保护，先重跑 Tunnel3 关闭 Adaptive 对照。保持初始化200、播放1倍速、RViz与其他参数不变；保留当前结果，另设诊断子目录。用户负责编译，FAST-LIO2源码不动。

该试验只回答稀疏扫描处理差异是否影响此次失稳，不能独立证明所有基础路径等价。之后再单独研究初始化差异、积分边界与预处理路径，不同时调整多项。启用 Adaptive Map 的完整矩阵应在基础路径审计后继续。
