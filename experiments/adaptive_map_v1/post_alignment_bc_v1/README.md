# 修正后 B/C 小规模验证矩阵

整理日期：2026-09-15。当前状态：SubT Hawkins 已完成 B/C 各三次；GEODE Tunnel5
初始 B/C 各三次后又重跑 C 四次。Tunnel5 的 C 累计 7 次中 4 次稳定、3 次完整发散，
呈现两个可重复分支；SPMS2 尚未开始。详见场景分析。

## 目的与边界

验证修正后内部 OFF 基线 B 与冻结架构 C 的精度、稳定性及入图行为。
这是开发场景上的方法筛查，不是独立测试集，不据此宣称普遍优于原版 FAST-LIO2 或创新性成立。
本轮不增加缓存、方向选择、滑动窗口、回环或后端功能。

源码基准：`6b9b863555b73003b941f1b03e5834afdc323ec0`。
本次检查 `src`、`scripts` 没有工作区修改；这不证明已安装二进制对应该 commit。
后续每次运行仍需记录实际代码、构建来源和配置，不能把本文件的基准追填成历史运行 commit。

## 历史数据审计与复用判定

`historical_inventory.csv` 枚举三个 validation 目录中 122 个 `runtime[0-9]*.csv`，
统计所有数据行中的 schema 和 adaptive_map 值、行数、首尾扫描结束时间。
schema 分布：无字段 46、v2 9、v3 14、v4 12、v5 13、v6 7、v7 18、v8 3。
无字段不代表数据损坏；schema 一致也不能替代代码和构建来源证明。

| 场景 | 历史记录 | 本轮判定 |
| --- | --- | --- |
| SubT Hawkins | formal_core_v1/B 三次 commit 均为 0bf0f0211797390595e2c80cd963af86068f3e03；未找到 v8 记录 | B/C 各补三次，旧结果保留为历史探索 |
| Tunnel5 | off_path_alignment_v2/B 三次均 v8、OFF；各 2316 行，覆盖约 231.500601 s | 三次 B 待确认复用；C 补三次 |
| SPMS2 | 未找到 v8 记录；旧 C1/C2/C3/C4/R1/R2 不是本轮 C 的定义 | B/C 各补三次 |

Tunnel5 历史 B 的既有评估 ATE 为 0.763688、0.763688、0.772151 m，均值 0.766509 m。
本次没有重算 ATE。run01 有参数快照；三次均未保存 commit.txt，run02/03 缺少各自参数快照，
播放倍率、启动等待、RViz 状态和二进制来源无法从这些文件完整恢复。
run01/02 CSV 字节相同；相同输出不等于没有独立重启，但不能把它们称为不同数值实现。

处理原则：不删除、不移动、不补造历史证据。现阶段不把 Tunnel5 B 标记为已完成正式新对照。
若后续可以确认三次独立运行以及代码、构建、参数和播放协议可比，明确记录证据类型
（包括用户回忆，而非冒充运行时日志）并决定是否条件复用；无法确认则新跑 B 三次。
现有三次仍可作辅助参考，复用决定不能依据它们 ATE 好坏。

确定补跑 15 次，另有 3 次 Tunnel5 B 待复用判定；最多 18 次。
`matrix.csv` 每行对应一个计划运行，状态 planned 或 reuse_pending_provenance 不表示运行已完成。
新数据放在本目录对应 scene/group/runNN 下，不覆盖旧目录；目前不创建虚假的运行产物。

## 配置冻结

每个场景提供完整 B/C YAML 快照，不修改项目默认配置。
B/C 之间仅 `adaptive_map.enable`、`adaptive_map.invalid_quality_filter_enable` 不同；
CSV 目标由每次启动单独指定。B 同时关闭子开关以避免配置歧义；主开关关闭时必须旁路自适应逻辑。

| 项目 | SubT Hawkins | Tunnel5 | SPMS2 |
| --- | --- | --- | --- |
| 配置文件前缀 | subt_hawkins | geode_tunnel5 | ntu_spms2 |
| preprocess.blind (m) | 0.5 | 2.0 | 1.0 |
| adaptive_map.min_range (m) | 0.5 | 2.0 | 1.0 |
| adaptive_map.max_range (m) | 80 | 200 | 150 |
| adaptive_map.min_effective_points | 120 | 200 | 300 |
| adaptive_map.min_effective_ratio | 0.08 | 0.1 | 0.1 |
| adaptive_map.min_normal_eigen_ratio | 0.04 | 0.02 | 0.02 |
| mapping.scan_match_min_effective_points | 20 | 20 | 20 |
| surface/map voxel (m) | 0.5/0.5 | 0.5/0.5 | 0.5/0.5 |

上述 adaptive_map 范围只在 C 生效；B 保留相同配置值但不施加额外自适应量程拒点。
因此第一阶段评估的是 C 整体策略，不是质量分类单因素；后续机制消融需控制量程因素。
20 是 C 的整帧入图门槛，120/200/300 是单帧退化判断门槛，不能混淆。

所有组：IMU 初始化计数 10；后端关闭；窗口关闭；方向选择关闭；Transient novel 配额关闭；
等点数控制关闭；低有效点放宽和转弯保护关闭；Persistent 总预算 max=0（窗口关闭，本身不激活）。
C 保留质量/MAD残差/量程筛选、退化时 invalid_quality 分类、候选排序及整帧门槛。

来源：

- SubT：validation/subt_hawkins/formal_core_v1/config/C_quality_only.yaml。
- Tunnel5：generalization/geode_tunnel4_v1/config/BC_geode_gamma_blind2.yaml，采用相同 Gamma 设备标定；不调整 blind/range。
- SPMS2：src/adaptive_fast_lio2/config/ntu_spms.yaml，保留既定 300 门槛，显式冻结 C 架构开关。

SPMS 原 YAML 的 adaptive_window.enable=true，本轮快照显式改为 false；
原 Persistent 预算设为失活的 1.0/0/0，方向等开关也显式写入，避免依赖默认值。
这些是新实验配置整理，不是新增算法。

## 播放与启动协议

| 场景 | bag（相对项目根目录） | start offset | bag 原始时长 |
| --- | --- | --- | --- |
| SubT | bag/SubT_MRS/SubT_points_ros2 | 1.0 s | 280.002064 s |
| Tunnel5 | bag/GEODE/Tunneling_tunnel_gamma/Tunneling_tunnel5_gamma_ros2 | 0 s | 232.022195 s |
| SPMS2 | bag/NTU/spms_02/spms_02_ros2 | 0 s | 398.142875 s |

新运行统一：`--clock --rate 1.0 --delay 5.0`；SubT 另加 `--start-offset 1.0`。
delay 是发布前墙钟等待，offset 才是跳过 bag 数据，二者不可替换。
RViz=false、backend=false、use_sim_time=true；保持 YAML 中相同的点云发布设置，不在 B/C 间变化。
节点就绪、参数快照保存后才播放；每次节点与播放器均独立重启，不并行运行不同组。
建议运行次序每场景 B01/C01、C02/B02、B03/C03；不依照已看到的 ATE 调整参数。

重要：现有 `adaptive_fast_lio2.launch.py` 会用 launch 参数覆盖 YAML。
使用对应 YAML 还不够，启动必须显式传递：

| launch 参数 | B | C |
| --- | --- | --- |
| adaptive_map_enable | false | true |
| invalid_quality_filter_enable | false | true |
| adaptive_window_enable | false | false |
| transient_novel_quota_enable | false | false |
| invalid_quality_low_effective_relax_enable | false | false |
| invalid_quality_turn_guard_enable | false | false |

`config_path` 指向本目录的 config；`config_file` 选场景_B.yaml 或场景_C.yaml；
`frontend_runtime_csv_path` 必须明确指向对应 runNN/runtime.csv。
YAML 中的 UNASSIGNED 路径是占位符，不能作为有效运行目录；launch 的默认输出路径也不能使用。
开始正式执行前给出各场景完整命令；本整理阶段不启动节点。

每次至少保留：commit.txt、代码差异/状态、实际二进制路径及 SHA256、参数 dump、
完整 launch/bag 指令、启动日志、runtime.csv、是否自然结束与提前终止原因。
commit 单独不能证明已安装程序版本；尚未验证当前 install 构建来源，不要求用户现在重新编译。

## 评估与决策

沿用各数据集现有评价脚本与时间约定，记录脚本/GT/输入哈希，不按单组成绩调整对齐。
GEODE 使用 gamma2GT_leica.py 与 rmse.py，扫描结束时间、SE3 无尺度；
既有 +0.1 s GT 偏移是项目约定，不应称为官方强制设置。
NTU 使用既有官方工具链；SubT 固定既有评价流程。执行评价前核对实际 manifest 参数。

先报告完整轨迹数量、终止情况、首尾时间、匹配覆盖，再报告 ATE 的每次值、均值、中位数及样本标准差。
提前结束的低 ATE 不与完整轨迹直接比较；单独列部分轨迹指标与覆盖范围，不补成“成功”。
CSV 首尾时间可核查日志覆盖，但行数不足不能单独认定算法丢帧，需要结合输入和日志。

分析退化/掉头前后的误差增长、各类拒点/接受量与整帧跳过。
OFF 的 MAD/SVD 等零值是未计算占位，不能用于声称几何良好或与 ON 做数值对照。
目前核对的 v8 runtime.csv 不含逐帧耗时字段；不能从时间戳间隔伪造 P95/P99 处理耗时。
性能指标需另行确认可用测量手段并让 B/C 使用相同开销的记录方式；本轮不擅自增加性能代码。
地图质量需要独立参考或统一参考位姿支持，不能只凭地图对自身的残差证明正确。

若收益可重复且未明显增加失败，再做分类机制消融；若收益消失或高度不稳定，先诊断，不直接扩全 bag。
三次重复不提供强统计保证；不设置事后成功阈值，也不从本次开发场景直接推断论文可发表。
