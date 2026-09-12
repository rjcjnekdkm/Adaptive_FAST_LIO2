# Adaptive map 针对性消融实验方案

C3低有效点恢复模式已加入源码：保留invalid_quality筛选，仅在退化且有效点低于`adaptive_map.min_effective_points`时放宽。20点整帧门槛先执行；当前候选上界采用300。launch参数为`invalid_quality_low_effective_relax_enable:=true`，默认false以保持C1。运行日志记录模式开关、是否激活、阈值、当帧及累计放宽数量。源码由用户编译。

R1掉头保护已加入源码：在C3/C4放宽候选成立、滑动窗口就绪且`window_yaw_change > adaptive_window.max_yaw_change`时，临时恢复invalid_quality拒点；正常质量点及其他入图逻辑不变，窗口航向变化回落后自动恢复低有效点放宽。launch参数为`invalid_quality_turn_guard_enable:=true`，默认false以保持历史C3/C4。运行日志schema升级为`map_diagnostics_v5`，新增保护开关、激活状态、yaw阈值、当帧及累计保护拒点数。该模式只保护地图更新，不能回滚同帧已完成的扫描匹配位姿。

SPMS2 R1/run01～04已完成，ATE为0.356373、4.331258、0.295508、4.718521m，两次低误差、两次高误差，掉头保护没有消除多分支。四次主要保护均覆盖约171.6～175.5s；失败轨迹在保护开启后约172.5s扩大到1m以上。见[R1四次分析](spms2_r1_turn_guard_run01.md)。不继续补R1重复，下一步做R2掉头期间整帧暂停地图插入；若仍失败，再处理扫描匹配/ESIKF层。

R2整帧地图冻结功能曾用于一次诊断运行，结果见后文；用户随后决定停止该方向。对应源码、launch参数和`map_diagnostics_v6`字段已删除，当前代码恢复到R1阶段的`map_diagnostics_v5`。R2原始CSV与分析记录保留，仅作为“地图冻结期间仍会漂移”的历史机制证据，不再提供启动入口。

SPMS2 R2/run01已完成，ATE为4.555899m。171.71～175.66s连续36帧整帧不入图且窗口正常更新，但轨迹仍约在172.79s差异超过1m、174.11s超过2m。该结果否定“掉头时继续入图是漂移必要条件”；失败阶段残差约0.37～0.40m且有效点显著下降，问题已进入扫描匹配/ESIKF状态更新层。见[R2分析](spms2_r2_turn_map_freeze_run01.md)。停止追加R2，下一步先补转向角速度及滤波更新前后增量日志，再设计前端保护。

Tunnel5 C3三次已完成，ATE为0.318074、0.318074、0.112406m，均完整或基本完整且未严重发散。三次所有退化帧都低于200点，低于20点的帧先跳过，其余退化帧均激活C3；累计放行140～169点。因此本场景支持低有效点恢复放宽，但不能单独区分C3与退化期间全放行。见[Tunnel5 C3结果](tunnel5_c3_low_effective_relax.md)。下一步转入SPMS1保护性回归，再验证SPMS2和短水道，不继续追加Tunnel5同类重复。

SPMS1 C3三次已完成，ATE为0.249179、0.260867、0.238890m。三次退化帧最低有效点为321、340、340，C3激活和放行均为0，证明200点条件在该场景保持C1路径并避开全局关闭的退步。数值优于历史C1不能归因于未触发的新分支。见[SPMS1 C3结果](spms1_c3_low_effective_relax.md)。下一步为SPMS2 C3，再做短水道C3。

SPMS2 C3三次已完成，ATE为7.130784、6.000969、5.173670m，均值6.101808m，未保留历史C2均值4.373589m的改善。200点条件只激活1、4、1帧，仍有540～1512个退化帧沿用C1拒点，说明根据最低有效点设定200不能覆盖该场景的主要退化阶段。见[SPMS2 C3结果](spms2_c3_low_effective_relax.md)。先完成短水道C3，再决定是否预注册300点阈值消融；不直接按单场景把阈值提高到接近SPMS1最低值的320。

短水道C3三次已完成，ATE为0.323075、0.323087、0.323087m，均值与历史C2同为0.323083m。三次全部退化帧有效点均在27～199，C3覆盖所有2763～2764个退化帧并复现C2约1.75%的改善。见[短水道C3结果](waterway_short_c3_low_effective_relax.md)。四场景C3首轮完成：200点条件保护SPMS1、覆盖Tunnel5和短水道，但未覆盖SPMS2主要退化阶段。下一步将300点登记为C4，仅先跑SPMS2三次；若失败，不继续逼近SPMS1最低321点。

SPMS2 C4已完成六次，ATE为4.739236、0.312004、4.416085、0.333847、3.092730、4.284376m；均值2.863046m、中位数3.688553m，但表现为明显多分支。源码复核确认C4把`adaptive_map.min_effective_points`改为300时，同时改变了静态退化判定、窗口状态和其他adaptive规则，并非只改变invalid_quality放宽阈值。见[C4六次分析与设计纠正](spms2_c4_threshold300.md)。用户决定接受这一耦合语义，以300作为候选最终设置，不再拆分C5。后续表述统一为“300点退化判定与恢复策略”，并直接转入跨场景回归。

SPMS2 C4三次已完成，阈值300使激活帧由C3的1～4帧增加到77、247、82帧；ATE为4.739236、0.312004、4.416085m。run02低误差评分覆盖完整且关联样本正常，但三次呈明显双模态，IMU同步批次在约3s起已不同，不能仅按均值认定稳定改善。见[SPMS2 C4结果](spms2_c4_threshold300.md)。不继续提高阈值；保持相同条件补run04～run06，若仍双模态则转向输入同步控制和状态条件。

SPMS1 C1/C2各三次已完成；关闭invalid_quality后三次ATE均恶化，均值由0.804742m升至1.721870m。见[SPMS1 C1/C2对照](spms1_c1_c2_comparison.md)。全局off候选淘汰，后续改为仅在有效点低于200的退化恢复阶段放宽，并重新做跨场景消融。

W1首跑在取消adaptive入图量程后约16.5s即因明显漂移停止，不再补重复。见[短水道W1失败记录](waterway_short_w1_failure.md)。该候选淘汰，保留200m最大入图距离。

短水道已完成C1/C2各三次；关闭invalid_quality后ATE均值由0.328839m降至0.323083m，约改善1.75%，三次方向一致，但未解决相对历史OFF的剩余退步。见[短水道C1/C2对照](waterway_short_c1_c2_comparison.md)。下一项W1仍按单变量原则基于C1隔离adaptive入图量程。

SPMS2已完成用户命名C1（开启invalid_quality）与C2（关闭）各三次；C2均值ATE较C1低约14.7%，但三次中一次略差且同步批次不完全一致。见[SPMS2 C1/C2对照](spms2_c1_c2_comparison.md)。

目录命名更新：用户现有 `T3/run01..04` 实际是重新开启invalid_quality的对照组，不是下表原定novel限额实验。见[T2/T3复核结果](tunnel5_t2_t3_comparison.md)。下表的原定T3仅表示计划项，后续若执行novel实验必须使用另一个明确的新目录。

T2 开关已实现：launch 参数 `invalid_quality_filter_enable` 默认 true，对应 ROS 参数 `adaptive_map.invalid_quality_filter_enable`。T2 使用 false，基于 `geode_gamma_blind2.yaml`，有效点整帧门槛保持20。当前 CSV schema 为 map_diagnostics_v3，追加 `invalid_quality_filter_enabled` 字段记录开关；默认开启行为等同原规则。关闭该项不会将有近邻的无效约束点归入 novel 分支，后续 Persistent 总配额仍有效。下方其他待实现开关状态不变。

目标：识别 Tunnel5 失稳的触发与放大环节、SPMS2 和短水道退步来源。此阶段以诊断因果和运行稳定性为目标，不做全局参数搜索。采用本轮新增 map_diagnostics_v2 日志。同一阶段所有组必须使用同一构建版本，旧 CSV 仅作历史参考。

## 0. 日志验收与配置冻结

新增字段：log_sequence（每条日志递增），map_update_skipped，map_skip_reason，window_updated，range_near_rejected，range_far_rejected，map_min_range，map_max_range，map_min_effective_points，runtime_schema_revision。

frame 保留原地图更新计数，跳过入图时可重复；按 lidar_end_time 对齐、按 log_sequence 检查记录连续性，不能再把 frame 当作扫描序号。有效点不足的记录 map_added=0、各候选拒点=0、累计入图不变；这表示整帧未执行候选筛选，不表示没有异常点。window_updated=0 时窗口字段是保留的历史状态，而 degenerate 是本帧静态检测值。OFF effective_ratio 现在记录实际有效点比例；OFF normal_eigen_ratio/degenerate 仍不用于几何对照。

本次覆盖已完成估计、进入 map_incremental 的低有效点跳过分支。初始化、空去畸变点云、下采样少于5点等更早返回仍不由此 CSV 覆盖。另录 /Odometry（实际话题以启动配置为准）和播放器/节点日志，用于区分跳过记录、进程退出、播放结束。不得宣称 CSV 已覆盖全部原始扫描。

每次运行后检查：

1. 表头含 runtime_schema_revision=map_diagnostics_v2 对应字段；无坏行，lidar_end_time 严格递增。
2. 正常入图行：total_rejected = range_near_rejected + range_far_rejected + quality_rejected + invalid_quality_rejected + direction_rejected + persistent_quota_rejected + novel_rejected。voxel_rejected 不含在 total_rejected 内。
3. 跳过行：map_skip_reason=low_effective_points，effective_points 小于记录的 map_min_effective_points，map_added=0、window_updated=0；累计入图不变。
4. ON/OFF 的 frame 含义和旧文件一致；log_sequence 包含跳过行。schema 不同的文件不得追加混写，应使用新路径。

保存源码/可执行文件 SHA256、git diff、完整运行参数导出、启动指令、bag 身份与哈希、播放速率、线程数、CPU 状态及终止原因。三场景各自沿用此前固定参数，Tunnel5 blind=2；不要用不同 blind 混做消融。CSV 新增量程字段记录的是 adaptive 入图范围，不是预处理 blind，后者仍需参数快照证明。

## 1. 同版本对照，先排除记录偏差

三个场景各做 C0=adaptive OFF、C1=当前 adaptive ON，各3次，共18次。按 C0/C1、C1/C0、C0/C1 交错运行，避免组别与运行顺序绑定。每次重启节点、清空地图，保持相同初始化和完整 bag 起止范围。先完成每组首跑并验收日志，再补剩余重复；无效运行保留并注明原因后补跑。

若 SPMS2 同步差异仍类似现有记录的数量级，先暂停其算法归因，处理重放与订阅完整性。可先统一降低播放速率再重建两组对照，不能仅降低其中一组。现有 IMU 数量和首尾时间相同仍不足以证明中间样本完全相同，严格核验需补充完整输入序列记录/哈希；当前日志没有这项功能。

## 2. 单项消融组

每一组均只相对于 C1 改动表中指定项，不叠加其他消融。每组3次，共21次；每项先看首跑的触发统计，再补重复。尚无独立开关的项目需先实施开关，并在所有组共同使用的构建中保持默认行为等同 C1。

| 组 | 场景 | 唯一改动 | 验证假设 | 当前可执行性 |
|---|---|---|---|---|
| T1 | Tunnel5 | mapping.scan_match_min_effective_points=0，解除低有效点整帧入图保护 | 保护是否妨碍低约束恢复 | 已有参数；其余筛选和判退化阈值保持不变 |
| T2 | Tunnel5 | 仅禁用有局部近邻但没有有效约束时的 invalid_quality 拒点 | 是否过早阻断地图恢复 | 已实现 invalid_quality_filter_enable:=false |
| T3 | Tunnel5 | transient_novel_quota_enable=true，恢复 Transient 的50个新点限额 | 是否限制失稳后的错误地图扩张或反而妨碍恢复 | 已有 launch 参数；Persistent 不变 |
| W1 | 短水道 | 仅绕过 adaptive min/max_range 筛选 | 额外量程过滤是否贡献主要退步 | 可用独立 YAML 将 adaptive min_range=0、max_range=1e9 近似隔离有限距离点；preprocess.blind 不变 |
| W2 | 短水道 | 从首次可执行地图增量更新起前2秒旁路 adaptive 入图筛选与少点门槛，之后恢复 C1；窗口仍按原正常调用规则更新 | 启动期筛选是否造成不利地图分叉 | 需新增按 LiDAR 时间计的 warmup 开关；2秒预先固定，不逐次寻优 |
| S1 | SPMS2 | 仅禁用 has_quality 分支中的 residual_limit 与 quality_score 拒点 | 质量门槛是否导致地图偏差 | 需新增独立开关 |
| S2 | SPMS2 | 仅禁用 invalid_quality 拒点 | 无有效约束拒点是否过强 | 与 T2 共用已实现开关 |

T1 不是直接优化建议，解除保护可能恶化地图；目的在于检验恢复路径。T3 如只限制发散规模而不减少发散发生，结论应是降低后果而非解决触发原因。W1 若有效，再将近距与远距分别隔离；W2 是启动期整体策略消融，若有效再拆质量/量程/无效约束贡献，不能归功于某一个子项。

本轮不优先消融方向或 Persistent 总配额：已有记录显示它们未参与 SPMS2/短水道拒点，且在 Tunnel5 初始失稳之后才拒点。不能据此排除候选排序影响，若前述消融不能解释结果，再安排“保留筛选但恢复候选原顺序”的后续试验。

## 3. 指标与判定

主表逐次列出 ATE、成功/失败/未完成、轨迹时长、与预期输出范围的覆盖差、最大记录间隔、跳过入图帧数、近/远距拒点数、质量及 invalid_quality 拒点数。保留三次原始值、均值和样本标准差；失败单列，不用删除失败后均值作为主结论。3次仅作初步稳定性证据。

沿用 GEODE 本地官方转换与 rmse.py，SE(3)，max_diff=0.1s，GT 偏移+0.1s；NTU 沿用当前 Python NTU 协议。新日志的跳过行也进入全记录 ATE，不能为了降低误差删掉。旧分数只覆盖旧记录行，不能直接把新旧变化认定为算法变化。

各组均报告自身全记录评分，并补同一时间支持的敏感性比较；共同时间比较不替代完整性和失败统计。若不完整，记录终止原因及覆盖，不通过截短所有组掩盖失败。缺失输入/记录导致的无效对照与真实算法发散分开标记。

Tunnel5 重点绘制120–155秒的位姿/GT误差、有效点、残差、map_added、novel_accepted、各拒点与跳过原因，定位失稳前后次序。SPMS2 重点150–200秒，并审计所有共同帧的 IMU 批次。短水道重点启动0–5秒、250–300秒及400秒之后，分开统计近距与远距。

只有在同版本、输入可比的重复试验中，收益伴随预期分支行为变化，才能支持对应机制。先筛出有效改动，再在 SPMS1、SPMS3、SubT 做回归，确认未丢失已有改善；回归不计入本阶段39次预算。

## 4. 目录与执行顺序

新数据建议：validation/<sequence>/targeted_ablation_v1/<C0|C1|T1|T2|T3|W1|W2|S1|S2>/run01..03/runtime.csv。同目录保存 params.yaml、launch.txt、source_hashes.txt、node.log、playback.log、termination.txt；原 ours_on/off 保留。

顺序：日志验收 → Tunnel5 C0/C1首跑 → SPMS2/短水道 C0/C1首跑与输入验收 → 补对照重复 → T1/T3/W1 → 实施并验证独立开关 → T2/W2/S1/S2 → 形成消融矩阵与候选改动 → 跨场景回归。若过程中换构建，正式比较的 C0/C1 也应使用新构建复核，不能默认构建变化没有影响。
