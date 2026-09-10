# NTU SPMS1 三次重复实验核查

检查对象：validation/ntu_spms1 下 ours_no_adaptive 和 ours_adaptive 的 run02/runtime1.csv～runtime3.csv。未修改算法、配置或 FAST-LIO2 baseline；仅新增诊断脚本与派生结果。

## 评分与采样核查

沿用项目 evaluate_ntu_viral.py 对 NTU 官方评估流程的 Python 实现：估计位置变换到棱镜测量点、有效真值插值、SE(3) 刚体对齐，不估计尺度。本次没有运行官方 MATLAB 工具，也没有套用 GEODE 的 0.1s offset。

| 组别 | runtime1 ATE(m) | runtime2 ATE(m) | runtime3 ATE(m) |
|---|---:|---:|---:|
| 关闭 Adaptive | 4.743129 | 4.934026 | 4.192112 |
| 开启 Adaptive | 1.610274 | 9.158088 | 4.740420 |

六份 CSV 哈希不同，开启标志分别符合分组；轨迹数值有限。六组共用的有效时间戳共有 1753 个，实际时间差为 0。仅在这些时间戳重新对齐评分：关闭组 3.548734、3.833234、3.313238m；开启组 1.131842、6.891913、3.410342m。因此差异并非完全由评分采样点不同造成。这个子集分数仅为诊断，不替换实验矩阵原有 ATE。

## 异常时段

下表时间相对第一条记录扫描的帧尾。针对开启组 runtime2：

| 时段(s) | 残差均值字段的中位数(m) | Transient比例 | 有效点中位数/最小值 | 入图候选拒绝率 | 放行候选数 |
|---|---:|---:|---:|---:|---:|
| 340–350 | 0.0659 | 17.6% | 560 / 250 | 66.3% | 21108 |
| 350–360 | 0.2206 | 100% | 561 / 147 | 95.9% | 13899 |
| 360–370 | 0.2404 | 100% | 612.5 / 235 | 96.7% | 9595 |
| 370–390 | 0.1957 | 95.1% | 1209.5 / 442 | 92.7% | 36291 |

拒绝率定义为 total_rejected / (total_rejected + map_added)，不包含前面的 voxel_rejected，不能称为原始点云删除率。map_added 是送入地图接口的候选数，不等于地图实际净增点数。

350–360s 拒绝总数 326533，其中 novel_rejected=196301（约60.1%）、invalid_quality_rejected=108041（约33.1%）、quality_rejected=22089、direction_rejected=99、persistent_quota_rejected=0。正常表现较好的 runtime1 同段拒绝率82.5%，放行21218个候选。

三次开启组的日志均未出现 Persistent；runtime2 为 Normal3235帧、Transient548帧。当前配置 blind 与 adaptive_map.min_range 均为1m，不存在此前 Tunnel3 那种盲区/入图最小距离不一致。

## 误差曲线的限制

全程 SE(3) 对齐会把后段漂移影响分摊到前段，因此不能把全程对齐后起点误差当作初始化错误。另用前60s的416个共用有效样本拟合刚体变换，作为诊断：runtime2在340–350s误差中位数约0.214m，350–360s约0.522m，370–390s约34.689m。后者仅有5个共用有效样本，不能解释成这20秒的连续测量。

360–370s三次开启组各自都没有满足评估规则的真值匹配点，不能确定该区间内部误差如何增长。图中使用散点，不跨真值缺口连接或补造误差。前60s对齐误差不是正式ATE，不能与9.158m混用。

## 对应源码与判断

当前 adaptive_laserMapping.cpp 的 allow_map_insert_point()：

1. 有有效约束的候选点经过残差和质量筛选。
2. 退化帧中，已有近邻但没有有效约束的点被 invalid_quality 分支拒绝。
3. 无可用近邻的新点受 max_novel_points_per_frame=50 限额，超额计入 novel_rejected。这一限额在 Transient 也生效，不仅在 Persistent。
4. ESIKF 更新发生在地图筛选之前；这些拒绝影响后续地图，不是直接删除当前帧全部测量约束。

已有日志有效点最小147，高于20门槛，因此有记录帧的主要异常不是“有效点小于20”。但日志在地图更新后写入，提前返回帧不完整可见，不能据此排除所有跳帧。当前源码与配置核查不等于对历史运行二进制/参数的完整证明。

最值得验证的假设是：匹配变差后新点与无效约束点大量被拒绝，地图补充速度下降，使匹配更难恢复。现有记录只能支持这种反馈机制的可能性，不能证明配额导致首次漂移；已有位姿偏差同样会导致近邻减少、novel计数增加。runtime3的高残差更早出现，Transient帧数更多，但最终ATE比runtime2低，说明模式次数不是唯一解释。

## 建议下一步

保留此次全部结果，不以最优值代替重复均值。先固定并保存运行参数及二进制版本，再仅针对 Transient 的 novel 配额进行单变量对照；保持 Normal质量筛选、有效点门槛、播放倍率和其他策略不变，保留所有重复结果。该试验用于检验地图扩展是否被配额限制，不能预先承诺放宽会改善（也可能写入错误点）。不要修改 baseline、初始化或引入回环来掩盖本次前端问题。

复现诊断：`MPLCONFIGDIR=/tmp/adaptive_eval_mpl /home/romi/evo_env/bin/python scripts/audit_spms1_repeats.py`。

详见同目录 metrics.json、windows.csv、common_errors.csv 和 spms1_repeats_diagnostics.png。
