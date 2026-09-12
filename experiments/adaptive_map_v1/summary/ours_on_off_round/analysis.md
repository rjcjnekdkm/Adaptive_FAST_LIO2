# 内部基线对齐后 Ours ON/OFF 本轮评估

后续[退步诊断](regression_diagnosis.md)发现：ON 有效点少于 20 时跳过 CSV 记录，因此下文“提前结束”只表示记录提前结束，不能证明进程停止；Tunnel5 严重发散仍成立。另发现短水道存在大量未单列的量程拒点，SPMS2 同步批次有显著差异。

已补充 map_diagnostics_v2 日志（少有效点跳过行、近/远量程拒点），下一阶段见[针对性消融方案](targeted_ablation_plan.md)。新 schema 需要新运行数据，不能回填历史缺失帧。

来源为各场景 `ours_on` / `ours_off`，没有混用旧 `ours_adaptive` 或 `ours_no_adaptive`。逐次分数、开关、时间范围、源码标识和评估清单路径见 [run_details.csv](run_details.csv)，本轮矩阵见 [comparison.csv](comparison.csv)。可用 `scripts/evaluate_ours_on_off_round.py` 复现。

GEODE 使用本地官方坐标转换脚本和 rmse.py，SE(3) 对齐，max_diff=0.1s，沿用 +0.1s 作用于 GT 的时间偏移（不是官方强制值）。NTU 使用现有 Python 实现的 NTU 协议，未执行官方 MATLAB；SubT 使用 evo SE(3)，max_diff=0.1s，offset=0。每份数据均保留源文件哈希及评估产物。

| 场景 | OFF ATE (m) | ON ATE (m) |
|---|---:|---:|
| Offroad1 | 0.083145 | 0.076725 |
| Offroad2 | 0.109786 | 0.108194 |
| Offroad3 | 0.092068 | 0.092139 |
| Tunnel1 | 0.378592 | 0.374297 |
| Tunnel2 | 1.354696 ± 0.835487 | 2.092863 |
| Tunnel3 | 0.636794 ± 0.835385 | 0.147075 |
| Tunnel4 | 0.129121 | 0.144934 |
| Tunnel5 | 0.269745 ± 0.100250 | 676.056726（失败、片段评分） |
| Waterway medium | 0.738090 | 0.773940 |
| Waterway short | 0.244923 | 0.353607 |
| SPMS1 | 2.125597 | 0.249326 |
| SPMS2 | 4.439032 | 5.471411 |
| SPMS3 | 1.107944 | 0.575295 |
| SubT Hawkins | 24.483131 | 12.058145 |

ON 每场景仅一次；Tunnel2/3/5 的 OFF 使用较新的 run02 三次均值和样本标准差，其他 OFF 使用 run01。旧 run01 隧道分数也保留在逐次表中，没有择优替换。单次 ON 对三次 OFF 均值只是描述性比较，不能推断显著增益。Tunnel3 OFF 两次已分别达到 0.159087、0.149896 m，ON 0.147075 m 并非远超其最好水平。

14 份 ON 全帧 adaptive_map=1，全部有 quality_rejected 记录；23 份 OFF 全帧 adaptive_map=0，质量、方向、持续配额拒点合计均为零。全部 CSV 自报 frontend_core_revision=fastlio2_internal_v1；该标识不能替代构建哈希或运行配置快照，CSV 也不能证明当时 blind 的具体值。

Tunnel5 ON 仅 1670 帧，OFF 约 2316 帧，提前 63.9 s 结束。末端位置约 (-1896,1003,-2701) m，属于严重发散，676.06 m 仅为现有片段的评分，不能当作完整运行 ATE。位置模长约在 142.0 s 超过 100 m、151.4 s 超过 1000 m；方向拒点首次发生于 150.5 s，持续配额拒点首次发生于 167.4 s。因此不能把早期发散直接归因于后发生的配额拒点。是否人为停止或进程异常，需要运行日志确认。

Tunnel3 ON 最大记录间隔 2.0 s。SPMS1/2/3 的最大间隔分别约 0.597/0.500/0.428 s，ON/OFF 帧数存在差异。当前各组按自身关联样本评分，尚未做统一时间支持或逐帧输入一致性验证。

本轮支持 SPMS1、SPMS3、SubT 的改善趋势，同时显示 Tunnel5 失败、SPMS2 和短水道恶化。建议优先诊断 Tunnel5 发散前的质量筛选、地图插入及 IMU 同步，再以相同输入和配置重复验证。稳定后补齐 ON 重复试验，不能仅凭当前一轮宣称 adaptive_map 整体有效。
