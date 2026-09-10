# Tunnel5 Transient新点配额对照

control和control_off各run01～run03共6份记录已评分。每份使用本地GEODE原始gamma2GT_leica.py及rmse.py；帧尾时间、SE(3)对齐，延续用户指定t_max_diff=0.1s、t_offset=+0.1s（第二输入GT）。offset不是宣称官方强制参数。原始数据/官方脚本未修改。

| 组别 | run01 | run02 | run03 | 平均ATE(m) |
|---|---:|---:|---:|---:|
| control | 0.447549 | 0.447549 | 0.447549 | 0.447549 |
| control_off | 0.447549 | 0.447549 | 0.447549 | 0.447549 |

全部2299行，时长229.800007s，最大CSV间隔0.105287s；Transient112帧、Persistent0帧。Transient新点接纳单帧最大20，novel_rejected全部为0。未触及50点上限，故无法检验取消配额是否有收益。

六份CSV时间戳、位姿及除map_size外全部数值字段完全一致。control/run02仅map_size不同，最大绝对差3616，不能仅凭该字段差异认定轨迹或入图决策不同。其余5份文件字节级一致，SHA256为72328323591f43623145cc4118d531648b222f0b19d863b2b471da58aa360cb8。零ATE标准差不能作为充分独立重复稳定性证据；CSV未记录该参数，未触发时不能验证两组开关是否不同。

与Tunnel2一样，本例只能记录为“配额未触发，两组输出轨迹相同”，不能泛化成无上限更好或配额无用。SPMS1与SPMS3结论存在场景/运行依赖，候选默认值不能视作已完成验证的最终方案。

下一步按计划做SubT长走廊配额对照，保留Persistent逻辑不变，并保存启动参数快照或日志。实验以ATE为主，核查Transient novel_rejected是否实际触发，再判断其解释力。不修改算法或覆盖主实验矩阵。

复现：`MPLCONFIGDIR=/tmp/adaptive_eval_mpl /home/romi/evo_env/bin/python scripts/evaluate_geode_novel_ab.py --tunnel 5`。
