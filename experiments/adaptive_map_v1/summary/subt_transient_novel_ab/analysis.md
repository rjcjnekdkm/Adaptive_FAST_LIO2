# SubT新点配额对照：control一次，control_off三次

关闭组三次由用户确认均关闭Transient配额；原先误存于control，现已归位control_off。开启组run01按control目录登记。原始CSV哈希及评分数值保留。

使用既定SubT评估：GT纳秒转秒，估计使用lidar_end_time，evo SE(3)无尺度对齐，t_max_diff=0.1s、offset=0。每次另保存0.2s敏感性评分日志。该评分不是GEODE脚本。原始CSV及源代码未修改。

| 组别 | run01 | run02 | run03 | 平均ATE ± 样本标准差(m) |
|---|---:|---:|---:|---:|
| control（开启限额） | 1.405533 | 缺失 | 缺失 | 1.405533（n=1） |
| control_off（关闭限额，用户确认） | 1.447292 | 1.410705 | 1.445497 | 1.434498 ± 0.020625 |

开启组run01共有151个Transient帧、125个Persistent帧；Transient novel_rejected为0，单帧novel_accepted最大8。关闭组三次Transient帧数66、89、110，Persistent帧数143、97、102；Transient novel_rejected均为0，单帧novel_accepted最大8、9、8。所有运行都远未达到50点上限。存在Persistent并不表示本次测试的Transient配额生效；Persistent分支不受该开关控制。

四个CSV哈希不同，位姿有限且时间戳严格递增。开启组记录2535行；关闭组记录2497、2477、2505行。各次起止时间一致，持续277.953249s，最大CSV间隔0.50～0.61s。开启组没有console.log，且运行计数未触发上限，因此仅能按输出目录及运行约定登记为control，无法从runtime.csv独立确认开关值。

开启组run01比关闭组均值低0.028965m，但样本数不平衡，且50点限额从未触发，不能把差值归因于配额。继续按相同设置补run02、run03只能估计普通运行波动，不能验证50点上限的作用。若目标是验证该机制，应改用能让Transient novel_accepted接近50的序列，或预先设定更低的诊断阈值后重新做成对实验。

复现：`MPLCONFIGDIR=/tmp/adaptive_eval_mpl /home/romi/evo_env/bin/python scripts/evaluate_subt_novel_ab.py`。
