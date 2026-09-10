# Tunnel2 Transient新点配额对照

发现control及control_off各run01～run03，共6份runtime.csv。每份执行本地GEODE原始gamma2GT_leica.py转换及rmse.py评分；帧尾时间戳，沿用用户约定的t_max_diff=0.1、t_offset=+0.1s（按官方脚本输入顺序作用于第二输入GT），SE(3)对齐。该offset不是宣称官方强制值。原始脚本及数据未修改。

| 组别 | run01 | run02 | run03 | 平均ATE(m) |
|---|---:|---:|---:|---:|
| control | 1.382934 | 1.382934 | 1.382934 | 1.382934 |
| control_off | 1.382934 | 1.382934 | 1.382934 | 1.382934 |

6份CSV字节级一致，SHA256均为7d41d549b6518f19be4c51caa97c64996cc4b817bc84d9a46bd060c0552bf522。记录数2574，Transient304帧，Persistent0帧；Transient新点接纳最大23，novel_rejected=0。时长259.3014s，最大CSV间隔0.59986s，位姿有限、时间严格递增、Adaptive全开启。

关键解释：记录中50点限制没有触发，因此这些记录不能支持“取消限额有收益”，也不能说明50点限制在其他场景无作用。这不是有效激发了配额差异的消融样本。

文件一致不等于断言用户复制了文件；在缺少运行参数快照/启动日志时，无法从本CSV验证两组开关状态或6次运行独立性。零样本标准差只是文件数据事实，不能作充分稳定性证据。

保留全部结果，不覆盖主矩阵、不改算法。建议下一步Tunnel5对照；运行时保存配额开关参数/日志。若开启组仍没有novel_rejected，标记为未触发配额，转向其他代表场景，不反复以该bag判断限额优劣。

复现：`MPLCONFIGDIR=/tmp/adaptive_eval_mpl /home/romi/evo_env/bin/python scripts/evaluate_geode_novel_ab.py --tunnel 2`。
