# SPMS1 Transient 新点配额消融

最新时序核查见[配额与建图诊断](../spms_quota_mapping_audit/findings.md)。当前替换后的control/run02为3773行，Transient novel_rejected=454533；下文242230及六份原始CSV完整性的描述属于旧记录，不能用于当前版本。control_off/run02只保留历史评分，本次现存原始数据诊断为开启3次、关闭2次。

> 用户已确认覆盖文件是配额开启实验，并要求替换control/run02。新CSV已归位control/run02并重新评分；旧control/run02完整备份至control/run02_superseded_5p100。control_off/run02保留5.665712m的历史评估，但其原始CSV已被用户覆盖，汇总raw_status明确标记缺失，不用其他CSV代替。

输入：validation/ntu_spms1/transient_novel_ab/{control,control_off}/run01～run03/runtime.csv。
评分沿用 evaluate_ntu_viral.py：棱镜点转换、有效GT插值、SE(3)对齐、无尺度修正；是NTU流程的本地Python实现，不是本次执行官方MATLAB工具。没有使用GEODE时间offset。

| 组别 | run01 | run02 | run03 | 均值 ± 样本标准差(m) |
|---|---:|---:|---:|---:|
| control，保留50点配额 | 0.332451 | 5.087781 | 18.410079 | 7.943437 ± 9.371032 |
| control_off，取消Transient配额 | 2.125791 | 5.665712 | 0.544869 | 2.778790 ± 2.622130 |

本次取消配额组均值低约65.0%，最大误差较小，但并不是每次均改善；run编号不代表相同随机种子或相同接收扫描，不能据此作严格配对推断。三次样本不足以宣称统计显著或稳定解决问题。均值差受control/run03大误差影响明显；最佳单次反而来自保留配额组，不应仅选最好值。

## 数据完整性和开关证据

- 六份文件哈希均不同；adaptive_map全部为1，时间戳严格递增，位姿数值有限。
- 起止时间全部相同，时长417.592694s；各组CSV行数3798～3876，评分关联数2906～2952。相同起止点不意味着中间扫描全部接收，最大CSV间隔0.400～0.600s。
- control的Transient新点接纳单帧最大值均为50，累计novel_rejected分别44471、242230、693346。
- control_off的Transient novel_rejected均为0；单帧novel_accepted最大值分别1333、1797、774。因此行为符合开关生效，不只是目录名不同。
- 六组Persistent帧数均为0。本次结果不涉及Persistent配额效果。
- 未提供运行时参数快照/二进制哈希，仅凭CSV不能证明其他参数和运行环境完全一致。

## 异常窗口

时间相对第一条日志帧尾。control/run03在340～350s已全部为Transient，残差中位数0.271m，候选拒绝率95.0%；350～370s约95.6%～96.0%。最终ATE18.410m。

取消配额并未消除高残差：control_off/run02在340～350s残差中位数0.164m，360～370s又达0.210m，最终ATE5.666m。该组350～360s虽然放行166325个候选、拒绝率只有13.3%，也没有保证最终低误差。可见“放行更多点”本身不是充分条件。

这里拒绝率为total_rejected/(total_rejected+map_added)，不含voxel_rejected；map_added是提交地图接口的候选数，不是地图净增点数。残差低也不能单独证明位姿正确，错误地图内部也可能形成低残差匹配。

## 结论与下一步

结果支持“Transient固定50点新点配额可能加重地图恢复困难”的假设，但不能证明其是首次漂移的唯一根因；取消后仍然有明显波动。当前不改默认值、不覆盖原始实验矩阵，也不修改baseline。

建议先比较control_off/run02与run03首次分离窗口，核查大批新点写入之前的位姿增量、残差和输入时间间隔，以及invalid_quality拒绝；优先确定剩余问题来自错误地图写入还是输入/匹配波动。不要同步放宽质量筛选与有效点门槛。若随后需要验证泛化，再在另一条已有对照序列做同一开关对照。

所有单次评估保存在各run目录的evaluation_v1中，含manifest、棱镜轨迹、误差及图。汇总见同目录ate.csv、statistics.csv、windows.csv。复现：

```bash
MPLCONFIGDIR=/tmp/adaptive_eval_mpl /home/romi/evo_env/bin/python scripts/evaluate_transient_novel_ab.py
```
