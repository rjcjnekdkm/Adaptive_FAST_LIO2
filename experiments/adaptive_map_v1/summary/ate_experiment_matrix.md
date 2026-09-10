# ATE实验矩阵（单位：m）

最新[基线公平性审计](baseline_fairness_audit/findings.md)：42条来源中40条原始CSV与评分哈希一致；Tunnel2、Tunnel5开启组所引`ours_adaptive_blind2/run02/runtime.csv`当前缺失，以下对应数值仅作历史评分。当前源码仍存在关闭Adaptive后保留的工程差异，主表尚不能作为已通过公平性验收的正式结论。

主矩阵在2米设置下：Tunnel1～5的FAST-LIO2和关闭Adaptive使用blind2/run01，开启组使用ours_adaptive_blind2/run02（入图下限2米）；其他序列仍用同名目录run01。按用户指定更新，不择优。每格来源见CSV的source列。原目录参考表保留此前结果。
空单元格由status列说明；switch_mismatch表示记录开关与实验角色不符，评分仍保留在ate_run_details.csv。scored仅表示评分完成，不等于轨迹成功或完整覆盖。
GEODE：原始本地官方坐标转换脚本+rmse.py，帧尾时间，SE(3)，max_diff=0.1s，offset=+0.1s作用于第二输入GT（沿用约定，非官方强制offset）。
NTU：现有Python实现的NTU协议，未执行官方MATLAB。SubT：evo SE(3)，max_diff=0.1s，offset=0。
各组按自身关联样本评分，尚未统一时间支持；初始化、代码版本及运行波动仍需审计。不能把现有矩阵直接作为模块因果增益结论。
开启Adaptive的隧道盲区：2。

| 序列 | FAST-LIO2 | Ours no adaptive | Ours adaptive |
|---|---:|---:|---:|
| geode_offroad1 | 0.097590 | 0.077155 | 0.086774 |
| geode_offroad2 | 0.105217 | 0.102416 | 0.107786 |
| geode_offroad3 | 0.087475 | 0.087325 | 0.093716 |
| geode_tunnel1 | 0.380394 | 0.363231 | 0.370164 |
| geode_tunnel2 | 18.292805 | 5.640907 | 1.382934 |
| geode_tunnel3 | 0.278646 | 0.146799 | 0.150348 |
| geode_tunnel4 | 0.130606 | 0.134015 | 0.131181 |
| geode_tunnel5 | 1.396598 | 1.852172 | 0.447549 |
| geode_waterway_short | 0.240711 | 0.248172 | 0.367010 |
| geode_waterway_medium | 0.745051 | 0.739867 | 0.840257 |
| ntu_spms1 | 2.162181 | 5.610317 | 0.267194 |
| ntu_spms2 | 2.421478 | 4.142781 | 4.828395 |
| ntu_spms3 | 1.543863 | 1.423322 | 0.452999 |
| subt_hawkins | 6.546465 | 1.530303 | 1.459600 |

## 专项A/B实验登记

帧尾公平性专项：[SPMS3 max/last三次对照](spms3_scan_end_ab/analysis.md)已评分，均值±样本标准差分别1.154806±0.236015m、0.999368±0.535015m。last均值较低但波动更大；初始化也随同步批次改变，暂不据此切换默认规则。该实验关闭Adaptive，不与新点配额消融混合。

五个场景已汇总至[新点配额消融统一矩阵](transient_novel_ab_matrix.md)，机器可读文件为[组别矩阵](transient_novel_ab_matrix.csv)和[逐次来源](transient_novel_ab_runs.csv)。

| 序列 | 开启/关闭现存记录数 | 配额触发与状态 |
|---|---:|---|
| ntu_spms1 | 3 / 2 | 触发；关闭run02仅保留历史评分 |
| ntu_spms3 | 3 / 3 | 触发；结果不支持取消后总有收益 |
| geode_tunnel2 | 3 / 3 | 未触发；六份文件相同，独立性未核实 |
| geode_tunnel5 | 3 / 3 | 未触发；轨迹相同，独立性未核实 |
| subt_hawkins | 1 / 3 | 未观察到拒绝；开关缺运行时证据 |

专项统计不替换上方三种主方法的单次ATE。后续沿用当前关闭限额候选版时，单独标记`ours_adaptive_transient_quota_off`；现有历史`ours_adaptive`列不追认开关状态。版本口径与证据限制见统一矩阵。
