# Tunnel3 入图距离下限2米重跑

本次数据位于ours_adaptive_blind2，未覆盖之前ours_adaptive记录。当前geode_gamma_blind2.yaml的preprocess.blind与adaptive_map.min_range均为2.0，CSV adaptive_map全为1；没有运行时完整参数快照，当前配置不能独立证明全部启动覆盖参数。

执行本地官方gamma2GT_leica.py与rmse.py，沿用帧尾时间、SE(3)、max_diff=0.1s、offset=+0.1s作用于第二输入GT；ATE RMSE为0.150348m。评分及源CSV哈希在evaluation_v1/manifest.json。

2504行，跨度252.000263s，最大CSV间隔1.799476s；最终位置(0.864246,1.519261,0.012134)m。

相对共同原点1706584908.5422764的132～134秒仅3条CSV记录，有效点22、20、22，入图候选合计4。CSV记录发生在map_incremental末尾，提前跳过入图的扫描不写该CSV；不能据空档断定里程计或传感器断流。

关闭Adaptive的同盲区ATE=0.146799m，上次min_range=4m的开启组ATE=4.756198m。当前结果比关闭组高0.003549m，未显示明确增益，但此前的大幅退步未在本次出现。单次重跑还不足以严格确定因果或稳定性；结果支持优先处理预处理与入图范围不一致，而不是直接删除Transient或放宽全部质量筛选。

额外有效点20门槛对应的记录空档仍存在，因此上次大幅漂移不能归因于该门槛单独作用。建议固定当前设置先重复一次、保留独立运行结果，再决定是否需要门槛消融。

该结果单列在summary/tunnel3_min_range_comparison.csv；原实验矩阵仍对应此前开启组设置，未静默替换为本次参数变体。
