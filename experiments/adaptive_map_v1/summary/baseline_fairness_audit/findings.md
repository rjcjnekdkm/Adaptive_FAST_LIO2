# 基线公平性检查与核心消融缺口

最新[剩余差异核查](remaining_differences.md)：共同iKD-Tree/IKFoM的14个文件字节一致；部分近邻分支因无限半径KNN而属于特殊边界，不能视为普通稀疏场景已触发。SPMS3已记录22160个IMU批次内的样本数与bag一致。下文部分近邻的优先级与“可达”解释以新核查为准。

受控回放进展：[SPMS3帧尾规则max/last三次对照](../spms3_scan_end_ab/analysis.md)已完成评分；同二进制/配置，规则改变了运行结果，last均值略低但重复波动较大。console证实同步批次改变也影响初始化输入，不能把总体差异全部归给后续去畸变。

后续[输入与初始化实证核实](input_verification.md)已确认：NTU SPMS1～3首段原始数据及历史输出存在last-point与max-point帧尾规则差异，最大约15.6～19.9ms。初始化规则的不同已作离线输入推算，尚未量化其历史轨迹影响。

依据：科研计划Word修订版v2，第4.2、5、12节。检查范围为当前源码、三个代表配置对、主矩阵42条原始来源，以及现有实验目录索引。本次不修改算法、不编译、不回放，也不把当前源码追认为历史运行程序。

结论：公平性尚未通过验收。关闭组行为符合关闭Adaptive，但初始化、时间同步、入图边界和调度存在工程差异，尚未量化各差异对轨迹的贡献。不能直接将FAST-LIO2与本框架的差距归因于Adaptive；框架内B/E比較也需要同版本和参数证据。

## 已核验的记录

- 主矩阵42条引用中40条原始CSV与evaluation_v1的SHA256一致。
- Tunnel2、Tunnel5的`ours_adaptive_blind2/run02/runtime.csv`目前缺失；原矩阵对应ATE作为历史评分保留，不能标为当前原始数据可追溯。未用其他同分或同名数据替代。
- 14条Ours w/o Adaptive的CSV全部记录adaptive_map=0、window_ready=0。源码中adaptive_map=false即绕过窗口处理，所以这些日志不能证明adaptive_window参数显式为false。
- 运行目录顶层缺少配置、启动命令和有效参数快照；SPMS1关闭组有notes.md，明确run01被替换过。评分manifest不能代替运行版本证据。
- 原始记录清单、哈希、起止时间、最大间隔、开关值及现有元数据见`main_run_audit.csv`。这些是当前可追溯性检查，不等于统一GT时间支持下完成了公平对照。

## 实现与协议差异表

| 检查项 | 当前FAST-LIO2 | 当前Ours关闭组路径 | 结论与证据 |
|---|---|---|---|
| Adaptive关闭 | 独立基线，无该模块 | allow_map_insert_point立即返回true；is_current_frame_degenerate关闭路径；窗口短路 | 额外筛选确实旁路；日志支持，但历史程序版本未知 |
| 窗口与后端 | 不适用 | launch窗口默认true；应显式false。后端也须按Word显式关闭 | B组不能只依赖目录或只传adaptive_map_enable=false；记录行为和参数是两种证据 |
| IMU初始化 | IMU_Processing.hpp中N逐IMU样本递增，超过MAX_INI_COUNT=10结束 | AdaptiveImuProcess按imu_init_num计数，三个当前配置均200 | 不是“10帧对200帧”，是不同样本规则，批次边界仍影响实际结束时刻；重力/零偏初始化可不同 |
| IMU传播与去畸变 | ImuProcess::Process/UndistortPcl | 调用带kf参数的Process，执行forwardPropagateIkfom和undistortPcl | 使用同类IKFoM传播，但实现不同；本次未证明逐样本数值等价，不能误把另一个无kf重载当成主路径 |
| 帧尾时间 | 用预处理后最后一个点curvature及平均扫描时长回退 | 遍历取最大点时间后回退 | 点时间无序时可能改变IMU截取、传播与评分时间；这是实质算法输入差异 |
| 标准点云预处理 | 按雷达类型的处理分支，Ouster直接读t | 动态读取时间字段和类型，并做有限值/盲区过滤 | 同一timestamp_unit不保证输出完全一致；需要同一消息的点序、时间和数量对照 |
| 下采样与更新 | IMU→局部地图裁剪→下采样→ESIKF→发布里程计→入图 | IMU→下采样→初始化检查→裁剪→ESIKF→入图→发布 | 都先滤波后入图；调度/发布与初始化流程有差异，不可仅凭不同顺序断言误差根因 |
| 少于5点保护 | 跳过匹配/入图 | 当前也跳过匹配/入图 | 基础保护一致；旧运行仍需版本证据 |
| 有效匹配数量门槛 | 观测模型控制有效性 | adaptive_map=false时绕过scan_match_min_effective_points附加入图门槛 | 关闭组不会因该20点门槛跳过入图；开启组该门槛属于额外机制 |
| 体素入图：部分近邻 | 近邻少于NUM_MATCH_POINTS时，体素中心比较循环直接退出 | 对实际已有的1～4个近邻仍作距离比较 | 即使Adaptive关闭也可能产生不同入图集合；MapManager保留非空部分近邻，因此分支可达 |
| 体素入图：初始化条件 | 近邻分支还受flg_EKF_inited约束 | 没有相同条件 | 启动阶段可能不同，需要逐帧证据，不推断所有场景都会触发 |
| iKD-Tree接口 | 初始化Build，后续Add_Points两路 | MapManager当前也是Build/Add_Points两路 | 包装器并非每帧重建整棵树；init_map注释有旧描述，应以实现为准。未审计所有依赖文件数值等价 |
| IMU QoS与调度 | IMU深度10默认QoS；ROS clock 10ms timer | SensorDataQoS深度2000；wall timer 10ms | 回放时接收、排队及sim time调度条件不同；CSV帧数不同不能直接证明丢包原因 |
| 输出时间戳 | Odometry使用lidar_end_time | Odometry使用lidar_end_time | 发布语义均为帧尾；帧尾计算和初始化发布范围仍不同 |

关键源码位置：

- `src/FAST_LIO/src/IMU_Processing.hpp:26,156,347`；`src/adaptive_fast_lio2/src/adaptive_imu_process.cpp:192,759`。
- `src/FAST_LIO/src/laserMapping.cpp:396,438,927,940,979`；`src/adaptive_fast_lio2/src/adaptive_laserMapping.cpp:675,1030,1365,2212,2758,2824`。
- `src/adaptive_fast_lio2/src/adaptive_map_manager.cpp:54,89`；`src/FAST_LIO/src/preprocess.cpp:261`；`src/adaptive_fast_lio2/src/adaptive_preprocess.cpp:277`。

源码/配置哈希见`audited_source_hashes.json`。行号为本次检查时位置。

## 当前配置对照

`current_config_comparison.csv`比较GEODE blind2、NTU、SubT的22项键值映射，共66项。包括主题、时间同步、盲区、点筛选、时间单位、噪声、外参、体素、迭代数、地图范围。65项相同；GEODE基线未显式设置timestamp_unit，而Ours设置2。该配置为Livox分支，直接使用offset_time，缺省键本身不能判定时间缩放错误。

配置中近邻数5、平方距离阈值5、平面拟合阈值0.1、有效性评分阈值0.9的当前Ours设置与基线源码常量相符，但平面拟合精度/类型、缓存更新及依赖库实现未做逐数值等价验证。三个配置均imu_init_num=200，基线源码固定MAX_INI_COUNT=10，初始化不在上述相同参数之列。

当前YAML仅说明工作区配置；旧运行缺快照，不据此填补历史有效参数。launch会覆盖Adaptive开关，运行审计必须记录覆盖后的值。

## A～F消融缺口

| 编号 | 设计目标 | 已有证据 | 当前缺口/处理 |
|---|---|---|---|
| A | FAST-LIO2基线 | 主矩阵14条基线轨迹 | 补源码/配置与输入条件证据；保留外部参照身份 |
| B | 框架关闭Adaptive及窗口 | 14条日志adaptive_map=0、window_ready=0 | 解释上表工程差异；有效窗口/后端参数未知；不能宣称与A等价 |
| C | 仅基础质量筛选 | 历史Tunnel4/5 normal_quality_ab目录可审计 | 历史Normal筛选开关不自动等于C；当前未确认有完整独立quality-only模式与合格记录 |
| D | 单帧自适应 | 源码支持adaptive_window_enable=false时Normal/Transient路径 | 尚未确认同版本D/E比较记录；需同时固定Transient新点开关，否则一次改变两个变量 |
| E | 完整窗口与Persistent | 主矩阵已有探索结果；SubT存在Persistent | 两个主来源缺失；版本与参数待核；不能用仅Transient新点开关实验代替D/E |
| F | 近似等点数、非方向选择 | 当前检查未找到明确模式或合格结果 | 先定义同质量/体素/预算的固定非方向选择规则及种子；不得仅将方向配额关闭当作等点数控制 |

SubT Persistent总配额历史数据位于`experiments/subt_mrs_hawkins_long_corridor/persistent_quota/`，包含ours_frontend、ours_frontend_no_total_quota、moderate、strong及no_quota重复记录。该目录有runtime和TUM，但本次未发现完整启动参数/版本快照，也未完成历史协议与实际配额行为核验；状态为“历史候选待审计”，不是“从未做过”或“必须重跑”。当前SubT配置的总配额scale=0.3、min=2、max=5；源码max=0可旁路总配额。总配额与Persistent新点缩放、方向配额需分别控制。

## 下一项具体工作

先完成工程差异的逐帧复核设计，优先顺序为初始化、同一消息的预处理/帧尾时间、1～4近邻入图分支，再检查QoS和调度。目标是解释A/B差异，不能按ATE更小择优修改基线。需要实现修改时另行明确单一改变；Word指定构建编译由用户执行并记录版本。

并行可审计已有SubT总配额与历史Normal质量筛选记录，确定哪些可重评分复用。公平性未解释前，不直接冻结前端、不新增F或Guard、不启动完整矩阵回放。

复现本次数据/配置审计：`/home/romi/evo_env/bin/python scripts/audit_baseline_fairness.py`。手工源码差异解释见本文。已有主表数值不被本次审计替换。
