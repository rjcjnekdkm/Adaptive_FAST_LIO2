# Ours 内部 FAST-LIO2 共享基础实现

版本标记：`fastlio2_internal_v1`。修改前备份：`7a05c9a`（已推送 GitHub）。
当前是**源码对齐后、待用户编译与回放验收**的状态，不能据此宣称轨迹或精度已经相同。

## 实现边界

Adaptive 开关都启动本包的 `adaptive_fastlio_mapping`，不启动、不链接、不依赖 `fast_lio` 包。
参考源是本仓库 `src/FAST_LIO`；不是以 FAST-LIO2 节点冒充 Ours baseline。

| 共享基础部分 | 本次处理 |
| --- | --- |
| 标准点云/Livox 预处理 | 将参考 preprocess 的类与算法移入本包；保留相同点字段、过滤、时间换算及输出顺序；去掉原来的首点时间重定位 |
| 帧同步 | 相同 ROS 时间转换；初始平均扫描时间 0；最后一个点决定帧末；相同少点/半均值回退与 IMU 队列边界 |
| 初始化与 IMU | 首个同步扫描跳过；移植 IMU_init、Process、UndistortPcl；N 从 1 开始，批次结束检查 N > 10；相同重力、零偏、P/Q、传播与反向补偿 |
| ESIKF | 保留本包 IKFoM；补齐 23 个收敛阈值均为 0.001；相同 SO3 运算顺序与雅可比 |
| 匹配 | 固定 5 邻域、float 5×3 QR 平面拟合、float 残差/分数及 s > 0.9；并行逐点匹配后按原索引串行收集 |
| 基础地图 | 去畸变后先裁剪再下采样；根节点为空且点数 > 5 才建图；匹配点数 < 5 跳过；相同 float cube 边界/移动/删除盒与移除点缓存回收 |
| 基础入图 | 复用最终近邻；补齐 flg_EKF_inited 门槛；近邻不足 5 时不做体素中心距离比较；先 Add_Points(true) 再 Add_Points(false) |
| 调度 | 与参考相同的 ROS clock 10ms 定时器、IMU depth=10 默认 QoS、CPU 条件下的匹配线程数 |

Adaptive 的差异限于滤波完成后的地图插入：有效点门槛、质量/范围筛选、退化窗口、方向与 novel 配额、Persistent 配额和候选排序。
这些差异会通过地图影响**后续帧**的匹配与位姿，这是算法作用路径；不允许 Adaptive 开关切换预处理、初始化或估计器实现。

Ours 自己的 CSV、退化统计、可视化、可选后端保留；诊断计算不修改 H、残差或滤波状态。
正式前端比较须关闭 backend、RViz 和不必要的地图发布，使用相同输入、参数、线程设置和回放速率。
额外诊断/发布开销仍可能影响在线消息送达，因此“同源码逻辑”不等于“异步在线输出逐位相等”。

## 参数与旧实验隔离

- 本包当前 YAML 的 imu_init_num 已改成 10；旧 100/200/400 配置可在备份或历史 run 快照中找到。
- 节点拒绝旧初始化门槛及偏离参考固定匹配/入图常量的参数，不静默回退。
- 帧末对照开关及 max/last 专用诊断字段已删除，固定采用 FAST-LIO2 的最后点时间/回退规则。旧 max/last run01–03 是旧核心诊断；历史 runner 已删除，复现实验应在独立工作区使用旧备份。
- CSV 增加 frontend_core_revision 列；默认 launch 输出改至 shared_core_v1/manual。正式每次运行必须指定新的独立 run 目录，不能复用历史输出路径。
- 旧实验矩阵和旧 run 文件未改写；它们不能直接作为新共享核心的重复实验。
- 三组已有参考配置（NTU、SubT Hawkins、GEODE gamma blind2）的传感器/滤波/地图参数有静态配对检查；这不意味着其他数据集配置自动公平。

## 验证状态与边界

已提供 `scripts/test_internal_core_alignment.py`，检查移植的完整预处理/IMU函数、同步和 float 平面/裁剪主体、IKFoM/ikd-tree 源码、初始化/主流程顺序、Adaptive 插入边界和同节点启动。
这些是**源码及启动配置契约检查**，不是 C++ 编译测试或数值回归测试。

2026-09-10 检查结果：17 项全部通过（含删除对照开关、同步函数与参考完整对照、CSV 列一致性）；修改文件的 git diff --check 通过。C++ 编译、短段回放、完整重复实验均未执行。

与参考保留的非算法接口差异：
1. 内部 Pose6D 使用私有 std::array 工作结构，不依赖参考包生成的 ROS 消息；原始 IMU 调试文件不再打开。
2. Ours 的时间回退保护同时清除 LiDAR/time 队列，参考只清 LiDAR 队列。公平回放要求单次、单调时间流；回退时的恢复不宣称等价。
3. 自动时间同步的 ROS 时间构造在参考中存在单位疑点，未把该非默认分支移入 Ours；当前配对配置均为 time_sync_en=false，使用数据集时间戳/相同手动偏移。自动同步不在本轮对齐验收范围。
4. 参考 IMU 的 acc_s_last 与 last_lidar_end_time_ 没有显式初值；本包在构造/Reset 中补零，避免读取不确定的历史状态。两个 Adaptive 模式一致使用此保护，测试将其列为显式适配；这不是对参考未定义行为的逐位复现保证。验收时仍须检查首帧传播。

### 接下来由用户编译

在工作区根目录执行（按实验方案，本轮代理没有运行 C++ 编译）：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select adaptive_fast_lio2 --executor sequential --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ROS_LOG_DIR=/tmp/ours_internal_core_tests /usr/bin/python3 scripts/test_internal_core_alignment.py
```

随后在**新目录**先做 FAST-LIO2 与 Ours-off 的短段一致性验收：
- 选择已配对的 NTU SPMS3 bag，统一配置与回放条件，分别新建 run；保存源代码 diff、二进制/config/launch 哈希与命令。
- 核实实际启动 Ours 可执行文件和 core revision；不要运行旧安装二进制后误认是新实现。
- 依次比较首个处理帧/首个建图帧、同步帧末和 IMU 数量、预处理/下采样点数、有效点、残差、轨迹和地图规模；先定位最早分歧，不只看最终 ATE。
- 若收包/首批 IMU 不一致，先解决输入调度和丢帧，再判断数值差异；不得通过调 Adaptive 参数掩盖基础偏差。
- 短段通过后重新做完整 Ours-off 与 FAST-LIO2 重复验证；再以同一新核心重跑 Adaptive 消融。历史三次 max/last 结果不能替代这一步。

## 源码来源与许可

移植源：本仓库 FAST_LIO 的 src/preprocess.h、src/preprocess.cpp、src/IMU_Processing.hpp、src/laserMapping.cpp 和 include/common_lib.h。
移植/适配日期：2026-09-10；名称、头文件依赖、内部消息存储和日志出口做本包适配，算法主体由测试对照。
参考 LICENSE 已作为 LICENSE.FAST_LIO 随本包保留，package.xml 增加对应 GPL-2.0-only 声明；原有 BSD 声明不覆盖这些上游移植代码。
