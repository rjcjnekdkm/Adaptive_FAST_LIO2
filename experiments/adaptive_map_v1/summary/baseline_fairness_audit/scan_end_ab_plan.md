# SPMS2帧尾规则受控验证

状态：源码与运行入口已准备，尚未编译或回放，暂无新ATE。选择Word中的开发序列SPMS2，先做max/run01与last/run01一对；两组均为当前框架关闭Adaptive、窗口、后端，初始化200样本、1倍回放、RViz关闭。相同二进制、配置及其他参数；只改变mapping.scan_end_use_last_point。

false=max：保持现有最大点时间逻辑。true=last：取预处理后的最后一点时间。平均扫描周期初值0.1秒及回退规则保持本框架原设置，两组相同；这不是完整模拟外部FAST-LIO2同步器。实际帧尾驱动同步与传播，不能只修改输出时间标签。

新增CSV列追加在原有列末尾：scan_end_use_last_point、scan_last_offset_s、scan_max_offset_s、scan_end_fallback、sync_imu_samples、sync_imu_first_time、sync_imu_last_time。日志仍只覆盖执行入图的帧；初始化等早退帧需结合console.log，不把这些列声称为全程传感器日志。不要将新格式追加到旧CSV。

按Word由用户编译，在工作区执行：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
colcon build --packages-select adaptive_fast_lio2 --symlink-install
source install/setup.bash
```

第一组，终端1：

```bash
python3 scripts/run_scan_end_ab.py max run01
```

节点准备好后，在已source ROS环境的终端2执行：

```bash
ros2 bag play bag/NTU/spms_02/spms_02_ros2 --clock --rate 1.0
```

播放结束并处理完缓冲后，在终端1按Ctrl+C结束节点。确保上一组节点和bag均退出，再以同样方式运行第二组：

```bash
python3 scripts/run_scan_end_ab.py last run01
```

入口拒绝复用已存在run目录，并检查所选安装程序是否含新参数；保存二进制哈希、配置、启动快照、源码差异和文件哈希、命令及控制台日志。参数存在检查不能保证编译完全对应当前源码，两组必须使用同一次成功构建。bag仅登记路径，正式归档时仍需校验输入文件标识。

完成一对后验收：确认开关列各自恒为0/1、Adaptive为0、窗口未就绪；按相同lidar_begin_time比较实际帧尾差与IMU边界；分别按实际帧尾作NTU棱镜评估，并补共同GT支持的误差时序、覆盖/间隔/失败状态。保留全部结果，不按更小ATE选择时间规则。不更改原始基线。一次对照仅作诊断，若比较重复性再给两组同等追加run02/run03。
