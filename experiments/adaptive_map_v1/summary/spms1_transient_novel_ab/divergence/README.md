# 数据版本不一致：暂不解释生成的比较图

本次检查发现 control_off/run02/runtime.csv 已变化，故本目录初步生成的 diagnostics.png / seconds.csv 不能用于解释此前 ATE=5.665712m 的那次实验。不要把它们作为已完成的原实验因果分析。诊断脚本已增加哈希一致性检查。

旧评分对应SHA256：b6b7917523363cf877697d72d71b307e135a2ca54f35a0bbfae2a590f9155655，3824行；Transient novel_rejected=0，单帧novel_accepted最大1797。

当前源文件SHA256：95c13c1fbcae6a74f26ce494155eba30a2372c82cf69eda334107e45ec2a4ccd，3773行；Transient novel_rejected=454533，单帧novel_accepted最大50。当前计数与保留配额行为一致，而不符合取消配额实验的预期。文件修改时间2026-09-09 14:13:34 +0800。

另外五份源文件仍与各自评分manifest一致。原评分及原导出轨迹未覆盖。请确认run02文件来源；恢复正确的取消配额日志后才能继续逐帧解释该次5.666m结果。当前实验组均值只能作为历史结果，不能视为当前磁盘六份原始文件的重新评分。
