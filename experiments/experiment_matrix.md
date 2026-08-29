# 实验矩阵总览

| 数据集类别 | 序列 | FAST-LIO2 | LIO-SAM | Ours w/o Adaptive | Ours-Frontend | Ours-Full |
|---|---|---:|---:|---:|---:|---:|
| SubT | Hawkins Long Corridor | ✓ | 尽量运行 | ✓ | ✓ | ✓ |
| Offroad | 1–3 | ✓ | 尽量运行 | ✓ | ✓ | ✓ |
| Tunneling | 1–5 | ✓ | 尽量运行 | ✓ | ✓ | ✓ |
| Waterway | Short / Medium | ✓ | 尽量运行 | ✓ | ✓ | ✓ |

说明：

- `Ours w/o Adaptive`：在本工程中关闭自适应点质量过滤和滑动窗口退化策略；后端关闭。
- `Ours-Frontend`：开启自适应点质量过滤、单帧退化判断和滑动窗口持续退化状态机；后端关闭。
- `Ours-Full`：在 Ours-Frontend 基础上开启回环检测与后端位姿图优化。
- LIO-SAM 若无法正常接收 Livox 点云、初始化失败或明显发散，记录为 `×/失败`，不强行调整为不公平的设置。
- Offroad 与 Inland Waterways 使用官方 `gamma2GT_gnss.py + rmse.py`；Tunneling 使用官方 `gamma2GT_leica.py + rmse.py`。
