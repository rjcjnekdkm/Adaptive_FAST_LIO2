# C group result

All three runs satisfy the C-group runtime invariants. Adaptive Map quality and
range rejection is enabled, while directional selection, equal-point-count
control and window updates remain inactive. All trajectories cover the full
278.46 s sequence without a gap larger than 0.5 s.

ATE RMSE values are 1.477357, 1.494576 and 4.921436 m. The mean is
2.631123 m, the sample standard deviation is 1.983488 m, and the median is
1.494576 m. Relative to B, the mean ATE is 61.34% lower and the sample standard
deviation is 43.85% lower. With only three repetitions, these are descriptive
effect sizes rather than a significance claim.

run03 has both the largest degenerate-frame count and the highest rejection
counts in C, alongside the group's highest ATE. D is required to determine
whether direction-aware selection reduces this remaining tail.

run02 and run03 lack live ROS parameter dumps. This is explicitly documented
instead of reconstructing snapshots; their targeted state is verified by v7
runtime columns and the frozen configuration.
