# E group result

All three runs satisfy the E-group runtime invariants. The window updates on
every logged frame, becomes ready after initialization, and enters Persistent
mode for 123, 151 and 136 frames. Persistent direction tightening has a real
insertion effect: 392, 572 and 159 direction rejections occur in Persistent
frames. Equal-count control and the Persistent total insertion cap remain off.
No novel-point rejection occurs on this sequence.

ATE RMSE values are 3.211447, 1.470187 and 3.444140 m. The mean is
2.708591 m, the sample standard deviation is 1.078782 m, and the median is
3.211447 m.

Relative to D, E reduces mean ATE by 60.88% and sample standard deviation by
70.58%, supporting an incremental stability benefit from the temporal
window/Persistent policy over single-frame direction selection. Relative to C,
E's mean is 2.94% higher while its sample standard deviation is 45.61% lower.
Thus E's strongest evidence is reduced tail risk and repeatability variance,
not lower mean error than quality-only C.

run02 and run03 lack live ROS parameter dumps; this is recorded without
reconstructing snapshots. Their v7 logs directly verify window and Persistent
activity.
