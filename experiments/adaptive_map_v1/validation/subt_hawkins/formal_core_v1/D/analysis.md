# D group result

All three runs satisfy the D-group runtime invariants. Directional selection is
enabled on every run, becomes active exactly on single-frame-degenerate frames,
and never rejects points on normal frames. Equal-count control, window updates
and Persistent quotas remain inactive.

ATE RMSE values are 4.892702, 11.156203 and 4.721581 m. The mean is
6.923495 m, the sample standard deviation is 3.666631 m, and the median is
4.892702 m. Relative to C, mean ATE increases by 163.14% and sample standard
deviation increases by 84.86%. H3 is therefore not supported on this sequence.

The direction rule is a weak intervention at the current threshold: only 39,
67 and 26 points are rejected over the complete runs, equivalent to 0.199,
0.469 and 0.128 points per degenerate frame. D/run02 also follows a substantially
different trajectory and inserts many more map points. Because mapping is a
closed-loop nonlinear process and n=3, the result supports “no demonstrated
incremental benefit” more strongly than a universal claim that direction
selection is always harmful.

run02 and run03 lack live ROS parameter dumps; this limitation is recorded
without reconstructing snapshots.
