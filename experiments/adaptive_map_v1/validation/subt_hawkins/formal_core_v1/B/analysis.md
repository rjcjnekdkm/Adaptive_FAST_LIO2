# B group result

All three runs satisfy the B-group runtime invariants: Adaptive Map,
directional selection, equal-point-count control and window updates remain
inactive. The trajectories cover the full 278.46 s sequence without a gap
larger than 0.5 s.

ATE RMSE values are 8.148716, 2.799133 and 9.471355 m. The mean is
6.806401 m, the sample standard deviation is 3.532845 m, and the median is
8.148716 m. This group exhibits substantial repeatability variance and a high
tail; no run is excluded as a playback failure.

run03 lacks a live ROS parameter dump. This is explicitly documented in its
provenance note rather than reconstructed. Its targeted ablation state is still
verified by the v7 runtime columns and the frozen configuration.
