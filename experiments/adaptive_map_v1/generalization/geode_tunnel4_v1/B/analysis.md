# Tunnel4 B baseline

All three runs cover 223.50 s with 2236 poses, finite outputs and no gap above
0.5 s. Adaptive Map, direction selection, window updates, equal-count control
and all adaptive rejection counters remain inactive.

The local official GEODE conversion and RMSE flow gives ATE RMSE 0.132993 m for
all three runs. Their exported pose trajectories have the same SHA256. The
run02 runtime CSV differs slightly in non-pose diagnostics, but produces the
same trajectory and evaluation result. All repetitions are retained.

Only run01 has a live ROS parameter dump. The frozen configuration, commit and
v7 runtime invariants establish the targeted B-group state for run02/run03.
