# Tunnel4 C candidate

All three runs cover 223.50 s with 2236 finite poses and no gap above 0.5 s.
The runtime CSVs and exported pose trajectories are each byte-identical across
the three restarts.

C is behaviorally active: 168 frames are classified as single-frame
degenerate, with 27048 quality rejections, 4654 invalid-quality rejections and
1650 far-range rejections per run. Direction selection, adaptive window,
Persistent handling, novel quota and equal-count control remain inactive.

The local official GEODE flow gives ATE RMSE 0.131982 m for every run. B gives
0.132993 m, so C improves by only 0.001011 m (0.76%). This is evidence of stable
cross-sensor non-regression, not a practically meaningful accuracy gain by
itself. Independent FAST-LIO2 A remains required for the final comparison.
