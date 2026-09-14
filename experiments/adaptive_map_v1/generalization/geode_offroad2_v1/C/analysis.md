# Offroad2 C candidate

All three runs cover 421.10 s with 4212 finite poses and no gap above 0.5 s.
The exported trajectories differ only slightly in run01 and give the same
official score.

C is active in each run: 268 frames are single-frame degenerate, approximately
87392 points are rejected by quality, 1747 by invalid-quality and 5810 by the
far-range limit. Direction selection, adaptive window, Persistent handling,
novel quota and equal-count control remain inactive.

The local official GEODE GNSS flow gives ATE RMSE 0.105584 m for every run.
Internal baseline B has mean 0.110634 m, so C improves by 0.005050 m (4.56%).
Independent FAST-LIO2 A remains required before making the final sequence-level
claim.
