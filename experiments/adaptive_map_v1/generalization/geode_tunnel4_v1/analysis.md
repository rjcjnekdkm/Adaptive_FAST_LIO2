# GEODE Tunnel4 generalization result

All nine trajectories are complete and satisfy their group invariants. A has
mean ATE 0.129898 m (sample standard deviation 0.000089 m), B has 0.132993 m
with identical trajectories, and C has 0.131982 m with identical trajectories.

C improves over internal baseline B by 0.001011 m (0.76%) but remains
0.002084 m (1.60%) worse than independent FAST-LIO2 A. It recovers 32.7% of the
A/B internal implementation gap.

C is not an empty intervention: each run has 168 single-frame-degenerate
frames, 27048 quality rejections, 4654 invalid-quality rejections and 1650
far-range rejections. Direction selection, the adaptive window, Persistent
handling and count control remain inactive.

The defensible held-out conclusion is stable cross-sensor non-regression
relative to the internal baseline with a small positive effect. Tunnel4 does
not support a claim that C outperforms independent FAST-LIO2, nor is the
0.76% difference by itself practically strong.
