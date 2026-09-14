# GEODE Offroad2 generalization result

All nine trajectories are complete. A has mean ATE 0.103403 m (sample standard
deviation 0.000420 m), B has 0.110634 m (0.000057 m), and C has 0.105584 m
with identical scores.

C improves over internal baseline B by 0.005050 m (4.56%) but remains
0.002181 m (2.11%) worse than independent FAST-LIO2 A. It recovers 69.84% of
the A/B internal implementation gap.

C is behaviorally active and reproducible, with quality, invalid-quality and
far-range rejection occurring in every run while direction and temporal
policies remain off. The result supports stable positive generalization
relative to the internal baseline, but not superiority to external FAST-LIO2.
