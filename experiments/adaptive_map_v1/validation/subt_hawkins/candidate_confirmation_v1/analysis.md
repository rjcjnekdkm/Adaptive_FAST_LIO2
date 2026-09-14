# C/E candidate confirmation

All six trajectories are complete and satisfy their runtime invariants. C has
no window, Persistent, direction or equal-count activity. E updates the window
on every frame and enters Persistent mode in all three runs.

The confirmation C ATE values are 1.469465, 1.717086 and 2.864914 m
(mean 2.017155 m, sample standard deviation 0.744547 m). Confirmation E gives
1.401467, 10.756171 and 1.434619 m (mean 4.530752 m, sample standard deviation
5.391396 m). E preserves excellent best-case accuracy but again produces a
large failure tail.

Across the formal and confirmation rounds combined, C has six-run mean
2.324139 m, sample standard deviation 1.381491 m, median 1.605831 m and maximum
4.921436 m. E has mean 3.619672 m, sample standard deviation 3.617796 m, median
2.340817 m and maximum 10.756171 m. Using 5 m only as a transparent descriptive
success boundary, C is 6/6 and E is 5/6.

E/run02 enters Persistent for only 57 frames and inserts 161792 points. Its two
good runs enter Persistent for 151 and 152 frames and insert 113363 and 113460
points. This association does not by itself prove causality, but it shows that
the current window state machine does not reliably maintain the intended
restrictive behavior before the trajectory/map feedback diverges.

Decision: freeze C, not E, as the frontend candidate for generalization. Keep E
as a documented temporal-policy ablation and failure case. Do not tune the
SubT thresholds again from these outcomes.
