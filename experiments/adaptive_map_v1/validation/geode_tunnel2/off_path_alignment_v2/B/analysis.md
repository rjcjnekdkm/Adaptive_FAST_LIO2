# Tunnel2 — aligned Ours OFF, B group

## Result

All three runs completed 2611 logged scans over 260.999525 s and associated
all 618 available GT poses. Evaluation used the local GEODE
`gamma2GT_leica.py` and `rmse.py`, end-of-scan timestamps, SE(3) alignment,
`t_max_diff=0.1 s`, and the established `+0.1 s` offset on the GT input.

| Run | ATE RMSE (m) | Rows | Max CSV gap (s) | CSV SHA256 |
|---|---:|---:|---:|---|
| run01 | 0.889942 | 2611 | 0.103125 | `c8a61e3c8c3c7551513543491dd1daac5aad42f6722a20cd3f4e4bceee41d5f5` |
| run02 | 0.890638 | 2611 | 0.103125 | `fc379c2185ebf9f31a49d1af86918bdaeea0c0d8b552cedabd033b930935d9ea` |
| run03 | 0.890638 | 2611 | 0.103125 | `fc379c2185ebf9f31a49d1af86918bdaeea0c0d8b552cedabd033b930935d9ea` |

- Mean: 0.890406 m
- Median: 0.890638 m
- Sample standard deviation: 0.000402 m
- Range: 0.889942–0.890638 m

## Runtime checks

- No run was truncated or severely divergent.
- Every row reports `adaptive_map=0` and runtime schema
  `map_diagnostics_v8`.
- Invalid-quality filtering/relaxation/turn guard, directional selection,
  equal-point-count control, and the adaptive window are inactive.
- No map update is skipped.
- Adaptive-only median, MAD, condition-number, and window diagnostics remain
  zero, confirming the OFF fast path is active.
- The run01 parameter dump records `blind=2.0 m`, map maximum range `200 m`,
  Adaptive Map disabled, and the related filters disabled.
- run02 and run03 are byte-identical. They cannot count as two distinct
  numerical outputs without independent-launch provenance.
- No `commit.txt` is present, so the exact source revision cannot be recovered
  independently from the run artifacts.

## Comparison with the recorded original FAST-LIO2 runs

The existing `fastlio2_blind2/run02` records give:

| Method | run01 | run02 | run03 | Mean | Median |
|---|---:|---:|---:|---:|---:|
| Aligned Ours OFF | 0.889942 | 0.890638 | 0.890638 | 0.890406 | 0.890638 |
| Original FAST-LIO2 records | 10.766534 | 7.713387 | 2.475331 | 6.985084 | 7.713387 |

The original FAST-LIO2 trajectories begin to differ from current Ours by more
than 0.1 m at about 46.1 s. The large separation occurs near 138 s: the three
FAST-LIO2 runs then follow different drift branches, while current Ours remains
in the same low-error branch. Full-trajectory position RMS separation from
current Ours is approximately 14.06, 9.67, and 2.81 m respectively.

This comparison does **not** establish implementation equivalence on Tunnel2.
Unlike current run01, the old FAST-LIO2 repeat folder contains neither a runtime
parameter dump nor a commit identifier. Therefore configuration/source
equivalence cannot be audited, and a highly degenerate sequence can amplify a
small timing, ordering, build, or implementation difference into a different
trajectory branch. A fresh original FAST-LIO2 control with captured commit,
parameters, playback command/rate, and three independent launches is required
before assigning this gap to a remaining source-code difference.

For context, the older Ours-OFF repeats were all 5.640907 m. The present
0.890406 m mean is much lower, but this before/after comparison spans code and
logging revisions and is not a paired estimate of any single change.

