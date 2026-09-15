# Tunnel5 — aligned Ours OFF, B group

## Result

All three runs completed 2316 logged scans over 231.500601 s, with 623 matched
GT pose pairs per run. Evaluation used
the local GEODE `gamma2GT_leica.py` and `rmse.py`, end-of-scan timestamps,
SE(3) alignment, `t_max_diff=0.1 s`, and the established `+0.1 s` offset on
the GT input.

| Run | ATE RMSE (m) | Rows | Max CSV gap (s) | CSV SHA256 |
|---|---:|---:|---:|---|
| run01 | 0.763688 | 2316 | 0.105287 | `702a4d65f6d9aa0cdc80328b4df8fe6a1ef27f072da9448ef03cf3423e2794ae` |
| run02 | 0.763688 | 2316 | 0.105287 | `702a4d65f6d9aa0cdc80328b4df8fe6a1ef27f072da9448ef03cf3423e2794ae` |
| run03 | 0.772151 | 2316 | 0.105287 | `7bd6a8b1842108f04976e3da587ba4674f1041da098a834b83d0cb1ff8f2cff1` |

- Mean: 0.766509 m
- Sample standard deviation: 0.004886 m
- Median: 0.763688 m
- Range: 0.763688–0.772151 m

## Checks

- No run was truncated or severely divergent.
- All rows report `adaptive_map=0`, runtime schema `map_diagnostics_v8`, and
  zero enabled flags for invalid-quality relaxation/turn guard, directional
  selection, and equal-point-count control.
- The run01 parameter dump confirms `blind=2.0 m`, map maximum range `200 m`,
  Adaptive Map disabled, adaptive window disabled, and all related filters
  disabled.
- Adaptive-only median, MAD, eigen-ratio, condition-number, and window fields
  remain zero, confirming the OFF fast path is active.
- run01 and run02 are byte-identical. They must not be treated as two distinct
  numerical realizations unless independent-launch provenance is supplied.
- No `commit.txt` was saved in these run folders; the exact Git revision is
  therefore not independently recoverable from the experiment artifacts.

## FAST-LIO2 comparison

The prior `fastlio2_blind2/run02` records, rescored with the same protocol,
give 0.784160, 0.769412, and 0.415565 m (mean 0.656379 m; median 0.769412 m).
The low FAST-LIO2 mean is driven by its third, lower-error trajectory branch.
The robust central values agree closely: aligned Ours OFF median 0.763688 m
versus FAST-LIO2 median 0.769412 m (difference -0.005724 m, about -0.74%).

The aligned Ours trajectories are also close to the first two FAST-LIO2
trajectories before alignment: paired full-trajectory position RMS differences
are approximately 0.046–0.051 m. This supports the conclusion that the
remaining Tunnel5 variation is dominated by the sequence's sensitive trajectory
branching rather than a systematic Ours-OFF implementation offset.

For historical context, the older Ours-OFF blind-2 repeats were 1.852172,
1.238008, and 1.852172 m (mean 1.647451 m). The current aligned baseline is
substantially closer to FAST-LIO2, but this before/after observation combines
code revisions and different runs and is not a paired causal estimate.
