# SPMS2 bounded map-only transition guard v1

## Integrity and state-machine behavior

- Runtime rows: 3505
- Received / synchronized scans: 3510 / 3509
- Scan-to-map update failures: 0
- Guard triggers: 25
- Guard-active frames: 125
- Every activation contains exactly five frames
- No continuous re-triggering or unbounded pause occurs
- Normal / Transient / Persistent frames: 2937 / 568 / 0

The refactored state machine behaves as designed and removes the previous 36-second lock-up failure.

## NTU VIRAL evaluation

| Metric | Guard off | Old weighted guard (successful run) | Bounded map-only v1 |
|---|---:|---:|---:|
| ATE RMSE (m) | 5.384868 | 0.531742 | 5.204858 |
| U-turn RMSE, relative time 200--220 s (m) | 4.672791 | 1.636729 | 4.340868 |
| Outside-U-turn RMSE (m) | 5.438645 | 0.287450 | 5.269857 |
| Maximum error (m) | 8.810746 | 3.843715 | 8.825262 |
| Final error (m) | 7.771455 | 0.061850 | 6.879392 |

Map-only v1 reduces ATE by only 3.34% relative to guard-off and does not recover after the U-turn. Therefore pausing map insertion alone is insufficient. The old successful guard indicates that limiting the erroneous LiDAR correction contributed most of the recovery, but its in-iteration implementation was unstable.

## Next design

Retain the bounded and re-armed state machine, but apply measurement-noise inflation outside the observation callback. A nominal update can first produce current-frame diagnostics; if a new transition is detected, restore the pre-update state/covariance and repeat the update once with a fixed larger measurement noise. The trigger decision must remain fixed during the retry. Carry-over protection frames may use the same fixed noise directly. This avoids the old residual-weight circular feedback while preserving the useful bounded LiDAR correction.
