# SPMS2 transition guard final validation

## Protocol

- Frontend only
- Adaptive map and adaptive window enabled
- Transition guard enabled
- Playback rate: 1.0
- NTU VIRAL evaluation: prism compensation, ground-truth interpolation with gaps below 0.1 s, and SE(3) alignment

## Integrity

- Runtime rows: 3514
- Received LiDAR messages: 3519
- Synchronized scans: 3518
- Scan-to-map update failures: 0
- Degeneracy modes (Normal / Transient / Persistent): 3229 / 285 / 0

## Transition guard

- Triggered frames: 172
- Active frames: 307
- Longest active interval: 10.599 s
- Longest interval: 1624181261.258--1624181271.857 s
- Overall map insertion ratio: 0.1330

## Accuracy

- Associated ground-truth samples: 2439
- ATE RMSE: 0.531742 m
- Mean position error: 0.320803 m
- Median position error: 0.271746 m
- Maximum position error: 3.843715 m
- Final associated position error: 0.061850 m
- Error below 0.5 m: 91.47%
- Error below 1.0 m: 97.38%
- ATE RMSE in relative time 200--220 s: 1.636729 m
- ATE RMSE outside relative time 200--220 s: 0.287450 m

The maximum error occurs during the U-turn and overlaps the longest transition-guard interval. The trajectory recovers immediately after this interval and ends with about 0.062 m position error, so this run is a transient registration loss rather than permanent divergence. The guard remains useful, but it activated more often than in the previous two v2 runs (172 triggers versus 132 and 133), which explains part of the higher ATE.

## Comparison

| Run | ATE RMSE (m) | Max error (m) |
|---|---:|---:|
| Transition guard v2 run1 | 0.347271 | 2.208813 |
| Transition guard v2 run2 | 0.334178 | 2.002530 |
| Final validation | 0.531742 | 3.843715 |
| Old Ours Frontend | 5.746227 | 9.696008 |
| Ours w/o Adaptive | 2.783591 | 5.145011 |
| FAST-LIO2 | 2.933981 | 4.561709 |

The final validation is worse than the two earlier transition-guard runs but remains substantially better than the old unprotected frontend result. A same-revision guard-off control run is still required before freezing the innovation-point-1 configuration.
