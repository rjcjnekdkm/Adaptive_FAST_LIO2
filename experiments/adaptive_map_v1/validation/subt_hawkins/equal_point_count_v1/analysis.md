# SubT Hawkins F24 count-only control analysis

Protocol: SubT evo APE translation with SE(3) Umeyama alignment, no scale,
`t_max_diff=0.1 s`, zero time offset. All runs used 1.0x playback with
`--start-offset 1.0`; backend, RViz and the adaptive window were disabled.

F retained the D group's range, residual, quality and invalid-quality filters,
disabled normal-direction bin selection on degenerate frames, and replaced it
with a predeclared cap of 24 accepted points per degenerate frame.

| Group | Run | ATE RMSE (m) | Associated poses | Degenerate frames | Degenerate points added | Count rejected | Total map added |
|---|---|---:|---:|---:|---:|---:|---:|
| F24 | run01 | 3.519416 | 1369 | 193 | 3341 | 2390 | 136241 |
| F24 | run02 | 4.153462 | 1357 | 222 | 4011 | 3035 | 147141 |
| F24 | run03 | 5.060017 | 1363 | 324 | 4817 | 3852 | 157408 |

F24 mean/median/sample-standard-deviation ATE is
`4.244298 / 4.153462 / 0.774307 m`. D is
`1.810297 / 1.985090 / 0.309971 m`. Direct paired F24-minus-D differences are
`+1.526021`, `+2.701057`, and `+3.074927 m`.

Using only GT timestamps associated in both members of each pair gives D/F24 ATE
of `1.992593/3.515646`, `1.454921/4.142927`, and `1.986574/5.058411 m`.
All three common-support comparisons retain the same direction, with a mean
F24-minus-D change of `+2.427632 m`.

Mechanism checks passed: every active frame logged target 24, no active frame
added more than 24 points, active-frame direction rejection was zero, Normal
frames never activated the count control, and all runs used schema
`map_diagnostics_v6` without zero-effective or skipped-map frames.

Limitation: the cap produced only 14.87--18.07 added points per degenerate frame,
versus 21.77--26.25 for D. Pooled degenerate-frame insertion was 27.8% lower.
Whole-run insertion was nevertheless similar in scale and actually 8.1% higher
for F24 on average (`146930` versus `135941`) because Normal frames dominate.
Thus F24 supports a directional-selection benefit over a count-only policy, but
is not a strict degenerate-frame count match. An offline cap replay over F24's
pre-cap eligible counts estimates that cap 49 would match D's pooled 23.65
degenerate points/frame; such a run must be declared as a separate calibrated F
and not substituted silently for this pilot.

## Pre-registered F49 follow-up

F49 was fixed in commit `cf34c4d` before its bag replays. It changed only the
degenerate-frame cap from 24 to 49. All mechanism checks passed: target 49 on
every active frame, no active-frame direction rejection, no activation on Normal
frames, no skipped-map or zero-effective frames, and schema `map_diagnostics_v6`.

| Group | Run | ATE RMSE (m) | Associated poses | Degenerate frames | Degenerate points added | Count rejected | Total map added |
|---|---|---:|---:|---:|---:|---:|---:|
| F49 | run01 | 15.029375 | 1364 | 208 | 6753 | 471 | 182208 |
| F49 | run02 | 1.426412 | 1363 | 198 | 4808 | 365 | 125524 |
| F49 | run03 | 1.652329 | 1368 | 203 | 5354 | 435 | 139057 |

F49 mean/median/sample-standard-deviation ATE is
`6.036039 / 1.652329 / 7.789277 m`. Its pooled degenerate-frame insertion is
`27.775` points/frame, 17.5% above D's `23.648`; whole-run mean insertion is
`148930`, 9.6% above D's `135941`.

On common GT timestamps, D/F49 ATE is `1.988928/14.994212`,
`1.452367/1.426709`, and `1.985887/1.646736 m`. Thus F49 can match or slightly
beat D in two runs, but also produces one severe high-error outcome. D remains
tightly repeatable across all three runs.

Joint interpretation: F24, which brackets D from below in degenerate-frame point
count, is worse in all three pairs. F49 brackets D from above and exposes a
large-tail failure despite two good runs. The defensible directional-selection
claim is therefore improved repeatability and reduced bad-run risk, not universal
best-case ATE reduction. No third post-hoc cap should be tuned on this sequence.
