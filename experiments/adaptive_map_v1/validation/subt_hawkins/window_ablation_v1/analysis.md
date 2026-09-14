# SubT Hawkins D/E window ablation v1

Protocol: SubT evo APE translation with SE(3) Umeyama alignment, no scale,
`t_max_diff=0.1 s`, zero time offset. All runs used 1.0x playback with
`--start-offset 1.0`; backend and RViz were disabled.

- D: Adaptive Map enabled, sliding window disabled. Only per-frame adaptation is active.
- E: Adaptive Map and sliding window enabled, using the previously recorded P0 runs
  without the rejected Persistent total cap. Persistent sorting and its existing
  direction/novel quota scaling remain active.

| Group | Run | ATE RMSE (m) | Associated poses | Transient frames | Persistent frames | Total map added |
|---|---|---:|---:|---:|---:|---:|
| D | run01 | 1.993395 | 1362 | 246 | 0 | 141897 |
| D | run02 | 1.452405 | 1364 | 212 | 0 | 114847 |
| D | run03 | 1.985090 | 1368 | 255 | 0 | 151078 |
| E | run01 | 7.733256 | 1362 | 94 | 126 | 164574 |
| E | run02 | 2.848672 | 1370 | 108 | 88 | 148828 |
| E | run03 | 5.280720 | 1364 | 48 | 42 | 173039 |

D mean/median/sample-standard-deviation ATE is
`1.810297 / 1.985090 / 0.309971 m`. E is
`5.287549 / 5.280720 / 2.442299 m`. The direct paired D-minus-E differences are
`-5.739861`, `-1.396267`, and `-3.295630 m`.

Using only GT timestamps associated in both members of each pair gives D/E ATE
of `1.991292/7.706633`, `1.455532/2.848384`, and `1.981948/5.280349 m`.
All three common-support comparisons therefore retain the same conclusion;
their mean D-minus-E change is `-3.468865 m`.

Conclusion: under the current internally aligned frontend, the full sliding-window
Persistent policy is not supported on SubT Hawkins. The single-frame D strategy is
both more accurate and substantially more repeatable. This comparison alone cannot
separate geometric selection from map-point-count effects, so the next required
experiment is the F equal-point-count, non-directional control.
