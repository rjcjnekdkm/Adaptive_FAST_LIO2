# SubT Hawkins Persistent total-quota ablation v1

Protocol: SubT evo APE translation with SE(3) Umeyama alignment, no scale,
`t_max_diff=0.1 s`, zero time offset. The bag was replayed at 1.0x with
`--start-offset 1.0`; backend and RViz were disabled. The earlier no-offset run is
stored separately and excluded.

P0 disables only the Persistent total insertion quota (`1.0/0/0`). P1 enables
the candidate quota (`0.3/2/5`). Adaptive Map, the sliding window, quality filter,
directional controls and all remaining parameters are identical.

| Group | Run | ATE RMSE (m) | Associated poses | Persistent frames | Persistent points added | Persistent quota rejected | Total map added |
|---|---|---:|---:|---:|---:|---:|---:|
| P0 | run01 | 7.733256 | 1362 | 126 | 2456 | 0 | 164574 |
| P0 | run02 | 2.848672 | 1370 | 88 | 1694 | 0 | 148828 |
| P0 | run03 | 5.280720 | 1364 | 42 | 2163 | 0 | 173039 |
| P1 | run01 | 10.903811 | 1278 | 44 | 220 | 3659 | 192245 |
| P1 | run02 | 10.763106 | 1268 | 88 | 440 | 8071 | 192787 |
| P1 | run03 | 5.562113 | 1358 | 19 | 95 | 1082 | 166281 |

P0 mean/median/sample-standard-deviation ATE is
`5.287549 / 5.280720 / 2.442299 m`. P1 is
`9.076343 / 10.763106 / 3.044226 m`. The mean change P1 minus P0 is
`+3.788794 m`; all three paired differences are worse (`+3.170555`,
`+7.914434`, `+0.281393 m`).

P1 exercised the intended mechanism: every Persistent frame added exactly at
most five points. However, it did not reduce total-run ATE or variability. A
common-GT-timestamp audit, removing the unequal association-support concern,
also made all three P1 runs worse by `+3.262516`, `+7.937177`, and
`+0.293354 m` respectively.

Conclusion: the `max=5` Persistent total quota is too restrictive under the
current internally aligned frontend and is rejected as a formal candidate.
P0 remains the full-window control without this extra total cap. With only three
pairs, this is a mechanism decision for the next ablation, not a population-level
significance claim.
