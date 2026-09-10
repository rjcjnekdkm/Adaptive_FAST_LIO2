# ours_no_adaptive round 1 evaluation

GEODE: original local conversion/rmse.py; end stamp; offset +0.1 s applied to GT by official input order.
NTU: existing local Python implementation, not official MATLAB execution. SubT: evo max_diff 0.1 s; 0.2 s sensitivity log retained.
Single runs, no significance claim. Runtime code/config snapshots were not captured; manifests identify this limitation.
Trajectory gaps and NTU start/end coverage remain audit items. ATE is computed over associated samples, not proof of full-run success.

| Sequence | ATE RMSE (m) | Rows | Max gap (s) |
|---|---:|---:|---:|
| geode_offroad1 | 0.077155 | 4028 | 0.105 |
| geode_offroad2 | 0.102416 | 4194 | 0.106 |
| geode_offroad3 | 0.087325 | 3202 | 0.108 |
| geode_tunnel1 | 0.372604 | 2064 | 0.381 |
| geode_tunnel2 | 38.004938 | 2575 | 1.100 |
| geode_tunnel3 | 9.624905 | 2506 | 0.657 |
| geode_tunnel4 | 0.129225 | 2218 | 0.105 |
| geode_tunnel5 | 6.931448 | 2290 | 0.417 |
| geode_waterway_short | 0.248172 | 4733 | 0.121 |
| geode_waterway_medium | 0.739867 | 8360 | 0.121 |
| ntu_spms1 | 5.610317 | 3617 | 0.700 |
| ntu_spms2 | 4.142781 | 3523 | 0.300 |
| ntu_spms3 | 1.423322 | 3611 | 0.500 |
| subt_hawkins | 1.530303 | 2540 | 0.403 |
