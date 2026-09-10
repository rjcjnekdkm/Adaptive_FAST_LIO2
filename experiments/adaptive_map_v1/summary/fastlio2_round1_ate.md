# FAST-LIO2 round 1 evaluation

GEODE: original local conversion/rmse.py; end stamp; offset +0.1 s applied to GT by official input order.
NTU: existing local Python implementation, not official MATLAB execution. SubT: evo max_diff 0.1 s; 0.2 s sensitivity log retained.
Single runs, no significance claim. Runtime code/config snapshots were not captured; manifests identify this limitation.
Trajectory gaps and NTU start/end coverage remain audit items. ATE is computed over associated samples, not proof of full-run success.

| Sequence | ATE RMSE (m) | Rows | Max gap (s) |
|---|---:|---:|---:|
| geode_offroad1 | 0.097590 | 4045 | 0.105 |
| geode_offroad2 | 0.105217 | 4212 | 0.106 |
| geode_offroad3 | 0.087475 | 3220 | 0.108 |
| geode_tunnel1 | 0.367261 | 2082 | 0.381 |
| geode_tunnel2 | 4.792301 | 2582 | 1.100 |
| geode_tunnel3 | 7.427399 | 2523 | 0.657 |
| geode_tunnel4 | 0.133886 | 2236 | 0.105 |
| geode_tunnel5 | 9.110966 | 2295 | 0.993 |
| geode_waterway_short | 0.240711 | 4750 | 0.121 |
| geode_waterway_medium | 0.745051 | 8376 | 0.121 |
| ntu_spms1 | 2.162181 | 3906 | 0.403 |
| ntu_spms2 | 2.421478 | 3403 | 0.523 |
| ntu_spms3 | 1.543863 | 3541 | 0.524 |
| subt_hawkins | 6.546465 | 2288 | 1.109 |
