# Historical A/B reuse audit

Audit conclusion: do not reuse the historical FAST-LIO2 or Ours-off trajectories
as formal A/B repetitions. They remain useful reference observations only.

| Historical result | Runs found | ATE RMSE (m) | Formal reuse |
|---|---:|---:|---|
| `fastlio2/run01` | 1 | 6.546465 | No |
| `ours_no_adaptive/run01` | 1 | 1.530303 | No |
| `ours_off/run01` | 1 | 24.483131 | No |

Reasons:

- Each candidate has only one run, not three independent restarts.
- The evaluation manifests explicitly state that the runtime source revision was
  not captured; the current Git revision cannot be inferred retroactively.
- No matching launch command, full parameter snapshot and commit tuple exists for
  all candidates.
- The CSV schemas and row counts differ, so same-build equivalence is not established.
- The historical start-offset protocol cannot be proven consistently from the
  retained artifacts alone.

Therefore the paper-quality comparison must rerun A and B under a documented
frozen protocol. The new B/C/D configurations in this directory establish the
internal core ablation; external FAST-LIO2 A should also receive three fresh runs
with its repository revision and effective configuration recorded.
