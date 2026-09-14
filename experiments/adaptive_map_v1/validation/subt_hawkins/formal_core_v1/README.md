# SubT Hawkins formal core ablation v1

This directory defines the formal same-version core ablation. Historical A/B
results are retained as reference only; `historical_ab_audit.md` explains why
they cannot be pooled with this experiment.

## Groups

| Group | Adaptive Map | Quality/range rejection | Direction selection | Window/Persistent | Equal-count control |
|---|---:|---:|---:|---:|---:|
| B | off | off through master switch | off | off | off |
| C | on | on | off | off | off |
| D | on | on | on | off | off |

B->C isolates the single-frame range/quality rejection policy. C->D isolates
normal-direction bin selection. The invalid-quality relaxation, turn guard,
sliding window, Persistent quotas and F/F49 equal-count control are all excluded.

## Frozen protocol

- Bag: `/home/romi/Adaptive_FAST_LIO2/bag/SubT_MRS/SubT_points_ros2`
- Playback: `--clock --rate 1.0 --start-offset 1.0`
- RViz: off
- Backend: off
- Three independent node restarts per group: `run01`, `run02`, `run03`
- Run order: B, C, D. Finish all three runs of one group before the next.
- Save `runtime.csv`, the node parameter dump and the frozen Git commit in each run directory.
- Do not tune thresholds after B starts. Any change creates a new experiment version.

The configuration files are complete standalone parameter files. Their embedded
CSV paths are fallbacks; launch commands must override `frontend_runtime_csv_path`
with the current run directory.

## Runtime acceptance checks

- All groups report runtime schema `map_diagnostics_v7`.
- B: `adaptive_map_enabled=0`; all adaptive rejection counts remain zero.
- C: `adaptive_map_enabled=1`, `directional_selection_enabled=0`, and
  `direction_rejected=0` for every row.
- D: `adaptive_map_enabled=1`, `directional_selection_enabled=1`; direction
  selection may become active only on degenerate frames.
- B/C/D: equal-point-count control remains disabled and the window mode remains off.

