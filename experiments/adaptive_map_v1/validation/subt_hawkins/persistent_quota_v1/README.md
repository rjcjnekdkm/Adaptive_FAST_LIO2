# SubT Hawkins Persistent total-quota ablation v1

- Candidate code commit: `06d18b9`
- Bag: `/home/romi/Adaptive_FAST_LIO2/bag/SubT_MRS/SubT_points_ros2`
- Playback: `--clock --rate 1.0`
- Common controls: Adaptive Map on, adaptive window on, transient novel quota off,
  invalid-quality filter on, low-effective relaxation off, turn guard off, backend off, RViz off.
- P0 changes only the Persistent total insertion quota to disabled (`scale=1.0`, `min=0`, `max=0`).
- P1 uses the candidate quota (`scale=0.3`, `min=2`, `max=5`).
- SubT remains at `adaptive_map.min_effective_points=120` because this is the fixed sparse 16-line LiDAR configuration.
- Paired order: P0/run01, P1/run01, P0/run02, P1/run02, P0/run03, P1/run03.
- Restart the launch process before every bag replay. Do not overwrite an existing run.

For each run, create its directory, launch the frontend with the matching config and
runtime path, dump `/adaptive_fastlio_mapping` parameters while the node is alive,
and play the bag in a second terminal.

The launch commands and parameter values must remain unchanged across repetitions;
only `P0`/`P1` and `run01`/`run02`/`run03` in paths may change.
