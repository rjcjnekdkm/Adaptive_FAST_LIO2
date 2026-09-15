# Existing backend evaluation on GEODE Tunnel4

This experiment starts stage 6 of the research plan. It evaluates the existing
loosely coupled backend without changing or tuning its implementation.

## Fixed inputs

- Source commit: `e0237cb`.
- Frontend: frozen candidate C.
- Frontend configuration:
  `experiments/adaptive_map_v1/generalization/geode_tunnel4_v1/config/BC_geode_gamma_blind2.yaml`.
- Bag: `bag/GEODE/Tunneling_tunnel_gamma/Tunneling_tunnel4_gamma_ros2`.
- Playback: `--clock --rate 1.0`, no start offset.
- RViz and optimized-map publication: disabled for quantitative runs.
- Existing backend defaults are frozen; bidirectional ICP and observability
  rejection remain disabled. No threshold may be selected from GT error.

## Paired comparison

The backend does not feed corrections back to the frontend state or iKD-Tree.
Each backend-enabled run therefore provides a paired comparison with identical
frontend input:

- Backend OFF result: convert that run's `runtime.csv` to the frontend TUM
  trajectory.
- Backend ON result: use that run's
  `adaptive_backend_optimized_full.tum`.

The keyframe-only `adaptive_backend_optimized.tum` is diagnostic output and
must not be compared directly with the full-rate frontend trajectory.

## Execution and stopping rule

Run `run01` first. Continue to `run02` and `run03` only if all of the following
hold:

1. frontend and full-rate backend trajectories cover the complete sequence;
2. the loop CSV contains at least one accepted loop;
3. accepted loops have finite ICP and consistency metrics;
4. the frontend ATE remains compatible with the frozen-C Tunnel4 repeats,
   ruling out obvious backend CPU interference.

If no reliable loop is accepted, record the backend as inactive on this
sequence instead of lowering a threshold. If an accepted loop causes a clear
error jump, retain it as a failure case and diagnose loop reliability before
enabling any optional backend extension.

## Required outputs per run

- `runtime.csv`
- `frontend.tum`
- `adaptive_backend_optimized.tum`
- `adaptive_backend_optimized_full.tum`
- `adaptive_backend_loops.csv`
- `adaptive_fastlio_mapping.yaml`
- `adaptive_degenerate_backend.yaml`
- `commit.txt`
- official frontend and backend evaluation outputs
