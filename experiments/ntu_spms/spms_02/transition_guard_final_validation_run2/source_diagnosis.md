# Transition-guard failed-run diagnosis

## Run status

This run is incomplete and must not be used as a full-sequence ATE result.

- Processed runtime rows: 2004 (successful run: 3514)
- Last LiDAR time: 1624181299.657 s
- Received / synchronized scans: 2016 / 2015
- Scan-to-map update failures: 7
- Last estimated position: (-33.402, -143.780, 5.638) m
- Partial-trajectory NTU ATE RMSE: 7.154164 m
- Partial-trajectory maximum error: 67.085790 m

## Guard behavior

- Active frames: 452
- Triggered frames: 383
- A continuous active interval lasts from 1624181261.258 to 1624181297.557 s
- Duration of the continuous interval: 36.299 s
- Re-triggers in that interval: 343

The successful current-version run remains guarded for only 10.599 s around the U-turn. The two trajectories first differ by more than 1 m at 1624181265.258 s, within the failed run's continuous guard interval, and differ by more than 10 m at 1624181278.157 s.

## Source issues

1. `apply_transition_guard()` runs inside every ESIKF inner iteration. Its own scaling changes the following iteration's residual, so detection and intervention form a circular nonlinear feedback loop.
2. The trigger uses `frame_localizability_f0` and `frame_localizability_lambda0`, but these values are updated only after `update_iterated_dyn_share_modified()` returns. Therefore the guard uses the preceding frame's geometry to alter the current frame.
3. Every high-residual frame resets the cooldown to five frames. There is no upper bound or re-arm condition, allowing a nominal five-frame guard to remain active for 36.3 s.
4. While active, the code both reduces the complete LiDAR update to weight 0.25 and rejects all map insertion. If the initial state is already inaccurate, the weaker LiDAR correction and increasingly stale map can prevent recovery. This creates the observed positive feedback: high residual -> weak correction / no map -> fewer correspondences -> higher residual.
5. The frame-level logging state is inconsistent on the final cooldown transition: `advance_transition_guard()` can clear `active` after measurement weight 0.25 was used, leaving `active=0` and `measurement_weight=0.25` in one row. This is not the main divergence cause but confirms the state transition is split across incompatible stages.

Scaling both the Jacobian and residual by `sqrt(weight)` is mathematically equivalent to increasing the scalar measurement noise from `R` to `R/weight` in the current IKFoM implementation. The failure is caused by when and for how long this scaling is selected, not by the square-root scaling formula itself.

## Recommended correction

Treat the transition guard as a map-update protection mechanism:

1. Remove whole-frame ESIKF measurement scaling from `h_share_model()`.
2. Compute the trigger after the nominal ESIKF update, using current-frame residual and current-frame localizability.
3. Pause or selectively restrict map insertion for a bounded number of frames.
4. Require recovery below a lower residual threshold before allowing another trigger, preventing continuous re-triggering.

This keeps the innovation within adaptive map management and avoids weakening the LiDAR correction needed to recover from a U-turn registration error.
