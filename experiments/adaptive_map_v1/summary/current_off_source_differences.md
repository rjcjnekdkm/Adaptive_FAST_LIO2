# Current Ours OFF / FAST-LIO2 implementation audit

Audited source: current workspace at e0237cb. This report supersedes stale
implementation descriptions in the September 8 audit; it does not identify
the executable or effective parameters of historical runs retrospectively.
No algorithm changes, compilation or bag replay were performed.

## Confirmed remaining differences

| Area | Ours | FAST-LIO2 | Interpretation |
|---|---|---|---|
| Observation diagnostics | Allocates per-point quality arrays, computes median/MAD, SVD and weak-direction scores even with Adaptive OFF | No equivalent diagnostics | Additional work; does not directly rewrite h_x or h. Scheduling impact remains unmeasured. |
| Diagnostic map | Every ten processed frames calls getMapCloud/flatten for /ikdtree_map, even with RViz off and map_en false | No active equivalent tree flatten in normal processing; debug flatten is inside if(0) | flatten calls Push_Down and is not a pure snapshot read. Concurrency with asynchronous rebuild needs verification. No race or trajectory effect has yet been demonstrated. |
| Map publication | Per-ten-frame processing, extra tree topic | Separate one-second ROS timer for accumulated scan map | Different workload and event ordering. |
| IMU members | Explicitly zeros acc_s_last and last_lidar_end_time_ in constructor/reset | Neither member explicitly initialized before first UndistortPcl use | Real uninitialized-read concern in the reference. Actual historical memory values unknown; identical first recorded pose argues against assuming a large startup discrepancy. |
| Timestamp rollback | Clears LiDAR and timestamp queues | Clears LiDAR queue only | Different exceptional recovery; no evidence of rollback in the compared Tunnel2 runs. |
| Automatic time sync | Converts corrected seconds with get_ros_time | Uses rclcpp::Time directly on the corrected numeric expression | Different timestamp construction; time_sync_en=false in the recorded current B configuration, so not its active path. |
| Buffers and point representation | Dynamic observation buffers, nearest-neighbor wrapper copies, fresh VoxelGrid object, copies full point fields on world transform | Fixed/reused buffers, direct nearest search, reused VoxelGrid, world transform writes XYZ/intensity | More allocation/copying and different auxiliary fields. No demonstrated XYZ selection difference for identical finite inputs. |

The current implementations agree on the preprocessing algorithm, IMU process
bodies apart from reviewed initialization/logging differences, sync_packages,
float plane fitting, local cube logic, and the checked IKFoM/iKD-Tree source
files. The 24 existing source/launch tests passed in the preceding inspection.
Current IMU QoS depth 10 and ROS-clock mapping timer are aligned; older reports
of depth 2000, wall timer, max-point scan end and initialization 200 are stale.
Both current build flag files specify O3, OpenMP and MP_PROC_NUM=3. These files
do not prove historical binaries used identical flags.

Adaptive OFF bypasses its insertion gate, rejection and candidate sorting.
Nonzero configured invalid-quality flags do not override this master switch.
This does not eliminate diagnostic overhead.

## Tunnel2 evidence

Compare positions at matching end timestamps, without alignment, in current
frozen_c_regression_v1/B/run01 against fastlio2_blind2/run02/runtime1.csv:

- First recorded position is identical to CSV precision.
- Second position differs by approximately 2.37e-8 m.
- Difference exceeds 0.1 mm at 4.199 s, 1 cm at 8.7 s, 10 cm at 46.1 s,
  and 1 m at 138.5 s from the first recorded timestamp.
- The other FAST-LIO2 repeats share the same early divergence pattern.
- FAST-LIO2 runtime1 versus runtime2/runtime3 first differs by more than
  1e-8 m at 108.200 s, then finishes with substantially different ATE.
- Old ours_off/run02 versus current B first exceeds 1e-8 m at approximately
  75, 104 or 105 s, depending on the old trajectory.

Current B/run01 effective-point median is 619 over 0–100 s, 346.5 over
100–130 s, and 31 over 130–145 s. The latter interval has a minimum of 15
effective points and 12 frames below 20. This supports low-constraint
sensitivity near the period when trajectories separate substantially.
It does not isolate scheduling, floating-point effects or map rebuild as
the cause. The logged condition number is not uniformly larger there, and
must not be used alone to assert observability loss.

The historical FAST-LIO2 directory lacks parameter and executable snapshots.
Its run-to-run variation cannot establish a systematic A/B implementation
advantage. Repeated identical CSVs also cannot prove independent restarts.

## Diagnostic priority

1. Freeze effective A/B parameters and identify executable builds. Capture
   actual per-scan IMU/LiDAR input fingerprints in both implementations.
2. Inspect the first differing scan: undistorted/downsampled XYZ, nearest
   neighbors, effective-mask, h_x/h, posterior state and inserted XYZ/order.
3. Isolate diagnostic tree publication first, then SVD/statistics overhead,
   using separate controls. Never pool these controls with frozen-C results.
4. Correlate repeat divergence with asynchronous rebuild events; changing
   rebuild policy is a diagnostic control, not a silent change to the baseline.

Well-constrained scenes can correct small perturbations; scenes with few or
ambiguous constraints may preserve and amplify them through state/map
feedback. That mechanism is consistent with these observations, but the
specific originating implementation difference remains unproven.
