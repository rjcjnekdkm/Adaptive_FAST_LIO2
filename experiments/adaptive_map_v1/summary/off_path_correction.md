# OFF-path correction after e0237cb

User-requested implementation cleanup; compilation and replay are pending
user execution. Do not pool new runs with the previously frozen B/C results.

- Adaptive OFF skips per-point adaptive quality buffer allocation/filling,
  residual median/MAD, pose-Jacobian SVD and weak-direction scores.
- Required FAST-LIO2 point-to-plane residual, score >0.9, effective-point
  selection, mean residual, h_x and h remain on the shared estimator path.
- OFF insertion no longer allocates an identity candidate-index vector.
  ON degenerate-frame candidate sorting retains its existing policy.
- The mapping node no longer publishes /ikdtree_map or calls getMapCloud to
  flatten the live tree. /Laser_map remains an accumulated scan visualization.
- /Laser_map now runs from a one-second ROS-clock timer, as in the reference,
  instead of every ten processed scans. map_en controls that publication.
- Runtime schema map_diagnostics_v8 reports master-gated enabled flags.
  YAML/ROS parameter values remain the configured values. For Adaptive OFF,
  median/MAD/condition diagnostics are uncomputed zero placeholders; zero
  must not be interpreted as a measured residual dispersion or condition.

## Additional corrections from the full difference review

- Nearest-neighbor caches now use the native aligned iKD-Tree PointVector.
  The manager writes search outputs directly into the caller's buffers.
- Insertion uses native PointVector containers and passes them directly to
  Add_Points. Initial point-cloud insertion also forwards the PCL point vector
  without constructing an extra copy in the wrapper.
- The scan VoxelGrid object and observation-normal storage persist across
  callbacks instead of being reconstructed at every use.
- Path publication supports publish.path_en and emits every tenth eligible
  scan. Shared output publishers use depth 20, matching the reference.
- Output order is Path, world cloud, body cloud; body publication requires
  both scan_publish_en and scan_bodyframe_pub_en.
- Runtime, odometry, Path and TF use the filter rotation quaternion directly,
  eliminating the extra rotation-matrix conversion and normalization.
- Odometry now contains the current filter covariance, with position/rotation
  order mapped to ROS. It is assigned BEFORE publication; the reference's
  publish-before-assignment behavior is not reproduced.
- Degeneracy messages are built only when a subscriber is present. Backend
  subscribers still receive OFF mode and its uncomputed diagnostic placeholders.

Safe initialization of IMU members and corrected exceptional timestamp
handling are retained. The reference's uninitialized members and timestamp
rollback/self-sync problems are not reproduced. Dynamic buffer layout,
extra runtime recording, safe point-field initialization and wrapper guards
still differ; bitwise equivalence with FAST-LIO2 has not been established.
The untouched reference repository remains available for A comparisons.

These changes address known extra work/tree access. They do not establish
which difference caused historical Tunnel2 ATE gaps. Validate the new OFF
path first with explicit configuration/build provenance and independent
restarts, then validate ON separately.
