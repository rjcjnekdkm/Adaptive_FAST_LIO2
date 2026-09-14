# GEODE Tunnel4 generalization v1

Tunnel4 is used as a held-out generalization sequence for the frozen C
architecture. Historical runs may be cited as reference but are not pooled with
this same-version experiment.

## Groups

- A: independent FAST-LIO2 using
  `src/FAST_LIO/config/geode_gamma_source_check_baseline.yaml`.
- B: internal frontend with Adaptive Map disabled.
- C: frozen quality/range candidate; direction selection, adaptive window,
  Persistent policy and equal-count control disabled.

The A and B/C sensor configurations share Livox AVIA input, external Xsens IMU,
`blind=2.0`, point filter 3, FAST-LIO2 process noise, fixed official
extrinsics, 0.5 m voxels and `det_range=200 m`. The Adaptive thresholds in C
are the pre-existing GEODE sensor-family values and are not tuned on Tunnel4.

## Protocol

- Bag:
  `bag/GEODE/Tunneling_tunnel_gamma/Tunneling_tunnel4_gamma_ros2`
- Playback: `--clock --rate 1.0`, without a start offset.
- Backend and RViz disabled.
- Three independent restarts per group.
- Run order: B, C, A. Do not tune or change configuration after B/run01 starts.
- Evaluation: local official `gamma2GT_leica.py` conversion followed by
  `rmse.py`; preserve the established +0.1 s GT-offset convention separately
  from the playback protocol.
