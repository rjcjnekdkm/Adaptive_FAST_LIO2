# Frozen adaptive-map candidate: C

The project default is frozen to candidate C after the SubT Hawkins formal
ablation, the independent C/E confirmation round, and held-out GEODE Tunnel4
and Offroad2 validation.

## Default policy

Candidate C changes map insertion only. It retains the aligned internal
FAST-LIO2 preprocessing, synchronization, IMU processing, scan matching and
state-estimation path.

- Adaptive map insertion: enabled.
- Single-frame degeneracy decision: enabled.
- Range, residual and quality rejection: enabled.
- Invalid-quality rejection: enabled.
- Invalid-quality low-effective relaxation and turn guard: disabled.
- Normal-direction selection (D): disabled.
- Equal-point-count control (F/F49): disabled.
- Sliding window and Persistent mode (E): disabled.
- Transient novel-point quota: disabled.
- Persistent total-insertion quota: disabled (`max: 0`).

Dataset-specific sensor limits, including blind range, adaptive-map range and
minimum effective-point thresholds, remain in the corresponding dataset
configuration. Freezing the architecture does not make one set of sensor
thresholds universal.

## Selection evidence

Across the six SubT C runs from the formal and confirmation rounds, C obtained
mean ATE 2.324139 m, sample standard deviation 1.381491 m and maximum
4.921436 m. E obtained mean 3.619672 m, sample standard deviation 3.617796 m
and maximum 10.756171 m. C completed all six runs below the descriptive 5 m
boundary; E completed five of six below it.

On the held-out sequences, C improved over the aligned internal baseline B by
0.76% on GEODE Tunnel4 and 4.56% on GEODE Offroad2. It remained 1.60% and
2.11% worse than independent FAST-LIO2 A, respectively. The supported claim is
therefore improved robustness relative to the internal baseline, not universal
superiority over FAST-LIO2.

## Reproducing historical D/E ablations

The D configuration explicitly enables `directional_selection_enable`. The
historical E/P0 and E/P1 configurations also pin direction selection on, and E
must additionally be launched with `adaptive_window_enable:=true`. These
explicit settings preserve the old ablation definitions after candidate C
became the project default.
