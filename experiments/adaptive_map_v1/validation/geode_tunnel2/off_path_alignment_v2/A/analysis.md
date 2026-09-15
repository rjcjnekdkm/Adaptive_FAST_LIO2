# Tunnel2 — fresh independent FAST-LIO2, A group

## Result

All four runs reached the end of the bag. run01 contains 2610 poses and its
first recorded output is one scan period (approximately 0.1 s) later than
run02/run03/run04, which contain 2611 poses. The artifacts cannot distinguish an
external-recorder miss from an input/output startup race in the mapping node.
All four still associate all 618 available GT poses.
Evaluation used the local GEODE `gamma2GT_leica.py` and `rmse.py`, end-of-scan
timestamps, SE(3) alignment, `t_max_diff=0.1 s`, and the established `+0.1 s`
offset on the GT input.

| Run | ATE RMSE (m) | Rows | Duration (s) | CSV SHA256 |
|---|---:|---:|---:|---|
| run01 | 6.670714 | 2610 | 260.899690 | `359d8aa08743434695d5382581376ddd1089f86710d90074e1ee643f63f22787` |
| run02 | 0.828850 | 2611 | 260.999525 | `d475fa23dcb5b73b2d0e6fec097d1511488f92b2fabefb8cd18161776b5c8769` |
| run03 | 0.825968 | 2611 | 260.999525 | `673bd446177adf5ba44415a97d0312d3fc5ddddf7939b81b464ff9639e01639d` |
| run04 (`--delay 5.0`) | 0.816074 | 2611 | 260.999525 | `affd54650324ed0e0039b576295be55290183a85d6899ac4fc0ec8e403727b1c` |

- Mean over all three: 2.775177 m
- Median: 0.828850 m
- Sample standard deviation: 3.373634 m
- Two non-divergent runs mean: 0.827409 m
- Startup-consistent run02/run03/run04 mean: 0.823631 m
- Startup-consistent sample standard deviation: 0.006701 m

run01 enters a high-error branch near the approximately 137 s turn and reaches
ATE 6.67 m. run02 and run03 remain close to one another (about 0.021 m direct
full-trajectory position RMS separation) and produce approximately 0.83 m ATE.
Thus fresh FAST-LIO2 is not intrinsically high-error on Tunnel2, but the
sequence exposes a repeat-dependent degenerate branch.

At run01's first recorded timestamp, its pose already differs by approximately
0.04 m from run02 at the same timestamp. Therefore the missing CSV row cannot
be assumed to be a harmless recorder-only omission. A startup discovery race
may have shifted one LiDAR cycle and thereby changed the IMU initialization
sample set and initial map. This is a plausible mechanism, not proven causality
for the much later drift.

run04 used rosbag `--delay 5.0` and recovered the expected first timestamp and
all 2611 rows. Its ATE is 0.816074 m, and its direct full-trajectory position
RMS difference is approximately 0.046 m from run02 and 0.050 m from run03.
This supports retaining a playback startup delay in subsequent trials, but it
does not prove that the missing initial cycle alone caused run01's later drift.

## Comparison with aligned Ours OFF (B)

| Method | run01 | run02 | run03 | Mean | Median |
|---|---:|---:|---:|---:|---:|
| FAST-LIO2 A | 6.670714 | 0.828850 | 0.825968 | 2.775177 | 0.828850 |
| Aligned Ours OFF B | 0.889942 | 0.890638 | 0.890638 | 0.890406 | 0.890638 |

The typical results are close: the median difference is 0.061788 m, with B
about 7.45% higher than A's median. B produced three low-error runs in this
batch while A produced two low-error runs and one high-error branch. This is
evidence of comparable nominal accuracy, not bitwise trajectory equivalence.
Any stability difference is still a property of the non-adaptive base
implementations or execution ordering and cannot be credited to Adaptive Map.

The run01 FAST-LIO2 parameter dump matches B's sensor topics, no time sync,
Livox AVIA preprocessing, `blind=2 m`, point filter 3, 0.5 m voxel filters,
three iterations, 200 m detection range, process noise, and fixed extrinsics.
However, no `commit.txt`, source hash files, or source-status files were saved;
run02/run03 also lack live parameter dumps. Exact run-time source provenance
therefore remains incomplete and must not be reconstructed post hoc from the
current checkout.

For the startup-controlled comparison, use FAST-LIO2 run02/run03/run04. Its
mean 0.823631 m is 0.066775 m (8.11%) below B's mean 0.890406 m. Both sets have
low variance and comparable nominal accuracy, but they are not bitwise
equivalent. Keep run01 as a startup-protocol-deviation record rather than
deleting it or mixing it into the controlled three-run mean.
