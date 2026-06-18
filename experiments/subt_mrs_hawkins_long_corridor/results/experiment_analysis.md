# SubT-MRS Hawkins Long Corridor Experimental Analysis

## Evaluation Protocol

- Ground-truth path length: 579.173 m.
- ATE uses translation error after SE(3) Umeyama alignment without scale correction.
- RPE uses translation error over 10 m intervals.
- Timestamp association tolerance: 0.12 s.
- Internal degeneracy flags are excluded from cross-algorithm comparison because each algorithm uses a different definition and threshold.

## Quantitative Results

| Method | ATE RMSE (m) | ATE Mean (m) | ATE Median (m) | ATE Max (m) | RPE 10 m RMSE (m) | RPE 10 m Max (m) |
|---|---:|---:|---:|---:|---:|---:|
| **Adaptive FAST-LIO2** | **1.511** | **1.264** | **0.907** | **3.364** | **0.240** | **1.110** |
| LIO-SAM | 1.682 | 1.429 | 1.082 | 3.909 | 0.416 | 2.067 |
| FAST-LIO | 17.996 | 14.541 | 9.836 | 41.223 | 0.580 | 2.723 |

## Trajectory and Stability Results

| Method | Duration (s) | Path Length (m) | Length Error (m) | Length Error (%) | Max Estimated Speed (m/s) | Run Status |
|---|---:|---:|---:|---:|---:|---|
| **Adaptive FAST-LIO2** | 274.524 | 585.514 | 6.341 | 1.095 | 3.735 | Complete and stable |
| LIO-SAM | 276.340 | 583.543 | **4.370** | **0.755** | 4.057 | Complete and stable |
| FAST-LIO | 276.440 | 595.835 | 16.662 | 2.877 | 14.366 | Complete, with large trajectory drift |

## Analysis

Adaptive FAST-LIO2 achieved the best trajectory accuracy. Its ATE RMSE was 1.511 m, which was 10.2% lower than LIO-SAM and 91.6% lower than FAST-LIO. Its 10 m RPE RMSE was 0.240 m, improving on LIO-SAM by 42.3% and FAST-LIO by 58.6%.

LIO-SAM obtained the smallest path-length error, but path length alone does not measure trajectory shape or spatial consistency. Adaptive FAST-LIO2 achieved lower ATE and RPE despite a slightly larger path-length error.

FAST-LIO exhibited substantial lateral and vertical drift. Its maximum estimated speed reached 14.366 m/s, while the dataset ground-truth maximum was approximately 4.80 m/s. The similar total path length therefore does not indicate an accurate trajectory.

The comparison supports the conclusion that Adaptive FAST-LIO2 provides the best overall accuracy and local odometry consistency in this 16-line LiDAR long-corridor experiment.
