# Latest evo APE / ATE / RPE analysis

Date: 2026-06-27

## Input trajectories

- Ground truth: `subt_ground_truth.tum`
- Adaptive FAST-LIO2 latest: `adaptive_runtime.csv` -> `adaptive_runtime_latest_evo.tum`
- FAST-LIO latest: `fastlio_runtime_external.csv` -> `fastlio_runtime_external_latest_evo.tum`
- LIO-SAM: `liosam_runtime_external.csv` -> `liosam_runtime_external_latest_evo.tum`

## evo settings

- APE / ATE command type: `evo_ape tum ... -a`
- Error item: translation part, unit: m
- Alignment: SE(3) Umeyama alignment
- RPE command type: `evo_rpe tum ... -a -d 10 -u m`
- RPE delta: 10 m, consecutive pairs

Note: evo APE translation RMSE is usually reported as ATE RMSE. This evo result may not be numerically identical to the older `three_way_comparison.csv`, because that table used the previous project-specific comparison pipeline.

## APE / ATE result

| Method | RMSE (m) | Mean (m) | Median (m) | Max (m) |
| --- | ---: | ---: | ---: | ---: |
| Adaptive FAST-LIO2 | 1.430807 | 1.181123 | 0.838675 | 3.277203 |
| FAST-LIO | 18.144224 | 14.560853 | 10.052614 | 44.873928 |
| LIO-SAM | 1.729736 | 1.545627 | 1.261134 | 3.558665 |

## RPE 10m result

| Method | RMSE (m) | Mean (m) | Median (m) | Max (m) |
| --- | ---: | ---: | ---: | ---: |
| Adaptive FAST-LIO2 | 0.253330 | 0.188151 | 0.127789 | 0.784866 |
| FAST-LIO | 0.619230 | 0.429044 | 0.262834 | 1.906854 |
| LIO-SAM | 0.304961 | 0.249142 | 0.180881 | 0.691251 |

## Conclusion

Under the evo official evaluation here, the latest Adaptive FAST-LIO2 result is the best among the three methods in both global APE / ATE RMSE and local 10m RPE RMSE.

Compared with LIO-SAM, Adaptive FAST-LIO2 has lower global error and lower local relative error. Compared with FAST-LIO, the difference is much larger: FAST-LIO shows obvious drift in this latest run, while Adaptive FAST-LIO2 remains stable.

## Generated comparison plots

- APE / ATE evo comparison plot: `evo_ape_latest_comparison_plot.pdf`
- RPE 10m evo comparison plot: `evo_rpe10m_latest_comparison_plot.pdf`
- APE / ATE RMSE bar plot: `evo_ape_ate_latest_rmse_bar.png`
- RPE 10m RMSE bar plot: `evo_rpe10m_latest_rmse_bar.png`
- Trajectory comparison, evo PDF: `evo_trajectory_latest_comparison_xy.pdf`
- Trajectory comparison, PNG: `evo_trajectory_latest_comparison_xy.png`
