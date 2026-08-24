# Final four-method evo comparison

Evaluation uses evo official style: APE/ATE with SE(3) alignment, and RPE over 10 m consecutive pairs.

## Main result

| Method | APE RMSE (m) | APE Mean (m) | APE Median (m) | APE Max (m) | RPE 10m RMSE (m) | RPE 10m Mean (m) | RPE 10m Median (m) | RPE 10m Max (m) |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| FAST-LIO | 18.144224 | 14.560853 | 10.052614 | 44.873928 | 0.619230 | 0.429044 | 0.262834 | 1.906854 |
| LIO-SAM | 1.729736 | 1.545627 | 1.261134 | 3.558665 | 0.304961 | 0.249142 | 0.180881 | 0.691251 |
| Ours Frontend | 1.430807 | 1.181123 | 0.838675 | 3.277203 | 0.253330 | 0.188151 | 0.127789 | 0.784866 |
| Ours Frontend+Backend | 1.275174 | 1.075757 | 0.777557 | 3.039653 | 0.514308 | 0.398206 | 0.289763 | 1.533367 |

## Observation

- Compared with Ours Frontend, Ours Frontend+Backend reduces APE RMSE from 1.430807 m to 1.275174 m.
- RPE 10m RMSE increases from 0.253330 m to 0.514308 m, showing that backend loop correction improves global consistency but can affect local relative consistency.
- Therefore, the current backend result is best interpreted as a global-consistency enhancement on top of the frontend degeneration-aware map update.
