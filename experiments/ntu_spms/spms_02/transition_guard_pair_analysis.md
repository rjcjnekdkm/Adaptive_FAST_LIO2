# SPMS2 transition-guard paired ablation

## Controlled setup

Both runs use the same source revision, `ntu_spms.yaml`, frontend-only mode, adaptive map, adaptive window, RViz, playback rate 1.0, and the same rosbag2 QoS override. The only changed launch argument is `transition_guard_enable`.

The evaluation follows the NTU VIRAL protocol implemented in `scripts/evaluate_ntu_viral.py`: Leica prism compensation, ground-truth interpolation only across gaps below 0.1 s, and SE(3) trajectory alignment.

## Results

| Metric | Guard off | Guard on | Effect |
|---|---:|---:|---:|
| Associated samples | 2448 | 2439 | comparable |
| ATE RMSE (m) | 5.384868 | 0.531742 | -90.13% |
| Mean error (m) | 5.065816 | 0.320803 | -93.67% |
| Median error (m) | 4.715752 | 0.271746 | -94.24% |
| Maximum error (m) | 8.810746 | 3.843715 | -56.37% |
| Final error (m) | 7.771455 | 0.061850 | -99.20% |
| RMSE at relative time 200--220 s (m) | 4.672791 | 1.636729 | -64.97% |
| RMSE outside 200--220 s (m) | 5.438645 | 0.287450 | -94.71% |
| Normal / Transient / Persistent frames | 2897 / 618 / 0 | 3229 / 285 / 0 | fewer propagated transient failures |
| Map insertion ratio | 0.161842 | 0.133033 | fewer risky insertions |
| Final quality rejections | 940803 | 273217 | less downstream map degradation |
| Scan-to-map update failures | 0 | 0 | both trajectories are complete |

## Interpretation

Without the guard, the U-turn registration error propagates into the map and remains for the rest of the sequence. With the guard, the error rises temporarily during the U-turn, peaks at 3.84 m, then recovers and ends at about 0.062 m. Persistent mode is never entered in either run, so this failure is a transient registration transition rather than persistent geometric degeneracy.

This controlled ablation supports retaining the transition guard as the Transient branch of innovation point 1. It should not be presented as a Persistent-degeneracy mechanism.
