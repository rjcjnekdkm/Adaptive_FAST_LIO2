# Tunnel2 OFF-path alignment conclusion

Fresh same-configuration trials replace the misleading impression from the old
FAST-LIO2 records. Independent FAST-LIO2 run01 missed the expected first output
cycle and later drifted to 6.671 m. A startup-delayed run04 recovered all 2611
rows and scored 0.816 m. The startup-consistent run02/run03/run04 set has mean
0.823631 m and standard deviation 0.006701 m. Aligned Ours OFF has three runs
near 0.890 m (mean 0.890406 m). Their nominal ATE values are close, while
Tunnel2 remains sensitive to startup coverage and the trajectory branch
selected around its weakly constrained turn.

The OFF fast path is verified in B: Adaptive Map and all associated filters and
windows are disabled, no map update is skipped, and adaptive-only SVD/MAD and
window diagnostics remain zero. The remaining behavior is therefore a base
frontend comparison rather than an Adaptive Map effect.

For research reporting, use B as the internal ablation baseline for measuring
the module's contribution. Use independent A as an external FAST-LIO2 reference
and report all repeats, including A/run01 as a startup-deviation result. Use
A/run02–run04 for the controlled three-run summary. Do not claim that B is
bitwise equal to FAST-LIO2 or attribute A-versus-B stability differences to
Adaptive Map.
