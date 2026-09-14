# Formal A-E result

## Main result

C has the lowest mean and median ATE. E nearly matches C's mean and has the
lowest sample standard deviation and lowest worst-run ATE. The basic
quality/range insertion policy is the clearest accuracy contribution; the
window/Persistent policy's clearest contribution is repeatability and tail-risk
reduction.

Relative to independent FAST-LIO2 A, C reduces mean ATE by 69.22%, while E
reduces mean ATE by 68.32% and sample standard deviation by 76.51%. Relative to
internal baseline B, C reduces mean ATE by 61.34%, and E reduces it by 60.21%.

D does not demonstrate an incremental benefit over C. Its direction rule is
weak at the current threshold and the high-error tail returns. E's active
Persistent state restores stability relative to D, but does not lower mean ATE
below C.

## A/B fairness

A and B use aligned preprocessing, initialization, mapping parameters, replay
rate, start offset and evaluation protocol. Their ATE ranges overlap and their
medians differ by only 0.328 m, while both have very large three-run variance.
B's mean is 20.38% lower than A, but this difference is smaller than either
group's sample standard deviation and is driven by one low-error B run.

The evidence does not prove implementation equivalence, but it also does not
show a decisive systematic engineering offset requiring the adaptive
conclusion to be suspended. Claims should report both A and B baselines and all
three repetitions rather than selecting a single run.

## Scope

These are descriptive n=3 results on one representative long-corridor
sequence. Missing live parameter dumps for some run02/run03 records are
explicitly documented. Frozen configurations, source revision, runtime
invariants and full trajectory coverage remain available.
