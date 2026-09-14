# SubT Hawkins F equal-point-count control v1

- F keeps Adaptive Map's range, residual, quality and invalid-quality gates.
- The sliding window and Persistent policy are disabled, matching D.
- Normal-direction bin selection is disabled only in degenerate frames.
- A fixed cap of 24 accepted points per degenerate frame replaces direction selection.
- The value 24 was declared before F runs from D's three-run degenerate-frame means:
  21.77, 26.25 and 23.29 points/frame (pooled mean approximately 23.7).
- Normal frames are unchanged; this is not a whole-frame global insertion cap.
- Backend and RViz remain off. Replay at 1.0x with a mandatory 1-second start offset.
- Run three independent restarts under F/run01, F/run02 and F/run03.

The completed F24 runs are retained as a valid pilot. Post-run auditing found
that 24 was a cap rather than an achieved count: degenerate-frame insertion was
27.8% below D, although whole-run insertion differed by only 8.1%. See
`analysis.md`; do not describe F24 as an exact per-degenerate-frame count match.

Acceptance criteria in runtime CSV: equal-point-count control is enabled, it is
active only on degenerate frames, its target is 24, and direction_rejected remains
zero whenever the count control is active.
