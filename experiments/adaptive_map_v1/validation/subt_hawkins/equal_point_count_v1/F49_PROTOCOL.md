# F49 calibrated equal-point-count protocol

- This is a separate follow-up to F24; F24 results remain unchanged.
- The cap is fixed at 49 before F49 bag replay.
- Calibration target is D's pooled 23.65 added points per degenerate frame.
- Cap 49 was estimated only from F24 pre-cap eligible counts; it was not selected from F49 ATE.
- All other parameters and runtime controls remain identical to F24 and D.
- Replay order: F49/run01, F49/run02, F49/run03, with a fresh node restart each time.
- Playback: 1.0x, mandatory 1-second start offset, RViz off, backend off.
