# SPMS1 repeat run provenance

The user reports deleting the original run01 CSV and recording this repeat into
the same path. The current runtime.csv is the replacement run, not the original.
The original raw trajectory is unavailable for paired trajectory analysis.

Previously observed original-run summary (conversation record only):
- Rows: 3077
- Duration: 417.493 s
- Maximum adjacent timestamp gap: 1.200 s

Replacement-run inspection:
- Rows: 3617
- Duration: 417.593 s
- Maximum adjacent timestamp gap: 0.700 s
- adaptive_map is 0 throughout; pose values are finite.

More recorded poses do not establish better trajectory accuracy. Treat both
the missing original and the timing variability as limitations when comparing
methods. Do not describe the replacement as a best-of-two selection.
