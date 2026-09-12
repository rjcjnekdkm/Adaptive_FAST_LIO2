# Invalid protocol run

This run is excluded from all ablation statistics.

- Cause: the bag was played from offset 0 instead of the required 1-second start offset.
- Symptom: frontend divergence; effective points became permanently zero at about 132.8 seconds.
- Persistent mode was never entered and the total-quota branch was never exercised.
- The data is retained only as a protocol-failure record.
