# V6 A7 estimated navigation smoke — engineering evidence

## Provenance and boundary

- Read-only reviewer runtime baseline: Module3
  `e328e27b4c4dcedc4748b59d39ec36bf38535152`, Integration
  `c1e411b6c579e4f07f72b9e768760ff1f13c2bb0`.
- Evidence directory: `/tmp/v6_a7_estimated_isaac.Ikapdb`.
- Verdict: **PASS (engineering smoke only)**.
- This observation is not a qualification run and does not satisfy any formal
  campaign count, scene/seed matrix, or acceptance receipt.

## Observed result

- G2 navigation completed in `34.8 s`.
- Recovery count: `0`.
- Collision count: `0`.
- Final remaining distance: `0.071 m`.
- EKF xy ATE RMSE: `0.0689 m`.
- AMCL xy ATE RMSE: `0.0629 m`.

The evidence supports one estimated-state engineering closure on the frozen
runtime baseline above. It must not be generalized to held-out robustness or
formal qualification.
