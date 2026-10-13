# Published research evidence

Read [research status and metric definitions](../docs/RESULTS.md) first.

| Artifact | What can be verified |
|---|---|
| `a79_summary.json` | Historical seven-sequence ATE table, activation, exact source/build/config/input identities, uncertainty and protocol deviation |
| `a79_se3_statistics.json` | All 28 A79 final rigid-fit RMSE calculations from paired-point sufficient statistics |
| `a79-configs/` | Historical base configuration and T overlay, each with a recorded SHA256 |
| `a79-varying-illu03/` | Generated U/T scoring poses and identity-checked full ATE scoring against user-obtained upstream GT |
| `a81_noise_recheck.json` | Four pooled noise statistics and the failed SmartPNT Doppler gate |
| `a82_development_diagnostic.json` | Reported large Data02 development errors, with archived independent-audit identity |

```bash
python3 tools/preview/recompute_results.py
python3 tools/preview/recompute_results.py --gt /path/to/Varying-illu03.txt
```

The default command verifies aggregate algebra. Only the second command performs
trajectory association and ATE scoring; it requires the original unshifted GT,
which is not bundled. Neither command performs a sensor replay or opens sealed
evaluation data. The code needs NumPy and otherwise uses the Python standard library.

The sufficient statistics are computed after pairing. For centered estimate
points X and reference points Y, let C = YᵀX and its singular values be s₁,s₂,s₃.
The minimal rigid-fit residual energy is
`||X||² + ||Y||² − 2(s₁+s₂+sign(det(UVᵀ))*s₃)`.
RMSE is the square root of this energy divided by the paired point count.
This verifies the final SE(3) calculation from numeric aggregates and cannot
independently check which original rows were selected.

The archives contain project-generated estimate poses, numeric facts and
configuration snapshots. Third-party ground truth, raw sensor recordings,
linked binaries, container images, credentials and private machine logs are
not part of this evidence directory. Source licenses and upstream attributions
are documented in [LICENSING](../docs/LICENSING.md).
