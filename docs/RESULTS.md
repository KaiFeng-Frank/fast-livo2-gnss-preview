# Research status and recorded results

This is an experimental raw-GNSS integration for FAST-LIVO2. The recorded
experiments do not establish the reported performance of FAST-LIVGO or a
repeatable improvement over the host across sequences. The historical benchmark
and the later failed calibration gate are retained with their identities.

The public preview tag identifies the source shipped in this repository. The
source/build identifiers below identify the historical experiments; they are
different identities. A new build on another CPU or toolchain is a functional
rebuild and is not certified to reproduce the archived replay bit for bit.

## A79: seven M3DGR sequences, MID-360, seed 0

The paper columns quote FAST-LIVGO v2 Table II. They were not rerun by this
project. The host, sensor choice, alignment details and input defects prevent
treating the comparison as a controlled reproduction of the paper.
The machine-readable [summary](../results/a79_summary.json) preserves unrounded
values, input hashes, clock shifts and run identities.

| Sequence | Paper FAST-LIVO2 | Paper FAST-LIVGO target | U e1c | T e1c | U e1o | T e1o | GNSS active |
|---|---:|---:|---:|---:|---:|---:|---|
| Varying-illu03 | 1.44 | 0.23 | 1.7148 | 2.2742 | 1.7214 | 2.3042 | Yes, at 12.7 s |
| Varying-illu04 | 0.35 | 0.19 | 0.3532 | 0.3817 | 0.3544 | 0.4072 | Yes, at 11.9 s |
| Varying-illu05 | 0.16 | 0.09 | 0.2957 | 0.2238 | 0.2500 | 0.2997 | Yes, at 32.4 s |
| Outdoor01 | 0.27 | 0.11 | 0.2132 | 0.1822 | 0.2849 | 0.2244 | Yes, at 17.3 s |
| Outdoor04 | 0.25 | 0.13 | 0.2245 | 0.2953 | 0.2780 | 0.3614 | Yes, at 18.7 s |
| Dark01 | 0.31 | 0.34 | 0.0662 | 0.0662 | 0.1812 | 0.1812 | No, T = U |
| Dark02 | 0.18 | 0.27 | 0.2431 | 0.2431 | 0.2577 | 0.2577 | No, T = U |

Values are position ATE RMSE in metres. Activation times start at the common
scoring span. Both metrics meet the paper target on 2/7 sequences, Dark01 and
Dark02, with GNSS inactive on both. Among the five active sequences, e1c improves
on two and worsens on three; e1o improves only on Outdoor01. Varying-illu05 changes
improvement sign with the metric, so the two columns must be read together.
Only one final T seed was run; these are individual outcomes, not statistical
claims. The cited targets come from [FAST-LIVGO](https://arxiv.org/abs/2606.19190).

`U` is the GNSS-disabled host. `T` enables pseudorange, Doppler and TDCP tight
coupling. T is scored using `traj_N_odom`; U uses `traj_W`. Before activation,
the registered activation-frame transform maps T's host poses into its output
frame. Both use the same full-sequence span and mask, timestamp association within
0.05 s, and SE(3) Umeyama alignment without scale. `e1o` directly compares the
estimated IMU position with the ground-truth antenna position. `e1c` first moves
the estimate to that antenna using `p + R*l`, with fixed IMU-frame lever
`l = [-0.01999096558, -0.1534376595, -0.04942328067]` m. Horizontal lever components
were fitted on development Grass01/Grass02, five seeds each; the vertical component
was fixed from the calibration chain. No lever was fitted on these seven results.

The registered development gate had failed before A79: Grass01 T−U was +11.2 mm
against a +10 mm limit; Grass02 TDCP white-equivalent NIS/dof was 0.721 against a
lower limit of 0.8. One final benchmark was run as an explicitly recorded deviation.
The evaluation split had been used four times by A79. No additional seeds or
post-result tuning followed this final run.

U and T have different build hashes, so the A55 identical-build check fails.
The preactivation host output matches U byte for byte in every sequence; the
entire Dark01/Dark02 host outputs match. This is useful verification of the
unactivated path but does not erase the build difference.

Clock alignment also carries uncertainty. Varying-illu03/04/05 and Dark02 used
record-time fallback because the registered GT clock-chain check failed; the
other three used the checked chain. Their historical scoring spans excluded no
GT rows. Outdoor01 used a repaired input bag that differs from the official
checksum and has an approximately 0.5 s gap in all topics at about 262.5 s.
The [summary](../results/a79_summary.json) records these facts per sequence.

| Historical identity | Value |
|---|---|
| U runtime source | `72a2633ce91af2eb577acdd484725182e116d313` |
| T runtime source | `ef08068f9c05e08b64db7aa1fc616015fc3cae43` |
| Final configuration record | `d5fdab5f1e5420ed2e29573a13639300723ace41` |
| Scoring tool source | `ed60a742d8992c5c51f58eb76520ba034cd44b8a` |
| U build SHA256 | `741145f12da0b8947526ee2ba8553330df06e76006709e6050a25a15ddc5de17` |
| T build SHA256 | `80eac0fa064e4ca653940c77fde58c003c211773ac6e0208bd39b4d71e8e9d99` |
| T normalized runtime configuration SHA256 | `7c31c43ea47cbd9e94ce339c4fb57d75b5cd56670f2c478a64b46acfc8053ba0` |

The corresponding [base configuration](../results/a79-configs/m3dgr_mid360.yaml)
and [T overlay](../results/a79-configs/T.yaml) are included as historical snapshots.
Their file hashes are recorded separately from the runtime configuration hash,
which omits paths, sequence name and seed. Original binaries, containers and
private operations logs are outside this public preview.

## A80–A82: later development stopped at the noise gate

A80 added a literal paper-description branch and SmartPNT/MARS data support.
A81 registered one noise proposal and one complete model recheck. A82 validated
the implementation and retained the terminal research state. The recheck's
pooled development diagnostics are below; the closed D4 interval is [0.8, 1.25].

| Development dataset | Channel | NIS sum | DOF sum | NIS / DOF | D4 |
|---|---|---:|---:|---:|---|
| SmartPNT mini | Doppler | 292051.5819028482 | 224901 | 1.2985784051776035 | Fail |
| SmartPNT mini | TDCP | 216544.44774425257 | 221848 | 0.9760937567354792 | Pass |
| MARS-LVIG | Doppler | 13890.991699144288 | 12668 | 1.0965418139520278 | Pass |
| MARS-LVIG | TDCP | 12220.104931920361 | 12177 | 1.003539864656349 | Pass |

The failed SmartPNT Doppler gate stopped this registered chain. No noise freeze,
median3 selection, `lambda_th` selection, frozen-development run, promoted final
arm or final evaluation followed. The three SmartPNT evaluation sequences remain
sealed. The remaining three pooled passes do not establish accuracy, and a pooled
pass can hide individual-sequence deviations.
The [published aggregates](../results/a81_noise_recheck.json) retain the exact
sums, counts, runtime identity and downstream state.

A separate initial development diagnostic on SmartPNT
`Data02_20240718_100C_CAR_complex` already had large absolute errors: U e1c
710.0218 m and `LIVGO-literal-paper-diag` e1c 714.0275 m, over 1895 GT samples and
a 19.76 km reference path. Independent raw-pose scoring confirmed both values.
These are completed development diagnostics, not final evaluations. Their
near-unity travelled-distance ratios did not imply correct trajectory shape;
the compact [diagnostic record](../results/a82_development_diagnostic.json)
preserves the values and interpretation.

## Recompute what is published

Install NumPy in a virtual environment, then run from the repository root:

```bash
python3 tools/preview/recompute_results.py
```

This checks 28 A79 rigid-fit RMSE values from published paired-point sufficient
statistics and recomputes the four A81 pooled noise gates. It also recomputes
the 2/7 target count and the active-sequence improvement counts. Reproducing the
recorded scientific gate rejection is a successful verifier run (exit 0).
This default mode verifies final scoring algebra and aggregate pooling. It does
not validate GT conversion, timestamp association, masking or the sensor replay.

For a full ATE scoring example, obtain the original unshifted
`Visual_Challenge/Outdoor/Varying-illu03.txt` GT from the official
[M3DGR sequence table](https://github.com/sjtuyinjie/M3DGR#5-dataset-sequences)
([official GT link](https://1drv.ms/t/c/2b4bfc0edf421186/EYYRQt8O_EsggCtHEAAAAAABns-I3zsf1W1ysuU-5aiyyA?e=O2M98r),
[alternative official share](https://www.alipan.com/s/6mgxxw4Kze1)). Then run:

```bash
python3 tools/preview/recompute_results.py --gt /path/to/Varying-illu03.txt
```

The scorer sorts and removes duplicate timestamps, serializes the cleaned rows
to six decimal places as the historical auditor did, checks that GT identity,
then scores those canonical rows. It applies the recorded +0.021956682205200195 s shift, associates within
0.05 s, and recomputes both metrics for the included generated U/T scoring poses.
Expected e1c is **U 1.714814021 m, T 2.274239005 m**, with 7149 matched pairs per
arm; GNSS activated at 12.7 s and T became worse. `gt_audited.tum` already has
the clock shift and will fail the input identity check; an unshifted historical
`gt_clean.tum` is accepted. The generated T poses already contain the registered
preactivation frame fill. This runs scoring of recorded poses, not a new sensor
replay. `--json` prints the verification details.

The release preparation independently rescored all seven archived raw W/N_odom
trajectories against the local historical GT with a separate NumPy implementation.
Every e1c/e1o value agreed with the archived evaluator within 2×10⁻¹² m.
The bundled aggregate recomputation agreed within 1×10⁻¹⁰ m; the full Varying-illu03
example was also executed successfully using the historical unshifted GT.
No sealed data or new evaluation replay was used for these release checks.

The preview bundles generated estimates, configurations and aggregate statistics.
M3DGR's repository has a software license; this does not by itself establish
redistribution terms for its externally hosted raw dataset. GT is obtained from
the official source instead of being bundled. Dataset downloads remain subject
to their authors' terms. See [licensing](LICENSING.md).
