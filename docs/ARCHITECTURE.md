# Architecture and outputs

The following describes the consistency-oriented `T` arm. `LIVGO-literal` follows the paper's fixed frame,
primary-frequency RAIM screen, Doppler and consecutive-epoch TDCP updates. Its one-time DTW alignment now runs
at the 50 m initialization gate before the frame least squares, using the registered acceptance checks (A82).
The two noise candidates remain development-only until calibration is frozen.

- **Host integration.** The FAST-LIVO2 ESIKF is extended through explicit hooks; this
  snapshot also contains the documented host bug fixes and initialization changes. All GNSS states live in an augmented block that forms one joint Gaussian with the host
  state through three read-only hooks: propagation (H1), exact conditioning on the host's own LIO/VIO updates (H2)
  and a joint GNSS update (H3). Historical disabled-GNSS regression was checked on the recorded reference environment;
  it is not a cross-platform guarantee.
- **Map drift is a state, not an error.** The registration between the LIO world frame and a local ENU frame,
  ξ = (tilt φ, yaw ψ, translation t), is a random walk driven by the distance travelled, applied around the current
  pivot. GNSS corrections therefore mostly go to ξ, the receiver clock and velocity/biases; the host position only
  takes a share along LIO-degenerate directions.
- **Measurements.** Pseudorange (1 Hz), Doppler (10 Hz) and TDCP with a 3-D random clone per pair and the
  receiver-clock increment removed by a Householder projection; GPS, Galileo, BeiDou (GLONASS Doppler/carrier only);
  inter-system biases; time-correlated noise covered by κ_AR variance inflation. TDCP pairs are chained end to end and
  the carrier-phase noise of the shared epoch is a state, so a run of pairs measures the displacement with the noise
  of its two end epochs only — the carrier's continuity (AMENDMENTS A61 F10/F11).
- **Two global outputs.** `traj_N` is the best estimate of where the platform is in ENU and keeps revising the whole
  trajectory's absolute position as pseudorange information accumulates; `traj_N_odom` holds the registration's
  estimate at activation fixed and keeps every drift correction since — GNSS-corrected odometry, the like-for-like of
  FAST-LIVGO's frame fixed after initialisation (A61 F13). An SE(3)-aligned ATE counts `traj_N`'s absolute revision as
  error, so the Table II comparison uses `traj_N_odom`; both are reported.
- **Outlier handling is 0/1 membership only**: signal masks, a GNSS-only robust solution, clock-jump detection,
  cycle-slip detectors, Baarda w-tests with iterative FDE, post-fit rollback, lock-out recovery and an integrity
  referee. No robust kernels or data-dependent covariance inflation inside the filter.
- **Initialisation, outages, integrity**: SPP-window or LIO-aided multi-epoch initialisation (few satellites),
  outage/recovery/re-initialisation, velocity and heading integrity checks.
- **Deterministic offline replay** (`fastlivo_replay`) reads a bag in record order, to support deterministic checks on a fixed machine and image; seeds select the first LiDAR frame.

The loose-coupling arm (receiver PVT, `mode: loose_pvt`) and ablations (without pseudorange / Doppler / TDCP) share
everything else, to support raw-observation versus receiver-solution ablations.

One upstream defect had to be fixed for reproducibility: `VIOManager::updateState` reads out of bounds near the image
border on the coarse pyramid levels, so two runs on the same bag diverged. A 5-line bounds guard makes runs
bit-identical; the U (FAST-LIVO2) arm is `0d2c034` + this guard (see `docs/dev/REF_R_GATES.md`).


### Outputs

Written to `output_dir` (replay) — the GNSS files only when `gnss.enable: true`:

| File | Content |
|---|---|
| `traj_W.tum` | host pose T_{W←I} at every step end (the FAST-LIVO2 trajectory) |
| `traj_N.tum`, `traj_ant_N.tum`, `traj_N_geo.csv`, `anchor.yaml` | global pose T_{N←I} in the local ENU frame N, antenna position, WGS-84, the ENU anchor (from activation on) |
| `traj_N_odom.tum`, `traj_ant_N_odom.tum` | the odometry output: the same pose / antenna with the registration's estimate at activation held fixed (A61 F13) |
| `gnss_epochs.csv` | one row per GNSS epoch: phase, satellites, per-class candidates/accepted/rejected by reason, NIS, gain shares, registration and clock states with σ |
| `gnss_sats.csv` | optional per-satellite rows (`gnss.output.sat_csv: true`) |
| `events.log`, `summary.json` | ACTIVATE, SLIP, CLOCK_JUMP, ROLLBACK, OUTAGE, ... and the run summary (mean NIS per class, acceptance, TDCP coverage) |
| `state_trace.txt`, `filter_cov.csv`, `run_manifest.json`, `replay_stats.json` | hexfloat state trace, covariance diagnostics, build/config/bag identity |

Topics published with GNSS on: `/gnss/global_odometry`, `/gnss/global_odometry_smooth`, `/gnss/fused_fix`,
`/gnss/spp_fix`, `/gnss/diagnostics` and the TF `enu → camera_init` (= T_{N←W}).


For build and smoke-test commands use [REPRODUCING.md](REPRODUCING.md). Historical
ROS replay and scoring procedures live in the design records; published results
and their exact output choices are in [RESULTS.md](RESULTS.md).
