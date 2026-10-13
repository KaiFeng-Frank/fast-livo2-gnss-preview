# FAST-LIVO2-GNSS · research preview

Experimental raw-GNSS fusion for [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2):
pseudorange, Doppler and time-differenced carrier phase (TDCP), with a ROS-free
C++ core, offline replay, data diagnostics and evaluation tools.

**Status: research preview, 2026-10-13.** This implementation has not reached the
reported FAST-LIVGO performance or established consistent improvements over its
FAST-LIVO2 host across sequences. Code, evaluation procedures and negative results
are provided for inspection and reuse. `LIVGO-literal` is our implementation of
the published method, not the FAST-LIVGO authors' source code.

**中文说明：** 这是原始 GNSS 紧耦合的实验性研究实现，目前尚未达到 FAST-LIVGO
的报告性能，也尚未建立跨序列稳定优于宿主的证据。公开代码、评测工具及已知失败案例，
供研究和复用。工程测试通过不代表融合精度已通过验收。

## Start here

Use the **`v0.1.0-research-preview`** source tag in this repository. Its GitHub
source archives and Git checkout refer to the same public source tree.

```bash
git clone --branch v0.1.0-research-preview --depth 1 \
  https://github.com/KaiFeng-Frank/fast-livo2-gnss-preview.git
cd fast-livo2-gnss-preview
```

1. [Build, test and run the synthetic example](docs/REPRODUCING.md).
2. [Read the current results and verify the published numerical evidence](docs/RESULTS.md).
3. [Check source/configuration identities and the publication scope](docs/PROVENANCE.md).
4. [Read licensing and third-party notices](docs/LICENSING.md).

The minimum example runs without ROS, a private build machine, a cloud account
or a dataset download. It exercises GNSS activation on simulated observations.
The historical evidence example verifies archived scoring statistics; a full
real-data trajectory rescore additionally requires ground truth obtained from
the dataset provider. Neither operation rebuilds the original sensor replay.

## What the evidence currently supports

| Experiment | Outcome | Interpretation |
|---|---|---|
| A79: seven M3DGR sequences, Mid-360 host, seed 0 | 2/7 meet the recorded paper targets; GNSS is inactive on both | These two passes do not demonstrate GNSS benefit |
| A79: five sequences with active GNSS | Antenna-point ATE improves on two and worsens on three | Fusion gains are not consistent; the uncorrected IMU-point score improves only on Outdoor01 |
| A81/A82: development noise recheck | SmartPNT Doppler NIS/dof **1.2985784051776035**, allowed upper bound **1.25** | The preregistered gate failed; no frozen configuration, final arm or sealed evaluation results |

For example, Varying-illu03 antenna-point ATE is **1.7148 m (U)** versus
**2.2742 m (T)**; its recorded paper target is **0.23 m**. The
[complete table](docs/RESULTS.md) keeps both scoring conventions, GNSS activation
and the historical source identities. Differences in host, sensor selection,
reference point and experimental procedure prevent a claim of strict reproduction.

## Code map

| Path | Purpose |
|---|---|
| `gnss/` | ROS-free GNSS estimator, simulator and C++ tests |
| `gnss_io/` | RINEX, UBX and runtime ROS1 wire-format decoding |
| `src/`, `include/`, `ros/` | FAST-LIVO2 host, integration and offline replay |
| `config/`, `launch/` | Dataset configurations and experimental arm overlays |
| `tools/` | Data checks, calibration, scoring and experiment controls |
| `results/` | Public numerical evidence and provenance |
| `docs/dev/` | Historical design, decisions and diagnostics; not the release quick start |

The `T` arm couples the host state to registration, clock and GNSS states.
`traj_W` is the host trajectory; `traj_N` is the changing global estimate;
`traj_N_odom` freezes the registration at activation for GNSS-corrected odometry.
Choosing an output and reference point changes what an ATE score measures.
[Architecture and output conventions](docs/ARCHITECTURE.md) describe these choices.

## Scope and limitations

There is no RTK/PPP or integer ambiguity resolution. Initialization can fail
with poor satellite availability; carrier slips, multipath and host drift affect
results. The map is not globally corrected. The ROS1 integration and dataset
tooling are retained for research; the portable preview checks cover the core,
tools and examples rather than a fresh full ROS sensor replay.

Historical bitwise regression applies to the recorded machine, compiler and
image. Rebuilding elsewhere does not promise bitwise-identical trajectories.
The archived SmartPNT evaluation split remains unused in this release.

## Licensing and credit

FAST-LIVO2-derived code and ROS glue retain GPL-2.0-only; the new ROS-free core,
I/O and Python tools use BSD-3-Clause. Vendored RTKLIB-EX and other exceptions
retain their own notices. See [the component breakdown](docs/LICENSING.md) and
[third-party credits](THIRD_PARTY_NOTICES.md).

This preview distributes source and selected research evidence. It does not
distribute the private archive's Docker images, executables or linked libraries.
The full ROS host's GPL-2.0-only/rpg_vikit GPLv3 compatibility issue remains
unresolved; source-only packaging is not a declaration that a linked combined
work is license-compatible.

Thanks to [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2),
[RTKLIB-EX](https://github.com/rtklibexplorer/RTKLIB),
[M3DGR](https://github.com/sjtuyinjie/M3DGR), and the FAST-LIVGO authors.
This is an independent research project, without an upstream endorsement.
