# Reproducing the research preview

This preview has a portable software check and an entirely synthetic GNSS example.
The recorded real-data results have a separate verification path. None of these
checks establish stable real-data fusion gains or a FAST-LIVGO reproduction.

## Clean local setup

Use Linux, a C++17 compiler, CMake 3.20 or later, Eigen 3 and GoogleTest. The
validated portable Python profile uses Python 3.11. The GitHub workflow uses a fresh
Ubuntu 24.04 runner with Python 3.11. ROS is unnecessary for these checks.

On Ubuntu, with a Python 3.11 interpreter available, from the repository root:

```bash
sudo apt-get update
sudo apt-get install -y build-essential cmake libeigen3-dev libgtest-dev python3-venv
python3.11 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-preview.txt
bash scripts/check_preview.sh
```

The workflow obtains Python 3.11 with `actions/setup-python`; local machines need
their own Python 3.11 installation. This receipt does not validate other Python
versions. If CMake selects dependencies
from an unrelated Conda environment, configure with `CMAKE_PREFIX_PATH=/usr`.

`bash scripts/check_preview.sh` is the complete check command. It builds the
ROS-free core, runs all 21 CTest executables, records individual GoogleTest case
counts, runs five Python suites, verifies the archived result aggregates, and
generates and scores the synthetic example. No sensor dataset is downloaded.

Outputs are under `build_preview/reports/<run>/`. `core.json` and `python.json`
contain pass, skip and failure counts; `core/*.xml` and the suite logs contain
individual skip reasons. Missing external fixtures are reported as skips. A
present but malformed fixture continues to fail its original checks.

You can select an output directory and build concurrency:

```bash
PREVIEW_BUILD_DIR=/tmp/fastlivo-preview-build PREVIEW_JOBS=2 bash scripts/check_preview.sh
```

The script defaults to two compilation jobs and one test executable at a time.
Set `PREVIEW_PYTHON` to a Python interpreter path when no environment is activated.
The test profile uses strict floating point contraction settings and native CPU
instructions. Rebuilding on another CPU/compiler need not produce identical
trajectory bytes.

## Synthetic GNSS activation and scoring

`tools/preview/make_nav.py` independently authors 24 idealized GPS ephemerides:
six orbital planes and four satellites per plane, expressed in RINEX 3.04. It
contains no recorded receiver measurements, dataset message definitions or IGS
navigation data. The date in the file is a simulated time origin.

`preview_demo` calls the existing ROS-free simulator with a synthetic host, seed
0, a 150 m path and the `open_sky` model-matched configuration. It exercises the
real `GnssFusion` state machine and its raw pseudorange, Doppler and TDCP updates.
It must activate and accept all three measurement classes; a failed activation
or missing accepted rows makes the command fail. This example does not execute
the ROS bag decoder or the FAST-LIVO2 sensor estimator.

To run it separately after building:

```bash
python tools/preview/make_nav.py build_preview/example/synthetic.nav
build_preview/preview_demo build_preview/example/synthetic.nav build_preview/example
python tools/preview/score_demo.py build_preview/example
```

The generated files are:

| File | Meaning |
|---|---|
| `synthetic.nav` | Entirely invented GPS ephemerides |
| `config.txt` | Full effective GNSS configuration; navigation filename normalized to a relative name |
| `truth.tum` | True synthetic antenna positions |
| `host.tum` | Synthetic host antenna positions in its map frame |
| `global.tum` | Fused global antenna positions, corresponding to `traj_ant_N` |
| `odometry.tum` | Fused antenna odometry, corresponding to `traj_ant_N_odom` |
| `summary.json` | Activation, accepted measurement rows and simulator ATE |
| `score.json` | Independent NumPy ATE and hashes of all scoring inputs |

The `.tum` files contain antenna positions with identity quaternions; they are
position-scoring inputs, not estimated attitudes. Timestamps are seconds since
the simulated scene start. All four files use the same samples, at 1 Hz from
activation plus a 5 s settling interval. Each estimate is aligned to truth with
a proper rigid rotation and translation, without scale. The reported value is
the position ATE RMSE in metres. `score_demo.py` recomputes it from the exported
samples and checks agreement with the C++ simulator.

The [preview validation receipt](PREVIEW_VALIDATION.json) records the observed
activation, accepted rows and scores. These synthetic scores check execution and
scoring. The simulated host and GNSS observations follow the model assumed by
the filter, so they are not independent evidence of real-data accuracy. The
global and odometry outputs are both reported even when their errors differ.

## Verify the published real-data findings

The default archived check recomputes 28 A79 ATE values from paired-point
sufficient statistics and the A81 noise factors from recorded NIS sums and
degrees of freedom:

```bash
python tools/preview/recompute_results.py
```

It verifies the published numerical calculations and recorded negative
conclusions. It does not repeat timestamp association, ground-truth preprocessing
or sensor replay. See [Results](RESULTS.md) for the configurations, output points
and evidence boundaries.

To perform full trajectory scoring of the included Varying-illu03 estimate poses,
obtain the original unshifted M3DGR ground-truth TUM independently and run:

```bash
python tools/preview/recompute_results.py --gt /path/to/Varying-illu03_GT.txt
```

The tool checks its normalized identity, applies the recorded time shift, repeats
association and rigid alignment, and checks both host and fused errors. The
ground truth is not bundled. These are archived estimate poses, so this command
does not rerun the sensors.

## What the portable test counts cover

The historical delivery's 482 passed / 6 skipped Python checks and 21/21 CTest
executables describe its original environment and available inputs. The public
tree omits the raw wire fixtures and private operational environment. Its exact
case counts are reported separately in [PREVIEW_VALIDATION.json](PREVIEW_VALIDATION.json)
and regenerated on each run.

The Python suites are `tools/tests`, `tools/calib/tests`, `tools/datasets/tests`,
`tools/eval/tests` and `tools/eval/livgo`. Expected optional skips include the ROS
Python checks, an M3DGR reference bag, evo, the original paper text/checksum file
and optionally the `7z` archive tool. Core skips include the omitted wire fixtures,
archived broadcast-orbit fixtures and explicitly optional study/replay modes.
An executable can pass while individual cases are skipped; use `core.json` to
assess coverage.

ROS sensor replay remains a separate integration path requiring ROS Noetic, host
dependencies, an independently obtained dataset and its calibration inputs.
The source and configuration files are provided; this portable check does not
validate a complete ROS build or a new real-data sensor replay. Consult the
[license and dependency notes](LICENSING.md) before distributing a linked host
build.
