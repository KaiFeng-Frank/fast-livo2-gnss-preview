#!/usr/bin/env bash
# SPDX-License-Identifier: BSD-3-Clause
# setup_evo_env.sh — make evo_ape usable inside fastlivo2-gnss:noetic without touching the image (P3-D; run on the
# remote host, idempotent).
#
# Image defect (found by P3-D, reported to the maintainer): the Dockerfile installs apt python3-scipy 1.3.3 and then
# pip "evo>=1.20,<2", which pulls numpy 1.24.4; scipy 1.3.3 still calls np.typeDict (removed in numpy 1.24), so
# `evo_ape` dies on `import evo.core.lie_algebra` -> scipy.spatial. Fix for the evaluation only: the cp38 wheel of
# scipy 1.10.1 (supports numpy >= 1.19.5, < 1.27; sha256 pinned below) is installed with the image's own pip into
# <root>/tools/eval_pylib/site, and tools/eval/eval_ate.py prepends that directory to PYTHONPATH when it runs evo_ape.
# Nothing else in the image changes; evo itself stays the image's 1.31.1.
set -Eeuo pipefail
ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
IMAGE=${FL2G_IMAGE:-fastlivo2-gnss:noetic}
INDEX=${FL2G_PIP_INDEX:-https://pypi.tuna.tsinghua.edu.cn/simple}
D=$ROOT/tools/eval_pylib
WHL=scipy-1.10.1-cp38-cp38-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
SHA=07c3457ce0b3ad5124f98a86533106b643dd811dd61b548e78cf4c8786652f6f
mkdir -p "$D/wheels"
if ! echo "$SHA  $D/wheels/$WHL" | sha256sum -c --status 2>/dev/null; then
  rm -f "$D/wheels/$WHL"
  pip3 download --no-deps --only-binary=:all: --python-version 3.8 --platform manylinux2014_x86_64 "scipy==1.10.1" \
    -d "$D/wheels" -i "$INDEX"
fi
echo "$SHA  $D/wheels/$WHL" | sha256sum -c
if [ ! -f "$D/site/scipy/__init__.py" ]; then
  sudo -n docker run --rm -u 1000:1000 -e HOME=/tmp -v "$ROOT:/r" "$IMAGE" \
    pip3 install --no-deps --no-index --target /r/tools/eval_pylib/site "/r/tools/eval_pylib/wheels/$WHL"
fi
sudo -n docker run --rm -u 1000:1000 -e HOME=/tmp -e PYTHONPATH=/r/tools/eval_pylib/site -v "$ROOT:/r" "$IMAGE" bash -c '
  python3 -c "import scipy, numpy, evo.main_ape; from importlib import metadata; print(\"scipy\", scipy.__version__, scipy.__file__, \"numpy\", numpy.__version__, \"evo\", metadata.version(\"evo\"))"
  evo_ape tum --help > /dev/null && echo "setup_evo_env: evo_ape OK"'
