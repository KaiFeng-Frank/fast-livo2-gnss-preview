# SPDX-License-Identifier: BSD-3-Clause
"""Shared helpers of the P3-D evaluation tools (tools/eval): registries, the A2 phase guard, project paths, TUM I/O,
atomic JSON, hashing. Python >= 3.8 (the project image), stdlib + numpy (numpy only where trajectories are read).

A2 (AMENDMENTS): on evaluation-set sequences nothing may compute or display an estimation accuracy, trajectory or
GNSS-filter result before the evaluation phase; the development set may be used freely. Every tool here calls
`check_phase` before it downloads, replays or scores a sequence, and an evaluation-class (or unknown) sequence needs
the explicit `--evaluation-phase` switch. The class itself comes from WP-g's tools/fl2g/sequences.py (one source).
"""
import datetime
import hashlib
import json
import os
import re
import socket
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
TOOLS = os.path.dirname(HERE)
REPO = os.path.dirname(TOOLS)
if TOOLS not in sys.path:
    sys.path.insert(0, TOOLS)

from fl2g.sequences import seq_class as _fl2g_seq_class  # noqa: E402

DEFAULT_ROOT = os.path.expanduser("~/fastlivo2_gnss_data")
IMAGE_DEFAULT = "fastlivo2-gnss:noetic"
SEEDS_A46 = (0, 1, 2, 3, 4)          # A46: five seeds per cell, A30 semantics (k-th LiDAR frame opens the view)
MAX_CONCURRENT_REPLAYS = 8           # A41
T_MAX_DIFF_S = 0.05                  # TEST_PLAN §4 E1 / task: evo_ape --t_max_diff 0.05


class PhaseError(Exception):
    """An evaluation-set (or unknown) sequence was requested outside the evaluation phase (A2)."""


class Blocked(Exception):
    """A step cannot run because an input owned by someone else is missing; the message says which."""


# ---------------------------------------------------------------------------------------------------------------
def project_root(explicit=None):
    """FL2G_ROOT; else /r inside the project container; else ~/fastlivo2_gnss_data."""
    if explicit:
        return explicit
    env = os.environ.get("FL2G_ROOT")
    if env:
        return env
    if os.path.isdir("/r/data") and os.path.isdir("/r/wt"):
        return "/r"
    return DEFAULT_ROOT


def utc_now():
    return datetime.datetime.now(datetime.timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def host_name():
    return os.environ.get("FL2G_HOST") or socket.gethostname()


def load_json(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def atomic_write_json(path, obj):
    """Write JSON through a temporary file in the same directory + rename: readers never see half a file."""
    d = os.path.dirname(os.path.abspath(path))
    os.makedirs(d, exist_ok=True)
    fd, tmp = tempfile.mkstemp(prefix="." + os.path.basename(path) + ".", dir=d)
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as f:
            json.dump(obj, f, indent=1, ensure_ascii=False, sort_keys=False)
            f.write("\n")
        umask = os.umask(0)
        os.umask(umask)
        os.chmod(tmp, 0o666 & ~umask)          # mkstemp makes 0600; records are shared with the containers' uid
        os.replace(tmp, path)
    except BaseException:
        if os.path.exists(tmp):
            os.remove(tmp)
        raise


def sha256_file(path, chunk=8 << 20):
    h = hashlib.sha256()
    with open(path, "rb") as f:
        while True:
            b = f.read(chunk)
            if not b:
                break
            h.update(b)
    return h.hexdigest()


def sha256_text(text):
    return hashlib.sha256(text.encode("utf-8")).hexdigest()


# ---------------------------------------------------------------------------------------------------------------
def load_sequences(path=None):
    """sequences.json -> (registry dict, {name: entry})."""
    reg = load_json(path or os.path.join(HERE, "sequences.json"))
    return reg, reg["sequences"]


def load_arms(path=None):
    return load_json(path or os.path.join(HERE, "arms.json"))["arms"]


def load_lidars(path=None):
    """lidars.json -> (default name, {name: profile}); the host LiDAR profiles of AMENDMENTS A76."""
    d = load_json(path or os.path.join(HERE, "lidars.json"))
    return d["default"], d["lidars"]


def lidar_profile(name=None, path=None):
    """The profile of host LiDAR `name` (default: lidars.json's, Avia); raises Blocked for an unknown name."""
    default, lidars = load_lidars(path)
    name = name or default
    if name not in lidars:
        raise Blocked("host lidar %r is not in tools/eval/lidars.json (%s)" % (name, ", ".join(sorted(lidars))))
    out = dict(lidars[name])
    out["name"] = name
    return out


def seq_class(name):
    """A2 class from tools/fl2g/sequences.py: evaluation | development | unknown (unknown is treated as evaluation)."""
    return _fl2g_seq_class(name)


def check_phase(seq, evaluation_phase, what="this step"):
    """Raise PhaseError unless `seq` is a development sequence or the evaluation phase was requested explicitly."""
    cls = seq_class(seq)
    if cls == "development":
        return cls
    if not evaluation_phase:
        raise PhaseError("%s: %s is a%s sequence (AMENDMENTS A2): %s is allowed only in the evaluation phase "
                         "(--evaluation-phase)" % (seq, seq, "n evaluation-set" if cls == "evaluation" else "n unknown-class",
                                                   what))
    return cls


def parse_seeds(text):
    """'0-4' | '0,2,5' | '3' -> sorted unique list of non-negative ints."""
    out = set()
    for part in str(text).split(","):
        part = part.strip()
        if not part:
            continue
        m = re.fullmatch(r"(\d+)-(\d+)", part)
        if m:
            a, b = int(m.group(1)), int(m.group(2))
            if b < a:
                raise ValueError("bad seed range %r" % part)
            out.update(range(a, b + 1))
        elif re.fullmatch(r"\d+", part):
            out.add(int(part))
        else:
            raise ValueError("bad seed %r" % part)
    if not out:
        raise ValueError("no seeds in %r" % text)
    return sorted(out)


def split_list(text):
    return [x.strip() for x in str(text).split(",") if x.strip()]


SAFE_NAME = re.compile(r"^[A-Za-z0-9][A-Za-z0-9_.-]*$")


def safe_name(s, what="name"):
    if not SAFE_NAME.match(s or "") or ".." in s:
        raise ValueError("%s %r must match %s" % (what, s, SAFE_NAME.pattern))
    return s


# ---------------------------------------------------------------------------------------------------------------
# TUM trajectories ("t x y z qx qy qz qw", the evo input format)
def read_tum(path):
    """-> numpy (n, 8) float64. Comments (#) and blank lines skipped; every other row must have 8 numbers."""
    import numpy as np
    rows = []
    with open(path) as f:
        for i, ln in enumerate(f, 1):
            s = ln.strip()
            if not s or s.startswith("#"):
                continue
            tok = s.split()
            if len(tok) != 8:
                raise ValueError("%s:%d: %d columns, TUM needs 8" % (path, i, len(tok)))
            rows.append([float(x) for x in tok])
    return np.array(rows, dtype=float).reshape(-1, 8)


def sanitize_tum(a):
    """Stable sort by time and collapse rows of equal stamp to the LAST one (the step-end state; traj_W repeats the
    initial stamp while the IMU initialises, as tools/fl2g/replay.py does for state_trace). -> (b, n_reordered, n_dup)."""
    import numpy as np
    if a.shape[0] == 0:
        return a, 0, 0
    order = np.argsort(a[:, 0], kind="stable")
    b = a[order]
    keep = np.ones(b.shape[0], dtype=bool)
    keep[:-1] = b[1:, 0] != b[:-1, 0]
    return b[keep], int(np.sum(order != np.arange(a.shape[0]))), int(np.sum(~keep))


def write_tum(path, a, digits=9):
    fmt = " ".join(["%%.%df" % digits] * 8) + "\n"
    with open(path, "w") as f:
        for r in a:
            f.write(fmt % tuple(r))


# ---------------------------------------------------------------------------------------------------------------
# replay-run completion (the criterion of docs/dev/ref_r_gates/replay_run.sh, reproduced verbatim)
_DIED = re.compile(r"process has died \[pid [0-9]*, exit code (-?[0-9]*)")


def console_verdict(text):
    """-> (node_rc or None, done_line 0/1) from a roslaunch console log. The node's exit code: 0 when roslaunch printed
    'process has finished cleanly'; otherwise the first 'process has died [pid N, exit code X' (X wins over clean)."""
    node_rc = None
    if "process has finished cleanly" in text:
        node_rc = "0"
    m = _DIED.search(text)
    if m:
        node_rc = m.group(1)
    return node_rc, int("fastlivo_replay: done" in text)


def stats_complete(run_dir):
    p = os.path.join(run_dir, "replay_stats.json")
    if not os.path.isfile(p):
        return 0
    with open(p, encoding="utf-8", errors="replace") as f:
        return int('"complete": true' in f.read())


def count_cycles(run_dir):
    p = os.path.join(run_dir, "state_trace.txt")
    if not os.path.isfile(p):
        return -1
    n = 0
    with open(p, "rb") as f:
        for _ in f:
            n += 1
    return n - 1


def fmt_m(x, digits=2):
    return "—" if x is None else ("%.*f" % (digits, x))
