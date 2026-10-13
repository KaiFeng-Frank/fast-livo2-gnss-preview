# SPDX-License-Identifier: BSD-3-Clause
"""Project-root layout of the A80 data (what tools/eval/run_matrix.py and eval_ate.py expect, under a project root of its own).

The new data live on their own disk, /data2/fl2g, which is used as a second project root:

    run_matrix.py ... --root /data2/fl2g --lidar smartpnt_mini|mars_avia     with FL2G_BUILD_ROOT=${FL2G_ROOT:?set FL2G_ROOT to the project directory}
                                                                              (the builds stay under the main root's wt/)

so that, relative to that root,
    <family>/bags/<seq>.bag(+.sha256)      the replay bag (lidars.json bag_dir), a relative symlink to the builder's output
    <family>/bags/<seq>.fetch.json         {"brdc": [paths relative to the root]} -> gnss.nav_rinex (run_matrix.gnss_inputs)
    audit/<seq>/gt/                        gt_clean.tum, gt_audited.tum, gt_timebase.yaml, trust_mask.csv (gtbase.load_gt)
Links are relative inside the root, so they resolve in the containers (which mount only the root).
"""
import json
import os


def _link(target_abs, link_abs):
    os.makedirs(os.path.dirname(link_abs), exist_ok=True)
    rel = os.path.relpath(target_abs, os.path.dirname(link_abs))
    if os.path.islink(link_abs) or os.path.exists(link_abs):
        os.remove(link_abs)
    os.symlink(rel, link_abs)


def publish(root, family, seq, bag, gt_dir, nav_files, sha256, extra=None):
    """Create the links and the fetch record under `root`. Paths given absolute, all inside `root`."""
    root = os.path.abspath(root)
    for p in [bag, gt_dir] + list(nav_files):
        if not os.path.abspath(p).startswith(root + os.sep):
            raise ValueError("%s is outside the project root %s" % (p, root))
    bags = os.path.join(root, family, "bags")
    _link(os.path.abspath(bag), os.path.join(bags, seq + ".bag"))
    with open(os.path.join(bags, seq + ".bag.sha256"), "w") as f:
        f.write("%s  %s.bag\n" % (sha256, seq))
    rec = {"sequence": seq, "bag": os.path.relpath(bag, root), "bag_sha256": sha256,
           "brdc": [os.path.relpath(p, root) for p in nav_files]}
    if extra:
        rec.update(extra)
    with open(os.path.join(bags, seq + ".fetch.json"), "w") as f:
        json.dump(rec, f, indent=1)
        f.write("\n")
    _link(os.path.abspath(gt_dir), os.path.join(root, "audit", seq, "gt"))
    return rec
