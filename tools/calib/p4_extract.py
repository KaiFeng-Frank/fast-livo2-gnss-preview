# SPDX-License-Identifier: BSD-3-Clause
"""P4 calibration (TEST_PLAN §3, AMENDMENTS A58): the receiver's own per-row numbers that the diagnostics CSVs do not
carry, read from a development-set bag (A2: development sequences only).

  python3 tools/calib/p4_extract.py <bag> <out_dir>

Writes <out_dir>/meas_rows.csv (primary signal of every satellite of every /ublox_driver/range_meas epoch: GPST week/tow
of the epoch, sat, frequency, C/N0, psr_std [m], dopp_std [Hz], cp_std, status, LLI — the core's primary-signal rule,
tools/fl2g/obs.py) and <out_dir>/pvt.csv (every /ublox_driver/receiver_pvt message). Needs the ROS Python of the image
(rosbag); run it through tools/dev/remote_tool.sh.
"""
import csv
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.dirname(HERE))

from fl2g import gnss_msgs, gnsstime, obs, rosio  # noqa: E402


def main(argv):
    if len(argv) != 3:
        print(__doc__)
        return 2
    bag, out = argv[1], argv[2]
    from fl2g.sequences import seq_class
    name = os.path.splitext(os.path.basename(bag))[0]
    if seq_class(name) != "development":
        print("refusing %s: not a development-set sequence (AMENDMENTS A2)" % name)
        return 3
    os.makedirs(out, exist_ok=True)
    rb = rosio.RawBag(bag)
    n_ep = n_rows = n_pvt = 0
    with open(os.path.join(out, "meas_rows.csv"), "w", newline="") as fm, open(os.path.join(out, "pvt.csv"), "w", newline="") as fp:
        wm, wp = csv.writer(fm), csv.writer(fp)
        wm.writerow(["week", "tow", "t_record", "sat", "sys", "freq_hz", "cn0", "psr_std_m", "dopp_std_hz", "cp_std", "status", "lli"])
        wp.writerow(["t_gpst", "week", "tow", "t_record", "fix_type", "valid_fix", "diff_soln", "carr_soln", "num_sv", "lat", "lon",
                     "h_ell", "h_acc", "v_acc", "p_dop", "vel_n", "vel_e", "vel_d", "vel_acc"])
        for topic, _dt, raw, t_rec, pytype in rb.iter_raw(topics=[gnss_msgs.TOPIC_MEAS, gnss_msgs.TOPIC_PVT]):
            m = rb.deserialize(pytype, raw)
            if topic == gnss_msgs.TOPIC_PVT:
                p = gnss_msgs.pvt(m, t_rec)
                wk, tow = gnsstime.week_tow(p["t_gpst"])
                wp.writerow(["%.9f" % p["t_gpst"], wk, "%.9f" % tow, "%.9f" % t_rec, p["fix_type"], int(p["valid_fix"]), int(p["diff_soln"]),
                             p["carr_soln"], p["num_sv"], "%.10f" % p["lat"], "%.10f" % p["lon"], "%.4f" % p["h_ell"], p["h_acc"],
                             p["v_acc"], p["p_dop"], p["vel_n"], p["vel_e"], p["vel_d"], p["vel_acc"]])
                n_pvt += 1
                continue
            ep = gnss_msgs.meas_epoch(m, t_rec)
            n_ep += 1
            for s in ep["sats"]:
                ps = obs.primary_slot(s)
                if ps is None:
                    continue
                _k, sl, _code, _kglo = ps
                wm.writerow([s["week"], "%.9f" % s["tow"], "%.9f" % t_rec, s["sat"], s["sys"], sl["freq"], sl["cn0"], sl["psr_std"],
                             sl["dopp_std"], sl["cp_std"], sl["status"], sl["lli"]])
                n_rows += 1
    rb.close()
    print("%s: %d epochs, %d primary-signal rows, %d PVT messages -> %s" % (name, n_ep, n_rows, n_pvt, out))
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv))
