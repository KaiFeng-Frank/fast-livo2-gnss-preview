#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Run each tool suite in its own interpreter, retaining exact pass/skip/failure counts."""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import unittest

ROOT = Path(__file__).resolve().parents[2]
SUITES = ("tools/tests", "tools/calib/tests", "tools/datasets/tests", "tools/eval/tests", "tools/eval/livgo")


def run_suite(suite, report_dir):
    tests = unittest.defaultTestLoader.discover(str(ROOT / suite), pattern="test_*.py")
    result = unittest.TextTestRunner(verbosity=2).run(tests)
    report = {
        "suite": suite,
        "tests": result.testsRun,
        "passed": result.testsRun - len(result.skipped) - len(result.failures) - len(result.errors),
        "skipped": len(result.skipped),
        "failures": len(result.failures),
        "errors": len(result.errors),
        "skip_details": [{"test": str(test), "reason": why} for test, why in result.skipped],
    }
    name = suite.replace("/", "_")
    (report_dir / (name + ".json")).write_text(json.dumps(report, indent=2) + "\n")
    return 0 if result.wasSuccessful() else 1


def run_all(report_dir):
    report_dir.mkdir(parents=True, exist_ok=True)
    totals = dict(tests=0, passed=0, skipped=0, failures=0, errors=0)
    reports = []
    failed = False
    for suite in SUITES:
        name = suite.replace("/", "_")
        with (report_dir / (name + ".log")).open("w") as log:
            process = subprocess.run(
                [sys.executable, str(Path(__file__).resolve()), str(report_dir), "--suite", suite],
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, check=False,
            )
        report_path = report_dir / (name + ".json")
        if not report_path.is_file():
            print(f"{suite}: no result; inspect {name}.log", flush=True)
            failed = True
            continue
        report = json.loads(report_path.read_text())
        reports.append(report)
        for key in totals:
            totals[key] += report[key]
        print(f"{suite}: {report['passed']} passed, {report['skipped']} skipped, "
              f"{report['failures']} failures, {report['errors']} errors", flush=True)
        failed |= process.returncode != 0
    aggregate = {"schema": "fastlivo.gnss.preview.python.v1", "totals": totals, "suites": reports}
    (report_dir / "python.json").write_text(json.dumps(aggregate, indent=2) + "\n")
    return 1 if failed else 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report_dir", type=Path)
    parser.add_argument("--suite", choices=SUITES)
    args = parser.parse_args()
    args.report_dir = args.report_dir.resolve()
    sys.exit(run_suite(args.suite, args.report_dir) if args.suite else run_all(args.report_dir))
