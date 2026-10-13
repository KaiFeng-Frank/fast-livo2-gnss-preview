#!/usr/bin/env python3
# SPDX-License-Identifier: BSD-3-Clause
"""Summarize individual GoogleTest cases; CTest executable counts alone hide skips."""
import argparse
import json
from pathlib import Path
import xml.etree.ElementTree as ET


def summarize(directory):
    files = sorted(directory.glob("*.xml"))
    if len(files) != 21:
        raise ValueError(f"expected 21 core executable reports, got {len(files)}")
    counts = dict(tests=0, passed=0, skipped=0, failures=0, disabled=0)
    skip_details = []
    for path in files:
        root = ET.parse(path).getroot()
        for key in ("tests", "failures", "disabled"):
            counts[key] += int(root.get(key, "0"))
        for case in root.iter("testcase"):
            skip = case.find("skipped")
            if skip is not None:
                counts["skipped"] += 1
                skip_details.append({"test": case.get("classname", "") + "." + case.get("name", ""),
                                     "reason": skip.get("message", "")})
    counts["passed"] = counts["tests"] - counts["failures"] - counts["skipped"] - counts["disabled"]
    report = {"schema": "fastlivo.gnss.preview.core.v1", "executables": len(files),
              "totals": counts, "skip_details": skip_details}
    (directory.parent / "core.json").write_text(json.dumps(report, indent=2) + "\n")
    print(f"Core cases in {len(files)} executables: {counts['passed']} passed, {counts['skipped']} skipped, "
          f"{counts['failures']} failures, {counts['disabled']} disabled")
    if counts["failures"]:
        raise ValueError("core case failures")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    summarize(parser.parse_args().directory)
