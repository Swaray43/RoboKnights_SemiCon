#!/usr/bin/env python3
"""
ci/load/compare.py -- Step 5 before/after table from run_workloads.py JSON files.

    python3 ci/load/compare.py base.json new.json [more.json ...] [--labels base,A+B,C]

Each file holds {"<set>": {"<scenario>": {metric: value, ...}}}; the first set
of each file is used. Prints a Markdown table (and appends it to
GITHUB_STEP_SUMMARY when set). Exits 1 if a scenario failed or a gated
metric got worse than ci/load/thresholds.json allows between the first and
the last column.
"""
import json
import os
import sys
from pathlib import Path

ROWS = [
    # (label, metric key, unit, better)  better: +1 higher is better, -1 lower is better
    ("NIC throughput",          "ch0_gbps",  "GB/s", +1),
    ("NIC packet rate",         "ch0_mpps",  "Mpps", +1),
    ("NIC packet latency p50",  "pkt_lat50", "ns",   -1),
    ("NIC packet latency p99",  "pkt_lat99", "ns",   -1),
    ("NIC TLP latency p99",     "ch0_lat99", "ns",   -1),
    ("ML throughput",           "ch1_gbps",  "GB/s", +1),
    ("ML TLP latency p99",      "ch1_lat99", "ns",   -1),
    ("interrupt rate",          "msix_mps",  "M/s",  +1),
    ("NIC share (mixed)",       "share0",    "",     0),
    ("Jain fairness (mixed)",   "jain",      "",     +1),
    ("replays",                 "replays",   "",     -1),
    ("FEC corrected",           "fec_fix",   "",     0),
    ("peak CDC FIFO",           "pk_cdc",    "of 16", 0),
    ("peak write buffer",       "pk_wdb",    "of 16", 0),
    ("peak replay outstanding", "pk_rpl",    "of 32", 0),
    ("peak RX Flit FIFO",       "pk_rxf",    "of 4", 0),
]


def fmt(v, key):
    if v is None:
        return "-"
    if key.endswith("gbps") or key in ("share0", "jain", "ch0_mpps", "msix_mps"):
        return f"{v:.3f}"
    return f"{v:.0f}"


def main():
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    labels = None
    for a in sys.argv[1:]:
        if a.startswith("--labels="):
            labels = a.split("=", 1)[1].split(",")
    sets = []
    for f in args:
        d = json.loads(Path(f).read_text())
        sets.append(next(iter(d.values())))
    labels = labels or [Path(f).stem for f in args]
    thr_f = Path(__file__).with_name("thresholds.json")
    thr = json.loads(thr_f.read_text()) if thr_f.exists() else {}
    out = ["### Step 5 workloads: " + " vs ".join(labels), ""]
    bad = []
    scen = [s for s in sets[0] if all(s in x for x in sets)]
    for s in scen:
        out += [f"**{s}** -- {sets[0][s]['desc']}", "",
                "| Metric | " + " | ".join(labels) + " | change |",
                "|---|" + "---:|" * len(labels) + "---:|"]
        for lab, key, unit, better in ROWS:
            vals = [x[s].get(key) for x in sets]
            if all(v is None or v == 0 for v in vals):
                continue
            ch = ""
            a, b = vals[0], vals[-1]
            if a not in (None, 0) and b is not None:
                r = b / a
                ch = f"{r:.1f}x" if r >= 10 else (f"1/{1 / r:.0f}" if 0 < r <= 0.1 else f"{(r - 1) * 100:+.1f}%")
                lim = thr.get(key)
                if lim is not None and better != 0:
                    worse = (b < a * (1 - lim)) if better > 0 else (b > a * (1 + lim))
                    if worse:
                        bad.append(f"{s} {key}: {a} -> {b}")
                        ch += " **worse**"
            out.append(f"| {lab}{' (' + unit + ')' if unit else ''} | " +
                       " | ".join(fmt(v, key) for v in vals) + f" | {ch} |")
        passed = [("PASS" if x[s]["pass"] else "FAIL") for x in sets]
        out.append("| data check | " + " | ".join(passed) + " | |")
        out.append("")
        if not sets[-1][s]["pass"]:
            bad.append(f"{s}: data check FAILED")
    if bad:
        out += ["**Regressions:**", ""] + [f"- {b}" for b in bad]
    text = "\n".join(out)
    print(text)
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as fh:
            fh.write(text + "\n")
    return 1 if bad else 0


if __name__ == "__main__":
    sys.exit(main())
