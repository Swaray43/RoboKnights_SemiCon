#!/usr/bin/env python3
"""
ci/load/run_workloads.py -- Phase 2 Step 5: SmartNIC / ML / mixed workloads on bridge_top.

Runs tb_bridge_rand in workload mode (+WL=n) for each scenario, parses the
"LOAD:" line every run prints, and writes one JSON file. With --base REF it
also builds that git ref in a temporary worktree, runs the same scenarios
there and prints a before/after table.

    python3 ci/load/run_workloads.py                     # quick set on the working tree
    python3 ci/load/run_workloads.py --full              # adds the long ML tensors
    python3 ci/load/run_workloads.py W1 W3_17            # selected scenarios
    python3 ci/load/run_workloads.py --base step4-base   # before/after table
    python3 ci/load/run_workloads.py --out x.json        # where to write results

Rates are at the 250 MHz design clock (B/clk x 0.25 = GB/s).
"""
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]

# id: (description, plusargs, quick?)
SCEN = {
    "W1":      ("SmartNIC IMIX 64/596/1520 B (7:4:1), bursty, 200 packets", ["+WL=1", "+WL_PKTS=200"], True),
    "W1_B2B":  ("SmartNIC back-to-back 64 B, 200 packets",                ["+WL=4", "+WL_PKTS=200"], True),
    "W2_64K":  ("ML 64 KB tensors x4, unaligned",                         ["+WL=2", "+WL_KB=64", "+WL_TENSORS=4"], True),
    "W2_1M":   ("ML 1 MB tensor x1, unaligned",                           ["+WL=2", "+WL_KB=1024", "+WL_TENSORS=1"], True),
    "W2_4K":   ("ML 64 KB tensors, bases 64 B below a 4 KB line",          ["+WL=5", "+WL_TENSORS=2"], True),
    "W3_11":   ("Mixed: ML saturating + NIC IMIX 100 pkts, weights 1:1",  ["+WL=3", "+WL_PKTS=100", "+WL_KB=256", "+W0=1", "+W1=1"], True),
    "W3_13":   ("Mixed, weights NIC:ML 1:3",                              ["+WL=3", "+WL_PKTS=100", "+WL_KB=256", "+W0=1", "+W1=3"], True),
    "W3_17":   ("Mixed, weights NIC:ML 1:7",                              ["+WL=3", "+WL_PKTS=100", "+WL_KB=256", "+W0=1", "+W1=7"], True),
    "W3_N300": ("Mixed 1:3 + channel noise 300 ppm, bursts of 4",          ["+WL=3", "+WL_PKTS=100", "+WL_KB=256", "+W0=1", "+W1=3", "+NOISE_PPM=300", "+BURST=4"], True),
    "W3_N1K":  ("Mixed 1:3 + channel noise 1000 ppm",                      ["+WL=3", "+WL_PKTS=100", "+WL_KB=256", "+W0=1", "+W1=3", "+NOISE_PPM=1000"], True),
    "W2_4M":   ("ML 4 MB tensor",                                          ["+WL=2", "+WL_KB=4096", "+WL_TENSORS=1"], False),
    "W2_16M":  ("ML 16 MB tensor (session limit, 16 MB - 4 KB)",           ["+WL=2", "+WL_KB=16380", "+WL_TENSORS=1"], False),
}


def bridge_srcs(root):
    out = []
    for line in (root / "bridge_top/bridge_top.f").read_text().splitlines():
        line = line.split("//")[0].strip()
        if not line:
            continue
        if line.startswith("+incdir+"):
            out.append("-I" + str(root / line[8:]))
        else:
            out.append(str(root / line))
    return out


def build(root, bench, tmp):
    vvp = Path(tmp) / "load.vvp"
    cmd = ["iverilog", "-g2012", "-DSIMULATION", "-s", "tb_bridge_rand", "-o", str(vvp)] + bridge_srcs(root) + [str(bench)]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        sys.exit("compile failed:\n" + p.stdout + p.stderr)
    return vvp


def run_set(vvp, ids, tmp):
    res = {}
    for sid in ids:
        desc, args, _ = SCEN[sid]
        p = subprocess.run(["vvp", "-n", str(vvp)] + args, cwd=tmp, capture_output=True, text=True, timeout=7200)
        out = p.stdout + p.stderr
        m = re.search(r"^LOAD:(.*)$", out, re.M)
        ok = bool(re.search(r"RESULT: PASS", out))
        d = {"desc": desc, "pass": ok}
        if m:
            for k, v in re.findall(r"(\w+)=(-?[\d.]+)", m.group(1)):
                d[k] = float(v)
        res[sid] = d
        print(f"  {sid:8s} {'PASS' if ok else 'FAIL'}  "
              f"ch0 {d.get('ch0_gbps', 0):.2f} GB/s  ch1 {d.get('ch1_gbps', 0):.2f} GB/s  "
              f"NIC p99 {d.get('pkt_lat99', 0):.0f} ns  replays {d.get('replays', 0):.0f}", flush=True)
    return res


def main():
    args = sys.argv[1:]
    full = "--full" in args
    base = None
    out = ROOT / "ci_out" / "load.json"
    ids = []
    i = 0
    while i < len(args):
        a = args[i]
        if a == "--base":
            base = args[i + 1]; i += 2; continue
        if a == "--out":
            out = Path(args[i + 1]); i += 2; continue
        if a in SCEN:
            ids.append(a)
        i += 1
    if not ids:
        ids = [k for k, v in SCEN.items() if v[2] or full]
    bench = ROOT / "bridge_top/tb/tb_bridge_rand.v"
    result = {}
    with tempfile.TemporaryDirectory() as tmp:
        print(f"== this tree ({ROOT})")
        result["new"] = run_set(build(ROOT, bench, tmp), ids, tmp)
        if base:
            wt = Path(tmp) / "base"
            subprocess.run(["git", "-C", str(ROOT), "worktree", "add", "-q", "--detach", str(wt), base], check=True)
            try:
                print(f"== base {base}")
                result["base"] = run_set(build(wt, bench, tmp), ids, tmp)
            finally:
                subprocess.run(["git", "-C", str(ROOT), "worktree", "remove", "--force", str(wt)])
    out.parent.mkdir(exist_ok=True)
    out.write_text(json.dumps(result, indent=1))
    print(f"results: {out}")
    return 0 if all(v["pass"] for r in result.values() for v in r.values()) else 1


if __name__ == "__main__":
    sys.exit(main())
