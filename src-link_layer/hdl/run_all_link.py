#!/usr/bin/env python3
"""
run_all_link.py -- run every link-layer unit testbench listed in files.f.

Mirrors protocol_PHY_layer/run_all.py. For each `tb` line in files.f it compiles
the bench + all `rtl` sources + that bench's extra deps with Icarus Verilog
(-g2012 -DSIMULATION), runs it, and looks for a "RESULT: PASS" / "RESULT: FAIL"
line. No RESULT line (crash, hang, stale bench) counts as FAIL.

A dep written as PHY:<file> is taken from protocol_PHY_layer/rtl/.

Usage:  python3 run_all_link.py [--files-f files.f] [--only NAME ...]
Exit code 0 only if every selected bench passes.
"""
import argparse, re, subprocess, sys, tempfile
from pathlib import Path

HERE = Path(__file__).resolve().parent
PHY_RTL = HERE.parent.parent / "protocol_PHY_layer" / "rtl"
SHARED = HERE.parent.parent / "shared"   # ticket G7: axis_tuser_defs.vh lives here
RESULT_RE = re.compile(r"RESULT:\s*(PASS|FAIL)")
# Older benches print "ALL TESTS PASSED" / "Results: N PASSED, M FAILED" instead.
ALT_PASS_RE = re.compile(r"ALL TESTS PASSED")
ALT_FAIL_RE = re.compile(r"Results:\s*\d+\s*PASSED,\s*([1-9]\d*)\s*FAILED")


def parse(path):
    rtl, tbs = [], []
    for raw in path.read_text().splitlines():
        line = raw.split("#", 1)[0].strip()
        if not line:
            continue
        p = line.split()
        if p[0] == "rtl":
            rtl.append(p[1])
        elif p[0] == "tb":
            tbs.append((p[1], p[2], p[3:]))
        else:
            print(f"warning: unrecognised line: {raw}", file=sys.stderr)
    return rtl, tbs


def resolve(dep):
    return PHY_RTL / dep[4:] if dep.startswith("PHY:") else HERE / dep


def run_one(rtl, name, bench, deps, timeout):
    bench_p = HERE / bench
    if not bench_p.exists():
        return "FAIL", f"bench not found: {bench}"
    srcs = [HERE / s for s in rtl if s.endswith(".v")] + [resolve(d) for d in deps] + [bench_p]
    missing = [str(s) for s in srcs if not s.exists()]
    if missing:
        return "FAIL", "missing sources: " + ", ".join(missing)
    with tempfile.TemporaryDirectory() as td:
        vvp = Path(td) / f"{name}.vvp"
        cmd = ["iverilog", "-g2012", "-DSIMULATION", "-I", str(HERE), "-I", str(SHARED),
               "-s", bench_p.stem, "-o", str(vvp)] + [str(s) for s in srcs]
        c = subprocess.run(cmd, capture_output=True, text=True)
        if c.returncode != 0:
            return "FAIL", "compile error: " + c.stderr.strip().splitlines()[-1]
        try:
            r = subprocess.run(["vvp", "-n", str(vvp)], capture_output=True, text=True,
                               timeout=timeout, cwd=td)
        except subprocess.TimeoutExpired:
            return "FAIL", f"timeout after {timeout}s"
    out = r.stdout + r.stderr
    m = RESULT_RE.findall(out)
    if not m:
        if ALT_FAIL_RE.search(out):
            return "FAIL", "bench reported failures"
        if ALT_PASS_RE.search(out):
            return "PASS", ""
        return "FAIL", "no self-check / no RESULT line printed"
    return ("FAIL" if "FAIL" in m else "PASS"), ""


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--files-f", default=str(HERE / "files.f"))
    ap.add_argument("--only", nargs="*")
    ap.add_argument("--timeout", type=int, default=600)
    a = ap.parse_args()
    rtl, tbs = parse(Path(a.files_f))
    if a.only:
        tbs = [t for t in tbs if t[0] in a.only]
    npass = 0
    for name, bench, deps in tbs:
        v, d = run_one(rtl, name, bench, deps, a.timeout)
        npass += v == "PASS"
        print(f"[{v}] {name}" + (f"  -- {d}" if d else ""), flush=True)
    print(f"\n{npass}/{len(tbs)} benches passed.")
    return 0 if npass == len(tbs) else 1


if __name__ == "__main__":
    sys.exit(main())
