#!/usr/bin/env python3
"""
ci/run_regression.py -- the full Phase 2 regression, one command.

Runs with Icarus Verilog (GitHub runners have no ModelSim licence). Works on
Linux, macOS and Windows; run from anywhere.

    python3 ci/run_regression.py                 # every suite except 'long' and 'formal'
    python3 ci/run_regression.py guards bridge   # selected suites
    python3 ci/run_regression.py --list

Suites
  guards        ci/check_protected.py: lost fixes, file lists, duplicate modules
  link          src-link_layer/hdl/run_all_link.py (all benches in files.f)
  interconnect  4 interconnect unit benches
  phy           protocol_PHY_layer/run_all.py
  cocotb        protocol_PHY_layer/cocotb (needs: pip install cocotb)
  bridge        tb_bridge_top, tb_bridge_ovf, tb_ingest_overflow_leak
  errors        tb_stream_path with bit-error injection + random backpressure
  asic          bridge + stream benches with the ASIC buffer sizes (asic/sim)
  long          tb_bridge_top +SESS=6 +BEATS=50 (600 TLPs)
  rand          tb_bridge_rand over several seeds, clean and with PAM4 noise
                (RAND_SEEDS=n, default 4; RAND_BASE=s, default 1), then the
                Step 3 C coverage table (ci/coverage_report.py). COV: lines go
                to ci_out/cov_<base>.txt; COV_STRICT=1 fails on an empty bin.
  formal        every *.sby under protocol_PHY_layer/formal and verification/formal
                (needs sby + yosys + solvers, e.g. the OSS CAD Suite)

Exit code 0 only if every selected suite passes. On GitHub Actions a summary
table is written to the job summary.
"""
import os
import re
import shutil
import subprocess
import sys
import tempfile
import time
import xml.etree.ElementTree as ET
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
PY = sys.executable
TIMEOUT = 1800
DEFAULT = ["guards", "link", "interconnect", "phy", "cocotb", "bridge", "errors", "asic", "rand"]
ALL = DEFAULT + ["long", "formal"]

results = []          # (suite, item, ok, detail, seconds)


def run(cmd, cwd=ROOT, timeout=TIMEOUT):
    env = dict(os.environ, PWD=str(cwd))     # cocotb's Makefile reads $(PWD)
    try:
        p = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, errors="replace",
                           timeout=timeout, stdin=subprocess.DEVNULL, env=env)
        return p.returncode, p.stdout + p.stderr
    except subprocess.TimeoutExpired as e:
        return 124, (e.stdout or "") + f"\nTIMEOUT after {timeout}s"
    except FileNotFoundError as e:
        return 127, str(e)


def record(suite, item, ok, detail, t0):
    dt = time.time() - t0
    results.append((suite, item, ok, detail, dt))
    print(f"  [{'PASS' if ok else 'FAIL'}] {suite}/{item}  ({dt:.0f}s)" + (f"  -- {detail}" if detail and not ok else ""),
          flush=True)


def tail(out, n=4):
    lines = [l for l in out.strip().splitlines() if l.strip()]
    return " | ".join(lines[-n:])


def has_pass(out):
    r = re.findall(r"RESULT:\s*(PASS|FAIL)", out)
    return bool(r) and "FAIL" not in r


def ivl_build_run(name, srcs, top, defines=(), incs=(), plusargs=(), extra_tops=()):
    """Compile with Icarus and run; return (ok, output)."""
    tmp = Path(tempfile.mkdtemp(prefix="reg_"))
    vvp = tmp / f"{name}.vvp"
    cmd = ["iverilog", "-g2012", "-o", str(vvp), "-s", top]
    for t in extra_tops:
        cmd += ["-s", t]
    for d in defines:
        cmd.append(f"-D{d}")
    for i in incs:
        cmd += ["-I", str(ROOT / i)]
    cmd += srcs
    rc, out = run(cmd)
    if rc != 0:
        shutil.rmtree(tmp, ignore_errors=True)
        return False, "compile error: " + tail(out, 3)
    rc, out = run(["vvp", "-n", str(vvp)] + list(plusargs), cwd=tmp)
    shutil.rmtree(tmp, ignore_errors=True)
    return rc == 0 and has_pass(out), out


def bridge_srcs(tb_files):
    return ["-c", str(ROOT / "bridge_top/bridge_top.f")] + [str(ROOT / f) for f in tb_files]


# ---------------------------------------------------------------- suites
def s_guards():
    t0 = time.time()
    rc, out = run([PY, str(ROOT / "ci/check_protected.py")])
    record("guards", "protected lines + file lists + duplicate modules", rc == 0, tail(out, 6), t0)


def s_link():
    t0 = time.time()
    rc, out = run([PY, str(ROOT / "src-link_layer/hdl/run_all_link.py")])
    m = re.search(r"(\d+)/(\d+) benches passed", out)
    for line in out.splitlines():
        if line.startswith("[FAIL]"):
            print("     " + line)
    record("link", m.group(0) if m else "run_all_link.py", rc == 0, tail(out), t0)


def s_interconnect():
    src = ["axi4s_ingest_port.v", "cdc_fifo_async.v", "rr_arbiter_2ch.v", "interconnect_top.v"]
    d = ROOT / "interconnect_layer/src"
    for tb in ["tb_axi4s_ingest_port", "tb_cdc_fifo_async", "tb_rr_arbiter_2ch", "tb_interconnect_top"]:
        t0 = time.time()
        tmp = Path(tempfile.mkdtemp(prefix="ic_"))
        rc, out = run(["iverilog", "-g2012", "-I", str(ROOT / "shared"), "-s", tb, "-o", str(tmp / "a.vvp")]
                      + [str(d / s) for s in src] + [str(d / f"{tb}.v")])
        if rc == 0:
            rc, out = run(["vvp", "-n", str(tmp / "a.vvp")], cwd=tmp)
            ok = rc == 0 and "ALL TESTS PASSED" in out and not re.search(r"\bFAIL", out)
        else:
            ok, out = False, "compile error: " + tail(out, 3)
        shutil.rmtree(tmp, ignore_errors=True)
        record("interconnect", tb, ok, tail(out), t0)


def s_phy():
    t0 = time.time()
    rc, out = run([PY, "run_all.py"], cwd=ROOT / "protocol_PHY_layer")
    m = re.search(r"(\d+)/(\d+) benches passed", out)
    for line in out.splitlines():
        if "FAILED" in line:
            print("     " + line)
    record("phy", m.group(0) if m else "run_all.py", rc == 0, tail(out), t0)


def s_cocotb():
    t0 = time.time()
    d = ROOT / "protocol_PHY_layer/cocotb"
    if shutil.which("cocotb-config") is None:
        record("cocotb", "pcie_phy_top", False, "cocotb not installed (pip install cocotb)", t0)
        return
    (d / "results.xml").unlink(missing_ok=True)
    rc, out = run(["make", "-s"], cwd=d)
    tests = fails = 0
    try:
        for tc in ET.parse(d / "results.xml").getroot().iter("testcase"):
            tests += 1
            if tc.find("failure") is not None or tc.find("error") is not None:
                fails += 1
    except Exception:
        pass
    shutil.rmtree(d / "sim_build", ignore_errors=True)
    record("cocotb", f"{tests - fails}/{tests} tests", rc == 0 and tests > 0 and fails == 0, tail(out), t0)


def s_bridge():
    for tb in ["tb_bridge_top", "tb_bridge_ovf", "tb_ingest_overflow_leak"]:
        t0 = time.time()
        ok, out = ivl_build_run(tb, bridge_srcs([f"bridge_top/tb/{tb}.v"]), tb, defines=["SIMULATION"])
        m = re.search(r"data TLPs received ok\s*:\s*(\d+)", out)
        record("bridge", tb + (f" ({m.group(1)} TLPs)" if m else ""), ok, tail(out), t0)


def stream_srcs():
    hdl = ROOT / "src-link_layer/hdl"
    rtl = []
    for l in (hdl / "files.f").read_text().splitlines():
        t = l.split("#", 1)[0].split()
        if len(t) >= 2 and t[0] == "rtl" and t[1].endswith(".v"):
            rtl.append(str(hdl / t[1]))
    return rtl + [str(hdl / "testbench/tb_stream_path.v")]


ERR_ARGS = ["+NBEATS=1500", "+ERR1=3000", "+ERR2=1000", "+READY_PCT=40", "+RXREADY_PCT=40"]


def s_errors():
    t0 = time.time()
    ok, out = ivl_build_run("stream_err", stream_srcs(), "tb_stream_path", defines=["SIMULATION"],
                            incs=["src-link_layer/hdl", "shared"], plusargs=ERR_ARGS)
    m = re.search(r"FEC corrected\s*:\s*(\d+).*?replays:\s*(\d+)", out)
    record("errors", "tb_stream_path bit errors" + (f" (FEC fixed {m.group(1)}, replays {m.group(2)})" if m else ""),
           ok, tail(out), t0)


def s_asic():
    for tb, prm in [("tb_bridge_top", "asic_params"), ("tb_bridge_ovf", "asic_params_ovf"),
                    ("tb_bridge_top", "asic_params_gds"), ("tb_bridge_ovf", "asic_params_gds_ovf")]:   # X.2: GDS build
        t0 = time.time()
        ok, out = ivl_build_run(tb + "_" + prm, bridge_srcs([f"bridge_top/tb/{tb}.v", f"asic/sim/{prm}.v"]), tb,
                                defines=["SIMULATION"], extra_tops=[prm],
                                plusargs=["+REARM_FAST=1"] if tb == "tb_bridge_top" else [])
        record("asic", f"{tb} with {prm}", ok, tail(out), t0)
    t0 = time.time()
    ok, out = ivl_build_run("stream_asic", stream_srcs() + [str(ROOT / "asic/sim/asic_params_sp.v")], "tb_stream_path",
                            defines=["SIMULATION"], incs=["src-link_layer/hdl", "shared"],
                            extra_tops=["asic_params_sp"], plusargs=ERR_ARGS)
    record("asic", "tb_stream_path with asic_params_sp", ok, tail(out), t0)


def s_long():
    t0 = time.time()
    ok, out = ivl_build_run("long", bridge_srcs(["bridge_top/tb/tb_bridge_top.v"]), "tb_bridge_top",
                            defines=["SIMULATION"], plusargs=["+SESS=6", "+BEATS=50"])
    m = re.search(r"data TLPs received ok\s*:\s*(\d+)", out)
    record("long", "tb_bridge_top 6x50" + (f" ({m.group(1)} TLPs)" if m else ""), ok, tail(out), t0)


def s_rand():
    """tb_bridge_rand: half the seeds clean, half with PAM4 noise (some bursty)."""
    n = int(os.environ.get("RAND_SEEDS", "4"))
    base = int(os.environ.get("RAND_BASE", "1"))
    tmp = Path(tempfile.mkdtemp(prefix="rand_"))
    vvp = tmp / "rand.vvp"
    t0 = time.time()
    rc, out = run(["iverilog", "-g2012", "-DSIMULATION", "-s", "tb_bridge_rand", "-o", str(vvp)]
                  + bridge_srcs(["bridge_top/tb/tb_bridge_rand.v"]))
    if rc != 0:
        record("rand", "compile", False, tail(out, 3), t0)
        shutil.rmtree(tmp, ignore_errors=True)
        return
    cov = []
    for i in range(n):
        seed = base + i
        args = [f"+SEED={seed}"]
        if i % 2 == 1:
            # 1000 ppm single-symbol errors, or 300 ppm hits of 4-symbol bursts
            args += ["+NOISE_PPM=300", "+BURST=4"] if i % 4 == 3 else ["+NOISE_PPM=1000"]
        t0 = time.time()
        rc, out = run(["vvp", "-n", str(vvp)] + args, cwd=tmp)
        cov += [l for l in out.splitlines() if l.startswith("COV:")]
        m = re.search(r"data TLPs received ok\s*:\s*(\d+)", out)
        r = re.search(r"replays (\d+)", out)
        label = " ".join(args) + (f" ({m.group(1)} TLPs" + (f", {r.group(1)} replays)" if r else ")") if m else "")
        record("rand", label, rc == 0 and has_pass(out), tail(out), t0)
    shutil.rmtree(tmp, ignore_errors=True)
    # Step 3 C: merge this run's coverage lines
    t0 = time.time()
    outd = ROOT / "ci_out"
    outd.mkdir(exist_ok=True)
    covf = outd / f"cov_{base}.txt"
    covf.write_text("\n".join(cov) + "\n")
    strict = os.environ.get("COV_STRICT", "0") == "1"
    rc, out = run([PY, str(ROOT / "ci/coverage_report.py")] + (["--strict"] if strict else []) + [str(covf)])
    print(out)
    m = re.search(r"Required bins covered: (\d+)/(\d+)", out)
    record("rand", "coverage" + (f" {m.group(1)}/{m.group(2)} required bins" if m else "")
           + (" (strict)" if strict else ""), rc == 0, tail(out, 1), t0)


def s_formal():
    if shutil.which("sby") is None:
        t0 = time.time()
        record("formal", "sby", False, "sby not installed (use the OSS CAD Suite)", t0)
        return
    for sby in sorted(list((ROOT / "protocol_PHY_layer/formal").glob("*.sby")) +
                      list((ROOT / "verification/formal").glob("*.sby"))):
        t0 = time.time()
        rc, out = run(["sby", "-f", sby.name], cwd=sby.parent)
        for d in sby.parent.glob(sby.stem + "*"):
            if d.is_dir():
                shutil.rmtree(d, ignore_errors=True)
        record("formal", str(sby.relative_to(ROOT)), rc == 0, tail(out, 3), t0)


SUITES = {"guards": s_guards, "link": s_link, "interconnect": s_interconnect, "phy": s_phy,
          "cocotb": s_cocotb, "bridge": s_bridge, "errors": s_errors, "asic": s_asic,
          "long": s_long, "rand": s_rand, "formal": s_formal}


def summary():
    passed = sum(1 for r in results if r[2])
    lines = [f"### Regression: {passed}/{len(results)} checks passed", "",
             "| Suite | Check | Result | Time |", "|---|---|---|---|"]
    for s, item, ok, detail, dt in results:
        lines.append(f"| {s} | {item} | {'PASS' if ok else 'FAIL: ' + detail.replace('|', '/')[:160]} | {dt:.0f}s |")
    text = "\n".join(lines) + "\n"
    if os.environ.get("GITHUB_STEP_SUMMARY"):
        with open(os.environ["GITHUB_STEP_SUMMARY"], "a") as f:
            f.write(text)
    print("\n" + text)
    return passed == len(results)


def main():
    args = sys.argv[1:]
    if "--list" in args:
        print("\n".join(ALL))
        return 0
    chosen = args or DEFAULT
    bad = [a for a in chosen if a not in SUITES]
    if bad:
        print(f"unknown suite(s): {bad}; choose from {ALL}")
        return 2
    for tool in ("iverilog", "vvp"):
        if shutil.which(tool) is None and any(c != "guards" and c != "formal" for c in chosen):
            print(f"error: {tool} not on PATH")
            return 2
    for c in chosen:
        print(f"== {c}", flush=True)
        SUITES[c]()
    return 0 if summary() else 1


if __name__ == "__main__":
    sys.exit(main())
