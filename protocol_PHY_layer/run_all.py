#!/usr/bin/env python3
"""
run_all.py - build and run every Verilog testbench with Icarus Verilog.
Works on Windows, macOS and Linux (no bash, no `timeout` command needed).

Usage:
    python run_all.py                    # all benches
    python run_all.py tb_pcie_phy_top    # one or more benches by name

Needs: iverilog and vvp on PATH (Icarus Verilog 11 or newer).
Logs go to sim_out/<bench>.build.log and sim_out/<bench>.log
"""
import re
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
OUT = ROOT / "sim_out"


def rtl_files():
    files = []
    for line in (ROOT / "files.f").read_text().splitlines():
        line = line.split("//")[0].strip()
        if line:
            files.append(str(ROOT / line))
    return files


def main():
    try:
        ver = subprocess.run(["iverilog", "-V"], capture_output=True, text=True, errors="replace",
                             stdin=subprocess.DEVNULL).stdout.splitlines()[0]
        print(f"Using {ver}\n")
    except Exception:
        pass
    for tool in ("iverilog", "vvp"):
        if shutil.which(tool) is None:
            print(f"ERROR: '{tool}' not found on PATH. Install Icarus Verilog and reopen the terminal.")
            return 2

    OUT.mkdir(exist_ok=True)
    benches = sys.argv[1:] or sorted(p.stem for p in (ROOT / "tb").glob("tb_*.v"))
    common = [str(ROOT / "tb" / "timescale.v")] + rtl_files() \
             + [str(p) for p in sorted((ROOT / "tb" / "models").glob("*.v"))] \
             + [str(ROOT / "tb" / "pcie_reset_controller_checker.v")]

    failed = 0
    for tb in benches:
        vvp_file = OUT / f"{tb}.vvp"
        build = subprocess.run(
            ["iverilog", "-g2012", "-o", str(vvp_file), "-s", tb] + common + [str(ROOT / "tb" / f"{tb}.v")],
            capture_output=True, text=True, errors="replace", stdin=subprocess.DEVNULL)
        (OUT / f"{tb}.build.log").write_text(build.stdout + build.stderr)
        if build.returncode != 0:
            print(f"{tb:<38} BUILD FAILED")
            print("    " + "\n    ".join((build.stdout + build.stderr).strip().splitlines()[:5]))
            failed += 1
            continue
        try:
            # stdin closed so a stray $stop can never wait for the interactive prompt
            run = subprocess.run(["vvp", "-n", str(vvp_file)], capture_output=True, text=True,
                                 errors="replace", stdin=subprocess.DEVNULL, timeout=300)
            log = run.stdout + run.stderr
            if not log.strip():
                code = run.returncode & 0xFFFFFFFF
                hint = " - vvp crashed (Windows access violation)" if code == 0xC0000005 else ""
                log = f"FAIL: vvp produced no output (exit code {code:#x}){hint}"
        except subprocess.TimeoutExpired:
            log = "FAIL: simulation timed out after 300 s"
        (OUT / f"{tb}.log").write_text(log)
        p = len(re.findall(r"PASS", log))
        f = len(re.findall(r"FAIL|ERROR|MISMATCH", log)) - len(re.findall(r"RESULT: FAIL", log))
        # project convention: every bench ends with "RESULT: PASS" / "RESULT: FAIL";
        # a missing RESULT line counts as a failure
        ok = f == 0 and p > 0 and re.search(r"RESULT: PASS", log) is not None
        if not re.search(r"RESULT: (PASS|FAIL)", log):
            log += "\nFAIL: no RESULT line (bench did not finish)"
            f += 1
        failed += 0 if ok else 1
        print(f"{tb:<38} {'ok' if ok else 'FAILED':<6}  pass={p:<3} fail={f}")
        if not ok:
            tail = [l for l in log.strip().splitlines() if l.strip()][-3:]
            print("    " + "\n    ".join(tail))

    print(f"\n{len(benches) - failed}/{len(benches)} benches passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
