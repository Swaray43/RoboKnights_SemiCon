#!/usr/bin/env python3
"""
ci/coverage_report.py -- Phase 2 Step 3 C: functional coverage of tb_bridge_rand.

Every tb_bridge_rand run prints one line

    COV: seed=7 noise=1 wr_00=6 rd_00=1 ... split=3 ... arb0_w4=17 ... cable=1 ...

This script merges those lines (from any number of log files or stdin) and
prints one table: each coverage bin, how often it was hit, in how many seeds,
and whether it is required. With --strict it exits 1 if a required bin was
never hit (the nightly 40-seed sweep runs it that way).

    python3 ci/coverage_report.py ci_out/cov_*.txt            # report only
    python3 ci/coverage_report.py --strict ci_out/cov_*.txt   # gate

Bins that tb_bridge_rand cannot reach by construction are listed with the
directed bench that covers them instead (required = no).
"""
import os
import re
import sys
from collections import defaultdict

# (group, bin key, description, required, where else it is covered)
BINS = [
    ("BAR0 registers (written)", "wr_00", "0x00 ARM (session start)", True, ""),
    ("BAR0 registers (written)", "wr_08", "0x08 arbiter weights", True, ""),
    ("BAR0 registers (written)", "wr_0c", "0x0C session limit", True, ""),
    ("BAR0 registers (written)", "wr_10", "0x10 MSI-X vectors", True, ""),
    ("BAR0 registers (written)", "wr_18", "0x18 traffic class (inbound MWr only)", True, ""),
    ("BAR0 registers (written)", "wr_1c", "0x1C XLAT_MISS clear", True, ""),
    ("BAR0 registers (written)", "wr_20", "0x20 ch0 base lo", True, ""),
    ("BAR0 registers (written)", "wr_24", "0x24 ch0 base hi", True, ""),
    ("BAR0 registers (written)", "wr_28", "0x28 ch1 base lo", True, ""),
    ("BAR0 registers (written)", "wr_2c", "0x2C ch1 base hi", True, ""),
] + [
    ("BAR0 registers (read)", f"rd_{a:02x}", f"0x{a:02X}", True, "") for a in range(0, 0x30, 4)
] + [
    ("BAR0 registers (read)", "rd_1800", "0x1800 MSI-X PBA", True, ""),
    ("Data shape", "split", "beat split at a 4 KB boundary", True, ""),
    ("Data shape", "nosplit", "beat not split", True, ""),
    ("Data shape", "single_ch", "session with one channel armed", True, ""),
] + [
    ("Data shape", f"last_{n}", f"last beat {n} DW", True, "") for n in range(1, 17)
] + [
    ("Arbiter (contested grant won)", f"arb{c}_w{w}", f"ch{c} wins with weight {w}", True, "")
    for w in range(1, 8) for c in (0, 1)
] + [
    ("Flow control", "credit_stall_ep", "TX stalled for Posted credits", False, "tb_credit_gate_proof (link suite); the link partner advertises infinite credits"),
    ("Errors / recovery", "fec_fix", "FEC corrected a symbol error", True, ""),
    ("Errors / recovery", "fec_bad", "FEC uncorrectable", True, ""),
    ("Errors / recovery", "crc", "CRC-64 fail", False, "tb_stream_path (errors suite)"),
    ("Errors / recovery", "replay", "replay (NAK / timeout)", True, ""),
    ("Errors / recovery", "xlat_cnt", "RX translation miss counted (XLAT_MISS)", True, ""),
    ("Errors / recovery", "xlat_err", "TX translation miss (err_xlat_miss)", False, "link suite (address translation benches)"),
    ("Errors / recovery", "framing", "PHY framing error", False, "phy suite"),
    ("Errors / recovery", "lane_err", "PHY lane / deskew error", False, "phy suite"),
    ("Errors / recovery", "ovf", "session-limit overflow (F5)", True, ""),
    ("Interrupts", "msix", "MSI-X delivered", True, ""),
    ("Interrupts", "mask_vec", "vector masked (F8)", True, ""),
    ("Interrupts", "mask_fn", "function masked (F8)", True, ""),
    ("Resets / link", "hotreset", "hot reset mid-session (F7)", True, ""),
    ("Resets / link", "cable", "cable pull mid-session (F7b)", True, ""),
    ("Resets / link", "glitch", "short glitch mid-session (F7b)", True, ""),
    ("Resets / link", "glitch_drop", "glitch took the link down", False, ""),
    ("Resets / link", "exit_loss", "L0 left on signal loss (K6)", True, ""),
    ("Resets / link", "exit_framing", "L0 left on lost framing (K6)", False, "tb_pcie_phy_top"),
    ("Resets / link", "exit_ts1", "L0 left on partner TS1 (K6)", False, "tb_pcie_phy_top / cocotb"),
    ("Lanes", "lanes_straight", "straight lanes (F6)", True, ""),
    ("Lanes", "lanes_rev", "reversed lanes (F6)", True, ""),
    ("Inbound BAR0 (F9)", "hw_weights", "weights written by inbound MWr", True, ""),
    ("Inbound BAR0 (F9)", "hw_tc", "TC written by inbound MWr", True, ""),
    ("Inbound BAR0 (F9)", "hw_base1", "ch1 base written by inbound MWr", True, ""),
]


def parse(lines):
    total = defaultdict(int)
    seeds = defaultdict(int)
    runs = 0
    for line in lines:
        m = re.search(r"COV:(.*)$", line)
        if not m:
            continue
        runs += 1
        for k, v in re.findall(r"(\w+)=(-?\d+)", m.group(1)):
            if k in ("seed", "noise"):
                continue
            total[k] += int(v)
            if int(v) > 0:
                seeds[k] += 1
    return runs, total, seeds


def report(runs, total, seeds):
    out = [f"### Step 3 C coverage: tb_bridge_rand, {runs} seeds", ""]
    missing = []
    group = None
    for g, key, desc, req, elsewhere in BINS:
        if g != group:
            out += ["", f"**{g}**", "", "| Bin | Hits | Seeds | Required | Status |", "|---|---:|---:|---|---|"]
            group = g
        hits = total.get(key, 0)
        if hits > 0:
            st = "covered"
        elif req:
            st = "**EMPTY**"
            missing.append(key)
        else:
            st = f"not reached here; {elsewhere}" if elsewhere else "not reached (optional)"
        out.append(f"| {desc} (`{key}`) | {hits} | {seeds.get(key, 0)} | {'yes' if req else 'no'} | {st} |")
    req_n = sum(1 for b in BINS if b[3])
    out += ["", f"Required bins covered: {req_n - len(missing)}/{req_n}"
            + (f"  -- empty: {', '.join(missing)}" if missing else "")]
    return "\n".join(out), missing


def main():
    args = sys.argv[1:]
    strict = "--strict" in args
    files = [a for a in args if a != "--strict"]
    lines = []
    if files:
        for f in files:
            with open(f, errors="replace") as fh:
                lines += fh.readlines()
    else:
        lines = sys.stdin.readlines()
    runs, total, seeds = parse(lines)
    if runs == 0:
        print("coverage_report: no COV: lines found")
        return 1
    text, missing = report(runs, total, seeds)
    print(text)
    s = os.environ.get("GITHUB_STEP_SUMMARY")
    if s:
        with open(s, "a") as fh:
            fh.write(text + "\n")
    return 1 if (strict and missing) else 0


if __name__ == "__main__":
    sys.exit(main())
