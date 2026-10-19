"""
run_tests.py - runs the cocotb testbenches without needing GNU Make.

Top level is now pcie_phy_top (pcie_link_top was folded into it; every
pcie_link_top port kept its name). Sources are read from ../files.f.

Usage:
    python run_tests.py                  # Icarus Verilog (default)
    python run_tests.py --sim verilator  # Verilator (once installed)

Requires: iverilog (or verilator) on PATH, and `pip install cocotb`.
Same file layout as the Makefile version:
    cocotb/
    ├── run_tests.py           (this file)
    ├── test_pcie_link_top.py  (original 4 tests, unchanged)
    ├── test_pcie_phy_top.py   (BAR0, session CDC, MSI-X, datapath loopback)
    └── models/
        └── rc_bfm.py
"""

import argparse
import sys
from pathlib import Path

from cocotb_tools.runner import get_runner

THIS_DIR = Path(__file__).resolve().parent

# Same source list as the Makefile: the single RTL file list, ../files.f
FILES_F = THIS_DIR / ".." / "files.f"
VERILOG_SOURCES = [
    THIS_DIR / ".." / line.strip()
    for line in FILES_F.read_text().splitlines()
    if line.strip() and not line.strip().startswith("//")
]
TOPLEVEL = "pcie_phy_top"
TEST_MODULES = ["test_pcie_link_top", "test_pcie_phy_top"]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--sim", default="icarus", choices=["icarus", "verilator"],
                         help="Simulator to run against (default: icarus)")
    args = parser.parse_args()

    missing = [str(p) for p in VERILOG_SOURCES if not p.resolve().exists()]
    if missing:
        print("ERROR: missing source file(s):")
        for m in missing:
            print(f"  {m}")
        print("Check that ../files.f matches the files in ../rtl/.")
        sys.exit(1)

    runner = get_runner(args.sim)

    build_args = ["-g2005"] if args.sim == "icarus" else []

    runner.build(
        sources=[str(p.resolve()) for p in VERILOG_SOURCES],
        hdl_toplevel=TOPLEVEL,
        build_args=build_args,
        build_dir=str(THIS_DIR / "sim_build"),
        timescale=("1ns", "1ps"),
        always=True,
    )

    runner.test(
        hdl_toplevel=TOPLEVEL,
        test_module=TEST_MODULES,
        test_dir=str(THIS_DIR),
        results_xml="results.xml",
    )


if __name__ == "__main__":
    main()
