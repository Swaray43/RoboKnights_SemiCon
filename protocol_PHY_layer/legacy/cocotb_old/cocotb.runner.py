"""
run_tests.py - runs the cocotb testbench without needing GNU Make.

Usage:
    python run_tests.py                  # Icarus Verilog (default)
    python run_tests.py --sim verilator  # Verilator (once installed)

Requires: iverilog (or verilator) on PATH, and `pip install cocotb`.
Same file layout as the Makefile version:
    cocotb/
    ├── run_tests.py        (this file)
    ├── pcie_link_top.v     (place next to your other .v sources)
    ├── test_pcie_link_top.py
    └── models/
        └── rc_bfm.py
"""

import argparse
import sys
from pathlib import Path

from cocotb_tools.runner import get_runner

THIS_DIR = Path(__file__).resolve().parent

# Same source list as the Makefile - add pcie_phy_datapath.v-derived files
# here once the PHY datapath is folded into pcie_link_top.v in a later pass.
VERILOG_SOURCES = [
    THIS_DIR / ".." / "pcie_phy_datapath.v",
    THIS_DIR / ".." / "pcie_tag_man.v",
    THIS_DIR / ".." / "pcie_config_msix.v",
    THIS_DIR / "pcie_link_top.v",
]


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
        print("Check that pcie_link_top.v sits next to your other .v files as described above this script's docstring.")
        sys.exit(1)

    runner = get_runner(args.sim)

    build_args = ["-g2001"] if args.sim == "icarus" else []

    runner.build(
        sources=[str(p.resolve()) for p in VERILOG_SOURCES],
        hdl_toplevel="pcie_link_top",
        build_args=build_args,
        build_dir=str(THIS_DIR / "sim_build"),
        always=True,
    )

    runner.test(
        hdl_toplevel="pcie_link_top",
        test_module="test_pcie_link_top",
        test_dir=str(THIS_DIR),
        results_xml="results.xml",
    )


if __name__ == "__main__":
    main()