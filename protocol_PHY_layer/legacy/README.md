# Protocol-to-PHY — legacy

| Path | What | Status |
|---|---|---|
| `legacy_axi4s_ingest_port.v`, `legacy_session_overflow_ctrl.v` (+ `tb_legacy_*`) | Kshitij's early stand-in for the interconnect ingest port (16-bit offsets). Renamed so it no longer clashes with `interconnect_layer/src/axi4s_ingest_port.v` | Benches pass (16 + 15 checks) |
| `pre_restructure/` | The pre-P5 multi-module files (`pci_stuff.v`, `pcie_tag_man.v`, `pcie_phy_datapath*.v`, …), old root benches and `.sby` files. The CHANGELOG lists them as deleted, but they were still in the repo. Every module in them now lives one-per-file in `../rtl/` | Snapshot only — they redefine `rtl/` modules, never compile them together with `rtl/` (ticket K2) |
| `cocotb_old/` | `pcie_link_top.v` (replaced by `rtl/pcie_phy_top.v`) and `cocotb.runner.py` (replaced by `cocotb/run_tests.py`) | `pcie_link_top.v` leaves `msix_controller.mwr_ready` floating (ticket K3) |
