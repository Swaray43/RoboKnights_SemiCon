# Protocol-to-PHY — legacy

| Path | What | Status |
|---|---|---|
| `legacy_axi4s_ingest_port.v`, `legacy_session_overflow_ctrl.v` (+ `tb_legacy_*`) | Kshitij's early stand-in for the interconnect ingest port (16-bit offsets). Renamed so it no longer clashes with `interconnect_layer/src/axi4s_ingest_port.v` | Benches pass (`RESULT: PASS`); reference only, not on any file list |

Removed (tickets K2, K3, K5): `pre_restructure/` (the pre-P5 multi-module files, old root
benches with the `sext_rst_n` typo, old `.sby` files) and `cocotb_old/` (`pcie_link_top.v`
with a floating `mwr_ready`, and `cocotb.runner.py`). Everything they contained lives
one-module-per-file in `../rtl/`, `../tb/`, `../formal/` and `../cocotb/`.
