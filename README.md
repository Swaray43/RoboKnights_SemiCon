# GDSII flow (LibreLane + SKY130)

RTL → GDSII for the RoboKnights AXI4-Stream → PCIe 6 bridge using
LibreLane (OpenROAD-based) on the
SkyWater **SKY130A** PDK with the **sky130_fd_sc_hd** standard-cell library.

## Configurations

| Folder | Top | What | Clocks |
| --- | --- | --- | --- |
| `gds/interconnect/` | `interconnect_top` | Interconnect layer alone | aclk 30 ns, pcie_clk 15 ns |
| `gds/bridge/` | `bridge_top` | Full bridge, **narrow GDS build** (X.2): `PHY_WIDE=0`, `REPLAY_DEPTH=4`, `IC_FIFO_AW=2`, `WDB_FIFO_D=4`, `WDB_HWM=3`, `PHY_TX_FIFO_D=8` | clk, aclk 30 ns |
| `gds/rxcheck/` | `link_layer_rx` | 2-minute synthesis-check run (`--to Checker.YosysSynthChecks`) | clk 30 ns |

Each folder has `config.json` (explicit RTL file list, include dirs,
`RUN_LINTER: false`) and an SDC with both clocks in
`set_clock_groups -asynchronous`, explicit port-name I/O delays and false paths
on the resets. The FPGA build uses the same RTL with the parameter defaults
(wide PHY, replay 32, FIFOs 16).

## Running

```bash
# from the repository root (Docker mounts only $HOME and the current folder)
librelane --dockerized --pdk sky130A --scl sky130_fd_sc_hd gds/bridge/config.json

# resume a run from a step instead of restarting
librelane --dockerized --pdk sky130A --scl sky130_fd_sc_hd gds/bridge/config.json \
  --run-tag <RUN_TAG> --from OpenROAD.DetailedRouting

# quick synthesis-check of link_layer_rx
librelane --dockerized --pdk sky130A --scl sky130_fd_sc_hd gds/rxcheck/config.json \
  --to Checker.YosysSynthChecks
```

A run passes when `flow.log` ends with “Flow complete.” and DRC, LVS and
antenna are clean in the manufacturability report.

## GDS patches (X.n)

Each X patch is applied separately with `git am` to the FPGA line
(`phase2/X`) and the GDS branch (`phase3/gds`); the branches are not merged.

| Patch | Problem | Fix |
| --- | --- | --- |
| X.1 | Interconnect setup −12 ns at slow corner: TKEEP → popcount → offset add → limit compare → TREADY → FIFO write ≈ 27 ns in one cycle | Offset = `off_base + bytes_q`; full beats accepted without TKEEP; a beat within 64 B of the limit is held one cycle while its size is checked |
| X.2 | Full bridge too large for laptop place-and-route | Buffer-size parameters plumbed through `bridge_top` (defaults = FPGA); narrow GDS build + matching ASIC sim profile in CI |
| X.3 | 230 Yosys check errors (latches, logic loops) in `link_layer_rx` | Defaults at top of the correction block; separate loop index for the clocked loop |
| X.4 | Newer Yosys still found 192 logic loops | FEC correction as constant-position gates (Hamming positions computed at elaboration) |

Flow fixes: Verilator linter off (cell-model timescales), explicit file lists
(testbenches broke synthesis), exact port names in SDC, run from the repo root
when it lives outside `$HOME`.

## Results (24 Oct 2026)

**Interconnect (`interconnect_top`), before X.1 — GDS produced**

- DRC, LVS and antenna **clean**; hold met at all corners
- Die 1.20 mm² at 41% utilisation, ~49k cells (9.8k flip-flops), 104 mW
- Setup −0.95 ns at ss_100C_1v60 (−0.52 ns after X.1; that rerun was stopped before stream-out)

**`link_layer_rx` alone, after X.4** — “Check for Yosys check errors clear”; 1.12 mm² of cells

**Full bridge, narrow build — placed and clock-tree-synthesised, not routed**
(run `RUN_2026-10-23_22-24-44`)

- Completed: synthesis, floorplan, PDN, placement, CTS, global routing (41 of ~80 steps)
- Die 13.17 mm² (~3.6 × 3.6 mm), core 13.05 mm²
- Standard cells 4.15 mm² (~32%): logic 2.40, flip-flops 1.51, taps 0.23 mm²
- 431,956 instances
- Pre-route hold: ss +0.46 ns, tt +0.10 ns, ff −0.036 ns
- Pre-route setup at ss −1,014 ns: timing repair was off and 100+-pin nets
were unbuffered, so this is not a real path delay; **ASIC timing closure not done**
- Detailed routing: killed once out of memory (~14 GB), resumed run stopped at
the deadline. **No full-bridge GDS.**

## Next steps

1. Detailed routing on a machine with ≥ 32 GB RAM, with `DRT_THREADS` limited to 4.
2. Timing repair on (post-CTS / post-GRT resizer) and buffering of the high-fanout nets.
3. Or harden `link_layer_top` as a separate macro and integrate it at the top.
