# RoboKnights SemiCon — AXI4-Stream to PCIe 6 bridge

Three layers, one top:

    fabric AXI4-Stream (2 ch) → interconnect_layer → src-link_layer → protocol_PHY_layer → 4-lane PAM4
                                  (Aditya)            (Swaray)          (Kshitij)

| Folder | What's in it |
|---|---|
| `bridge_top/` | Full-stack top (`bridge_top.v`), end-to-end benches, `bridge_top.f`, Vivado/Quartus project scripts, constraints. **Start here.** |
| `interconnect_layer/` | `src/`: ingest ports, weighted round-robin arbiter, CDC FIFO + unit benches. `docs/`: spec and DMA addressing. `sim_logs/`: bench output. |
| `src-link_layer/hdl/` | `link_layer_top`: stream TX path, credit gate, Flit framing, replay/ACK-NAK, CRC-64/FEC, RX TLP decode to AXI BAR0 writes. `link_phy_bridge_top`: link + PHY glue. `files.f` + `run_all_link.py`: active RTL and benches. `legacy/`: retired blocks. `testbench/wcfg/`: Vivado waveform configs. |
| `protocol_PHY_layer/` | `rtl/` (one module per file, `files.f`), `tb/`, `formal/`, `cocotb/`, `run_all.py`. `legacy/`: pre-restructure snapshot. |
| `docs/` | Problem statement, figures, round-1 report, glossary, work distribution, `TICKETS.md`. |
| `asic/` | LibreLane (OpenLane 2) configs + SDCs per block and flat, ASIC-profile sim params. |

## Run everything

```sh
# full stack (from repo root)
iverilog -g2012 -DSIMULATION -s tb_bridge_top -o b.vvp -c bridge_top/bridge_top.f -c bridge_top/bridge_top_tb.f && vvp b.vvp
iverilog -g2012 -DSIMULATION -s tb_bridge_ovf -o o.vvp -c bridge_top/bridge_top.f -c bridge_top/bridge_top_tb.f && vvp o.vvp

# per layer
python3 src-link_layer/hdl/run_all_link.py              # 16 link benches (files.f)
cd protocol_PHY_layer && python3 run_all.py             # 24 PHY benches
cd protocol_PHY_layer/cocotb && make                    # 9 cocotb tests
# interconnect: each tb in interconnect_layer/src/ with the 4 RTL files there
```

Vivado: `source bridge_top/vivado_create_project.tcl` · Quartus: `quartus_sh -t bridge_top/quartus_create_project.tcl` ·
LibreLane: see `asic/README.md`.

## Interface between interconnect and link (Phase 2)

`interconnect_top.m_tuser[25:0] = {tlast_flag, session_offset[23:0], stream_id}`, the link layer's
native TUSER. It connects straight to `link_phy_bridge_top.s_axis_tuser`; there is no remap module.

## Status (this snapshot)

| Suite | Result |
|---|---|
| `tb_bridge_top` (144 and 600 TLPs), `tb_bridge_ovf`, `tb_ingest_overflow_leak` | PASS (Icarus 12) |
| Same benches with the ASIC buffer sizes (`asic/sim/`) | PASS |
| Link benches (`run_all_link.py`) | 15/16 (`tx_engine` has no self-check) |
| Interconnect unit benches | 4/4 |
| PHY benches / cocotb | 24/24, 9/9 |
| Yosys elaboration of `bridge_top` | 0 latches |
| Vivado/Quartus synthesis and timing | Re-run after this snapshot |

Open work is tracked in `docs/TICKETS.md` (round 2); round 1 is in `docs/TICKETS_round1.md`.

**Legacy rule:** anything under a `legacy/` folder is kept for reference, isn't on any file list, and
may redefine active modules. Fix it, then move it back.
