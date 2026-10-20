# RoboKnights SemiCon — AXI4-Stream to PCIe 6 bridge

Three layers, one top:

    fabric AXI4-Stream (2 ch) → interconnect_layer → src-link_layer → protocol_PHY_layer → 4-lane PAM4
                                  (Aditya)            (Swaray)          (Kshitij)

| Folder | What's in it |
|---|---|
| `bridge_top/` | Full-stack top (`bridge_top.v`), end-to-end benches, `bridge_top.f`, Vivado/Quartus project scripts, constraints. **Start here.** |
| `interconnect_layer/` | `src/`: ingest ports, weighted round-robin arbiter, CDC FIFO + unit benches. `docs/`: spec and DMA addressing. `sim_logs/`: bench output. |
| `src-link_layer/hdl/` | Stream TX path, Flit framing, replay/ACK-NAK, CRC-64/FEC, RX TLP delimiter. `files.f` lists the active RTL. `legacy/`: old AXI-MM path and two-way building blocks. `testbench/`: active benches; `testbench/legacy/`: parked benches (see `docs/TICKETS.md`). |
| `protocol_PHY_layer/` | `rtl/` (one module per file, `files.f`), `tb/`, `formal/`, `cocotb/`, `run_all.py`. `legacy/`: pre-restructure snapshot. |
| `docs/` | Problem statement, figures, round-1 report, glossary, work distribution, `TICKETS.md`. |

## Run everything

```sh
# full stack (from repo root)
iverilog -g2012 -DSIMULATION -s tb_bridge_top -o b.vvp -c bridge_top/bridge_top.f -c bridge_top/bridge_top_tb.f && vvp b.vvp
iverilog -g2012 -DSIMULATION -s tb_bridge_ovf -o o.vvp -c bridge_top/bridge_top.f -c bridge_top/bridge_top_tb.f && vvp o.vvp

# per layer
cd protocol_PHY_layer && python run_all.py            # 24 benches
cd protocol_PHY_layer/cocotb && python run_tests.py   # 8 cocotb tests
# link: each bench in src-link_layer/hdl/testbench/ with  -I src-link_layer/hdl  src-link_layer/hdl/*.v
# interconnect: each tb in interconnect_layer/src/ with the 4 RTL files there
```

Vivado: `source bridge_top/vivado_create_project.tcl` · Quartus: `quartus_sh -t bridge_top/quartus_create_project.tcl`

## Status (this snapshot)

| Suite | Result |
|---|---|
| `tb_bridge_top`, `tb_bridge_ovf`, `tb_ingest_overflow_leak` | PASS (Icarus 12 and Vivado XSim) |
| Interconnect unit benches | 4/4 |
| Link active benches (`tb_stream_path` with bit-error injection, address translation, 4KB splitter, cfg bridge) | 4/4 |
| PHY benches / cocotb | 24/24, 8/8 |
| Yosys elaboration of `bridge_top` | 0 problems, 0 latches |
| Vivado/Quartus synthesis and timing | Not yet run |

Open work is tracked in `docs/TICKETS.md`.

**Legacy rule:** anything under a `legacy/` folder is kept for reference, isn't on any file list, and
may redefine active modules. Fix it, then move it back.
