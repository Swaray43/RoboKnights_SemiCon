# bridge_top — full AXI4-Stream → PCIe 6 bridge build

This folder adds the one top level that wires all three layers together:

    interconnect_top (Aditya) → link_phy_bridge_top (Swaray + Kshitij) → 4-lane PAM4 symbol bus

Applied to the repo in this zip (already done; `plumbing_fixes.patch` is included for reference):

| File | Change |
|---|---|
| `interconnect_layer/src/axi4s_ingest_port.v` | The over-limit beat can no longer handshake (`!would_overflow` in `s_tready`); an overflow abort sends one zero-length TLAST marker so the session closes in order and raises MSI-X |
| `protocol_PHY_layer/rtl/pcie_phy_top.v` | Sessions end on the link layer's `session_done` (data drained), not when the interconnect stops accepting — fixes the re-arm address race |
| `src-link_layer/hdl/flit_defs.vh` | FEC `while` loops → `if` so Yosys/OpenLane can synthesize it (identical results) |

## Load it

**Vivado** — Tcl console or batch:

    source bridge_top/vivado_create_project.tcl        # project in ./vivado_bridge
    # part: picks an installed one (Alveo first); to choose: set ::env(VIVADO_PART) <part> before sourcing

**Quartus** —

    quartus_sh -t bridge_top/quartus_create_project.tcl   # project in ./quartus_bridge
    # family/device: QUARTUS_FAMILY / QUARTUS_DEVICE env vars, default Arria 10 / AUTO

**Any simulator** (run from the repo root):

    iverilog -g2012 -DSIMULATION -s tb_bridge_top -o sim.vvp -c bridge_top/bridge_top.f -c bridge_top/bridge_top_tb.f && vvp sim.vvp
    vlog +define+SIMULATION -f bridge_top/bridge_top.f -f bridge_top/bridge_top_tb.f      # Questa

Use the file list rather than adding whole folders. Stale and superseded files now live in
`legacy/` folders, which redefine some modules; never add a `legacy/` folder to a build.

## Testbenches (`bridge_top/tb/`)

| Top | Checks | Plusargs |
|---|---|---|
| `tb_bridge_top` | Both channels, several sessions, every TLP's address/length/payload, MSI-X order | `+BEATS +SESS +GAP +REARM_FAST +SEED` |
| `tb_bridge_ovf` | Overflow abort on ch0 mid-session, then re-arm both channels | `+OVF_BEATS +LIMIT` |
| `tb_ingest_overflow_leak` | Ingest-port unit test: no data beat past the session limit | — |

The SerDes is a wire loopback (`rx_level = tx_level`); the link ACKs its own Flits.

## Verified (Icarus Verilog 12, Yosys 0.33)

- `tb_bridge_top`, `tb_bridge_ovf`, `tb_ingest_overflow_leak`: PASS
- Interconnect unit tests 4/4, PHY benches 24/24, link `tb_stream_path` with bit errors: PASS
- Yosys elaboration of `bridge_top` from `bridge_top.f`: 0 problems, 0 latches
- **Not run here:** Vivado or Quartus synthesis/implementation, timing closure

## Ports to note

- No pin constraints: `tx_level`/`rx_level` are meant for SerDes IP, outside this project.
- `bar0_*` and `tbl_*` are driven by the testbench until an inbound MWr/MRd decoder exists.
- Clocks: `clk` (pcie_clk, 4 ns) and `aclk` (5.4 ns) are asynchronous; see `bridge_top.xdc` / `.sdc`.
