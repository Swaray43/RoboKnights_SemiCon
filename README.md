# RoboKnights SemiCon — AXI4-Stream to PCIe 6.0 Bridge

**SARCathon 2026 · SemiCon Hackathon · Team RoboKnights**

| Member | Layer | Owns |
|---|---|---|
| **Aditya Jain** | Interconnect + integration lead | AXI4-Stream ingest, arbitration, clock crossing, top-level integration, CI, load analyser, GDS flow |
| **Swaray Agrawal** | Link layer | TLP engine, Flit framing, CRC-64 / FEC, replay (ACK/NAK), flow control, RX decode |
| **Kshitij Seshadri** | Protocol-to-PHY | Logical PHY (Gray / PAM4 lanes), LTSSM, reset controller, config space, BAR0, MSI-X |

A synthesizable RTL bridge that moves data from two FPGA-fabric AXI4-Stream producers — a **SmartNIC** (channel 0) and an **ML inference accelerator** (channel 1) — into host memory over a PCIe 6.0 Flit-mode link. It was built in two rounds, verified with directed, randomized, formal and coverage-driven testbenches under GitHub Actions, optimised to **255 MHz on a Xilinx xcu250** with timing fully met, and taken through **RTL → GDSII on SkyWater SKY130** with LibreLane.

> **This branch (`main`) holds documentation only.** All RTL, testbenches, CI and build scripts live on the code branches listed in [Branches](#branches). Start with [`docs/report_round2.pdf`](docs/report_round2.pdf) for the full technical report and [`docs/LEDGER.md`](docs/LEDGER.md) for every bug, ticket, patch and optimisation.

---

## Contents

1. [Headline results](#headline-results)
2. [Repository map: branches, tags, files](#repository-map)
3. [Problem and scope](#problem-and-scope)
4. [Architecture](#architecture)
5. [How the project was built (timeline)](#timeline)
6. [Verification and GitHub Actions](#verification-and-github-actions)
7. [Step 5: in-house traffic analyser](#step-5-in-house-traffic-analyser)
8. [FPGA synthesis and timing](#fpga-synthesis-and-timing)
9. [ASIC: RTL → GDSII on SKY130](#asic-rtl--gdsii-on-sky130)
10. [How to run everything](#how-to-run-everything)
11. [Documentation index](#documentation-index)
12. [Limitations and open items](#limitations-and-open-items)
13. [Lessons learned](#lessons-learned)

---

## Headline results

| Measure | First integrated build (`step4-base`) | Final (X.1–X.4) | Change |
|---|---|---|---|
| FPGA Fmax, PCIe clock (xcu250, Vivado 2026.1) | 118 MHz (19,046 failing endpoints) | **255 MHz, WNS +0.072 ns, 0 failing** | 2.2× |
| FPGA Fmax, AXI clock (target 185 MHz) | — | 221 MHz | met |
| LUTs / flip-flops | 56k / 24k | 57k / 39k | +2 % LUT, +63 % FF (pipelining) |
| ML tensor throughput (64 KB tensors) | 0.06 GB/s | **6.17 GB/s** | ~100× |
| SmartNIC IMIX packet rate | 0.15 Mpps | **4.84 Mpps** | 32× |
| SmartNIC IMIX p99 latency | 58 µs | **0.59 µs** | ~99× lower |
| Back-to-back 64 B packet rate | 0.48 Mpps | 11.6 Mpps | 24× |
| SmartNIC p99 under ML load (weights 1:7) | 241 µs | 4.2 µs | 57× lower |
| Interconnect ASIC (SKY130) | — | **GDSII: DRC / LVS / antenna clean**, hold met, 1.20 mm² | setup −0.95 ns at slow corner |
| Regression on final code | — | link 17/17, PHY 25/25, cocotb 10/10, interconnect 4/4, bridge 3/3, formal 4/4, coverage 73/73 bins | all pass |

Throughput and latency are from the Step 5 traffic analyser in simulation, normalised to the 250 MHz design clock; they are not board measurements.

---

## Repository map

### Branches

| Branch | What it is | Use it for |
|---|---|---|
| `main` | Documentation only: this README, both round reports, supporting reports, the ledger, problem statement, glossary (merged from `docs`) | Reading about the project |
| `docs` | The branch the documentation was assembled on before replacing `main`'s contents | — |
| `RTL-FPGA` | Final RTL: the Step 4 freeze plus GDS patches X.1, X.3, X.4 (commit `e29db76`). 255 MHz with every endpoint met. Vivado reports in `results/fpga/e29db76/`. **No X.2** (X.2 only adds GDS size parameters whose defaults equal the FPGA build). | FPGA synthesis, simulation, the reference RTL |
| `GDSII` | Physical-design branch: Phase 2 RTL + X.1, X.2, X.3, X.4, LibreLane configs and SDCs under `gds/`, interconnect GDS + metrics, layout renders in `gds/GDSII_Images/`. `*.gds` tracked with Git LFS; LibreLane `runs/` folders are never committed. | ASIC flow, viewing the layout |
| `Pre-GDS-RTLCodeBase-(Internal-Ref)` | Formerly `phase2/X`: the Phase 2 Step 4 code base (`phase2/step4`) with the X patches applied one by one (`git am`), kept as the internal reference line the FPGA and GDS branches were cut from | Internal reference; diffing the RTL against `RTL-FPGA` / `GDSII` |

The X patches were written for the ASIC flow but each was applied **separately** (`git am`) to the FPGA line and to the GDS line; the two lines were never merged into each other, so each stays buildable on its own.

### Tags

The measured build points are tagged so any number in the reports can be reproduced (`git tag -l` lists all of them):

| Tag | Point |
|---|---|
| *(no tag)* `step4-base` | End of Step 3: integrated, fully verified bridge before optimisation (118 MHz). Not a git tag: the Vivado base run is commit `49a467b`, and the load analyser's committed base results are named `step4-base` |
| `waveAB` | Step 4 Waves A + B: 512-bit TLP engine, wide PHY, 16 DW/clk RX, merging (49 MHz, the CRC-64 regression) |
| `waveC` | Step 4 Wave C: CRC split into parts, merge unit restructured (147 MHz) |
| `step4-final` | Freeze: XOR-tree CRC/FEC, registered inbound beat (248 MHz) |
| `X.1` | Ingest TREADY independent of TKEEP; interconnect GDS config + SDC |
| `X.2` | Full-bridge GDS build parameters (narrow PHY, small buffers, 30 ns) |
| `X.3` | `link_layer_rx` FEC correction without latches / logic loops |
| `X.4` | FEC correction as constant-position gates; Yosys synthesis checks clear |

### Layout of a code branch

```
.githooks/            pre-push hook: blocks direct pushes to main, runs the guard first
.github/workflows/    regression.yml, load.yml, main-guard.yml
bridge_top/           bridge_top.v (full stack), bridge_top.f, benches, Vivado/Quartus scripts,
                      vivado_baseline.tcl (one-page summary + failing-endpoint histogram)
interconnect_layer/   src/: axi4s_ingest_port, rr_arbiter_2ch, cdc_fifo_async, interconnect_top + unit benches
src-link_layer/hdl/   link_layer_top, tlp_tx_wide, link_layer_tx/rx, flit_crc64_parts, flit_fec_syn,
                      replay buffer, credit manager, RX delimiter, link_phy_bridge_top; files.f, run_all_link.py
protocol_PHY_layer/   rtl/ (one module per file, files.f), tb/, formal/, cocotb/, run_all.py, CHANGELOG.md
shared/               axis_tuser_defs.vh: the single definition of the interconnect → link TUSER layout
asic/                 ASIC-profile simulation parameters (asic/sim/asic_params*.v)
gds/                  (GDSII branch) interconnect/, bridge/, rxcheck/ configs + SDCs, GDSII_Images/
ci/                   run_regression.py, check_protected.py + protected_lines.txt, coverage_report.py,
                      load/ (run_workloads.py, compare.py, thresholds.json, make_report.py, base results)
verification/formal/  tlp_header_parser.sby
docs/                 problem statement, round 1 report, STEP4_ROADMAP.md (register map, as-built notes)
results/fpga/         (RTL-FPGA branch) committed Vivado reports per measured commit
```

**Legacy rule:** anything under a `legacy/` folder is kept for reference only, is on no file list, and may redefine active modules.

---

## Problem and scope

The SARCathon problem statement asks for an AXI4 ↔ PCIe 6.0 bridge: RTL, simulation, transaction and reset/error verification for the qualifier; optimisation, workload evaluation, synthesis, open-source physical design and GDSII for the final round.

Two workloads drive the design, both **device-to-host writes**:

- **SmartNIC packet ingestion** — variable-size frames (IMIX 64 / 596 / 1520 B), high packet rate, tail latency matters.
- **ML inference output streaming** — long output tensors (64 KB to 16 MB), sustained bulk throughput matters.

Two decisions taken in Round 1 shaped everything after:

1. **AXI4-Stream only (AD-SCOPE-1).** Two independent 512-bit AXI4-Stream ingest ports. Not implemented: the AXI4 memory-mapped AW/W/B and AR/R channels, the read-completion clock crossing and per-beat 64-bit addresses. This is a documented scope decision, not a claim of full bidirectional AXI4.
2. **Address split across layers (AD-SCOPE-2).** The interconnect never carries a 64-bit address. It produces a per-channel running byte offset; the link layer forms `real_addr = base_addr_ch[N] + session_offset_ch[N]`, where the base addresses are host-written BAR0 registers. One host buffer and one MSI-X vector per channel. The interconnect datapath stays 24 bits wide for addressing instead of 64.

The receive direction is supported only as **inbound host memory writes to BAR0** (configuration from the host), by design. The PHY is a **logical** model (Gray-coded PAM4 symbols, lanes, training); it is not an analog SerDes and no PCIe compliance testing is claimed.

---

## Architecture

```
  ch0: SmartNIC AXI4-Stream ─┐                         ┌─ ch1: ML accelerator AXI4-Stream
                             │  512-bit TDATA, 64-bit TKEEP, TLAST, TVALID/TREADY
╔════════════════════════════╪═════════════════════════╪═══════════════════════════════╗
║ LAYER 1 — INTERCONNECT (Aditya)            interconnect_top          clock: aclk     ║
║   axi4s_ingest_port (ch0)          axi4s_ingest_port (ch1)                           ║
║     session offset, limit, overflow   session offset, limit, overflow                ║
║                 └──────────── rr_arbiter_2ch ─────────────┘                          ║
║                    interleaved weighted round-robin, starvation guard                ║
║                               cdc_fifo_async  (aclk → pcie_clk)                      ║
╚═══════════════════════════════════════╪══════════════════════════════════════════════╝
            512 b data + 64 b keep + 26 b TUSER {tlast_flag, offset[23:0], stream_id}
╔═══════════════════════════════════════╪══════════════════════════════════════════════╗
║ LAYER 2 — LINK (Swaray)              link_layer_top                clock: pcie_clk   ║
║   TX: tlp_tx_wide (512-bit TLP engine, merge to max payload, 4 KB split)             ║
║       credit gate (InitFC / UpdateFC) → Flit framer (236 B TLP + 6 B DLP +           ║
║       8 B CRC-64 + 6 B FEC = 256 B) → replay buffer (ACK/NAK, 2048-clk timer)        ║
║   RX: FEC syndromes + SEC-DED correction → CRC-64 check → 16-DW/clk TLP delimiter    ║
║       → header decode → inbound MWr to BAR0, DLLP parser (ACK/NAK, FC)               ║
╚═══════════════════════════════════════╪══════════════════════════════════════════════╝
                         Flit stream with ready / back-pressure
╔═══════════════════════════════════════╪══════════════════════════════════════════════╗
║ LAYER 3 — PROTOCOL / PHY (Kshitij)   link_phy_bridge_top → pcie_phy_top               ║
║   TX Flit FIFO + high-watermark back-pressure; PHY_WIDE=1: 8 B/lane/clk (FPGA),      ║
║   PHY_WIDE=0: narrow (ASIC); 4-lane striper, Gray + PAM4 map, SOF insert, idle fill  ║
║   RX: PAM4 demap, deskew, lane reversal, SOF strip, destriper                         ║
║   Control: LTSSM (recovery-lite), reset controller (PERST#, hot reset, FLR),         ║
║   config space, BAR0 DMA registers, error counters, MSI-X (2048 vectors, mask, PBA)  ║
╚═══════════════════════════════════════╪══════════════════════════════════════════════╝
                       4 logical PAM4 lanes  (looped back in simulation)
```

### Layer 1 — Interconnect (Aditya)

| Module | Function |
|---|---|
| `axi4s_ingest_port` (× 2) | AXI4-Stream slave per channel. Tracks the byte offset inside the current session (popcount of TKEEP), compares against the per-channel session limit, raises a sticky `session_overflow` and sends a zero-length TLAST marker so the link layer closes an aborted session. After X.1, TREADY no longer depends on TKEEP: full beats are accepted on a registered offset, and only a beat within 64 B of the limit is held one cycle for an exact size check. |
| `rr_arbiter_2ch` | Weighted round-robin with runtime weights from BAR0, one beat per grant (interleaved, S4-7), starvation guard after 64 cycles, registered grants so no combinational path reaches the 512-bit mux. |
| `cdc_fifo_async` | Gray-code asynchronous FIFO, the only clock crossing between `aclk` and `pcie_clk`. 2-flop synchronisers, first-word-fall-through read, `almost_full` used as shared back-pressure. LUT RAM. |
| `interconnect_top` | Two ports → crossbar → arbiter → FIFO → link. Emits the link-native TUSER through `shared/axis_tuser_defs.vh` (`tuser_pack()`), with a `$fatal` width check. `pulse_sync` carries `session_start` from `pcie_clk` into `aclk`. |

### Layer 2 — Link (Swaray)

| Module | Function |
|---|---|
| `tlp_tx_wide` | 512-bit TLP engine (replaced a 512→32-bit serialiser + 32-bit packetizer). Merges beats of one session into max-payload memory-write TLPs (up to 64 DW), never crossing a 4 KB boundary. Address = channel base + session offset. |
| Credit manager / FC FSM | Real PCIe flow control: InitFC1 / InitFC2 / UpdateFC DLLPs; TLPs wait for Posted header and data credits. |
| Flit framer (`link_layer_tx`) | Packs TLPs into 256-byte Flits: 236 B TLP area, 6 B DLP (sequence number, ACK/NAK), 8 B CRC-64, 6 B FEC. 3-stage pipeline, one Flit per 8 clocks. |
| `flit_crc64_parts` | CRC-64 computed per chunk and combined, written as constant-mask XOR trees (the change that took Fmax from 49 to 147+ MHz). |
| `flit_fec_syn` | FEC syndromes as XOR trees; 3-way interleaved SEC-DED corrects one bit per interleave. |
| Replay buffer | Keeps sent Flits until ACK; NAK or a 2,048-clock timeout replays them. Depth 32 Flits on FPGA. |
| `link_layer_rx` | FEC correction (constant-position gates after X.4), CRC-64 check, NAK on error. |
| RX delimiter + header decode | 16 DW per clock TLP boundary detection and header parse; inbound MWr routed to BAR0; MWr/MRd/Cpl/CplLk decode proven formally. |
| `link_phy_bridge_top` | Glue between link and PHY; forwards MSI-X requests, base addresses and limits from BAR0. |

### Layer 3 — Protocol / PHY (Kshitij)

| Module | Function |
|---|---|
| `pcie_phy_top` | Composed PHY: TX Flit FIFO with `phy_tx_ready`, SOF insertion, 4-lane striping, Gray/PAM4 mapping; RX demap, elastic deskew, lane-reversal detect and latch, SOF strip, destripe. Logical idle (`0x1B`) in every free L0 slot. |
| LTSSM (recovery-lite, K6) | DETECT → training (TS1/TS2) → L0. Leaves L0 on signal loss (64 clocks), lost framing (512 unframed cycles) or partner TS1s; datapath reset pulse and retrain in ~1,700 clocks. |
| `pcie_reset_controller` | PERST#, hot reset, FLR; `axi_rst_n` released only after training and the first UpdateFC. |
| Config space + `bar0_dma_regs` | Type 0 header, BARs; BAR0 DMA registers: per-channel base addresses, arbiter weights, traffic class, session limits, XLAT_MISS counter, sticky error counters. |
| `msix_controller` | Up to 2048 vectors, per-vector and function masking, pending-bit array; one vector per channel on session end. |

### Interfaces and clocks

| Interface | Signals |
|---|---|
| AXI ingress (each channel) | 512-bit TDATA, 64-bit TKEEP, TLAST, TVALID / TREADY |
| Interconnect → link | 512-bit TDATA, 64-bit TKEEP, 26-bit TUSER `{tlast_flag, session_offset[23:0], stream_id}`, valid / ready |
| Link ↔ PHY | Flit data + SOF/EOF, `phy_tx_ready` back-pressure, CRC / FEC error flags |
| Host control | BAR0 reads/writes (config and inbound MWr), MSI-X, PERST#, hot reset |
| Clocks | `aclk` (AXI side, 5.4 ns FPGA constraint) and `pcie_clk` / `clk` (4.0 ns = 250 MHz FPGA target); asynchronous to each other |

### BAR0 register highlights

| Offset | Register |
|---|---|
| 0x0C, 0x34 | Per-channel session limit (bytes) |
| 0x1C | XLAT_MISS counter (sticky, write to clear) |
| 0x20, 0x28 | Channel 0 / channel 1 host buffer base address |
| 0x40 – 0x64 | Sticky error counters (FEC corrected, uncorrectable, CRC, replays, link-down, …), cleared only by PERST# |

The full register map with as-built notes is in `docs/STEP4_ROADMAP.md` on the code branches.

---

## Timeline

### Round 1 — Stage 1 (to 15 Oct 2026)

Each layer built its RTL and unit benches independently; `Final/system_top.v` chained all three with a PHY loopback.

- **Interconnect:** complete; 4 real bugs fixed (FIFO re-write under back-pressure, arbiter deadlock with one requester, stale first FIFO read, combinational loop on the full flag); integration bench **128 / 128 beats, 0 errors**.
- **Link layer:** data-link core wired (write buffer, replay buffer, TX/RX Flit framing, DLLP parser); transaction-layer blocks (tag and credit managers, packetizers, address translation, reorder buffer, RX engine) unit-tested but not integrated.
- **PHY:** full lane chain bit-exact in loopback; reset / config / MSI-X control plane; SymbiYosys proofs for Gray and PAM4; cocotb 4 / 4.
- **System top:** connectivity smoke tests pass — a beat goes in, a reconstructed Flit comes out. Not yet a data-correctness result: no TLP packetizer, and no back-pressure between link and PHY.

Report: [`docs/report_round1.pdf`](docs/report_round1.pdf).

### Round 2 — Integration to GDSII (16–24 Oct 2026)

| Step | Dates | Work | Outcome |
|---|---|---|---|
| Review + tickets | 17–20 Oct | Layer-by-layer review of the merged repo; ticket rounds T1 and T2 | 27 design bugs fixed (7 blockers, 12 data-corrupting); benches repaired |
| Step 1 | 20 Oct | Integration gaps G1–G7 | Real flow control, host BAR0 writes, XLAT_MISS, shared TUSER header; **144 / 144 TLPs end to end** |
| Step 2 | 21 Oct | Vivado baseline, perf monitor, first optimisations | 39 MHz → 118 MHz; two stale-copy regressions caught and repaired |
| Step 3 | 21–22 Oct | CI, randomized bench, coverage, formal | 46 guard rules, 40-seed sweep, 73 / 73 bins, 4 / 4 proofs; found S7, S13, K6 |
| Step 4 | 22–23 Oct | Three optimisation waves + freeze | 118 → 248 MHz; datapath 512 bits end to end |
| Step 5 | 23 Oct | In-house traffic analyser | ML 0.06 → 6.17 GB/s; NIC p99 58 → 0.59 µs |
| GDS | 23–24 Oct | LibreLane on SKY130, patches X.1–X.4 | Interconnect GDSII clean; FPGA 255 MHz with timing met |

In total **44 design bugs and 21 testbench bugs** were found and fixed in Round 2. Every one, with what found it, who fixed it and which patch carries it, is in [`docs/LEDGER.md`](docs/LEDGER.md).

**What the 19 Oct review found (worst first):** a duplicate `axi4s_ingest_port` that stopped the layers compiling together; a serialiser emitting 15 DWs to a 16-DW packetizer; a replay buffer with no ACK source (stopped after 31 beats); no ready between link and PHY (PHY 32× slower); TUSER bit order mismatched between interconnect and link (every beat routed as channel 0); sequence numbers written over data bits; 40 bits of every fourth beat dropped; FEC with no correction.

**Step 4 waves:**

| Wave | Changes | Fmax |
|---|---|---|
| Start | `step4-base` | 118 MHz |
| A | 3-stage RX CRC/FEC check; 3-stage TX framer (18 → 8 clocks per Flit); sticky BAR0 error counters; per-channel session limit + interleaved WRR | — |
| B | 512-bit TLP engine; merging to max payload; wide PHY (8 B/lane/clk); 16-DW/clk RX delimiter | 49 MHz (unrolled CRC-64: 51 logic levels) |
| C | CRC-64 split into parts; merge unit from registers; replay timer 2,048 clocks | 147 MHz |
| Freeze | CRC/FEC as constant-mask XOR trees; inbound beat registered before header gather | 248 MHz |
| + X.1, X.3, X.4 | TREADY independent of TKEEP; FEC correction as gates | **255 MHz, all endpoints met** |

Report: [`docs/report_round2.pdf`](docs/report_round2.pdf).

---

## Verification and GitHub Actions

All simulation runs on **Icarus Verilog** in CI (the team also uses ModelSim locally), plus **cocotb** and **SymbiYosys**.

### Test suites (`ci/run_regression.py`)

| Suite | Contents | Final result |
|---|---|---|
| `guards` | `ci/check_protected.py`: 46 rules, each a line an earlier fix must keep in the code; complete file lists; no module defined twice | 46 / 46 |
| `link` | `src-link_layer/hdl/run_all_link.py`: TX engine order monitor, RX engine (incl. CplLk/CplDLk), parser, replay, credit-gate proof, Flit framing, loopback | 17 / 17 |
| `phy` | `protocol_PHY_layer/run_all.py`: lanes, Gray/PAM4, deskew, SOF, reset controller, LTSSM incl. link-loss retrain (Part K) | 25 / 25 |
| `cocotb` | PHY control plane: reset, config, BAR sizing, MSI-X masking, link-loss retrain | 10 / 10 |
| `interconnect` | ingest port, arbiter, CDC FIFO, interconnect top | 4 / 4 |
| `bridge` | `tb_bridge_top` (143 TLPs, MSI-X), `tb_bridge_ovf` (overflow, 34 TLPs), `tb_ingest_overflow_leak` | 3 / 3 |
| `errors` | `tb_stream_path` with bit-error injection and random back-pressure | pass: 19 FEC corrections, 4 replays, all data correct |
| `asic` | bridge and stream benches with ASIC / GDS buffer sizes (`asic/sim/`) | 5 / 5 (GDSII branch), 3 / 3 (RTL-FPGA) |
| `rand` | `tb_bridge_rand` over many seeds, clean and with PAM4 noise | pass |
| `long` | `tb_bridge_top`, 6 sessions × 50 beats = 600 TLPs | pass |
| `formal` | Gray encode/decode inverse, PAM4 map vs spec, start-of-Flit strip, TLP header parser | 4 / 4 proofs |

### Randomized bench `tb_bridge_rand` (features F1–F9, F7b)

| Feature | Randomises |
|---|---|
| F1–F4 | Session count, beats per session, last-beat sizes (1–16 DW), gaps, arbiter weights, unaligned base addresses, PAM4 symbol noise |
| F5 | Session-limit overflow at a random point |
| F6 | Lane reversal |
| F7 | Hot reset mid-session |
| F7b | Cable pull and signal glitch mid-session (link-down and retrain) |
| F8 | MSI-X vector and function masking |
| F9 | Host writes into BAR0 (weights, traffic class, base addresses) |

### Coverage (40 seeds, final code): 73 / 73 required bins

Every BAR0 register written (10) and read back (13); 55 beats split at a 4 KB boundary; every last-beat size 1–16 DW; arbiter wins for both channels at every weight 1–7; 1,069 FEC corrections, 433 uncorrectable Flits, 175 CRC-64 failures, 262 replays, 2,197 XLAT_MISS counts, 53 session-limit overflows — all with correct data; 248 MSI-X deliveries; 10 hot resets, 7 cable pulls, 5 glitches; straight (29 seeds) and reversed (11) lanes. Optional bins not reached randomly are covered by directed benches.

### GitHub Actions workflows

| Workflow | Trigger | What it does |
|---|---|---|
| `regression.yml` › guards | every push and PR | Protected-line guard, file lists, duplicate modules |
| `regression.yml` › sim matrix | every push and PR | One job per suite: link, PHY, cocotb, interconnect, bridge, errors, asic, rand |
| `regression.yml` › long | nightly 02:00 IST | 600-TLP run |
| `regression.yml` › rand sweep + coverage | nightly | 40 seeds of `tb_bridge_rand`, merged `COV:` lines, fails on an empty required bin |
| `regression.yml` › formal | nightly | SymbiYosys proofs |
| `load.yml` | pushes/PRs to `phase2/step4*`, nightly (4 MB, 16 MB tensors) | Step 5 workloads; fails if throughput or packet rate drops > 2 %, p99 latency rises > 5 %, or replays rise > 10 % vs `step4-base`. Uploads JSON results. Jain fairness reported, not gated (a weighted arbiter is unfair by design). |
| `main-guard.yml` | every push to `main` (code era) | Runs guards + key suites; on failure resets `main` to the last good commit with one auto-revert commit and opens an issue naming the commit, author and log |
| `.githooks/pre-push` (local) | every push | Blocks direct pushes to `main`, runs the guard before anything leaves the laptop. Enable with `git config core.hooksPath .githooks`. |

The guard exists because of one bad day: in Step 2, two fixes were applied to stale copies of files and **nine earlier fixes were silently lost**. The guard catches any repeat; the stale copies were deleted.

---

## Step 5: in-house traffic analyser

Built to answer "does the bridge serve its two users?", which Fmax alone does not.

- **Generator:** `tb_bridge_rand +WL=n` — 1 SmartNIC IMIX, 2 ML tensors, 3 mixed NIC + ML with `+W0`/`+W1` weights, 4 back-to-back 64 B, 5 tensors 64 B below a 4 KB line; `+NOISE_PPM`, `+BURST` inject PAM4 errors.
- **Measurement:** every beat timestamped at AXI ingress and RX egress; per-channel bytes and packets, p50 / p99 latency, MSI-X rate, peak occupancy of every buffer, replays, FEC fixes, credit stalls → one `LOAD:` line per run.
- **Runner:** `ci/load/run_workloads.py` runs named scenarios, writes JSON; `--base <ref>` builds another ref in a temporary worktree.
- **Gate:** `compare.py` + `thresholds.json` against committed `step4-base` results; `make_report.py` renders the before/after HTML.

| Scenario | Traffic | Base | Final |
|---|---|---|---|
| W1 | SmartNIC IMIX 64/596/1520 B at 7:4:1 | 0.15 Mpps, p99 58 µs | 4.84 Mpps (1.72 GB/s), p99 0.59 µs |
| W1_B2B | 200 back-to-back 64 B | 0.48 Mpps | 11.6 Mpps |
| W2_64K | 4 × 64 KB ML tensors, unaligned | 0.061 GB/s | 6.17 GB/s, p99 1.26 µs |
| W2_1M | 1 MB tensor | 0.061 GB/s | 6.17 GB/s |
| W2_4K | tensors 64 B below a 4 KB line | 0.062 GB/s | 6.17 GB/s |
| W3 1:1 / 1:3 / 1:7 | ML saturating + 100 NIC packets | NIC p99 up to 241 µs | NIC / ML 0.70 / 5.24, 0.59 / 5.68, 0.43 / 5.69 GB/s; NIC p99 4.2 µs at 1:7 |
| W3 + 300 ppm noise | mixed 1:3 with channel errors | ~0 GB/s, 1,641 replays | ~0.1 GB/s, 624 replays |

Every scenario delivers every byte in order. The gains come from Wave B (wide engine, merging, wide RX and PHY); the timing waves after it left behaviour unchanged, confirmed by rerunning all scenarios on the final code. Heavy channel noise remains the weak spot.

---

## FPGA synthesis and timing

Target: Xilinx **xcu250-figd2104-2L-e**, Vivado 2026.1, out-of-context, PCIe clock 4.0 ns (250 MHz), AXI clock 5.4 ns.

| Point | LUTs | FFs | WNS @ 4 ns | Failing endpoints | Worst path |
|---|---|---|---|---|---|
| `step4-base` | 56,043 | 24,482 | −4.439 ns | 19,046 | RX raw Flit → `crc_err`, 19 levels |
| `waveAB` | 67,999 | 36,244 | −16.285 ns | — | TX body → CRC, 51 levels |
| `waveC` | 61,914 | 39,147 | −2.789 ns | 18,164 | RX delimiter TKEEP → `inb_mwr_data`, 28 levels |
| `step4-final` | 60,816 | 39,304 | −0.027 ns | 82 | PHY TX FIFO count → Flit enable, 3 levels |
| **Final X.1–X.4** | **57,245** | **39,269** | **+0.072 ns** | **0** | `tlp_tx_wide` enable, 11 levels (met; WHS +0.019 ns) |

No BRAM, URAM or DSP is used; all buffers are LUT RAM; ~3.3 % of the device's LUTs. Vivado power estimate (synthesised, vectorless): 3.50 W total, 0.545 W dynamic. Reports for the final run: `results/fpga/e29db76/` on `RTL-FPGA`.

---

## ASIC: RTL → GDSII on SKY130

**Flow:** LibreLane (the OpenLane 2 successor, OpenROAD-based) · PDK **SkyWater SKY130A** · library `sky130_fd_sc_hd` · run in Docker (`--dockerized`) or Nix · two asynchronous clocks in the SDC · multi-corner STA (slowest corner `ss_100C_1v60`).

### GDS patches

| Patch | Problem | Fix |
|---|---|---|
| **X.1** | Interconnect failed setup by −12 ns: at the slow corner TKEEP → popcount → offset add → limit compare → TREADY → FIFO write took ~27 ns in one cycle | Offset kept as registered `off_base + bytes_q`; full beats accepted without looking at TKEEP; a beat within 64 B of the limit held one cycle for an exact check. Interconnect GDS config + SDC. |
| **X.2** | Full bridge (wide PHY, 32-Flit replay buffer) too large to place and route in the time available | Buffer sizes plumbed as `bridge_top` parameters (defaults = FPGA build); narrow GDS build: `PHY_WIDE=0`, replay depth 4, FIFOs 4 / 8, 30 ns; matching simulation profile in CI. |
| **X.3** | LibreLane's Yosys: 230 check errors in `link_layer_rx` (latches and logic loops in FEC correction; one loop integer driven from two blocks) | Defaults at top of the block; separate index for the clocked loop. |
| **X.4** | A newer Yosys still found 192 logic loops in the loop form | Each protected bit's Hamming position made an elaboration-time constant; correction written as plain gates. Same error-injection results; `gds/rxcheck` 2-minute synthesis-check config. |

Flow fixes along the way: linter off (Verilator rejected the cell models' timescales), explicit RTL file lists (testbenches in `src/` broke synthesis), exact port names in the SDC, run from the repository root when it lives outside `$HOME` (Docker mounts only home and the current folder).

### Results

| Run | Clocks | Result |
|---|---|---|
| **`interconnect_top`** | `aclk` 30 ns, `pcie_clk` 15 ns | **GDSII produced. DRC, LVS and antenna clean; hold met.** Die 1.20 mm² at 41 % utilisation, ~49k cells (9.8k flip-flops), 104 mW. Setup −0.95 ns at the slow corner (−0.52 ns after X.1). |
| `link_layer_rx` alone (`gds/rxcheck`) | 30 ns | After X.4 all Yosys synthesis checks clear (was 230, then 192). 1.12 mm² of standard cells. |
| Full bridge, narrow build (X.2) | 30 ns | Synthesis, floorplan, placement, CTS and global routing completed (~3.9 mm² of cells). **Detailed routing did not converge**: the first attempt ran out of memory (~14 GB), and the restart with fewer threads saw violations rise (3,200 → 9,748) instead of falling — the floorplan is too congested at 30 % core utilisation. Stopped at the 24 Oct cutoff. |

Layout renders of the interconnect GDS (KLayout): `gds/GDSII_Images/` on the `GDSII` branch; the GDS itself is `gds/interconnect/results/interconnect_top.gds` (Git LFS) with `metrics.json`.

**Next step for the full bridge:** relax the floorplan (`FP_CORE_UTIL` ≈ 20 %, `PL_TARGET_DENSITY_PCT` ≈ 30 %), cap router threads to fit memory, rerun from floorplan.

---

## How to run everything

Check out a code branch first (`git checkout RTL-FPGA` or `git checkout GDSII`).

### Simulation

```sh
# full regression (everything except 'long' and 'formal')
python3 ci/run_regression.py
python3 ci/run_regression.py guards bridge link      # selected suites
python3 ci/run_regression.py --list

# single benches
iverilog -g2012 -DSIMULATION -s tb_bridge_top -o b.vvp -c bridge_top/bridge_top.f -c bridge_top/bridge_top_tb.f && vvp b.vvp
python3 src-link_layer/hdl/run_all_link.py
cd protocol_PHY_layer && python3 run_all.py
cd protocol_PHY_layer/cocotb && make
```

### Load analyser

```sh
python3 ci/load/run_workloads.py                  # all scenarios → JSON
python3 ci/load/run_workloads.py --base step4-base
python3 ci/load/compare.py <base.json> <new.json>
python3 ci/load/make_report.py <json files>       # HTML report
```

### FPGA (Vivado)

```powershell
$env:BASELINE_SIM=0; $env:BASELINE_JOBS=4
& "C:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat" -mode batch -source bridge_top/vivado_baseline.tcl -log baseline.log
```
(Keep the project path free of spaces and brackets.)

### ASIC (LibreLane, `GDSII` branch)

```sh
# 2-minute synthesis check of link_layer_rx
librelane --dockerized --pdk sky130A --scl sky130_fd_sc_hd gds/rxcheck/config.json --to Checker.YosysSynthChecks
# interconnect
librelane --dockerized --pdk sky130A --scl sky130_fd_sc_hd gds/interconnect/config.json
# full bridge (narrow build)
librelane --dockerized --pdk sky130A --scl sky130_fd_sc_hd gds/bridge/config.json
# resume a failed run from a step
librelane ... config.json --run-tag <RUN_TAG> --from <StepName>
# view
klayout gds/interconnect/results/interconnect_top.gds
```

Run from the repository root; LibreLane writes `runs/` next to each config (never committed).

---

## Documentation index

| File (`main`) | What |
|---|---|
| [`docs/report_round2.pdf`](docs/report_round2.pdf) | Round 2 technical report: integration, verification, optimisation, FPGA, GDSII, schematics |
| [`docs/report_round1.pdf`](docs/report_round1.pdf) | Stage 1 report: per-layer RTL and unit verification |
| [`docs/LEDGER.md`](docs/LEDGER.md) | Complete ledger: every bug, ticket, patch, optimisation and test, by round and step, with chains of what led to what |
| [`docs/supporting/Round1_Report_Stage1.pdf`](docs/supporting/Round1_Report_Stage1.pdf) | Round 1 narrative: scope decisions, per-layer modules and bugs, Stage 1 test results, how each open item closed |
| [`docs/supporting/Round2_Report_Integration_to_GDSII.pdf`](docs/supporting/Round2_Report_Integration_to_GDSII.pdf) | Round 2 narrative: review findings, Steps 1–5, GDS patches, open items and lessons |
| [`docs/supporting/Bridge_Results_and_Deliverables.pdf`](docs/supporting/Bridge_Results_and_Deliverables.pdf) | All results in one place: build points, FPGA per-run tables, CI checks, coverage, Step 5 scenarios, GDS runs, deliverables index |
| [`docs/PS/`](docs/PS/) | SARCathon problem statement |
| `docs/Glossary_Semicon - Sheet1.pdf` | Glossary of PCIe / AXI terms used |
| `docs/work_distribution.md` | Original per-member work split (Round 1 plan) |

On the code branches: `docs/STEP4_ROADMAP.md` (Step 4 plan, as-built notes, register map), `protocol_PHY_layer/CHANGELOG.md`, `asic/README.md`.

---

## Limitations and open items

- **Scope:** device-to-host AXI4-Stream only; inbound traffic limited to host MWr into BAR0. No AXI4 memory-mapped ports.
- **PHY:** logical model only (Gray/PAM4 symbols, lanes, training). Not a characterised SerDes; no compliance testing; loopback is simulated.
- **Throughput figures** are simulation results at the 250 MHz design clock, not board measurements.
- **Heavy channel noise:** at 300 ppm throughput falls to ~0.1 GB/s (replay-bound). At 2,000–3,000 ppm, the MSI-X (F8) or BAR0-write (F9) checks can time out; data is never corrupted (every Flit is CRC-64 checked). Faster NAK / shorter replay timer is the next lever.
- **FPGA timing:** internal paths met; many I/O ports have no input/output delay constraint, so interface timing is not closed.
- **ASIC:** only the interconnect block has a sign-off-clean GDSII; it misses setup at the slow corner. The full bridge stopped at detailed routing (congestion).
- **Tooling nits:** a guard rule containing `//` never matches; the PHY reset checker prints `VIOLATION` lines `run_all.py` does not count.

---

## Lessons learned

1. **Edit the repository, never a copy.** Nine fixes were lost once to stale copies; a guard that checks every past fix is present, plus deleting the copies, ended it.
2. **Review before integrating.** The layer review and ticket rounds removed every blocker and 12 of 13 data-corrupting bugs before integration started.
3. **Random traffic finds what directed tests miss.** S7, S13 and the need for link-loss recovery (K6) appeared only after the random bench, coverage and cable-pull tests existed.
4. **Write for every tool from day one.** Code Icarus and Vivado accepted was rejected by Yosys twice (inline loop variables, procedural correction loops); an early synthesis check would have caught it.
5. **Measure the user's traffic, not just timing.** The load analyser showed the 100× gain and the noise weakness that Fmax alone hid.
6. **Size physical design early.** The full bridge needed more die area and memory than a laptop run allowed; a floorplan trial on day one would have set the utilisation correctly.
