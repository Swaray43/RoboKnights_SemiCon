# RoboKnights SemiCon — Ticket / Patch / Fix Ledger

AXI4-Stream (2 × 512 b) → PCIe 6.0 bridge. Owners: **Aditya** (interconnect layer, integration lead),
**Swaray** (link layer), **Kshitij** (protocol-to-PHY layer).

This file is the single cross-reference for both round reports (`report_round1.pdf`, `report_round2.pdf`).
Every row is one bug, test gap, optimisation, infra item or scope item, with where it came from, who
fixed it, which patch or commit carries it, and what it led to.

Final version, 24 Oct 2026. Sources: team working notes, git history of `Swaray43/RoboKnights_SemiCon`,
the round 1 and round 2 ticket files (now merged here), `STEP4_ROADMAP.md`, the 19 Oct layer review,
`report_round1.pdf`, `protocol_PHY_layer/CHANGELOG.md`. Rows marked **CONFIRM (Swaray)** are link-layer
items whose itemised Round 1 detail was not recorded separately.

---

## 1. Conventions

### 1.1 Rounds

| Round | Dates | Meaning |
|---|---|---|
| **R1** | up to 15 Oct 2026 | SARCathon Stage 1: RTL + testbenches + Stage 1 report (`docs/report_round1.pdf`). Git commits 6–13 Sep. |
| **R2** | 16 Oct 2026 onward | Integration, Phase 2 Steps 1–5, GDSII (X patches). |

### 1.2 Phase 2 steps (R2)

| Step | Purpose | Typical type |
|---|---|---|
| Pre-P2 | Review of the merged three-layer repo (17–19 Oct), ticket rounds T1 and T2 (20 Oct) | BUG, VERIF |
| Step 1 | Integration gaps G1–G7 (G8 started, superseded by S4-8) | BUG / FEATURE |
| Step 2 | Baseline: Vivado scripts, perf monitor, first bug fixes, early optimisation ranks 4/5 | BUG, OPT, INFRA |
| Step 3 | CI, randomized bench (F1–F9, F7b), coverage, formal; bugs found by these | VERIF, BUG |
| Step 4 | Last optimisation cycle: Waves A, B, C, freeze `step4-final` | OPT |
| Step 5 | Load analyser: SmartNIC / ML / mixed workloads | VERIF / PERF |
| GDS | RTL → GDSII on SKY130 with LibreLane; patches X.1, X.2, … | BUG (synthesis), OPT, INFRA |

### 1.3 Types

| Type | Meaning |
|---|---|
| **BUG** | Wrong behaviour or broken build in RTL. |
| **TB-BUG** | Bug in a testbench or script, not the design. |
| **VERIF** | New test, check, coverage or CI capability. |
| **OPT** | Timing, area or throughput improvement with the same function. |
| **FEATURE** | New function needed by the problem statement. |
| **INFRA** | Build, file lists, CI, tooling, repo hygiene. |
| **SCOPE** | Question about what the problem statement requires, or a decision to defer. |

Severity for BUGs: **BLOCKER** (nothing works / won't build), **CORRUPTS** (wrong data), **FUNC** (wrong behaviour in a case), **MINOR**.

### 1.4 ID namespaces

| Prefix | Source | Example |
|---|---|---|
| `R1-IC/LL/PHY-x` | Round 1 by layer | `R1-IC-2` |
| `REV-Ix`, `REV-Lx`, `REV-Px`, `REV-Seam-A..D` | Layer review, 19 Oct | `REV-L1` |
| `T1-Sx`, `T1-Kx` | Ticket round 1 (20 Oct) | `T1-S7` = reorder-buffer bench |
| `T2-Rx`, `T2-P2-x` | Ticket round 2 (20 Oct) | `T2-P2-5` |
| `SW-xx` | Found by/for Swaray's layer while doing tickets | `SW-H2` |
| `G1..G8`, `G3a`, `G5a..c`, `G6a` | Step 1 gaps and sub-bugs found while fixing them | `G5a` |
| `S2-xx` | Step 2 items | `S2-OPT-4` |
| `F1..F9`, `F7b` | Step 3 randomized-bench features (tests) | `F7b` |
| `S7, S12, S13, K6, F1-FIX, F6-FIX` | Bugs raised in Step 3 | `S13` |
| `S4-x`, `S4-Tx`, `S4-C-x` | Step 4 waves | `S4-2` |
| `X.n` | GDSII-phase patches | `X.3` |
| `AD-…`, `KS-…` | Rows from Aditya's / Kshitij's working notes | `KS-K6-IDLE` |

Note: `T1-S7` (reorder-buffer bench) ≠ `S7` (TX DLLP slot bug). `REV-P5` (PHY module duplication) ≠ `SW-P5` (packetizer DW0). Aditya's answers to Kshitij's tickets use `AD-KT-n` to avoid clashing with `K6`.

---

## 2. Round 1 (≤ 15 Oct): Stage 1

### 2.1 What was built

| Ref | Type | Owner | Area | Summary | Status |
|---|---|---|---|---|---|
| AD-SCOPE-1 | SCOPE | Aditya | all | Scope set to AXI4-Stream device-to-host writes only (two ingest ports: SmartNIC, ML). Dropped AXI4-MM AW/W/B, AR/R, read-completion CDC, per-beat `addr64`. | Done |
| AD-SCOPE-2 | SCOPE | Aditya → Swaray, Kshitij | DMA addressing | Interconnect never handles a 64-bit address: `real_addr = base_addr_ch[N] + session_offset_ch[N]` in the link layer; base addresses host-written in BAR0 (Kshitij); one MSI-X vector and one host buffer per channel. | Done |
| AD-IC-FEAT-INGEST | FEATURE | Aditya | `axi4s_ingest_port` | Per-channel AXI4-Stream slave; byte-accurate offset via popcount(TKEEP); look-ahead overflow guard; sticky `session_overflow`; `req`/`accepted` split. | Done |
| AD-IC-FEAT-ARB | FEATURE | Aditya | `rr_arbiter_2ch` | Weighted round-robin, runtime weights, starvation guard (`STARVE_CYCLES` 64), registered grants. Limit: 3:1 gives bursts, not interleave (fixed in S4-7). | Done |
| AD-IC-FEAT-FIFO | FEATURE | Aditya | `cdc_fifo_async` | Gray-code async FIFO, depth 16, 2-flop sync, `almost_full` at 75% as shared back-pressure; 602-bit word. | Done |
| AD-IC-FEAT-TOP | FEATURE | Aditya | `interconnect_top`, `session_arm_ctrl`, `bridge_top` (Stage 1) | Crossbar + FIFO + output; single `m_tuser` bus to match Swaray's contract; per-channel session-arm FSM (IDLE→LATCH→ACTIVE, `session_valid` level). | Done |
| R1-LL-BUILD | FEATURE | Swaray | link layer | `write_data_buffer`, `replay_buffer`, `link_layer_tx/rx` (per-channel Flit assembly, CRC/FEC framing), DLLP parser, `data_link_layer_top` (256-bit PHY bus). Built and unit-tested but not yet in `system_top`: tag manager, credit manager, TLP packetizers, address translation, AR splitter, reorder buffer, TX engine, RX engine, FEC enc/dec, Flit framer. | Done (bug list: CONFIRM Swaray) |
| R1-PHY-BUILD | FEATURE | Kshitij | PHY | Gray enc/dec, PAM4 map/demap, byte↔symbol, 4-lane striper/destriper, elastic deskew, SOF strip, TS1/TS2 detect + lane reversal latch, PCIe reset controller (PERST#, hot reset, FLR), config space + MSI-X (2048 vectors, mask, PBA), `pcie_phy_link_datapath` with synthesizable auto-SOF FSM. | Done |
| R1-INT-1 | INFRA | All | `Final/system_top.v` | First 3-layer chain with PHY loopback; auto-SOF FSM; `tb_system_top_smoke` (connectivity, not data correctness). | Superseded by `bridge_top` (R2) |

### 2.2 R1 bugs and test issues

| Ref | Type | Owner | Area | Summary | Resolution | Status |
|---|---|---|---|---|---|---|
| R1-IC-1 | BUG (FUNC) | Aditya | `interconnect_top` FIFO write | A beat held under back-pressure was written again every cycle (FIFO write gated on raw `grant`). | Ingest port exports `accepted`; `xbar_valid = (grant0 && ch0_accepted) \| (grant1 && ch1_accepted)`. | Fixed |
| R1-IC-2 | BUG (BLOCKER) | Aditya | `rr_arbiter_2ch` | Joint `round_done` deadlocked when only one channel requested. | Independent `round_done0/1`. | Fixed |
| R1-IC-3 | BUG (FUNC) | Aditya | `cdc_fifo_async` | Registered read vs AXI4-Stream FWFT: stale data on first reads. | `assign rd_data = mem[rd_ptr_bin]`. | Fixed |
| R1-IC-4 | BUG (BLOCKER) | Aditya | `cdc_fifo_async` | `wr_full` combinational loop hung the simulator. | Unconditional look-ahead pointer. | Fixed |
| AD-IC-OVF-NOEND | BUG (FUNC) | Aditya (design) | `axi4s_ingest_port` | Overflow abort sent nothing downstream, so the link layer never saw the session end. | Zero-length TLAST marker, in F1-FIX (R2). | Fixed R2 |
| AD-TB-RACE-1 | TB-BUG (MINOR) | Aditya | `tb_cdc_fifo_async` | Trailing `wr_en=0` with no settle delay dropped the last write. | `#1`; bench FIFO depth 8 → 256. | Fixed |
| AD-TB-SAMPLE-1 | TB-BUG (MINOR) | Aditya | `tb_interconnect_top` | "1 beat per channel missing": driver sampled `s_tready` post-NBA. RTL correct, proven by parsing the VCD pointer transitions. | Zero-delay acceptance check. 128/128. | Fixed |
| AD-TB-SAMPLE-1b | TB-BUG | Aditya | same | A wrong intermediate "fix" (settle delays on debug counters) misreported FIFO loss. | Replaced by VCD-based checking. | Closed |
| AD-VERIF-SAMPLE-RULE | VERIF | Aditya | all benches | Rule: observe settled state after a delay, check handshakes with zero delay at the edge. | Documented. | Done |
| AD-VERIF-* | VERIF | Aditya | interconnect benches | Unit benches (ingest, arbiter, FIFO), parameterised integration bench, pointer-gated logs + VCD, CDC lag formula `lag ≈ N·max(0,Tr−Tw) + k·Tr`. | — | Done |
| AD-INFRA-CONS | INFRA | Aditya | `.xdc`, `.sdc`, `openlane_config.json` | Async clock groups, multicycle on gray synchronisers, placeholder periods. OpenLane not run in R1. | — | Done, not run |
| R1-LL-* | BUG | Swaray | link layer | Commits 6–13 Sep mention "bugs fixed in modules", "edge cases sorted". | — | **CONFIRM (Swaray)** |
| R1-PHY-SOF | SCOPE | Kshitij | `flit_sof_strip` | Known limitation: 0xFB in genuine payload. Stated, not solved in R1. | — | Open in R1 |
| R1-PHY-* | BUG | Kshitij | PHY | Review later found the `sext_rst_n` bench typo (REV-TB-1 / T1-K5). Further Stage 1 fixes in CHANGELOG Updates 1–8. | — | See CHANGELOG |

### 2.3 R1 scope / ticket exchanges between layers

| Ref | Type | Between | Summary | Status |
|---|---|---|---|---|
| AD-ST-A / AD-ST-B | SCOPE | Swaray → Aditya | TUSER latched per burst; interleaving prohibition. Closed: Swaray keeps one Flit buffer per stream. | Closed |
| AD-ST-C | SCOPE | Aditya → Swaray | `m_tuser` width; SLVERR/B-channel flag not applicable to AXI4-Stream. | Minor, historical |
| AD-KT-1 | SCOPE | Kshitij → Aditya | `session_valid` pulse vs level. Became REV-I3 (pulse_sync). | Closed R2 |
| AD-KT-2 | SCOPE | Kshitij → Aditya | Compile-time `SLICE_SIZE`: held (breaking port change). Ended up as runtime limit registers (S4-7). | Closed R2 |
| AD-KT-3-6 | SCOPE | Kshitij → Aditya | Overflow policy, TUSER fields, Flit alignment, shared back-pressure: answered, matched RTL. | Replied |

### 2.4 R1 numbers

| Number | Source |
|---|---|
| Interconnect integration: 128 beats, `wr_cnt = rd_cnt = 128`, `accepted0 = accepted1 = 64`, 0 errors | Stage 1 report §5.1 |
| Interconnect unit benches 3/3 + integration PASS (Icarus) | Stage 1 report §5.1 |
| `tb_bridge_smoke`, `tb_system_top_smoke`: PASS (connectivity) | Stage 1 report §5.2 |
| PHY `tb_end_to_end_datapath` bit-exact; cocotb 4/4; formal `gray.sby`, `pam4.sby`, `verify_reset.sby` | Stage 1 report §5.3 |
| Link layer: 9 unit benches individually passing, not chained into `system_top` | Stage 1 report §5.4 |

---

## 3. Round 2: pre-Phase-2 review and ticket rounds (16–20 Oct)

### 3.1 Layer review, 19 Oct

| Ref | Type | Owner | Area | Summary | Resolution / patch | Led to | Status |
|---|---|---|---|---|---|---|---|
| REV-Seam-A / REV-I1 | BUG (CORRUPTS) | Aditya + Swaray | interconnect → link TUSER | TUSER bit order differed: every beat routed as ch0, address at half offset. | Shim `ic2link_tuser_remap.v` → `interconnect_top` emits link-native `{tlast_flag, session_offset, stream_id}` → shared header (G7). | T2-R2, G7 | Fixed |
| REV-I2 | BUG (BLOCKER) | Kshitij | `pcie_tag_man.v` | Second `axi4s_ingest_port` definition. | Kshitij's copy renamed `legacy_axi4s_ingest_port` and moved to `protocol_PHY_layer/legacy/` (not on any file list). | — | Fixed |
| REV-I3 | BUG (FUNC) | Kshitij / Aditya | session start CDC | `session_start` must be an aclk pulse. | `pulse_sync u_start_cdc0/1` (clk → aclk), `u_end_cdc0/1` (debug). | — | Fixed |
| REV-I4 | PERF | Swaray | link drain | 1 beat / 20 clk → interconnect stalled 95%. | 512-bit TX engine | S4-1 | Fixed (Step 4) |
| REV-I5 | SCOPE | All | RX direction | No receive path. | Inbound MWr → BAR0 | G1 | Partly (MWr-only by design) |
| REV-L1 | BUG (BLOCKER) | Swaray | serialiser | Emitted 15 DWs; packetizer waited for 16. | Load `popcount(TKEEP)/4`; REV-L2/L10/Seam-C folded into the same edit | — | Fixed 20 Oct |
| REV-L2 | BUG (BLOCKER) | Swaray | descriptor gen | FIFO head consumed without pop. | Gate on serialiser free | — | Fixed |
| REV-L3 | BUG (BLOCKER) | Swaray | replay buffer | No ACK source. | ACK/NAK in DLP + control Flits | — | Fixed |
| REV-L4 | BUG (CORRUPTS) | Swaray | replay buffer | Sequence number overwrote data bits. | Seq in DLP field | — | Fixed |
| REV-L5 | BUG (CORRUPTS) | Swaray | replay buffer | tuser/tlast bypassed the buffer. | L6 redesign | — | Fixed |
| REV-L6 | BUG (CORRUPTS) | Swaray | Flit packing | 40 bits of every 4th beat dropped. | 236 B TLP area, `flit_payload_builder` | — | Fixed |
| REV-L7 | BUG (CORRUPTS) | Swaray | partial Flit seal | Last beat always in slot 3. | Contiguous DW packing | — | Fixed |
| REV-L8 | FEATURE | Swaray | FEC | 8 parity bits, no correction. | 3-way interleaved SEC-DED + CRC-64 | S2-OPT-4, S4-T1, X.3/X.4 | Fixed |
| REV-L9 | PERF | Swaray | 32-bit packetizer | ~20 clk per beat; one TLP per Flit. | 512-bit engine + merging | S4-1, S4-2 | Fixed (Step 4) |
| REV-L10 | FEATURE | Swaray | MSI-X | No MSI-X injection port. | `msix_req_*`, `session_done` | Seam D | Fixed |
| REV-Seam-B | FEATURE | Kshitij | BAR0 | No base / weight / limit registers. | `bar0_dma_regs` | REV-P4, G1 | Fixed |
| REV-Seam-C / REV-P3 | BUG (BLOCKER) | Swaray + Kshitij | link → PHY | No ready; PHY 32× slower. | PHY Flit FIFO + `phy_tx_ready` | S4-3 | Fixed |
| REV-Seam-D | FEATURE | Kshitij + Swaray | MSI-X | Not wired to link. | `mwr_*` → `msix_req_*` | — | Fixed |
| REV-P1 | FEATURE | Kshitij | PHY | No 4-lane symbol output. | `lane_tx` / `lane_rx` | — | Fixed |
| REV-P2 | FEATURE | Kshitij | PHY | No composed `pcie_phy_top`. | `pcie_phy_top` with SOF insert / strip | — | Fixed |
| REV-P4 | FEATURE | Kshitij | config | No BAR0 register file. | `bar0_dma_regs` | G1 | Fixed |
| REV-P5 | INFRA | Kshitij | PHY | Modules duplicated across 7 files. | `pre_restructure/`, `cocotb_old/` removed (K2/K3/K5); one module per file | — | Fixed |
| REV-TB-1 | TB-BUG | Kshitij | `tb_full_system_integration` | `sext_rst_n` typo. | Fixed copies in `tb/` | T1-K5 | Fixed |

### 3.2 Ticket round 1 (20 Oct): benches broken by renames

| Ref | Type | Owner (layer) | Summary | Resolution | Status |
|---|---|---|---|---|---|
| T1-S1 | TB-BUG | Swaray | Packetizer benches for an old interface | Rewritten; drive at negedge | Done |
| T1-S2 | TB-BUG | Swaray | Replay-buffer bench | Rewritten | Done |
| T1-S3 | TB-BUG | Swaray | RX-engine bench stimulus deleted | Rewritten from scratch: 4 cases | Done |
| T1-S4 | INFRA | Swaray | No link `run_all` | `run_all_link.py` + `files.f` | Done |
| T1-S5 | TB-BUG | Swaray | Link loopback bench | Rewritten at Flit level | Done |
| T1-S6 | TB-BUG + FEATURE | Swaray | Credit manager not on TX path | Wired in; one-shot bootstrap (Posted limits `0x7F`/`0xFFF`); proven live by AD-CGP → real FC in G2 | Done |
| T1-S7 | TB-BUG | Swaray | Reorder-buffer bench name mismatch | Fixed | Done |
| T1-S8 | TB-BUG | Swaray | FEC bench for legacy framer | `tb_step9_flit_fec.v` deleted | Done |
| T1-S9 | TB-BUG | Swaray | Tag manager / TX engine print-only | Checks added (TX engine completed in G5) | Done |
| T1-S10 | TB-BUG | Swaray | Stale pipeline bench | `tb_top_pipeline.v` deleted | Done |
| T1-S11 | INFRA | Swaray | Old Vivado project | `git rm -r`, regenerate from tcl | Done |
| T1-K1 | BUG | Kshitij | `session_end_aclk` dead | Kept as debug only | Done |
| T1-K2..K5 | INFRA | Kshitij | Legacy files, cocotb runner, rename, stale bench copies | Cleaned | Done |
| SW-H2 | BUG (CORRUPTS) | Swaray | Parser consumed 2 header words for 4-DW headers | Fixed | Fixed |
| SW-H3 | BUG (CORRUPTS) | Swaray | Header DWs passed through as payload | Stripped | Fixed |
| SW-P5 | BUG (CORRUPTS) | Swaray | Packetizer DW0 built from 31 bits | Exact 32-bit DW0 | Fixed |
| SW-S1 | BUG (FUNC) | Swaray | Splitter `s_ready` registered | Combinational | Fixed |
| SW-W2 | BUG (CORRUPTS) | Swaray | Write buffer re-read a stale entry | Gate on `wr_ptr != rd_ptr` | Fixed |
| SW-GAP-1..3 | INFRA | Swaray | Missing files in `files.f`; RESULT-line mismatch (benches never matched the regex, always FAIL); credit manager not wired | Fixed | Fixed |
| SW-LPB | FEATURE | Swaray | `link_phy_bridge_top` missing | Written + bench; `cfg_base_addr` arbiter for two per-channel pulses | Done |
| AD-LPB-TUSER | BUG (CORRUPTS) | Swaray's layer | First `link_phy_bridge_top` packed `{s_axis_tlast, ic_tuser}` — the order later found corrupting | Retired in T2-R2 | Superseded |
| AD-CGP | VERIF | Swaray's layer | `tb_credit_gate_proof`: zero credits stall 197 cycles, 0 TLPs escape, resume on release | — | Done |

### 3.3 Ticket round 2 (20 Oct)

| Ref | Type | Owner | Summary | Resolution | Led to | Status |
|---|---|---|---|---|---|---|
| T2-R1 | BUG (BLOCKER) | Swaray | `link_layer_top.v` held a copy of `link_phy_bridge_top` | Restored | — | Fixed |
| T2-R2 | BUG (CORRUPTS) | Swaray / Aditya | TUSER order again | Link-native order | G7 | Fixed |
| T2-R3 | BUG (CORRUPTS) | Swaray | Parser address one DW early; RX engine sampled at SOF | `REQ_ON_HDR_DONE` | — | Fixed |
| T2-R4 | BUG (FUNC) | Swaray | `m_rx_axis_*` tied 0 | `mon_rx_axis_*` tap | R4 guard | Fixed |
| T2-R5 | BUG (BLOCKER) | Swaray | FEC `while` loops (Yosys) | `while` → `if` | — | Fixed |
| T2-R6 | INFRA | Swaray | `files.f`, `run_all_link.py` missing | Restored | — | Fixed |
| T2-R7 | BUG (FUNC) | Kshitij / Swaray | 12-bit BAR0 truncated MSI-X table | `bar0_addr[15:0]` | — | Fixed |
| T2-R8 | INFRA | All | File lists, `asic/` | Updated | — | Fixed |
| T2-P2-1 | FEATURE | Swaray + Kshitij | Inbound MWr not routed to BAR0 | — | G1 | Fixed |
| T2-P2-2 | FEATURE | Swaray | Fake "infinite" credit sentinel; no UpdateFC | — | G2 | Fixed |
| T2-P2-3 | BUG (FUNC) | Swaray | Translation miss not latched | XLAT_MISS counter | G3, S13 | Fixed |
| T2-P2-4 | PERF | Swaray | RX unpacker 1 DW / clk | — | G8 → S4-8 | Fixed (Step 4) |
| T2-P2-5 | BUG (FUNC) | Swaray | `is_mwr` matched MRd | `parsed_has_data` | G4, F6-FIX | Fixed |
| T2-P2-6 | TB-BUG | Swaray | `tb_tx_engine` print-only (link suite 15/16) | Order monitor + checks + RESULT | G5 | Fixed |
| T2-P2-7 | BUG (MINOR) | Swaray | Shared module-level loop integer (reorder buffer) | Named-block local integers | G6; same pattern in X.3 | Fixed |
| T2-P2-8 | INFRA | All | Glue edits regressing earlier fixes | Regression before merge → CI + guard | S3-A | Done |
| AD-OBS-TXENG | TB-BUG | Swaray | `[FAIL] tx_engine — no RESULT line` seen in the G7 regression | = T2-P2-6 | G5 | Fixed |

---

## 4. Round 2, Phase 2

### 4.1 Step 1: integration gaps (20 Oct)

G1–G6 were made as separate patches, replayed as clean commits, and landed in `5d47645`.

| Ref | Type | Owner (layer) | Summary | Patch / commit | Status |
|---|---|---|---|---|---|
| G1 | FEATURE | Swaray + Kshitij | Inbound host MWr → BAR0 (`inb_mwr_*`, `cfg_bar0_addr`); `tb_g1_host_bar0` | `5d47645` | Done |
| G2 | FEATURE | Swaray | Real flow control: InitFC1/InitFC2/UpdateFC, FC init FSM | `5d47645` | Done |
| G3 | FEATURE + BUG | Swaray + Kshitij | XLAT_MISS counter at BAR0 0x1C with clear; a request arriving while `axi_addr_ready` was low was dropped → now queued 1-deep and drained without a bubble | `5d47645` | Done |
| G3a | BUG (FUNC) | Swaray's layer | `proc_valid = stalled ? pend_valid : req_valid` dropped the queued request on the cycle the stall cleared → `pend_valid \|\| req_valid` | G3 patch | Fixed |
| G4 | BUG (FUNC) | Swaray | MWr vs MRd decode (`parsed_has_data`) = T2-P2-5 | `5d47645` | Done |
| G5 | TB-BUG / VERIF | Swaray's layer | = T2-P2-6: `tb_tx_engine` order monitor (`{posted, tag}` at `STATE_HDR0`), checker task, RESULT line; mutation-tested (broken Posted priority makes it fail) | `5d47645` | Done |
| G5a | TB-BUG (FUNC) | Swaray's layer | Bench's "zero NP credits" was the pool's first update (InitFC, 0 = infinite) under G2 semantics → real InitFC(1) then UpdateFC(0) | G5 patch | Fixed |
| G5b | TB-BUG (MINOR) | Swaray's layer | `$display` label `[255:0]` truncated text over 32 chars → `[511:0]` | G5 patch | Fixed |
| G5c | TB-BUG (FUNC) | Swaray's layer | Same counter used to record and to check → separate `g5_check_idx` | G5 patch | Fixed |
| G6 | BUG (MINOR) | Swaray's layer | = T2-P2-7: one `integer i` shared by 3 loops in `axi_pcie_reorder_buffer` → 2 Yosys conflicting drivers; each loop scoped to a named block | `5d47645` | Done |
| G6a | BUG (synthesis, MINOR) | Swaray's layer | First attempt used SV inline `for (integer i…)`: Icarus accepts, Yosys plain-Verilog rejects → named blocks | G6 patch | Fixed |
| G7 | INFRA | Aditya | Shared TUSER header `shared/axis_tuser_defs.vh` (`tuser_pack()` etc., `$fatal` on width mismatch). Latent hand-duplication in 8+ files. | `2380044` | Done |
| AD-G7-TB / -INC / -PATCHPATH | VERIF / INFRA | Aditya | Benches use `tuser_pack()`; `+incdir+shared` in iverilog flows; patch regenerated with `git diff` paths after `git apply` failed | G7 patch | Done |
| G8 | PERF | Swaray | Widen RX decode 4–16 DW/clk: investigated, no RTL in Step 1 | superseded by S4-8 | Closed by S4-8 |

Step 1 numbers: link `run_all_link.py` 16/16; `tb_bridge_top` 144/144 TLPs, 6/6 MSI-X; `tb_bridge_ovf` 39/39, 4/4 MSI-X; Yosys elaboration of `bridge_top` (56 files): 0 latches, 0 conflicting drivers, 0 implicit nets. Interconnect bench still 128/128 after G7.

### 4.2 Step 2: baseline, first fixes, ranks 4/5 (21 Oct)

| Ref | Type | Owner | Summary | Patch / commit | Led to | Status |
|---|---|---|---|---|---|---|
| S2-BASE | INFRA | Aditya | Vivado baseline scripts, `perf_monitor`, Step 2 baseline | `c8e017a` | — | Done |
| S2-INC | INFRA | Aditya | `shared/` include path in Vivado / Quartus / LibreLane after G7 (G7 only covered iverilog) | Step 2 baseline | — | Fixed |
| S2-OPT-4 | OPT | Swaray's layer | TX CRC/FEC one 256-bit chunk per clock: `ST_CALC` 1 → 9 cycles, back-to-back Flits 10 → 18 clk. Baseline WNS −21.45 ns (~39 MHz), all worst paths CRC/FEC in `link_layer_tx`. Bit-exact vs reference on 8 vectors. | `b518a89` | S4-T3 | Done |
| S2-OPT-5 | OPT | Swaray + Kshitij + Aditya | Memory arrays in reset-free blocks: `replay_buffer` (60,416 bits), `write_data_buffer`, `rxf_mem`, reorder `buffer_data`; `cdc_fifo_async` (`ram_style="distributed"`; FWFT read keeps it LUT RAM). Baseline 0 BRAM, ~half of 116,932 FFs in buffers. | `b518a89`, `54425a9` | — | Done |
| S2-RANK5-IC | OPT | Aditya | Rank 5 interconnect changes | `1d9a49c` | — | Done |
| S2-FORMAL | VERIF | Kshitij | SymbiYosys proof of `tlp_header_parser` found **F5** (MRd decoded as MWr = T2-P2-5) and **F6** (CplLk/CplDLk) | `03bb926` | F6-FIX | Done |
| F6-FIX | BUG (FUNC) | Swaray (found by S2-FORMAL) | Locked completions not counted as completions → `malformed_tlp_err`; `is_cpld = tlp_type[4:1]==4'b0101` | `887e74f` | KS-REV-T7 → S12 | Fixed |
| F1-FIX | BUG (FUNC) | Aditya's layer | (1) `would_overflow` compared in 24 bits and wrapped near 2^24 → one bit wider; (2) on overflow abort, send one zero-length TLAST marker (`tkeep=0`) so the link closes the session (closes AD-IC-OVF-NOEND) | ingest offset-wrap patch | F1 guard rule; AD-F1-CHECKS | Fixed |
| AD-F1-CHECKS | VERIF | Aditya | Zero-length marker accepted only with `tlast`, length-0 descriptor handled, correct offset/stream id | — | — | Covered by `tb_bridge_ovf`, `tb_ingest_overflow_leak`, F5 (bin `ovf` 53 hits / 40 seeds) |
| S2-REG-1 | BUG (regression) | Kshitij | "Fixed bug (4.5)" edited an old `pcie_phy_top.v`, dropping G1/G3 ports | `rank45_repair` | guard (S3-A) | Fixed |
| S2-REG-2 | BUG (regression) | Swaray's layer | F6 entry re-derived on the 19 Oct baseline removed `parsed_hdr_done` / `REQ_ON_HDR_DONE` and G1/G3 | `rank45_repair` | guard (S3-A) | Fixed |
| S2-YOSYS-CRC | BUG (synthesis) | Swaray | CRC loops not Yosys-safe | `rank45_repair` | — | Fixed |
| AD-OPT-ITEM6 | SCOPE | Aditya | Item 6 (sticky error counters in BAR0) deferred in Step 2 | — | S4-6 | Closed by S4-6 |
| AD-OPT-ITEM8 | SCOPE | Aditya | Item 8 (RX decode widen) deferred | — | S4-8 | Closed by S4-8 |

### 4.3 Step 3: CI, randomized bench, coverage (21–22 Oct)

| Ref | Type | Owner | Summary | Patch | Found / led to | Status |
|---|---|---|---|---|---|---|
| S3-A | VERIF / INFRA | Aditya | `regression.yml` (guards, sim matrix, long, rand sweep, coverage, formal), `main-guard.yml` auto-revert, pre-push hook, `ci/check_protected.py` (33 → 39 with K6 → 46 rules) | merge-gate patch | Caught all 9 fixes lost in S2-REG | Done |
| F1–F4 | VERIF | Aditya | `tb_bridge_rand`: random sessions / beats / last-beat sizes / gaps, weights, unaligned bases, PAM4 noise | rand F1–F4 patch | — | Done |
| F5–F7 | VERIF | Aditya | Overflow at random point, lane reversal, hot reset mid-session | rand F5–F7 patch | — | Done |
| F8–F9 | VERIF | Aditya | MSI-X masking, inbound BAR0 writes from host | rand F8–F9 patch | S13 | Done |
| K6 | BUG (FUNC) | Integration review ticket → Kshitij | Once in L0 only reset left it: signal loss left `link_trained`/`axi_rst_n`/sessions high and data stopped silently. Recovery-lite: KS-K6-IDLE, -RX, -EXIT, -RST. | K6 patch (12 files, +599/−38) | F7b, KS-K6-GUARD, S4-6, KS-PERFMON | Fixed |
| KS-K6-IDLE | BUG (FUNC) | Kshitij | TX lanes went quiet with no Flit data, so "count `rx_level_valid` low" would drop every idle link. TX sends logical idle `0x1B` in every free L0 slot; mid-Flit underrun filler after 8 slots | K6 patch | KS-PERFMON | Fixed |
| KS-K6-RST | BUG (FUNC) | Kshitij | L0 → DETECT could not retrain (deskew lock, lane-reversal latch, link-layer state only cleared on reset). `datapath_rst_n` 3-clk pulse on link-down | K6 patch | F7b | Fixed |
| KS-K6-RX / KS-K6-EXIT | FEATURE | Kshitij | `hunting`, `gap_ok/gap_bad`; L0 exits: (1) signal loss 64 clk, (2) 512 unframed between-Flit cycles, (3) 2 TS1s in L0; exits 1/2 armed only after a clean between-Flit cycle | K6 patch | — | Done |
| KS-K6-TB1..3 | VERIF | Kshitij | `tb_pcie_phy_top` Part K (K0–K5, 85 → 118 checks); reset controller test 5 (15 → 17); cocotb `test_link_loss_retrain` (9 → 10) | K6 patch | — | Done |
| KS-K6-GUARD | INFRA | Kshitij → Aditya | K6 guard rules merged (7 K6 lines in `protected_lines.txt`) | — | — | Applied |
| F7b | VERIF | Aditya | Cable pull / glitch mid-session: link drop and recovery (tests K6) | F7b + coverage patch | — | Done |
| S3-COV | VERIF | Aditya | `COV:` bins + `coverage_report.py` | F7b + coverage patch | — | Done (73/73 required on 40 seeds) |
| S7 | BUG (CORRUPTS) | Swaray (found by noise-free random replays) | `link_layer_tx` consumed a payload Flit while sending an FC DLLP Flit → payload lost, replay | S7 TX DLLP slot fix | — | Fixed |
| AD-S7-VERIFY | VERIF | Aditya | Independent re-run: `REPLAY_COUNT` 1 → 0; `tb_bridge_top` 181.38 → 173.06 µs; `tb_bridge_ovf` 74.26 → 65.94 µs; link 17/17 | — | — | Done |
| S13 | BUG (FUNC) | Swaray (found by F9) | Host BAR0 write counted as a translation miss (`excl_en/base/mask`) | S13 patch | — | Fixed |
| S12 | VERIF | Kshitij → Swaray | RX-engine tests 7a CplDLk / 7b CplLk; fails if F6 reverted | S12 patch | from F6-FIX via KS-REV-T7 | Done |
| KS-REV-T7 | TB-BUG (MINOR) | Kshitij | A stale copy of `tb_pcie_rx_engine.v` Test 7 mislabelled CplLk/CplDLk and passed for the wrong reason | — | → S12 | Closed by S12 |
| S3-TS | INFRA | Kshitij | Missing `timescale` in PHY files | timescale patch | — | Fixed |
| S3-TB-1 | TB-BUG | Aditya | XLAT_MISS expectation, F8 timing, F9 readback race, drain cap | in F-patches | — | Fixed |
| KS-GUARD-CMT | TB-BUG (MINOR) | Kshitij | `check_protected.py` strips `//` from source but not from rule text: a rule containing a comment never matches | — | — | **Open** |
| KS-CHK-P3 | TB-BUG (MINOR) | Kshitij | `SVA-EQUIV P3 VIOLATION` printed twice in `tb_pcie_reset_controller`; `run_all.py` doesn't count `VIOLATION` | — | — | **Open** |
| KS-NOISE-1 | TB-BUG / BUG | Kshitij → Swaray / Aditya | Far above CI noise (2000–3000 ppm): F8 MSI-X late or F9 inbound MWr not delivered within its wait. Also fails on the step3 RTL (seed 16). Data never corrupted (CRC-64 on every Flit). | — | Step 4 rule 6 (wait on events) | **Open** |
| KS-PERFMON | INFRA | Kshitij → Aditya | After KS-K6-IDLE, `perf_monitor` stage 4 counts idle symbols | — | — | Reported |

### 4.4 Step 4: optimisation (22–23 Oct), freeze `step4-final`

Vivado xcu250, target 250 MHz. Base 118 MHz → Wave A+B 49 MHz → Wave C 147 MHz → freeze 248 MHz → with X.1, X.3, X.4 **255 MHz** (commit `e29db76`, now branch `RTL-FPGA`; reports in `results/fpga/e29db76/`, commit `3d0446f`).

| Ref | Wave | Type | Owner (layer) | Summary | Effect |
|---|---|---|---|---|---|
| S4-HIST | — | INFRA | Aditya | Vivado summary: commit stamp + failing-endpoint histogram | — |
| S4-1 | B | OPT | Swaray's layer | `tlp_tx_wide`: 512-bit TLP engine replaces serialiser + 32-bit packetizer + packer | REV-L9, REV-I4 closed |
| S4-2 | B | OPT | Swaray's layer | Merge beats up to 256 B MPS, no 4 KB crossing; DW-stream reference model | ML 0.06 → 6.17 GB/s |
| S4-T1 | A | OPT | Swaray's layer | RX FEC/CRC decode in 3 stages | −4.44 ns path removed |
| S4-T3 | A | OPT | Swaray's layer | TX framer 3-stage pipeline, 8 clk / Flit (was 18) | — |
| S4-8 / S4-T2 | B | OPT | Swaray's layer | 16-DW/clk RX delimiter + wide header decode | T2-P2-4, G8 closed |
| S4-3 | B | OPT / FEATURE | Kshitij's layer | Wide PHY 8 B / lane / clock (`PHY_WIDE`) | — |
| S4-6 | A | FEATURE | Kshitij's layer | Sticky error counters BAR0 0x40–0x64 (PERST#-only) | closes AD-OPT-ITEM6 |
| S4-7 | A | FEATURE | Aditya's layer | Per-channel session limit (0x34), `session_end`, interleaved WRR (one beat per grant) | closes AD-KT-2 |
| S4-C-1 | C | OPT | Swaray's layer | Merge-unit sizes / addresses from registers, per-DW pick | — |
| S4-C-2 | C | OPT | Swaray's layer | CRC-64 split into per-chunk parts + combine | — |
| S4-C-3 | C | OPT | Swaray's layer | Replay timer 2048 clk with wide PHY | Noise runs 6× better |
| S4-C-4 | C | OPT | Swaray's layer | CRC / FEC syndromes as constant-mask XOR trees (Vivado 51 levels) | 49 → 147 MHz |
| S4-C-5 | C (freeze) | OPT | Swaray's layer | Register inbound beat before header gather (28 levels) | 147 → 248 MHz |
| S4-CI | — | INFRA | Aditya | Load gate: Jain fairness reported, not gated | — |

### 4.5 Step 5: load analyser (23 Oct)

| Ref | Type | Summary |
|---|---|---|
| S5-1 | VERIF | Workload modes in `tb_bridge_rand` (`+WL=1..5`): SmartNIC IMIX, back-to-back 64 B, ML tensors 64 KB–16 MB, 4 KB-line, mixed 1:1 / 1:3 / 1:7, noise 300 / 1000 ppm |
| S5-2 | INFRA | `ci/load/run_workloads.py`, `compare.py`, `thresholds.json`, `make_report.py`, `.github/workflows/load.yml`, committed base results |
| S5-3 | PERF | Final code (24 Oct rerun = Wave C): ML 0.06 → 6.17 GB/s; NIC IMIX 0.15 → 4.84 Mpps, p99 58 → 0.59 µs; back-to-back 64 B 0.48 → 11.6 Mpps; mixed 1:7 NIC p99 241 → 4.2 µs; noise 300 ppm ≈ 0.1 GB/s (open weakness) |

### 4.6 GDSII phase (23–24 Oct): X patches

| Ref | Type | Summary | Found by | Status |
|---|---|---|---|---|
| X.1 | OPT (timing) | Ingest TREADY independent of TKEEP (exact limit check only near the limit); interconnect GDS config + SDC | SKY130 slow corner: ~27 ns TKEEP → TREADY path | Applied (`RTL-FPGA`, `GDSII`) |
| X.2 | INFRA | Full-bridge GDS build parameters on `bridge_top`; narrow PHY + small buffers, 30 ns; `asic_params_gds*` sim | GDS size / time budget | Applied (`GDSII` only) |
| X.3 | BUG (synthesis) | `link_layer_rx` FEC correction: latch / logic loop + shared loop integer (230 Yosys check errors) — same pattern as T2-P2-7 / G6a | LibreLane synth checks | Applied (`RTL-FPGA`, `GDSII`) |
| X.4 | BUG (synthesis) | Same block as constant-position gates (192 remaining loops in newer Yosys) + `gds/rxcheck` | LibreLane synth checks | Applied, verified ("Yosys check errors clear") |
| GDS-CFG-1 | INFRA | `RUN_LINTER: false` (Verilator TIMESCALEMOD on cell models) | LibreLane lint | Done |
| GDS-CFG-2 | INFRA | Explicit file list (testbenches in `src/` broke Yosys) | LibreLane | Done |
| GDS-CFG-3 | INFRA | Two-clock async SDC, exact port names | STA | Done |
| GDS-ENV-1 | INFRA | Docker mounts only home + cwd: run from repo root | LibreLane in Docker | Done |
| GDS-IC-1 | RESULT | Interconnect GDS (pre-X.1): DRC / LVS / antenna clean, hold met, 1.20 mm², 49k cells, 104 mW; setup −0.95 ns at slow corner (−0.52 ns after X.1, run stopped before stream-out). GDS + `metrics.json` + KLayout renders committed on `GDSII`. | LibreLane | Done |
| GDS-RX-1 | RESULT | `link_layer_rx` alone: synthesis checks clear after X.4; 1.12 mm² cells | LibreLane | Done |
| GDS-BRIDGE | RESULT | Full bridge, narrow build (X.2, 30 ns, 30 % utilisation): synthesis → global routing complete (~3.9 mm² cells). Detailed routing: first attempt killed out of memory (~14 GB); resumed with fewer threads, violations rose 3,200 → 9,748 instead of converging (congestion). Stopped at the 24 Oct cutoff. Parallel 50 ns and full-design runs abandoned for time and machine availability. | LibreLane | **Not completed** |
| GDS-REPO-1 | INFRA | First push of `gds/*/runs/` rejected by GitHub (files > 100 MB). Runs purged from history (`git filter-branch`), `runs/` ignored, `.gds` tracked with Git LFS. | GitHub push limits | Done |

### 4.7 Housekeeping

| Ref | Type | Owner | Summary | Status |
|---|---|---|---|---|
| AD-CLEAN-1 | INFRA | Aditya | 22 Oct: 78 superseded working-copy docs deleted after checking each fix is in the repo — stops stale copies being edited (the cause of S2-REG-1/2) | Done |
| REPO-BRANCHES | INFRA | All | 24 Oct: `main` made documentation-only; code on `RTL-FPGA` (was `FPGA_optmised_freeze`), `GDSII` (was `phase3/gds`) and `Pre-GDS-RTLCodeBase-(Internal-Ref)` (was `phase2/X`); superseded `phase2/*` branches removed. Tags `step4-base`, `waveAB`, `waveC`, `step4-final` keep every measured point. | Done |
| KS-S4-PLAN | SCOPE | Kshitij | Step 4 #6 then #3 recommended (S4-6, S4-3) | Handed off |

---

## 5. Chains (what led to what)

```
AD-SCOPE-2 ─ REV-Seam-B ─ G1 / G3 ─ S4-7
REV-Seam-A ─ shim ─ T2-R2 ─ G7 (shared header) ─ S2-INC (include paths)
REV-L8 ─ FEC/CRC-64 ─ S2-OPT-4 ─ S4-T1/T3 ─ S4-C-2/C-4 ─ X.3 ─ X.4
REV-L9 + REV-I4 ─ S4-1 ─ S4-2 ─ (Step 5 ML 100×)
T1-S6 ─ AD-CGP ─ T2-P2-2 ─ G2 ─ G5a (bench semantics) ; G2 ─ (rand noise-free replays) ─ S7
T2-P2-1 ─ G1 ─ F9 (bench) ─ S13
T2-P2-3 ─ G3 (+G3a) ─ S13
T2-P2-5 ─ G4 ; S2-FORMAL ─ F5 (=T2-P2-5) + F6-FIX ─ KS-REV-T7 ─ S12
T2-P2-5/F6 entry on old baseline ─ S2-REG-2 ─ rank45_repair ─ S3-A guard
S2-REG-1/2 ─ T2-P2-8 ─ S3-A guard + CI ─ AD-CLEAN-1
K6 (ticket) ─ KS-K6-IDLE / KS-K6-RST ─ F7b (bench) ; K6 ─ S4-6 ; K6 ─ KS-PERFMON
T2-P2-4 ─ G8 ─ AD-OPT-ITEM8 ─ S4-8
AD-OPT-ITEM6 ─ S4-6
AD-IC-OVF-NOEND ─ F1-FIX (zero-length TLAST) ─ AD-F1-CHECKS
AD-KT-2 ─ S4-7 ; AD-IC-FEAT-ARB (burst limit) ─ S4-7 (interleaved WRR)
T2-P2-7 ─ G6 / G6a (named-block loops) ─ same pattern in link_layer_rx ─ X.3 ─ X.4
SKY130 slow corner ─ X.1 ; GDS size ─ X.2 ─ GDS-BRIDGE (routing congestion)
```

---

## 6. Totals and open items

**Round 2 totals:** 44 design bugs and 21 testbench bugs found and fixed; 4 GDS patches (X.1–X.4).

Open at the 24 Oct cutoff:

1. **GDS-BRIDGE:** full-bridge detailed routing did not converge; rerun with `FP_CORE_UTIL` ≈ 20 %, `PL_TARGET_DENSITY_PCT` ≈ 30 % and router threads capped to available memory.
2. **KS-NOISE-1:** F8/F9 timeouts far above CI noise (2000–3000 ppm); fails on the step3 baseline too. No data corruption.
3. **KS-GUARD-CMT:** guard rule text containing `//` never matches.
4. **KS-CHK-P3:** `VIOLATION` lines not counted by `run_all.py`.
5. **R1-LL-*:** itemised Round 1 link-layer fixes (CONFIRM Swaray).
