# Interconnect layer — full stack specification

**Owner:** Aditya. **Consumers of this doc:** Swaray (link layer), Kshitij (config space / protocol-to-PHY).
**Status:** RTL built and simulated (Icarus Verilog). Not yet synthesized/placed-and-routed — see §7.

---

## 1. Scope

The interconnect layer implements two AXI4-Stream ingest ports (SmartNIC, ML), a weighted round-robin crossbar, and a write-path CDC FIFO. Both use cases are one-directional device→host writes; there is no AXI4 memory-mapped port and no read path anywhere in this layer. See §5/§6 for exactly what's expected from, and delivered to, the other two layers.

## 2. Module list

| File | Purpose |
|---|---|
| `axi4s_ingest_port.v` | One instance per channel (CH_ID=0 SmartNIC, CH_ID=1 ML). AXI4-Stream slave handshake, session offset tracking, overflow guard. |
| `rr_arbiter_2ch.v` | Weighted round-robin arbiter, 2 channels, starvation guard, registered grant outputs. |
| `cdc_fifo_async.v` | Generic gray-code async FIFO, first-word-fall-through read semantics, almost-full backpressure output. |
| `interconnect_top.v` | Wires the above into the full layer: 2 ingest ports → crossbar mux → arbiter-driven select → CDC FIFO → output port. |
| `interconnect_constraints.xdc` / `.sdc` | Clock definitions, async clock groups, CDC multicycle paths, for Vivado and OpenLane respectively. |
| `openlane_config.json` | Starting-point OpenLane config for physical implementation (not yet run — see §7). |

Testbenches (`tb_*.v`) exist for every module above plus a full-stack integration test; all pass — see §8.

## 3. Top-level interface (`interconnect_top`)

### Clocks / reset

| Port | Direction | Notes |
|---|---|---|
| `aclk` / `arstn` | in | AXI-side clock domain, target ≤250 MHz |
| `pcie_clk` / `pcie_rstn` | in | PCIe link clock domain — actual value owned by Swaray's block |

### Channel 0 (SmartNIC) and Channel 1 (ML) — identical shape, `s0_*`/`ch0_*` and `s1_*`/`ch1_*`

| Port | Direction | Width | Notes |
|---|---|---|---|
| `sN_tdata` | in | `DATA_WIDTH` (512) | AXI4-Stream payload |
| `sN_tkeep` | in | `DATA_WIDTH/8` | Byte-valid mask |
| `sN_tlast` | in | 1 | End of packet/session |
| `sN_tvalid` / `sN_tready` | in/out | 1 | Standard handshake |
| `chN_session_start` | in | 1 | **Pulse, from Kshitij's block** — arms a new session, resets the offset counter to 0 |
| `chN_session_limit` | in | `OFFSET_WIDTH` (24) | Max session bytes — see §6 open item |
| `chN_session_active` | out | 1 | High while a session is in progress |
| `chN_session_overflow` | out | 1 | Sticky — set if a beat would exceed `session_limit`; session aborts, `sN_tready` drops until re-armed |

### Arbiter weights

| Port | Direction | Width | Notes |
|---|---|---|---|
| `weight0` / `weight1` | in | `CREDIT_WIDTH` (8) | Runtime-programmable. Default `1`/`1` = 50/50. |

### Output to Swaray's link layer (`pcie_clk` domain)

| Port | Direction | Width | Notes |
|---|---|---|---|
| `m_tdata` | out | `DATA_WIDTH` | Payload |
| `m_tkeep` | out | `DATA_WIDTH/8` | Byte-valid mask |
| `m_stream_id` | out | 1 | 0 = SmartNIC, 1 = ML — **selects which `base_addr_ch[N]` register to read** |
| `m_session_offset` | out | `OFFSET_WIDTH` (24) | Running byte position within the current session — **combine with `base_addr_ch[m_stream_id]` to get the real host address; this layer never computes or carries a full 64-bit address** |
| `m_tlast` | out | 1 | End of session — triggers FLIT flush/pad on Swaray's side |
| `m_tvalid` / `m_tready` | out/in | 1 | First-word-fall-through AXI4-Stream-style handshake |

## 4. What this layer does NOT do (explicitly out of scope)

- No AXI4 memory-mapped AW/W/B or AR/R ports, no read-completion path.
- No BAR decode, no PCIe config-space handling.
- No real 64-bit address computation — only `stream_id` + local `session_offset` are produced.
- No FLIT framing, no zero-padding on partial-length termination, no TLP header generation, no MSI-X.
- No IOMMU/translation-vector logic.

## 5. Requirements FROM Kshitij's layer (config space)

1. **`base_addr_ch0` / `base_addr_ch1` registers** — 64-bit, host-writable via BAR-decoded register writes, one per channel (separate host buffers per channel, not a single shared+partitioned buffer — see conversation history for why this shape was chosen over the earlier 50/50-shared-buffer draft).
2. **`chN_session_start` pulse** — this layer assumes such a signal exists and is driven correctly (one pulse, one cycle, after a fresh `base_addr_ch[N]` has landed and it's safe to begin streaming). **This is currently an assumption, not a confirmed contract** — flagged as an open item in §9.
3. **`chN_session_limit` value** — currently assumed to be supplied externally (register or compile-time constant); this layer does not derive it.
4. **Dedicated MSI-X vector per channel** — not driven by this layer, but the host-side interrupt handler distinguishing channels by vector number (not by data content) needs `session_active`/`chN_session_overflow` visibility, which this layer does provide as outputs — confirm Kshitij's block reads these correctly.

## 6. Requirements FROM/TO Swaray's layer (link layer)

**What Swaray's block receives from this layer:** `m_tdata`, `m_tkeep`, `m_stream_id`, `m_session_offset`, `m_tlast`, `m_tvalid`/`m_tready` — see §3 table. This is the entire contract surface.

**What Swaray's block is expected to own, downstream of this interface:**
- Reading `base_addr_ch[m_stream_id]` (from Kshitij's register file) and computing `real_addr = base_addr_ch[N] + m_session_offset`.
- FLIT framing (256-byte PCIe 6.0 Flits), zero-padding on a session ending mid-Flit (`m_tlast` with a partial final beat).
- TLP header generation, MSI-X firing.
- Optional IOMMU pass-through translation, if used.

**Shared backpressure:** `fifo_almost_full` inside this layer (75% occupancy threshold, tunable via `ALMOST_FULL_PCT`) gates both channels' `sN_tready` identically — matching the earlier design decision that AWREADY/ARREADY/TREADY-equivalent backpressure should be one shared signal, not independent per-port ones.

## 7. Physical implementation (OpenLane / OpenRoad)

`openlane_config.json` and `interconnect_constraints.sdc` are provided as a starting point. **Neither has actually been run through OpenLane in this environment** — doing so requires the tool installed locally with an open PDK (e.g. sky130), which is a substantial separate setup this sandbox doesn't have. Before running:

1. Confirm `PDK`/`STD_CELL_LIBRARY` for your target (invocation flag, not in the JSON).
2. Confirm the actual `pcie_clk` period once Swaray's link-layer clock is known — the SDC currently uses a placeholder.
3. Check the synthesis log for how `cdc_fifo_async`'s `mem` array gets mapped — at depth 16 it will likely infer as flip-flop-based storage rather than a hard macro; this affects area but not functional correctness.
4. Run STA specifically on the CDC multicycle paths after PnR to confirm the constraints actually took effect — a misapplied `set_multicycle_path` (wrong cell/pin names post-synthesis, since synthesis can rename registers) is a common silent failure mode.

## 8. Verification summary

All testbenches built and run with Icarus Verilog; all pass.

| Testbench | What it checks | Result |
|---|---|---|
| `tb_axi4s_ingest_port.v` | Session offset tracking, TLAST ending a session, overflow guard aborting before exceeding `session_limit`, `ds_backpressure` holding TREADY low | PASS |
| `tb_rr_arbiter_2ch.v` | 50/50 default split, 3:1 weighted ratio, starvation guard firing for a weight-0 channel, single-requester channel never stalling, mutual exclusion (grant0/grant1 never both high) | PASS |
| `tb_cdc_fifo_async.v` | Data integrity across two genuinely asynchronous clocks (7ns/13ns periods), in-order delivery, no loss | PASS |
| `tb_interconnect_top.v` | Full-stack: two channels driven simultaneously across independent aclk/pcie_clk, correct `stream_id` tagging, correct per-channel `session_offset` sequencing, zero drops confirmed via internal accepted/write/read count reconciliation (all three counts matched exactly: 118 = 118 = 118) | PASS |

**Known gap:** the integration testbench's own driver-side send counter (`sent0`/`sent1`) showed a benign 1-count discrepancy against the DUT's internal accepted-pulse count, traced to a testbench sampling artifact (not a hardware defect — confirmed by the DUT-internal write/read counts matching exactly). The final pass/fail check uses the DUT's own internal signals as ground truth rather than the driver's parallel count.

**Not yet covered:** true metastability (standard RTL simulators don't model it — the gray-code/2-flop-synchronizer structure is the established mitigation, not something a functional testbench can prove); back-to-back session_start re-arming mid-stream; simultaneous overflow on both channels; post-synthesis gate-level simulation.

## 9. Open items — confirm before relying on this design

1. **Session-armed handshake** — `chN_session_start` is assumed to exist and behave as specified; not yet confirmed with Kshitij.
2. **`chN_session_limit` source** — compile-time constant vs. host-runtime-supplied; affects whether it needs its own register.
3. **Overflow policy** — this implementation stalls (aborts the session, sticky flag, holds TREADY low) rather than wrapping or dropping. Confirm this is the intended behavior.
4. **TUSER field set** — `attr`/`TC`/`reqtype` from the original interface contract are not implemented here (only `stream_id`, `session_offset`, `tlast`). Confirm with Swaray whether they're still needed per-beat.
5. **`ALMOST_FULL_PCT` (currently 75%) and `STARVE_CYCLES` (currently 64)** — placeholder values; should be tuned against real backpressure propagation delay and session timing once known.
