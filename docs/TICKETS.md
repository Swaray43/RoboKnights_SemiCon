# Tickets — files broken by port/parameter renames

Raised by Aditya after integrating all three layers (`bridge_top`). Ranked by **utility**: how much
fixing the file helps us catch real bugs, pass review, or unblock the next feature. Effort is a rough
estimate.

Every file named here has already been moved to a `legacy/` folder so the active build stays clean.
Fix it, make it end with a `RESULT: PASS` / `RESULT: FAIL` line, then move it back.

**Already done in this zip (no action needed):**
- FEC decoder `while` → `if` in `src-link_layer/hdl/flit_defs.vh`. Yosys/OpenLane could not
  synthesize the `while` loops. Results are identical under bit-error injection.
- `pcie_phy_top.v`: sessions now end on the link layer's `session_done`, not on the interconnect's
  `session_end_aclk`. Fixes the re-arm address race. See K1 for the follow-up.
- Ingest-port overflow fixes (Aditya's layer).

## Ranked list

| Rank | ID | Owner | File | Problem | Utility | Effort |
|---|---|---|---|---|---|---|
| 1 | S1 | Swaray | `tb_tlp_packetizer.v`, `tb_tlp_packetizer_mps.v` | Written for an old parallel interface; won't build | High | 2 h |
| 2 | S2 | Swaray | `tb_replay_buffer.v` | Parameter and outputs renamed; won't build | High | 2 h |
| 3 | S3 | Swaray | `tb_pcie_rx_engine.v` | Stimulus deleted, replaced by a placeholder comment; hangs | High (two-way) | 1–2 h |
| 4 | K1 | Kshitij | `rtl/pcie_phy_top.v`, `CHANGELOG.md`, BAR0 doc | `session_end_aclk` path is now dead logic; docs out of date | Med–High | 1 h |
| 5 | S4 | Swaray | (new) link `run_all` script | No single command runs the link benches, so stale ones go unnoticed | Medium | 1 h |
| 6 | K2 | Kshitij | `legacy/pre_restructure/*` | CHANGELOG says deleted, but the files were still in the repo | Medium | 15 min |
| 7 | S5 | Swaray | `tb_link_layer_loopback.v` | `link_layer_tx/rx` interface changed; won't build | Medium | 1 h or retire |
| 8 | S6 | Swaray | `tb_credit_manager.v` | Credit ports renamed; won't build | Medium (real host) | 1 h |
| 9 | S7 | Swaray | `tb_axi_pcie_reorder_buffer.v` | File/module name mismatch; print-only | Low–Med (two-way) | 45 min |
| 10 | K3 | Kshitij | `legacy/cocotb_old/pcie_link_top.v`, `cocotb.runner.py` | `mwr_ready` left floating; runner points at a deleted top | Low | 10 min |
| 11 | S8 | Swaray | `tb_step9_flit_fec.v` | Ends before 2 of 3 TLPs decode; print-only | Low | 30 min or retire |
| 12 | S9 | Swaray | `tb_tag_manager.v`, `tb_tx_engine.v` | Print-only, no pass/fail | Low | 30 min each |
| 13 | S10 | Swaray | `tb_top_pipeline.v` | Two packetizer generations stale | Low | Retire |
| 14 | S11 | Swaray | `src-link_layer/legacy/vivado_project_old/` | Stale file list; sim set points at a missing file | Low | Delete |
| 15 | K4 | Kshitij | `cocotb/test_pcie_link_top.py` | Name says `pcie_link_top`, now runs against `pcie_phy_top` | Cosmetic | 5 min |
| 16 | K5 | Kshitij | `legacy/pre_restructure/tb_*system_integration.v` | Old copies with the `sext_rst_n` typo; fixed copies live in `tb/` | Cosmetic | Delete |

---

## Swaray

### S1 — Revive the packetizer unit tests (rank 1)
`src-link_layer/hdl/testbench/legacy/tb_tlp_packetizer.v`, `tb_tlp_packetizer_mps.v`

**Problem:** Both use `tlp_packetizer dut (.*)` against an interface that no longer exists:
`req_data[255:0]` in, `tlp_header[127:0]` / `tlp_data[255:0]` out. The current module takes
`wdata_tdata/tvalid/tready` (32-bit DW stream) and emits `tlp_out_valid/data/sof/eof` with
`tlp_out_ready`. Elaboration fails at port 10 (`wdata_tdata`).

**Why it matters:** `tlp_packetizer` is on the bridge path. `tb_stream_path` only reports a whole-TLP
mismatch; a unit test pins down which header field (fmt/type, length, first/last BE, address) is wrong.

**Fix:**
- [ ] Rewrite the stimulus for the DW-stream interface (drive `wdata_*`, collect `tlp_out_*`)
- [ ] Check header DW0–DW3 and payload for 1, 2, 16 and 64 DW writes, and the BE rules for 1-DW writes
- [ ] MPS variant: a request longer than `MAX_PAYLOAD_DW` splits into correct back-to-back TLPs
- [ ] Random `tlp_out_ready` backpressure; end with `RESULT: PASS/FAIL`

### S2 — Revive the replay-buffer unit test (rank 2)
`testbench/legacy/tb_replay_buffer.v`

**Problem:** `.FLIT_WIDTH(...)` → the parameter is now `PAYLOAD_W` (1888). New outputs `m_seq`,
`outstanding` and `replay_event` aren't connected. The buffer now stores whole Flit payloads with
per-Flit sequence numbers, and replays on timeout (`REPLAY_TIMEOUT`).

**Why it matters:** Replay is the riskiest state machine on the link. Today it's only exercised
indirectly by error injection in `tb_stream_path`.

**Fix:**
- [ ] Update parameter and ports
- [ ] Check: an ACK frees exactly the acknowledged Flits (`outstanding` drops)
- [ ] Check: a NAK rewinds to the NAKed sequence and resends in order (`replay_event` pulses)
- [ ] Check: no ACK within `REPLAY_TIMEOUT` → replay
- [ ] Check: buffer full → `s_ready` low, nothing dropped

### S3 — Restore the truncated RX-engine bench (rank 3)
`testbench/legacy/tb_pcie_rx_engine.v`

**Problem:** The stimulus block was replaced by a pasted placeholder comment ("Rest of testbench
remains identical to previous version…"). Only the clock and reset remain, there's no `$finish`, and
the bench hangs.

**Why it matters:** `pcie_rx_engine` is the starting point for the inbound path (host MWr → BAR0,
completions), which two-way needs.

**Fix:**
- [ ] Recover the old stimulus from git: `git log -p -- src-link_layer/hdl/testbench/tb_pcie_rx_engine.v`
- [ ] If it can't be recovered, rewrite: MWr inside/outside the window, CplD for an allocated tag, a malformed TLP
- [ ] Add `$finish` and a `RESULT` line

### S4 — Add a link-layer `run_all` (rank 5)

**Problem:** Six benches broke without anyone noticing, because nothing runs them all.

**Fix:**
- [ ] Copy Kshitij's `protocol_PHY_layer/run_all.py` pattern, using the new `src-link_layer/hdl/files.f`
- [ ] Each bench must print `RESULT: PASS` or `RESULT: FAIL`; treat a missing line as FAIL

### S5 — `tb_link_layer_loopback.v`: rewrite or retire (rank 7)

**Problem:** It passes `.AXIS_DATA_W(...)` and drives AXI beats into `link_layer_tx`. The module now
takes Flit payloads from the replay buffer, and only has `SYS_BUS_W` and `SEQ_W` parameters.

**Fix (pick one):**
- [ ] Retire it: `tb_stream_path` already loops TX→RX with error injection
- [ ] Or rewrite at the Flit level: one Flit `link_layer_tx` → `link_layer_rx`, flip 1 or 2 bits, check `fec_corrected`/`fec_err`/`crc_err`

### S6 — `tb_credit_manager.v` (rank 8)

**Problem:** It connects `alloc_hdr_credits`, `alloc_data_credits`, `avail_hdr_credits` and
`avail_data_credits`, which no longer exist. The module now takes `alloc_req_type` +
`alloc_req_dw_len` and returns `alloc_req_grant` / `stall`.

**Why it matters:** The stream path has no posted-credit flow control. In loopback that's fine,
because the link NAKs when its RX buffer is full, but a real root complex expects UpdateFC credits.

**Fix:**
- [ ] Update the bench to the new ports; check grant/stall per credit type, including infinite credits
- [ ] Separate feature: wire `credit_manager` (Posted type) into `stream_tx_path_top`

### S7 — `tb_axi_pcie_reorder_buffer.v` (rank 9)

**Problem:** The file is `tb_axi_pcie_reorder_buffer.v` but the module is `tb_axi_backpressure`, so
scripted runs can't find the top. It builds and runs, but only prints.

**Fix:**
- [ ] Rename the module to match the file
- [ ] Check that out-of-order completions come out in tag order, and that `tlast` is right

### S8 — `tb_step9_flit_fec.v` (rank 11)

**Problem:** `$finish` fires 5 ns after the last injection, so only 1 of the 3 TLPs is decoded, and
that one reports `Len: 1`. There are no checks. It tests `flit_framer` (legacy MM path).

**Fix:**
- [ ] Retire, or wait for the RX side to drain and check: clean TLP decoded, 1-bit error corrected, 2-bit error flagged

### S9 — `tb_tag_manager.v`, `tb_tx_engine.v` (rank 12)

**Problem:** Both build and run but only print; nothing fails automatically.

**Fix:**
- [ ] Add a `RESULT` line with checks for tag reuse/timeout UR, and for posted-vs-non-posted ordering

### S10 — `tb_top_pipeline.v` (rank 13)

**Problem:** It instantiates `tlp_packetizer` with `MAX_PAYLOAD_SIZE_DW`, `REQ_ID`, `req_is_mwr` and
`req_len`, from two interface generations ago. This was already broken in the first zip.

**Fix:**
- [ ] Retire (delete). `tb_stream_path` covers the pipeline end to end.

### S11 — Old Vivado project (rank 14)

**Problem:** `src-link_layer/legacy/vivado_project_old/Link_layer.xpr` lists the pre-rewrite file
set (`DLLP_parser.v`, old `replay_buffer.v` …). Only `tx_engine` was ever synthesized, and the sim set
points at `hdl/tb_replay_buffer.v`, which doesn't exist.

**Fix:**
- [ ] Delete it; use `bridge_top/vivado_create_project.tcl`, or regenerate from `src-link_layer/hdl/files.f`

---

## Kshitij

### K1 — Follow up on the session-end change (rank 4)
`protocol_PHY_layer/rtl/pcie_phy_top.v`, `CHANGELOG.md`, `rtl/bar0_dma_regs.v` header

**What changed:** `session_arm_ctrl.session_end` is now driven by `session_done` from the link
layer. Before, it came from the interconnect's `session_end_aclk` through `pulse_sync`. The
interconnect said "done" while up to ~32 beats were still queued, so an early re-arm put those
beats at the next session's base address. Simulation: 46 of 144 TLPs went to the wrong address
before the fix, 0 after. All 24 benches and 8 cocotb tests still pass.

**Problem:** `session_end_aclk` and the `u_end_cdc0/1` synchronisers now drive nothing. Synthesis
will trim them, but lint will warn, and the CHANGELOG (I3) still describes the old path.

**Fix:**
- [ ] Decide: remove the `session_end_aclk` port (also update `link_phy_bridge_top` and `bridge_top`), or keep it as a debug bit in BAR0 STATUS
- [ ] Update the CHANGELOG I3 entry, and document BAR0 STATUS `session_valid` as "clears once the session's data has left the link layer → safe to re-arm"
- [ ] Add a check to `tb_pcie_phy_top`: an arm while the session is still draining is ignored and counted in ARM_DEBUG

### K2 — Make sure the deleted files are really deleted (rank 6)
`protocol_PHY_layer/legacy/pre_restructure/`

**Problem:** The CHANGELOG lists `pci_stuff.v`, `pcie_tag_man.v`, `pcie_phy_datapath*.v`, `reset.v`
and others as deleted, but all of them were still in the repo root. `pcie_tag_man.v` redefines
`axi4s_ingest_port`, so a folder-wide compile breaks again. They've been moved to `legacy/pre_restructure/`.

**Fix:**
- [ ] `git rm` them (or keep the `legacy/pre_restructure/` layout) and push
- [ ] Run `git status` before pushing so local deletions actually reach the repo

### K3 — Old `pcie_link_top.v` and cocotb runner (rank 10)
`protocol_PHY_layer/legacy/cocotb_old/`

**Problem:** `msix_controller` gained a required `mwr_ready` input. The old `pcie_link_top.v` doesn't
connect it, so the input floats, the MSI-X write never completes, and the vector stays pending. Icarus
warns: "dangling input port 17 (mwr_ready) floating". `cocotb.runner.py` still lists `pcie_link_top.v`
as its source.

**Fix:**
- [ ] Delete both (`pcie_phy_top.v` + `run_tests.py` replace them), or tie `.mwr_ready(1'b1)` if you keep `pcie_link_top` as a reference

### K4 — Rename `test_pcie_link_top.py` (rank 15)

**Problem:** It now runs against `pcie_phy_top`; the name is misleading. Purely cosmetic.

**Fix:**
- [ ] Rename to something like `test_pcie_phy_top_base.py` and update `run_tests.py` and the `Makefile`

### K5 — Old system-integration bench copies (rank 16)

**Problem:** `legacy/pre_restructure/tb_full_system_integration.v` and
`tb_config_msix_system_integration.v` are older copies, including the `sext_rst_n` typo. The fixed
versions are in `tb/` and pass.

**Fix:**
- [ ] Delete

---

*Not rename problems, but from Kshitij's own CHANGELOG "known gaps": no TS1/TS2 transmit generator,
no RX lane deskew, and BAR0/MSI-X not gated by `mem_space_enable` / `msix_enable`. These are needed
for a real link, not for the loopback demo.*

---
---

# Progress — Swaray's tickets (TICKETS.md), resolved

Companion report to the ticket list above. Covers: (1) the L1 scope question, (2) every
Swaray-owned ticket S1–S11. Kshitij's tickets (K1–K5) are his own layer and
are out of scope here — not touched.

All revived/rewritten testbenches end with a `RESULT: PASS` / `RESULT: FAIL`
line per the project convention, and were run individually with Icarus
Verilog (`iverilog -g2005 -Wall`) plus together through the new
`run_all_link.py` harness (S4) — **10/10 PASS**.

---

## L1: fixed and now in the project (was only scoped before, not applied)

Checked directly against the original review's ledger entry:

> **L1 | BLOCKER** | The serialiser loads its count with `DWS_PER_BEAT-1` and
> stops at 1, so it emits 15 DWs. The packetizer waits forever for the 16th.
> `stream_tx_path_top.v:551` | Fix: Load `DWS_PER_BEAT`. Verified.

**Scope:** the bug and its fix are entirely internal to `stream_tx_path_top.v`'s
512→32 serialiser (an off-by-one in a down-counter load value) — it doesn't
touch `stream_tx_path_top`'s port list in a way that affects the
interconnect, and `link_phy_bridge_top`/`bridge_top` already wire the
module's full interface correctly. **No changes needed in the
interconnect layer or in `bridge_top`.**

**Status:** the project's `stream_tx_path_top.v` doc was still carrying the
*unfixed* serialiser (`DWS_PER_BEAT[DW_CNT_W-1:0] - 1'b1`) when this pass
started — the L1 bug was scoped earlier but the fix had never actually been
written back into the project. It's fixed now: the serialiser loads the
beat's real DW count (`popcount(TKEEP)/4`, the same count the descriptor
carries) instead of a hardcoded 15/16, so it emits exactly the right number
of DWs even for a partial last beat (a fixed stray-16-DW load would've also
been wrong there). This also folded in L2 (the descriptor side no longer
double-accepts the FIFO head while the serialiser is still draining a beat)
and the already-integrated L10/MSI-X and Seam-C wiring, since all four live
in the same `stream_tx_path_top.v` doc and `link_phy_bridge_top.v` already
expects this exact interface.

Verified: compiled and ran `tb_stream_path.v` against the fixed module —
**400 beats in, 420/420 TLPs correct (incl. 17 MSI-X), 0 bad/extra/missing,
RESULT: PASS.**

---

## Ranked results

| Rank | ID | File(s) | Resolution |
|---|---|---|---|
| 1 | S1 | `tb_tlp_packetizer.v`, `tb_tlp_packetizer_mps.v` | **Rewritten.** New DW-stream interface; header/payload/BE checks for 1,2,16,64 DW + MPS split (32→32/32/16) + random backpressure. |
| 2 | S2 | `tb_replay_buffer.v` | **Rewritten.** `PAYLOAD_W`/`m_seq`/`outstanding`/`replay_event`; ACK, NAK-rewind, timeout-replay, buffer-full checks. |
| 3 | S3 | `tb_pcie_rx_engine.v` | **Rewritten** (stimulus could not be recovered from git in this environment — rebuilt per the ticket's fallback). MWr in/out of window, CplD for an allocated tag, malformed TLP. |
| 5 | S4 | `run_all_link.py`, `files.f` (new) | **Added.** Single command runs all link-layer benches; missing `RESULT:` line = FAIL. 10/10 PASS. |
| 7 | S5 | `tb_link_layer_loopback.v` | **Rewritten at the Flit level** (chose rewrite over retire — `tb_stream_path` covers the end-to-end pipeline, but this is the only place that unit-tests FEC correction/detection and the ACK/NAK piggyback in isolation). Clean Flit, 1-bit correctable error, 2-bit uncorrectable error (same interleave), ACK/NAK control-Flit piggyback. |
| 8 | S6 | `tb_credit_manager.v` | **Rewritten.** `update_fc_type`/`alloc_req_type`/`alloc_req_dw_len`; grant/stall per pool, exhaustion, UpdateFC replenishment, infinite-credit mode. |
| 9 | S7 | `tb_axi_pcie_reorder_buffer.v` | **Fixed.** Module renamed to match the file; added real checks (out-of-order wire arrival still drains in tag/allocation order, `tlast` correctness, backpressure). |
| 11 | S8 | `tb_step9_flit_fec.v` | **Retired (deleted).** Tests `flit_framer` (legacy MM-mode path) via `tlp_header_parser`, confirmed via `Bridge_RTL_Guide.html` as *not on the active `bridge_top.f` file list*. No replacement needed — Flit-mode framing/FEC is covered by the new S5 bench and by `tb_stream_path`. |
| 12 | S9 | `tb_tag_manager.v`, `tb_tx_engine.v` | **Fixed.** Added `RESULT:` lines with real checks: tag-manager (first-alloc determinism, pool exhaustion/distinctness, timeout→UR, posted bypass); tx-engine (Posted-bypasses-stalled-NP and NP-must-not-bypass-stalled-Posted, verified via a completion-order log). |
| 13 | S10 | `tb_top_pipeline.v` | **Retired (deleted)**, per the ticket's own recommendation — `tb_stream_path` already covers the pipeline end to end, and this bench used two interface generations ago. |
| 14 | S11 | old Vivado project | **Documented only** (see below) — the actual project files aren't present in this environment to delete. |

---

## Details

### S1 — `tb_tlp_packetizer.v` / `tb_tlp_packetizer_mps.v`
Rewritten against the current `req_valid/is_write/addr/length/rid/tag/ready`,
`wdata_tdata/tvalid/tready`, `tlp_out_valid/data/sof/eof/ready` interface.
`tb_tlp_packetizer.v` checks DW0–DW3 for 1/2/16/64-DW writes, FirstBE at all
four byte offsets, LastBE=0 on 1-DW writes, a 16-DW MRd (EOF on HDR3, no data
phase), and random `tlp_out_ready` backpressure (no drops/dupes).
`tb_tlp_packetizer_mps.v` lowers `MAX_PAYLOAD_DW` to 32 and confirms an
80-DW write splits into exactly 32/32/16-DW TLPs with the address advancing
correctly and payload words landing in the right TLP, in order.
Both: `RESULT: PASS`.

*Testbench note:* an early version of each raced the DUT (driving
`wdata_tvalid`/next-word data right at `@(posedge clk)`, same pattern flagged
in an earlier session). Fixed by driving stimulus at `@(negedge clk)` and
only ever reading handshake signals at `@(posedge clk)`.

### S2 — `tb_replay_buffer.v`
Updated to `PAYLOAD_W`/`SEQ_WIDTH`/`BUF_DEPTH`/`REPLAY_TIMEOUT` and the new
`m_seq`/`outstanding`/`replay_event` outputs (scaled down to
PAYLOAD_W=64/SEQ_WIDTH=4/BUF_DEPTH=8/REPLAY_TIMEOUT=20 for a fast sim). Four
checks, all passing: ACK frees exactly the acked Flits; NAK rewinds and
resends from `seq+1` in order; no ACK within `REPLAY_TIMEOUT` triggers a
timeout replay of everything outstanding; buffer-full correctly drops
`s_ready` and nothing is lost (confirmed by draining all 9 Flits, including
the one that was stalled, bit-exact and in order).

### S3 — `tb_pcie_rx_engine.v`
`git log -p` isn't available in this environment, so per the ticket's own
fallback the stimulus was rewritten from scratch against the current
side-band-field interface (`tlp_type`/`tlp_tag`/`tlp_addr`/... alongside the
raw `tlp_rx_data` stream). Four cases: MWr translated correctly inside the
configured window; MWr outside the window raising `translation_err` with no
`AWVALID`; a 2-DW CplD for a tag the reorder buffer had actually allocated,
drained correctly with `tlast` on the last word; a malformed TLP (type field
matching none of MWr/CplD/Cfg) raising a one-cycle `malformed_tlp_err`
pulse. Added `$finish` and `RESULT: PASS`.

### S4 — link-layer `run_all`
`run_all_link.py` + `files.f`, mirroring `protocol_PHY_layer/run_all.py`'s
pattern: `files.f` lists the link-layer RTL sources and every testbench (one
`tb` line each, with any extra cross-layer deps); the script compiles+runs
each with Icarus, greps for a `RESULT:` line, and treats a *missing* line as
FAIL — so a bench nobody touched going stale is caught automatically from
now on. Verified against S1/S2/S3/S5/S6/S7/S9's benches: **10/10 PASS**.

### S5 — `tb_link_layer_loopback.v`
The old bench drove AXI-Stream beats into `link_layer_tx`/`rx`; neither
module has an AXI-Stream side any more (both move whole Flit payloads over
`SYS_BUS_W`). Rewritten at the Flit level per the ticket's second option.
Four checks: a clean Flit decodes bit-exact with no errors; a single-bit
error (flipped in FEC interleave 0) is corrected (`fec_corrected`, CRC still
passes, payload still bit-exact); a two-bit error in the *same* FEC
interleave is correctly flagged uncorrectable (`fec_err`, `rx_flit_ok=0`);
an ACK request with no payload pending produces a control-only Flit
(`kind==CONTROL`) carrying the ACK and its sequence number through to the RX
side.

### S6 — `tb_credit_manager.v`
Updated to `update_fc_type`/`alloc_req_type`/`alloc_req_dw_len` (no more
`avail_hdr_credits`/`avail_data_credits` ports — checked purely through
`alloc_req_grant`/`stall`). Six checks: no-credits stall; grant within
provisioned Posted credits; exhausting the Posted data-credit pool stalls a
request that needs more than is left; the Non-Posted pool is independent
(stalls at 0, grants once UpdateFC arrives); a stalled request is granted
once UpdateFC replenishes it; infinite-credit mode (`0x7F`/`0xFFF`) grants
unconditionally, repeatedly.

### S7 — `tb_axi_pcie_reorder_buffer.v`
Module renamed from `tb_axi_backpressure` to `tb_axi_pcie_reorder_buffer` to
match the filename. Added real checks: allocate tag A then tag B, deliver
B's completion data on the "wire" *before* A's — the AXI master still sees
A's two words first, then B's (allocation order, not wire-arrival order),
with `tlast` correct on each slot's final beat; kept the original
backpressure (mid-completion `TREADY` stall) coverage, now with an assertion
that all beats drain intact afterward.

### S8 — `tb_step9_flit_fec.v`: retired
Tests `flit_framer` + `tlp_header_parser` — the legacy MM-mode framing path.
`Bridge_RTL_Guide.html`'s file index confirms `flit_framer.v` is **not** on
the active `bridge_top.f` list (Flit-mode framing in `link_layer_tx`/`rx`
replaced it). Retiring it loses nothing: FEC correction/detection is now
covered by S5's rewritten `tb_link_layer_loopback.v`, and end-to-end framing
by `tb_stream_path`. Deleted.

### S9 — `tb_tag_manager.v` / `tb_tx_engine.v`
Both kept their existing stimulus shape and gained real checks + a `RESULT:`
line. `tb_tag_manager.v`: first allocation is deterministically tag 0;
pool exhaustion allocates exactly `N` distinct tags before `s_tready` drops;
a timed-out tag fires `ur_valid` for one of the outstanding tags; a posted
(MWr) request never pulls a tag. `tb_tx_engine.v`: added a completion-order
log (`'P'`/`'N'` on `p_req_ready`/`np_req_ready`) and checked both ordering
rules directly — Rule 1 (Posted bypasses a stalled Non-Posted request) and
Rule 2 (Non-Posted must *not* bypass a stalled Posted request, since
`p_req_valid` staying high blocks `want_np` outright regardless of NP's own
credit state).

### S10 — `tb_top_pipeline.v`: retired
Deleted per the ticket's own recommendation. It instantiated
`tlp_packetizer` with parameters/ports (`MAX_PAYLOAD_SIZE_DW`, `REQ_ID`,
`req_is_mwr`, `req_len`) from two interface generations ago, and
`tb_stream_path` already covers the same pipeline end to end against the
current interface.

### S11 — old Vivado project: documented, not deleted
`src-link_layer/legacy/vivado_project_old/Link_layer.xpr` (and its sim set
pointing at a `tb_replay_buffer.v` that no longer exists in that form) isn't
present as a file this environment can read or delete — it's a binary/XML
Vivado project artifact, not one of the project's text docs. Recommended
action, to run from the actual repo: `git rm -r
src-link_layer/legacy/vivado_project_old/`, then regenerate a project either
from `bridge_top/vivado_create_project.tcl` or directly from
`src-link_layer/hdl/files.f` (S4's file list now covers exactly the current
RTL + bench set, so it's a safe source for that regeneration).

---

## Out of scope (Kshitij's tickets)
K1–K5 are Kshitij's PHY layer and weren't touched: K1 (`session_end_aclk`
dead-logic follow-up), K2 (confirm `legacy/pre_restructure/` deletions
reached git), K3 (`pcie_link_top.v`/cocotb runner — dangling `mwr_ready`),
K4 (rename `test_pcie_link_top.py`), K5 (delete stale `sext_rst_n`-typo
bench copies).