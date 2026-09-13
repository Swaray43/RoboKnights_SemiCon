# Interconnect layer — per-file documentation

Companion to `interconnect_layer_full_spec.md` (which covers the interface contract and cross-team requirements). This document walks through **each file individually** — what it does internally, every port and parameter, the key design decisions and why they were made that way, and known limitations. Read this when you need to modify or debug a specific file; read the full spec when you need the big picture or the contract with Swaray/Kshitij.

---

## 1. `axi4s_ingest_port.v`

**Purpose:** One AXI4-Stream ingest port. Instantiated twice in `interconnect_top` — once per channel (`CH_ID=0` for SmartNIC, `CH_ID=1` for ML). Owns the AXI4-Stream slave handshake and per-session byte-offset tracking for its channel.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `DATA_WIDTH` | 512 | AXI4-Stream data bus width |
| `KEEP_WIDTH` | `DATA_WIDTH/8` | TKEEP width (one bit per byte) |
| `OFFSET_WIDTH` | 24 | Width of the session offset counter — sized to the largest expected session/slice size, not to a full address |
| `CH_ID` | 0 | Which channel this instance is (informational; doesn't gate any logic internally — the *instantiating* module, `interconnect_top`, is what ties `stream_id` to a physical port) |

### Ports

| Port | Dir | Meaning |
|---|---|---|
| `clk` / `rstn` | in | AXI-side clock domain, active-low async reset |
| `s_tdata` / `s_tkeep` / `s_tlast` / `s_tvalid` / `s_tready` | AXI4-Stream slave | Standard signals. `s_tready` is an output computed by this module |
| `session_start` | in | One-cycle pulse from the config-space block. Resets `session_offset` to 0 and sets `session_active` — see §9.1 of the full spec, this is an **unconfirmed assumption**, not a verified contract |
| `session_limit` | in | Max bytes allowed in the current session before overflow triggers |
| `session_active` | out | High while a session is in progress; drops on TLAST (normal end) or overflow (abort) |
| `session_overflow` | out | **Sticky** — once set, stays set until the next `session_start`. This is deliberate: a sticky flag survives long enough for the host/interrupt handler to actually observe it, rather than a one-cycle pulse that could be missed |
| `req` | out | "I have a beat ready and want the shared bus" — feeds the arbiter |
| `grant` | in | Arbiter's registered decision — "you may drive the bus this cycle" |
| `ds_backpressure` | in | Shared downstream-full signal (from the CDC FIFO's `almost_full`, wired identically to both channels in `interconnect_top`) |
| `out_tdata` / `out_tkeep` / `out_tlast` / `out_session_offset` | out | This channel's current beat, forwarded toward the crossbar mux — valid regardless of `grant` (the crossbar picks which channel's `out_*` to use) |
| `accepted` | out | Pulses exactly when a real AXI4-Stream handshake completed this cycle (`s_tvalid && s_tready`) |

### Internal logic walkthrough

**Byte counting (`popcount`).** A beat's actual byte count isn't always the full bus width — the final beat of a session is typically partial, with `TKEEP` marking which bytes are valid. `popcount` is a synthesizable for-loop function counting set bits in `s_tkeep`, giving `beat_bytes` — this is what `session_offset` actually advances by, not a hardcoded `KEEP_WIDTH` constant.

**Overflow check (`would_overflow`).** Computed *before* accepting the beat: `session_offset + beat_bytes > session_limit`. This is a look-ahead check — it fires on the cycle that *would* cause the overflow, not after the fact, so the offending beat is never actually counted into `session_offset` or forwarded downstream.

**`req` vs. `s_tready` — why they're different expressions.** This is the most important subtlety in the file:
- `req` = `s_tvalid && session_active && !session_overflow && !would_overflow` — does **not** depend on `grant` or `ds_backpressure`. This is what the arbiter uses to decide who to grant; it must reflect "do I have something to send," not "am I currently allowed to send it."
- `s_tready` = `grant && session_active && !session_overflow && !ds_backpressure` — does **not** depend on `s_tvalid` directly (though in practice `s_tready` only matters when `s_tvalid` is also high, per AXI4-Stream semantics). This is what actually gates the transfer.

Conflating these two was the root cause of a real bug found during integration testing (see `tb_interconnect_top.v` notes below) — `accepted` (`s_tvalid && s_tready`) is the only signal that should ever be used to know a transfer *actually happened*, and that's exactly why it's exposed as its own output rather than left for the consumer to recompute.

**Why `accepted` exists as an explicit output.** Early versions of `interconnect_top` gated the CDC FIFO write purely on `grant`. That's wrong: `grant` can be high on a cycle where `ds_backpressure` or `session_overflow` is *also* high, blocking the real handshake — in which case the AXI4-Stream master is (correctly, per protocol) holding the same beat steady, unchanged, waiting for `s_tready`. Writing on `grant` alone would re-write that same still-pending beat into the FIFO on every such cycle, corrupting the stream with duplicates. `accepted` is the one signal that's only ever high when a transfer genuinely completed.

**Sequential block.** Priority order matters: `session_start` (re-arm) is checked first and unconditionally resets state, `would_overflow` (abort) is checked next, and normal accumulation (`beat_hs`) last. This ordering means a `session_start` pulse always wins even if it happens to coincide with what would otherwise be an overflow condition from a stale prior session.

### Known limitations

- No protection against `session_start` arriving while `session_active` is already high (mid-session re-arm). Simulated behavior would be an abrupt reset of the offset counter — functionally "fine" but not something explicitly tested; flagged in the full spec's verification gaps.
- `session_limit` is a fixed value for the whole session — there's no path to change it mid-session, which is correct given the design intent but worth knowing if a future requirement needs dynamic resizing.

---

## 2. `rr_arbiter_2ch.v`

**Purpose:** Decides, every cycle, which of the two channels' beats gets to drive the shared crossbar bus. Weighted round-robin with a starvation guard, two channels only (not generalized to N).

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `CREDIT_WIDTH` | 8 | Width of the weight/credit registers — max weight value 255 |
| `STARVE_WIDTH` | 16 | Width of the wait-cycle counters |
| `STARVE_CYCLES` | 64 | Cycles a channel can wait before starvation override forces a grant — placeholder, needs tuning against real timing (§9.5 of full spec) |

### Ports

| Port | Dir | Meaning |
|---|---|---|
| `clk` / `rstn` | in | Same domain as the ingest ports |
| `req0` / `req1` | in | From each channel's `req` output |
| `weight0` / `weight1` | in | Runtime-programmable relative weights |
| `grant0` / `grant1` | out | **Registered** — valid for the mux select on the cycle *after* the decision was made, never combinational |

### Internal logic walkthrough

**Credits, not a simple pointer.** Each channel has a credit counter. A channel is `eligible` only while it has a request *and* nonzero credit. When both channels are simultaneously eligible, a `last_grant` flip-flop breaks the tie by always favoring whichever channel was *not* granted last — this is what produces the round-robin behavior on top of the weighting.

**Per-channel independent refill — the deadlock fix.** The first working version refilled both credits together, only when *both* had hit zero (`round_done = credit0==0 && credit1==0`). That deadlocks the moment only one channel is ever requesting: the idle channel's credit never decrements (nothing consumes it), so the joint "both zero" condition can never become true again after the first refill, permanently starving the active channel once its own credit runs out. The fix (`round_done0` / `round_done1`) makes each channel's refill condition depend on **its own** exhaustion plus (the other channel not requesting, OR the other channel also exhausted) — so a channel with no competition can freely refill itself without waiting on a competitor that isn't even in the running.

**`cred_avail0`/`cred_avail1` — avoiding a one-cycle bubble.** Refill happens on a clock edge (registered), so naively, a channel that just hit zero credit would show `credit==0` for one full cycle before the refill lands, causing eligibility to falsely drop and a cycle to go to waste even when a channel legitimately deserves the bus. `cred_avail` folds in "credit is 0 but *about to* be refilled this cycle" as if credit were already available, closing that gap.

**Starvation override.** `wait0`/`wait1` count consecutive cycles a channel has a request but wasn't granted. Crossing `STARVE_CYCLES` forces a grant regardless of credit state — this is what guarantees forward progress for a channel whose weight is set to 0 (verified explicitly in the testbench), and is a hard backstop against any subtler fairness bug in the credit logic.

**Registered decision.** `grant0`/`grant1` are the *registered* output of a purely combinational `next_g0`/`next_g1` decision. This one-cycle latency is deliberate and required — see the crossbar design discussion: a combinational path from `req`/`weight` straight through a 512-bit mux select is a timing-closure risk at the target clock; registering the decision breaks that path.

### Known limitation

Credit-based WRR grants a channel's **full weight in a row** before switching (e.g. weight 3:1 → ch0,ch0,ch0,ch1,repeat), not a smoothed/interleaved pattern. Documented in the file header as an explicit simplification — fine for coarse bandwidth-share tuning, not fine if genuinely interleaved service is required. Flagged, not silently assumed away.

---

## 3. `cdc_fifo_async.v`

**Purpose:** Generic gray-code asynchronous FIFO — the one and only sanctioned crossing point between `aclk` and `pcie_clk`. Everything upstream of this file lives entirely in the AXI clock domain; everything downstream (Swaray's block) lives in the PCIe link clock domain.

### Parameters

| Parameter | Default | Meaning |
|---|---|---|
| `WIDTH` | 512 | Data width — in `interconnect_top`, sized to carry `{tdata, tkeep, stream_id, session_offset, tlast}` together as one atomic word |
| `ADDR_WIDTH` | 4 | `log2(depth)` — depth 16 |
| `ALMOST_FULL_PCT` | 75 | Occupancy percentage at which `wr_almost_full` asserts, ahead of true full |

### Ports

| Port | Dir | Meaning |
|---|---|---|
| `wr_clk`/`wr_rstn`/`wr_en`/`wr_data` | in | Write side, `aclk` domain |
| `wr_full` | out | True full — writing while this is high does nothing (write is silently dropped by design; the write-side contract is that the caller must never assert `wr_en` when `wr_full` is high) |
| `wr_almost_full` | out | Early-warning backpressure signal, margin before true full |
| `rd_clk`/`rd_rstn`/`rd_en`/`rd_data` | | Read side, `pcie_clk` domain |
| `rd_empty` | out | No valid data at the current read pointer |

### Internal logic walkthrough — and the two real bugs found here

**Gray code, and why.** Binary pointer increments can flip multiple bits at once (`0111→1000`). If a synchronizer samples mid-transition, a torn value can result — not just "off by one" but potentially wildly wrong. Gray code guarantees exactly one bit changes per increment, so a synchronizer catching a transition mid-flight reads either the old or the new value, never garbage. Two-flop synchronizers (`*_gray_wr1/wr2` and `*_gray_rd1/rd2`) give a full extra cycle for any metastable first-stage flop to resolve before the value is used.

**Bug #1 — combinational loop in the full-flag logic (design-breaking, would hang any simulator or synthesis tool).** The first version computed `wr_full` from a "next pointer" that was itself gated by `wr_en && !wr_full` — meaning `wr_full`'s own definition depended on `wr_full`. That's a zero-delay combinational cycle; the simulator span forever trying to resolve it (confirmed by an actual hung `vvp` run during development). **Fix:** the lookahead pointer used for the full-flag comparison (`wr_ptr_bin_lookahead = wr_ptr_bin + 1`) is now *unconditional* — it doesn't check `wr_en` or `wr_full` at all, so `wr_full`'s formula only ever depends on two registers (`wr_ptr_bin`, `rd_ptr_gray_wr2`), never on itself. The *actual* conditional pointer advance (gated by `wr_en && !wr_full`) is a separate sequential update that reads the now-safely-computed `wr_full` value — no cycle.

**Bug #2 — read-data protocol mismatch (FWFT vs. registered-read).** The first version registered `rd_data` one cycle *after* `rd_en` fired — a completely standard synchronous-read-port memory pattern, but wrong for this interface: `interconnect_top` ties `m_tvalid = !fifo_empty` and expects `m_tdata` to be valid in that same cycle, matching AXI4-Stream semantics (TVALID means TDATA is the beat to be transferred *this* cycle). With registered-read timing, the very first time `!fifo_empty` went high, `rd_data` still held its post-reset zero value for one more cycle — the consumer read that as a real (garbage) beat. **Fix:** switched to first-word-fall-through — `rd_data` is now a *combinational* read of `mem[rd_ptr_bin]` at the current pointer, valid before `rd_en` is even asserted; `rd_en` only advances the pointer to the next entry, it doesn't gate what's currently visible.

**`almost_full` — safe to convert gray→binary, and why only here.** `full`/`empty` are computed from pure gray comparisons for CDC safety (never converted back to binary before a synchronizer has resolved them). `almost_full`'s occupancy estimate *does* convert the already-two-flop-synchronized `rd_ptr_gray_wr2` back to binary via `gray2bin` — this is safe specifically because the conversion happens strictly *after* synchronization, never on a signal that's mid-crossing.

### Known limitation

Depth is small (16) relative to the 512-bit+sideband width — at synthesis this will very likely map to flip-flop/LUTRAM-based storage rather than a block RAM primitive. Noted in `openlane_config.json`'s comments as something to verify in the synthesis log rather than assume.

---

## 4. `interconnect_top.v`

**Purpose:** Wires the three modules above into the complete layer. This file *is* the contract boundary with the rest of the project — every port here is either driven by Kshitij's block, consumed by Swaray's block, or internal.

### Key wiring decisions

**Crossbar mux is a plain 2:1 select on `grant0`/`grant1`, not a real crossbar structure.** With only two channels and mutual exclusion already guaranteed by the arbiter (verified in `tb_rr_arbiter_2ch.v`'s Test 5), a ternary select is sufficient — `xbar_data = grant0 ? ch0_data : ch1_data`, same pattern for `tkeep`/`tlast`/`offset`, plus `xbar_id` tagging which channel won.

**`xbar_valid` gates on `accepted`, not raw `grant` — the fix for the duplicate-write bug.** `xbar_valid = (grant0 && ch0_accepted) | (grant1 && ch1_accepted)`. This was the third real bug found during integration: gating purely on `grant0 | grant1` wrote a channel's held-steady (not-yet-accepted) beat into the FIFO repeatedly whenever that channel was granted but blocked by backpressure or overflow — flooding the FIFO with duplicate entries and triggering `almost_full` almost immediately, which froze the whole pipeline (observed directly: 12 real beats sent, 224 duplicate entries received, before the fix).

**Shared backpressure.** `fifo_almost_full` feeds `ds_backpressure` on *both* ingest port instances identically — matching the design decision (made earlier, before this file existed) that downstream-full backpressure should be one shared signal, not independent per-channel ones.

**FIFO word packing.** `{xbar_data, xbar_keep, xbar_id, xbar_offset, xbar_last}` — packed and unpacked via simple concatenation on both sides of the CDC FIFO, so a beat's data, channel identity, offset, and end-of-session marker all cross the clock domain together as one atomic word, never torn apart from each other.

### Parameters

Same shape as the constituent modules, plus `FIFO_ADDR_WIDTH` (depth = 2^this, default 4 → depth 16).

---

## 5. `interconnect_constraints.xdc` / `interconnect_constraints.sdc`

**Purpose:** Timing constraints for the Vivado flow (XDC) and the OpenLane/OpenSTA flow (SDC) respectively. Content is intentionally near-identical between the two — SDC and XDC syntax differ slightly (`get_cells` vs `get_pins` targeting conventions in particular), but the constraints they express are the same.

### What's actually constrained

1. **Two independent clocks** (`aclk`, `pcie_clk`), declared with placeholder periods — **these need real values** once the actual target frequencies are known; `aclk` at 4.000ns (250MHz) matches the project's stated AXI-side target, `pcie_clk` at 3.000ns is a pure placeholder pending Swaray's block.
2. **`set_clock_groups -asynchronous`** — tells the tool these two clocks have no fixed phase relationship, so it doesn't attempt (and fail) to close timing on a direct single-cycle path between them.
3. **Multicycle paths on the gray-code pointer synchronizers**, in both directions (write pointer → read domain, read pointer → write domain). Without this, static timing analysis either falsely flags the crossing as a violation (since it doesn't understand the two-flop synchronizer is deliberately allowed to take an extra cycle to settle) or — the more dangerous failure mode — doesn't constrain it at all and lets an actually-unsafe single-cycle path through unflagged.

### Known limitation — explicitly flagged in the file itself

The cell/pin name patterns used in the multicycle path constraints (`*/u_fifo/wr_ptr_gray_reg*` etc.) assume synthesis preserves those register names. **This is not guaranteed** — synthesis tools frequently rename or merge registers. §7 of the full spec explicitly calls out re-checking these constraints post-synthesis as a required step, not an optional one; a silently-unapplied multicycle constraint is a common and dangerous failure mode that produces no error, just an unconstrained (and possibly unsafe) path.

---

## 6. `openlane_config.json`

**Purpose:** Starting-point configuration for running `interconnect_top` through the OpenLane physical implementation flow, per the project's stated tooling requirement (OpenRoad/OpenLane with open PDKs).

### What it specifies

- `DESIGN_NAME`, `VERILOG_FILES` (all four RTL files, no testbenches), `CLOCK_PORT`/`CLOCK_PERIOD` for the primary clock.
- `FALLBACK_SDC_FILE` pointing at the `.sdc` file above — this is how the second clock (`pcie_clk`) and the CDC multicycle constraints actually get applied, since OpenLane's top-level JSON schema has no native way to declare a second clock domain.
- Conservative placement/utilization defaults (`PL_TARGET_DENSITY: 0.45`, `FP_CORE_UTIL: 40`) as reasonable starting points, not tuned/optimized values.

### Explicitly documented as NOT verified

The file contains inline `_comment_*` fields stating plainly that this has not actually been run through OpenLane in this environment — doing so needs the tool installed locally with a real PDK (sky130, gf180, etc.), which wasn't available here. Treat this as a checked-in starting point for whoever runs the physical flow, not as evidence the design has been placed and routed.

---

## 7. Testbenches

### `tb_axi4s_ingest_port.v`

Four scenarios, each isolated: (1) session offset accumulates correctly across several beats matching `TKEEP` popcount, (2) `TLAST` correctly ends a session (`session_active` drops), (3) a beat that would exceed `session_limit` correctly sets `session_overflow` and aborts *before* actually exceeding the limit, (4) `ds_backpressure` correctly holds `TREADY` low even while granted. Uses a smaller `DATA_WIDTH=64` for readability — the module is fully parameterized so this doesn't affect coverage of the logic itself.

### `tb_rr_arbiter_2ch.v`

Five scenarios: 50/50 default split lands near-even (19/20 over 40 cycles), a 3:1 weight ratio lands near the target ratio (58:21 ≈ 2.76:1), a weight-0 channel still gets serviced via the starvation guard (10 grants over 60 cycles despite zero weight), a lone requesting channel gets granted essentially every cycle with no idle-channel-induced stall, and grant0/grant1 are never simultaneously asserted across a full run. This testbench is what caught both the original joint-refill deadlock and the one-cycle eligibility bubble.

### `tb_cdc_fifo_async.v`

Drives genuinely asynchronous clocks (7ns and 13ns periods — deliberately not related by a small integer ratio, to avoid accidentally-aligned edges masking a real bug) with independent writer/reader processes, verifying 110 words pushed exactly match 110 words popped, in order, with no corruption. This is where both CDC FIFO bugs (combinational loop, FWFT mismatch) were actually caught — and also where a testbench-only race (driving stimulus at the exact moment the DUT samples it, with no ordering guarantee between two concurrent Verilog processes) was found and fixed by inserting a small settle delay after each clock edge before driving new stimulus.

### `tb_interconnect_top.v`

Full-stack integration: two independent clock domains, both channels driven continuously and simultaneously, checking correct `stream_id` tagging and correct per-channel `session_offset` sequencing on the output side. This is where the duplicate-write bug was caught (`recv0`/`recv1` far exceeding `sent0`/`sent1` before the fix). Includes hierarchical debug instrumentation (`accepted0_cnt`, `accepted1_cnt`, `wr_cnt`, `rd_cnt`) added specifically to distinguish a genuine data-loss bug from a benign testbench-side counting artifact — the final pass criteria trusts the DUT's own internal signal counts rather than the driver process's parallel bookkeeping, since those were shown to disagree by exactly one count for reasons traced to simulation sampling timing, not RTL behavior.
