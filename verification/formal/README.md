# Formal proofs (Phase 2, section 2)

SymbiYosys proofs for the five blocks in the Phase 2 plan. Aditya's and Swaray's RTL is **not modified**: the
harnesses (`fv_*.v`) check their blocks only through their ports. The `session_arm_ctrl` properties
live inside that file (PHY, Kshitij) under `` `ifdef FORMAL ``.

```sh
cd verification/formal
python run_formal.py              # every .sby here; ends with RESULT: PASS / FAIL
python run_formal.py rr_arbiter   # one file
```

Tools: Yosys (with `yosys-abc`, `yosys-smtbmc`), SymbiYosys matching your Yosys version, and
**z3 ≥ 4.12**. z3 4.8.x, the Ubuntu default, crashes on some of these models with "Unexpected
EOF from solver"; `pip install z3-solver` gives a working one. The unbounded proofs use
`abc pdr`, which is built into Yosys and needs no external solver.

## Results

| Block | Properties | Result |
|---|---|---|
| `rr_arbiter_2ch` | Never grants both. No grant without a request. **A waiting request is granted within `STARVE_CYCLES + 1` cycles** for any weights (0 included) and any behaviour of the other channel. The bound is tight: the cover reaches it. Proved unbounded at small parameters **and at the design's** (8-bit weights, `STARVE_CYCLES = 64`). | **PASS** |
| `cdc_fifo_async` | Two unrelated clocks, arbitrary edge interleaving. Never overflows, never underflows, never loses, duplicates or reorders data (sequence-number check), full is never late. Non-vacuous: the FIFO reaches capacity and the pointers wrap. | **PASS** |
| `axi4s_ingest_port` | Never accepts a data beat past the session limit. The offset sent downstream equals the bytes accepted. No handshake outside an active session or under backpressure. An abort marker has `tkeep = 0` and `tlast = 1`, at most one per session. Overflow is sticky until the next arm. | **FAIL: real bug F1.** PASS with the one-line fix (`findings/ingest_offset_wrap_fix.patch`) |
| `tlp_header_parser` (link layer RX) | On any well-formed DW stream (gaps allowed, any payload length): `parsed_hdr_done` pulses **exactly once per TLP**, on its last header DW: DW2 for a 3DW request or a completion, DW3 for a 4DW request. Never on payload, never outside a TLP. With it, `parsed_addr` = DW2 (3DW) or {DW2, DW3} (4DW), low 2 bits zero; type, length, the 10-bit tag ({DW0[23], DW0[19], Tag[7:0]}) and `parsed_has_data` (Fmt[1]) are right. All completion types are included, CplLk too. `parsed_rx_*` is the input delayed by one clock. Covers reach a 3DW MWr, a 4DW request, a completion, back-to-back TLPs and a gap. **Mutation check:** re-adding an early `hdr_done` on DW2 of a 4DW header makes the proof fail at once. | **PASS** |
| `session_arm_ctrl` | Only the 3 legal states and legal transitions. **An arm while ACTIVE is ignored**: no restart, base address and TC unchanged, counted in the saturating debug counter. `session_valid` is exactly "ACTIVE". | **PASS** |
| PHY (`protocol_PHY_layer/formal/`) | Gray inverse, PAM4 table, SOF-strip FSM, and new: **`pam4_adjacent`**, data on adjacent PAM4 levels differs in exactly 1 bit across the whole enc→map→demap→dec chain. That proof failed on the old level table: Gray was applied twice, found by the noise model (CHANGELOG Update 9, N1). | **PASS** (re-checked with z3 5.3) |

`run_formal.py` also runs tasks marked `expect fail`. These are reachability checks: "FIFO can
fill", "pointers wrap", plus the documented no-contract hazard F2. They count as OK when they find
their counterexample.

## Findings

**F1: session offset wraps past 16 MB with no overflow (bug, Aditya, `axi4s_ingest_port.v:78`).**
`(session_offset + beat_bytes) > session_limit` is evaluated at `OFFSET_WIDTH` (24) bits. Near
the top of the range the sum wraps, the check passes, the beat is accepted, and the offset restarts
at 0. The BAR0 default limit is `0xFFFFFF`. In a full-width simulation
(`findings/tb_ingest_offset_wrap.v`), beat 262,145 of a session longer than 16 MB is accepted at
offset `0x000000` with `session_overflow = 0`, so it overwrites the start of the host buffer. The
plan's ML workload ("tensors up to the 16 MB session limit") would hit this. **Fix:** one extra bit
in the compare, see `findings/ingest_offset_wrap_fix.patch`. With it, all ingest-port properties
prove, the simulation overflows correctly, and `tb_bridge_top` and `tb_bridge_ovf` still pass.
Related: with a 24-bit limit, the largest session is 16 MB − 1 byte. A full 16 MB tensor needs
`LW = 25`, or the limit to mean "last valid offset".

**F2: early re-arm hazard (latent, Aditya, `axi4s_ingest_port.v`).** `session_start` does not clear
a pending abort marker, and a beat accepted in the same cycle as `session_start` isn't counted. So
if a channel were re-armed before its overflow marker went out, the old session's marker would end
the new session. **Not reachable in the bridge today:** since ticket K1, the PHY only re-arms after
the link layer's `session_done`, which needs that marker to have gone through. The `prove` task
states this contract as an assumption. Task `no_arm_contract` drops it and is *expected* to fail.
Worth a one-line hardening (clear `abort_pend` on `session_start`) if re-arming ever comes from
somewhere else.

**F3: the CDC FIFO holds 15 entries, not 16 (behaviour, not a bug).** `wr_full` uses a look-ahead
pointer, so it asserts at DEPTH−1 entries. This is proven: occupancy never exceeds DEPTH−1, and
DEPTH−1 is reachable. Backpressure uses `almost_full` (12), so nothing changes in practice; just
size with 15.

**F4: the arbiter's worst-case wait is `STARVE_CYCLES + 1` = 65 cycles** (the plan says 64).
Exact and reachable.

**F5: an MRd looked like an MWr downstream (ticket P2-5, Swaray). FIXED.** The parser exported
Type but not Fmt, so `pcie_rx_engine`'s `is_mwr` couldn't tell a Memory Read from a Memory Write.
A host read of BAR0 was decoded as a write. Its `is_mwr` also matched Type 1_0000, which is a
Message in the spec. Fix: the parser exports `parsed_has_data` (Fmt[1] = DW0[30]), and
`pcie_rx_engine` uses MWr = Type 0 && has_data. MRd/MRdLk and Messages are recognised and ignored,
not written and not flagged malformed. The proof checks the new output (T6). `tb_pcie_rx_engine`
gained T5 (MRd: no AXI write) and T6 (Msg: no AXI write).
Still open: no completer returns CplD for a host read, so a host BAR0 read would time out.

**F6: CplLk was parsed as a request. FIXED.** `is_cpl` matched only Type 0_1010. A
locked-read completion (0_1011) took the request path, so its tag came from DW1 instead of DW2 and
`parsed_addr` was overwritten. `is_cpl` now matches `Type[4:1] == 4'b0101`, and the proof's
environment includes CplLk.

## Files

| File | What |
|---|---|
| `rr_arbiter.sby`, `fv_rr_arbiter.v` | Arbiter: `prove` (small params), `prove_full` (design params), `cover` |
| `cdc_fifo.sby`, `fv_cdc_fifo.v` | CDC FIFO (multiclock): `prove`, `reach_full`, `reach_wrap` (expected fail = reachable) |
| `ingest_port.sby`, `fv_ingest_port.v` | Ingest port: `prove`, `no_arm_contract` (expected fail, F2), `cover` |
| `session_arm_ctrl.sby` | Uses the properties in `protocol_PHY_layer/rtl/session_arm_ctrl.v` |
| `tlp_header_parser.sby`, `fv_tlp_header_parser.v` | Header parser: `prove`, `cover` |
| `findings/` | F1 fix patch and the full-width simulation demo |
| `run_formal.py` | Runs everything, checks each task against its expectation |

Output folders (`*_prove/`, `*_cover/`, …) are generated; don't commit them.
