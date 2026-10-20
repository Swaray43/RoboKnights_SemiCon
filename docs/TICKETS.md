# Tickets — round 2 (review of the post-ticket snapshot, before the Phase 2 cycle)

Round 1 (S1–S11, K1–K5) is in `TICKETS_round1.md`. Every round-1 bench now builds and passes
except `tb_tx_engine` (still print-only, S9).

## A. Found in the snapshot and already fixed in this zip

| ID | Owner | File(s) | What was wrong | Impact if shipped | Fix applied |
|---|---|---|---|---|---|
| R1 | Swaray | `src-link_layer/hdl/link_layer_top.v` | File held a second copy of `link_phy_bridge_top` (identical, same module name). `link_layer_top` itself was missing. | Nothing above the link layer could build (duplicate module + missing module). | Restored the real `link_layer_top`. |
| R2 | Swaray / Aditya | `link_phy_bridge_top.v`, `interconnect_top.v`, `bridge_top.v` | TUSER built as `{tlast, ic_tuser}` with `ic_tuser = {stream_id, offset}`; the link reads `{tlast, offset, stream_id}`. | Reproduced end to end: every ch0 TLP lands at offset/2 (+8 MB when the id bit lands in the offset), ch1 traffic goes to ch0's base, bench hangs. | `interconnect_top.m_tuser` now **is** the link's native 26-bit `{tlast_flag, session_offset, stream_id}`; passed straight through. Shim retired. Interconnect bench checks `tlast_flag == m_tlast` on every beat. |
| R3 | Swaray | `tlp_header_parser.v`, `pcie_rx_engine.v` | Parser took the address from DW1/DW2 (one DW early: DW1 is Requester ID/Tag), and the RX engine sampled the address at SOF, before it had arrived. Completions also took Tag/Lower Address from DW1 instead of DW2. | The new RX decode never produced a correct BAR0 address. `tb_link_layer_top` and `tb_link_phy_bridge_top` timed out. | Parser indexes DW2/DW3 correctly and emits `parsed_hdr_done`. `pcie_rx_engine` has `REQ_ON_HDR_DONE` (1 in `link_layer_top`, 0 = old behaviour for the unit bench). Both benches pass. |
| R4 | Swaray | `link_phy_bridge_top.v` | `m_rx_axis_*` tied to 0. | `tb_bridge_top`'s TLP checker sees nothing; the full-stack bench can't verify data. | `link_layer_top` exports `mon_rx_axis_*` (observe-only copy of every inbound beat); `m_rx_axis_*` carries it. `m_rx_axis_tready` is ignored (decode owns flow control). |
| R5 | Swaray | `flit_defs.vh` | FEC `while` loops back (round-1 fix lost). | Yosys/LibreLane can't synthesize the link layer. | `while` → `if` again (equivalent; p never hits two powers of 2 in a row). |
| R6 | Swaray | `files.f`, `run_all_link.py` | Both missing from the snapshot. | No one-command link regression. | Restored. `files.f` now covers all 16 benches; `PHY:` deps for the bridge bench; runner also understands the older `ALL TESTS PASSED` format. |
| R7 | Kshitij / Swaray | `bridge_integration.patch` | Patch didn't apply to `link_phy_bridge_top.v` (line drift). | 12-bit BAR0 address truncates MSI-X table (0x1000) and PBA (0x1800). | Applied the rest of the patch; changed `bar0_addr` to `[15:0]` by hand. |
| R8 | Aditya | `bridge_top.f`, `asic/*` | File lists missing `credit_manager`, `link_layer_top`, the RX decode and the new PHY files (LTSSM, deskew, TS gen). `asic/` was missing from the snapshot. | Vivado/Quartus/LibreLane elaboration fails. | Lists updated (56 files); `asic/link` now hardens `link_layer_top`; SDCs and ASIC-profile defparams updated. |

## B. Open — for the Phase 2 cycle (ranked by utility)

| Rank | ID | Owner | File | Problem | Suggested fix | Effort |
|---|---|---|---|---|---|---|
| 1 | P2-1 | Swaray + Kshitij | `link_phy_bridge_top.v`, `pcie_phy_top.v`, `bridge_top.v` | Decoded inbound MWr (`m_axi_aw*`) goes nowhere; BAR0/MSI-X/config are still written by testbench ports. | Route decoded MWr (address + first data DW) into `bar0_dma_regs` as a second write source; keep the test ports for cocotb. This is what makes the bridge host-driven. | 1 day |
| 2 | P2-2 | Swaray | `stream_tx_path_top.v`, `credit_manager.v` | Credit gate is live but fed a one-shot "infinite" sentinel (0x7F / 0xFFF). No UpdateFC source. Also: PCIe advertises infinite as **0**, and 0x7F is a legal finite header count. | Take InitFC/UpdateFC from the PHY's DLLP path; map 0 → infinite; drop the 0x7F/0xFFF sentinel. | 0.5–1 day |
| 3 | P2-3 | Swaray | `pcie_address_translator.v` | Window miss: `awvalid` stays 0, `awerr` pulses for one cycle, nothing latches it. A new request while `awvalid && !awready` is dropped. | Latch a sticky miss counter/status bit (BAR0 STATUS); queue or back-pressure when not ready (`awready` is tied 1 today, so latent). | 2 h |
| 4 | P2-4 | Swaray | `link_layer_top.v` RX unpacker | One DW per clk: a full 512-bit beat takes 16 clk. Inbound bulk traffic would fill the DLL RX FIFO → NAK → replay. Fine for host register writes (measured 0 replays in `tb_bridge_top`). | Widen the parser (4–16 DW/clk) only if inbound bulk is added in Phase 2. | 1 day (if needed) |
| 5 | P2-5 | Swaray | `pcie_rx_engine.v`, `tlp_header_parser.v` | `is_mwr` also matches MRd (Type 0_0000 with Fmt without data). Parser doesn't export Fmt. | Export `parsed_has_data` (Fmt[1]); MWr = Type 0 && has_data. | 30 min |
| 6 | P2-6 | Swaray | `testbench/tb_tx_engine.v` | Print-only, no pass/fail (round-1 S9). | Add expected-order checks and a `RESULT:` line. | 30 min |
| 7 | P2-7 | Swaray | `axi_pcie_reorder_buffer.v` | Module-level `integer i` as a loop index in a clocked block: Yosys `check` reports 2 "conflicting driver" warnings. Harmless, but noise in signoff logs. | Use a loop variable local to the block (`for (integer i ...)` or a named block). | 10 min |
| 8 | P2-8 | All | process | Glue/interface edits keep regressing things fixed earlier (R1, R2, R5 were all round-1 fixes or contracts). | Before merging: `python3 src-link_layer/hdl/run_all_link.py`, `protocol_PHY_layer/run_all.py`, interconnect benches, `tb_bridge_top`, `tb_bridge_ovf`. Put the TUSER layout in one header (`axis_tuser_defs.vh`) shared by both layers. | 1 h |

## C. Verified on this zip (Icarus 12, cocotb 2.1, Yosys 0.33)

| Suite | Result |
|---|---|
| `tb_bridge_top` (3×2 sessions, 24 beats) | 144/144 TLPs, 6/6 MSI-X, 0 replays |
| `tb_bridge_top +SESS=6 +BEATS=50` | 600/600 TLPs, 12/12 MSI-X |
| `tb_bridge_ovf`, `tb_ingest_overflow_leak` | PASS, 0 leaked beats |
| ASIC profile (replay 8, FIFOs 8): `tb_bridge_top`, `tb_bridge_ovf`, `tb_stream_path` with bit errors | PASS |
| Link `run_all_link.py` | 15/16 (only `tx_engine`: no self-check) |
| Interconnect unit benches | 4/4 |
| PHY `run_all.py` / cocotb | 24/24, 9/9 |
| Yosys elaboration of `bridge_top` (56 files) | 0 latches; 2 cosmetic warnings (P2-7) |
