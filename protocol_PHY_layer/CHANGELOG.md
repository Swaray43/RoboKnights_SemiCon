# Protocol-to-PHY layer: review fixes

This change set covers Kshitij's items from the *AXI4–PCIe6 Bridge Review*: **P1–P5**, **I2**, **I3**, the PHY side of **Seams B, C and D**, and the testbench bugs. Every bench runs in Icarus Verilog 12.

## Results

| | Before | After |
|---|---|---|
| Verilog benches (`python run_all.py`) | 19 pass, `tb_full_system_integration` 5 FAIL, `tb_pam4_roundtrip` won't build | **24 / 24 pass** (incl. new `tb_pcie_phy_top`, 83 checks); every bench ends with `RESULT: PASS` |
| cocotb (`cocotb/run_tests.py`) | 4 / 4 on `pcie_link_top` | **9 / 9** on `pcie_phy_top` (4 original + 5 new) |
| Synthesis (`yosys synth -top pcie_phy_top`) | no top existed | clean, 0 latches, 0 `check` problems |
| Formal | `gray.sby` OK; `pam4.sby` and `verify_reset.sby` pointed at missing files | `gray.sby`, `pam4.sby`, `verify_sof_strip.sby` all PASS |
| PHY drain rate | n/a (no top) | **260 clk per 256-byte Flit** (floor 256 + 4 for the SOF byte) |

## Fixes by review ID

| ID | What changed | Where |
|---|---|---|
| **P1** | `lane_symbol_pipe` split into `lane_tx` (byte → symbols → Gray → PAM4, registered level out) and `lane_rx` (level in → slicer → Gray decode → byte). The top exposes `tx_level[11:0]` / `tx_level_valid[3:0]` and `rx_level` / `rx_level_valid` (lane *i* = `[3*i +: 3]`, signed −3/−1/+1/+3). The ideal channel now lives only in the testbench; `lane_symbol_pipe` survives as a sim model in `tb/models/` so the old benches still run. | `rtl/lane_tx.v`, `rtl/lane_rx.v`, `tb/models/lane_symbol_pipe.v` |
| **P2** | New synthesizable `pcie_phy_tx` / `pcie_phy_rx` / `pcie_phy_top`. TX inserts the 0xFB SOF itself (it used to come from the testbench's `sof_phase`). RX rebuilds `sof`/`eof` per Flit for `link_layer_rx`. | `rtl/pcie_phy_*.v` |
| **P3 / Seam C** | Flit FIFO (16 × 256 b) with a valid/ready handshake: `phy_tx_ready`. A lane scheduler feeds all four lanes in lockstep, one byte every 4 clocks, which is the full 2 bits/clk/lane. The deskew FIFO now reports `room_ok`, so the striper can't overflow it. Before, `elastic_fifo` dropped bytes silently when full. | `rtl/pcie_phy_tx.v`, `rtl/tx_elastic_deskew.v` |
| **P4 / Seam B** | New BAR0 register block holding `base_addr_ch0/1` (0x20/0x28), arm bits (0x00), weights (0x08), session limit (0x0C), MSI-X vector per channel (0x10), TC (0x18), status and debug. It's driven by a `bar0_*` port, from cocotb for now, until an inbound MWr path exists. Register map is in the file header. | `rtl/bar0_dma_regs.v` |
| **P5** | One module per file in `rtl/`, a single list in `files.f`, testbenches in `tb/`, proofs in `formal/`. Duplicates removed (copies came from 7 files). Gray ports standardized to `(a, g)` / `(g, a)`. The `ifdef FORMAL` block from `pci_stuff.v`'s copy of `flit_sof_strip` was kept, moved below the declarations so it elaborates. | whole tree |
| **I2** | Kshitij's `axi4s_ingest_port` (16-bit offsets) and its `session_overflow_ctrl` were renamed `legacy_*` and moved to `legacy/`, off the file list. Aditya's 24-bit `axi4s_ingest_port` is the one that stays. All three layers can now compile together. | `legacy/` |
| **I3** | `session_valid` (a level on `clk`) → rising edge → toggle synchroniser → `session_start_aclk`, a 1-cycle pulse on `aclk`. The overflow flags get a 2-flop sync into BAR0 STATUS. **Session end (changed by K1, see Update 8):** a session now ends on the link layer's `session_done`, not on `session_end_aclk`; `session_end_aclk` is crossed to `clk` only as a debug bit (BAR0 STATUS[5:4]). | `rtl/pulse_sync.v`, `rtl/pcie_phy_top.v` |
| **Seam D** (PHY side) | `msix_controller` now uses valid/ready (`mwr_ready`), so the MWr is held until the link layer takes it. A completion that arrives while busy goes to PBA instead of being lost. Pending vectors are sent when unmasked and their PBA bit clears, per the PCIe rule. `session_done[1:0]` from the link layer picks that channel's vector from BAR0. | `rtl/msix_controller.v`, `rtl/pcie_phy_top.v` |
| **Tests** | `tb_full_system_integration`: `sext_rst_n` → `ext_rst_n`; drives `DEAD_BEEF…` to match the check; loopback moved into the testbench (`lane_tx` → wires → `lane_rx`); `default_nettype none` added to catch the next typo; leftover `$monitor` removed. Result: 14 pass, 0 fail. `tb_pam4_roundtrip` now builds. | `tb/` |

## Update 2: training over the lanes

*(Update 6 replaces the TS1-only scheme below with a full TS1→TS2 LTSSM. `ts1_os_gen` became `ts_os_gen`, and `TS_AFTER_UP` was removed.)*

The link now trains over the channel itself. Before, the TS1 bytes had to be injected on the `ts_byte_*` ports.

| What | Where |
|---|---|
| **TX TS1 generator.** From reset until `link_trained`, the lanes send TS1 ordered sets: COM `0xBC`, link 0, lane number = physical lane, then `0x4A` ×6. That's the same format the detectors already expect. Ordered sets are always sent whole. 2 extra are sent after link-up (`TS_AFTER_UP`), so a slower partner also sees two in a row. Flits that arrive during training wait in the Flit FIFO. New output: `phy_ts_sending`. | `rtl/ts1_os_gen.v` (new), `rtl/pcie_phy_tx.v` |
| **Detectors read the received lanes.** The 4 TS1/TS2 detectors take `pcie_phy_rx`'s raw lane bytes *before* the lane swap, so a crossed channel shows up as lane numbers 3,2,1,0 and `lane_reversed_locked` sets on its own. `ts_byte_*` stays as an injection override while `ts_byte_valid` is high, and the 4 original cocotb tests still use it. | `rtl/pcie_phy_top.v`, `rtl/pcie_phy_rx.v` |
| **Symbol lock in `lane_rx`.** While the link isn't up, a sliding 8-bit window snaps the byte boundary to the COM symbol. This handles a channel that comes up partway through a byte. In TS1, `0xBC` only appears on a true byte boundary, and once the link is up the boundary is frozen, so `0xBC` in Flit data is harmless. With `align_en` tied low it behaves exactly as before. | `rtl/lane_rx.v` |

New tests:
- `tb_pcie_phy_top` A1/B1 now train over the lanes, with no injection.
  - Straight channel: trains in 135 clk.
  - Crossed channel: reversal is detected from the lane numbers alone.
- Part D: Flits pushed before link-up wait behind TS1 and arrive bit-exact.
- Part E: the channel connects 37 clk after reset, mid-byte. Lock on COM, training and data all succeed. The same test fails if alignment is disabled.
- The cocotb datapath tests now start the channel model first and train over it.

Note: the `ts_byte_*` injection port trains the link without aligning the lanes. Use it only for sideband tests where nothing goes through the lanes, as the original 4 cocotb tests do.

## Update 3: config-space gating

| What | Where |
|---|---|
| **Command register bit fix.** Memory Space Enable is now bit 1 and Bus Master Enable bit 2, as in the PCI spec. Before, they were bits 0 and 1, and bit 0 is actually I/O Space. The port comments already said bit1/bit2. **Hosts must now write `0x6`, not `0x3`, to enable both.** | `rtl/pcie_config_space.v` |
| **BAR0 gated on Memory Space Enable.** While it's 0, BAR0 writes are ignored and reads get no `bar0_rd_valid` (unsupported request), as for a real function. | `rtl/pcie_phy_top.v` |
| **MSI-X gated on Message Control.** With MSI-X Enable = 0, completions are dropped. With Function Mask = 1, they set PBA bits; clearing the mask sends them. | `rtl/msix_controller.v` |
| **MSI-X table and PBA live in BAR0.** Config 0x54 = `0x1000` and 0x58 = `0x1800`, both BIR 0. The table and PBA are reachable there with DWORD reads/writes, using the standard 16-byte entry (addr lo, addr hi, data, vector control). The `tbl_*` port still works. | `rtl/pcie_config_space.v`, `rtl/msix_controller.v`, `rtl/pcie_phy_top.v` |

**BAR0 map (64 KB):** `0x0000–0x0FFF` DMA/session regs · `0x1000–0x17FF` MSI-X table · `0x1800–0x1FFF` PBA. **`bar0_addr` is now 16 bits** (was 12).

Tests:
- Existing benches and cocotb tests now enable Memory Space and MSI-X first, the way a driver would. BFM: `enable_function()`.
- Unit benches tie `msix_enable=1` / `func_mask=0`.
- New `tb_pcie_phy_top` Part F checks:
  - offsets advertised in config space;
  - a vector programmed through BAR0 fires correctly;
  - function mask pends, and clearing it drains;
  - MSI-X disabled drops the event;
  - Memory Space off blocks BAR0.
- New cocotb test: `test_msix_table_through_bar0`.

## Update 4: RX lane deskew

| What | Where |
|---|---|
| **New `rx_lane_deskew`.** It sits between the 4 `lane_rx` blocks and the lane swap. During training, each lane starts filling its own 16-byte FIFO at its first COM (`0xBC`). Once all 4 have armed, every FIFO head is the same COM, so it locks and from then on reads all 4 lanes together. That removes the channel's lane-to-lane skew. If one lane gets more than 8 bytes ahead before all have armed, they armed on different ordered sets: it clears and retries on the next COMs. Alignment is frozen at link-up. | `rtl/rx_lane_deskew.v` (new), `rtl/pcie_phy_rx.v` |
| **TS detectors now read deskewed bytes** (still before the lane swap). `lane_reversal_detector` needs all 4 lanes' ordered sets in the same cycle, so with any skew the link could never train without this. | `rtl/pcie_phy_top.v`, `rtl/pcie_phy_rx.v` |
| **Status changes.** New output `phy_deskew_locked`. `phy_lane_err` now means "a deskew FIFO overflowed after lock"; it used to mean "lane valids didn't match". | `rtl/pcie_phy_top.v` |

Tests: the `tb_pcie_phy_top` channel model now has a separate delay per lane. New Part G covers:
- skew of 0/5/13/22 clk;
- 30/0/17/3 clk;
- 9/27/1/14 clk on top of crossed lanes.

All three deskew, train (in ~160–170 clk), detect reversal correctly, and deliver 4 Flits bit-exact, including the all-`0xFB` words.

## Update 5: hot reset retrains the link

| What | Where |
|---|---|
| **Training chain reset moved from PERST# to `datapath_rst_n`.** That covers the 4 TS detectors, the reversal detector and the latch. A hot reset already pulses `datapath_rst_n`, so `link_trained` now clears: TX sends TS1 again, RX re-locks and re-deskews, and the link retrains over the lanes. `axi_rst_n` stays low until a fresh UpdateFC, which the reset controller already enforced. Before, `link_trained` stayed high after a hot reset, TX never resent TS1, and the receiver came back un-aligned. **No deadlock:** `datapath_rst_n` is released by the reset controller on its own and never waits for `link_trained`. | `rtl/pcie_phy_top.v` |

Test: new `tb_pcie_phy_top` Part H, run with lane skew.
1. Link up, AXI released, data passes.
2. Hot reset: `link_trained` drops, TS1 resumes, AXI goes into reset.
3. The link retrains in 155 clk. AXI waits for UpdateFC.
4. Flits pass bit-exact again.

With the old PERST#-only reset, the link never retrains and the data after the hot reset fails.

Like a real hot reset, it also resets config space and the BAR0 registers. In-flight Flits are flushed, so the host must reprogram before the next session.

## Update 6: LTSSM with TS1 and TS2

| What | Where |
|---|---|
| **New `pcie_ltssm`:** a simplified PCIe link-training state machine. **Detect** keeps TX quiet for 32 clk; it stands in for receiver detection, which happens in the analog SerDes. **Polling** sends TS1 until 8 ordered sets have been received on all lanes. **Configuration** sends TS2 until 8 TS2 have been received, at least 16 TS2 have been *sent* since the first one arrived (the spec's rule, so the partner sees enough of ours), and the lane order is resolved. Then it enters **L0**. If Polling or Configuration doesn't finish within `TIMEOUT_CLKS` (16384), it goes back to Detect. `link_trained` now means "LTSSM in L0". | `rtl/pcie_ltssm.v` (new) |
| **TS2 on the TX lanes.** `ts1_os_gen` is replaced by `ts_os_gen` (TS1 ID `0x4A`, TS2 ID `0x45`); the type is fixed at byte 0, so an ordered set is never mixed. TX takes `os_mode` from the LTSSM and pulses `os_done` per ordered set. **Flit data now only leaves in L0** (`data_en`); before, in Detect, it could leak out before the receiver had aligned (the new test caught this). | `rtl/ts_os_gen.v` (new), `rtl/pcie_phy_tx.v` |
| **The detectors report every ordered set.** New outputs `os_valid` / `os_ts2` on `ts_ordered_set_detector`, which drive the LTSSM's counts. The existing ports and behaviour are unchanged. | `rtl/ts_ordered_set_detector.v` |
| **Top.** New output `ltssm_state` (0 Detect, 1 Polling, 2 Configuration, 3 L0) and parameters `LTSSM_DETECT_CLKS`, `LTSSM_RX_TS_MIN`, `LTSSM_TX_TS2_MIN`, `LTSSM_TIMEOUT_CLKS`. The latch's output now means "lane order identified", which is a Configuration exit condition. | `rtl/pcie_phy_top.v` |

Training now takes about 1,700 clk (8 TS1 + 16 TS2 at 64 clk each), up from about 135. A hot reset goes back to Detect and retrains the same way.

Tests:
- `tb_pcie_phy_top` A1: checks the state order Detect → Polling → Configuration → L0, with no illegal jumps, and that ≥8 TS1 and ≥8 TS2 arrived on all lanes.
- Part D: Detect keeps the lanes quiet, and queued Flits wait for L0.
- Part H: a hot reset returns to Detect, then Polling.
- New Part I: with no partner, Polling times out to Detect and retries; once a partner appears, the link trains.
- cocotb BFM `train_link()` (injection path) now plays the partner: it waits out Detect, sends 8 TS1, then TS2 until L0. The 4 original tests are unchanged and pass.

## Update 7: test runner hardening

**Real cause of the `pass=0` / `0xC0000005` failures:** PATH order. The Yosys install (OSS CAD Suite) ships its own `iverilog`/`vvp`, and it came before the standalone Icarus install, so its `vvp` was the one running and it crashed. **Put the Icarus install first on PATH**, or check with `where vvp` (Windows) / `which vvp`.

These changes don't fix that, but they're kept because they make the benches more robust and failures easier to read:
- **Every testbench now ends with `$finish` instead of `$stop`.**
- **Common timescale.** `tb/timescale.v` (`` `timescale 1ns/1ps ``) is compiled first for every bench, the same setting `tb_pcie_phy_top` and cocotb already used.
- **`run_all.py`:**
  - prints the Icarus version it found at the start;
  - runs `vvp` with stdin closed;
  - reports `vvp`'s exit code (in hex) when there's no output;
  - prints the last log lines of any failing bench.

## Update 8: tickets K1–K5 and integration into `bridge_top`

This update brings the Update 2–7 PHY (lane training, LTSSM, deskew, config gating, hot-reset retrain) into the integrated repo's `protocol_PHY_layer/`. The previously pushed version was the first one, with injection-only training, and it also carried Aditya's session-end fix. That fix is kept.

| Ticket | Resolution | Where |
|---|---|---|
| **K1** | **Kept Aditya's fix:** `session_arm_ctrl.session_end` = `session_done` from the link layer. **`session_end_aclk` port kept** as a debug signal rather than removed, so no other layer's ports change. It's crossed to `clk` and drives `ic_end_seen`, shown in **BAR0 STATUS[5:4]**: "the interconnect finished accepting; data still draining". It clears when the session ends or on a new arm. The synchronisers now drive something, so there's no dead logic or lint warning. **BAR0 STATUS `session_valid` is now documented** as clearing once the session's data has left the link layer, which is when it's safe to re-arm; an arm while it's still 1 is ignored and counted in ARM_DEBUG. The I3 entry above is updated. **New check** in `tb_pcie_phy_top` A5: an interconnect end alone doesn't end the session, STATUS shows draining, an early arm is ignored and counted, and `session_done` ends the session and clears the draining bit. | `rtl/pcie_phy_top.v`, `rtl/bar0_dma_regs.v`, `tb/tb_pcie_phy_top.v` |
| **K2** | `legacy/pre_restructure/` deleted (see "Files deleted"). | — |
| **K3** | `legacy/cocotb_old/` deleted: `pcie_link_top.v` (floating `mwr_ready`) and `cocotb.runner.py`. `rtl/pcie_phy_top.v` + `cocotb/run_tests.py` replace them. | — |
| **K4** | `cocotb/test_pcie_link_top.py` → **`cocotb/test_pcie_phy_top_base.py`**. `run_tests.py` and `Makefile` are updated. | `cocotb/` |
| **K5** | The old `tb_*system_integration.v` copies with the `sext_rst_n` typo went with `legacy/pre_restructure/`. The fixed copies are in `tb/`. | — |
| **Convention** | Every PHY bench (and the two legacy ones) now ends with `RESULT: PASS` / `RESULT: FAIL`. `run_all.py` treats a missing `RESULT` line as a failure, the same rule as the link layer's `run_all_link.py`. | `tb/*.v`, `legacy/tb_*.v`, `run_all.py` |

**Integration into `bridge_top`: to-dos for Aditya and Swaray (not applied here).** This push only changes `protocol_PHY_layer/`. The new PHY needs a few edits in the other layers' files before `bridge_top` builds and passes again. Each one is listed with its exact diff in **`integration/BRIDGE_TODO.html`**, and all of them are in **`integration/bridge_integration.patch`**:
- `bridge_top.f` gets the 3 new files.
- `bar0_addr` goes to 16 bits in `link_phy_bridge_top.v` and `bridge_top.v`.
- `bridge_top.v` exposes the config port.
- The two bridge benches wait for link-up over the loopback and enable Memory Space and MSI-X.

Applying it, from the repo root: `git apply protocol_PHY_layer/integration/bridge_integration.patch`.

Tested with the patch applied (Icarus 12):
- `tb_bridge_top`: 144/144 TLPs, 6/6 MSI-X, `RESULT: PASS`. Same with `+SEED=2`, `+SEED=7 +GAP=0`, `+REARM_FAST=0` and `+SESS=6 +BEATS=50` (600/600).
- `tb_bridge_ovf`: 39/39 TLPs, 0 leaked, `RESULT: PASS`.
- Yosys elaboration of `bridge_top`: 0 problems.

Without the patch, `bridge_top` doesn't build: `pcie_ltssm`, `rx_lane_deskew` and `ts_os_gen` aren't on its file list.

## Update 9: ticket K6, leaving L0 on link loss ("Recovery-lite")

**Problem:** once in L0, only `rst_n` (PERST# or hot reset) left it. If the receiver lost the signal, `link_trained`, `axi_rst_n` and the sessions all stayed high and data silently stopped.

**Why "count `rx_level_valid` low" alone was not enough.** Two things in the PHY had to change first:
1. **The TX lanes went quiet whenever there was no Flit data in L0.** `tx_level_valid` dropped to 0 when idle, so in loopback `rx_level_valid` low was the *normal* idle state; a counter would have dropped every idle link after 64 clocks.
2. **Retraining needs a reset.** `rx_lane_deskew.locked` and `lane_reversal_latch` only clear on reset, and the link layer, which runs on `datapath_rst_n`, keeps its sequence, replay and credit state otherwise. A plain L0 → DETECT transition could not retrain, and stale DLL state would survive the outage.

| Change | Where |
|---|---|
| **Logical idle in L0.** Every byte slot without a Flit byte carries `IDL_BYTE = 0x1B` on all 4 lanes, so the lanes are never quiet in L0 (the PCIe rule: IDLE Flits / logical idle). `0x1B` = symbols 0,1,2,3, so none of its symbol rotations (`0x6C`, `0xB1`, `0xC6`) is itself or SOF, and a slipped alignment can't hide. Idle never goes *inside* a Flit: `fl_left` counts payload lane-cycles exactly as `flit_sof_strip` does at the far end. A mid-Flit producer underrun stays quiet for up to `UNDERRUN_SLOTS = 8` slots, then sends IDL filler that counts as payload, so TX/RX framing stays in step and the far end doesn't see a dead link. That Flit is corrupt and link-layer CRC/replay recovers it; it never happens with `link_layer_tx`, which sends Flits back to back. `tx_idle` now means "no Flit data anywhere" rather than "lanes quiet". Parameter `L0_IDLE_FILL = 1`. | `rtl/pcie_phy_tx.v` |
| **Between-Flit check on RX.** `flit_sof_strip` has a new `hunting` output. `pcie_phy_rx` outputs `gap_ok` (between Flits, all lanes IDL or all SOF) and `gap_bad` (anything else). | `rtl/flit_sof_strip.v`, `rtl/pcie_phy_rx.v` |
| **LTSSM L0 exits.** After the RX is *armed* (first `gap_ok` in this L0), L0 → DETECT on: **(1) signal loss**, any lane's `rx_level_valid` low or `lane_err` for `LOSS_CLKS = 64` consecutive clocks; **(2) lost framing**, `GAP_ERR_MAX = 512` unframed between-Flit lane cycles with no clean one in between, which catches a short glitch that slipped the frozen byte boundary; **(3) partner retraining**, `L0_TS1_EXIT = 2` TS1s received on all lanes. TS2s are not counted, because the partner's last Configuration TS2s legitimately arrive after L0 entry. Before arming, exits 1 and 2 are off, so training through the `ts_byte_*` injection port with no lane traffic, as the 4 original cocotb tests do, never drops. Debug outputs: `l0_exit` pulse, `exit_reason` (1/2/3), `rx_armed` (wires `ltssm_l0_exit` / `ltssm_exit_reason` / `ltssm_rx_armed` in `pcie_phy_top`; no top-level port changes). `ltssm_state` is still 2 bits. | `rtl/pcie_ltssm.v`, `rtl/pcie_phy_top.v` |
| **Link down resets the datapath.** When `link_trained` falls while LINK_UP and no hot reset is pending, `datapath_rst_n` pulses low for 3 clocks, exactly like a hot reset (an Upstream Port going DL_Down is reset). Deskew, lane order, link layer and function state start clean, the LTSSM retrains from DETECT, and `axi_rst_n` is held until re-training plus a fresh UpdateFC, as before. No deadlock: the pulse starts on a falling edge, ends on its own and never waits for `link_trained`. Parameters `LINKDOWN_RESET = 1`, `LINKDOWN_PULSE = 3`. | `rtl/pcie_reset_controller.v` |
| **New top parameters** (defaults shown): `LTSSM_LOSS_CLKS = 64`, `LTSSM_GAP_ERR_MAX = 512`, `LTSSM_L0_TS1_EXIT = 2`, `L0_IDLE_FILL = 1`. `L0_IDLE_FILL = 0` gives the old quiet lanes and also turns exits 1 and 2 off. Exit 3 and the link-down reset stay; with this setting Parts A–I pass unchanged. | `rtl/pcie_phy_top.v` |

**Why these thresholds survive channel noise.** Noise flips levels, not `rx_level_valid`, so exit 1 is immune. For exit 2, a noise hit that loses one SOF costs one Flit (at most 65 unframed cycles), and the count restarts at the next clean cycle. It would take about 8 SOFs lost in a row to reach 512. The rand suite's `+NOISE_PPM=1000` and `+NOISE_PPM=300 +BURST=4` seeds pass with no link drop (see below).

**New tests**
- `tb_pcie_phy_top` **Part K** (33 checks; bench now 118 checks):
  - **K0**: lanes carry idle in L0 and the RX arms.
  - **K1**: a 32-clk dropout (< `LOSS_CLKS`, whole bytes) is tolerated, with no reset, the session kept and data bit-exact.
  - **K2**: a 7-clk dropout slips the symbol alignment. It's detected as lost framing about 2,060 clk later, then the link resets, retrains and data is bit-exact.
  - **K3**: cable pull mid-session with Flits in flight. L0 is left 65 clk later on signal loss. There's one datapath reset pulse; `axi_rst_n` goes low; sessions and the Command register are cleared. While pulled, Polling keeps sending TS1 with no reset storm. With the cable back, the link retrains (about 1,700 clk), AXI waits for a fresh UpdateFC, the host re-configures, there's one new session start and data is bit-exact with nothing stale.
  - **K4**: one lane lost drops the link, which retrains when the lane returns.
  - **K5**: 3 TS2s in L0 keep the link up; 2 TS1s drop it, and it retrains.
  - **A6** now also checks the lanes are never quiet in L0 (including producer stalls) and that there's no spurious L0 exit.
- `tb_pcie_reset_controller` Test 5: an unexpected link-down pulses `datapath_rst_n` for 2–4 clk, which then releases on its own.
- cocotb **`test_link_loss_retrain`**: cable pull via the Python channel (`bfm.lane_cut`). Link down within 72 clk, AXI reset, sessions cleared, stays down while pulled; cable back → retrain → bit-exact data. cocotb is now **10 / 10**.

**Cost**
- About +400 cells (+1.8%) in `yosys synth -top pcie_phy_top`, 0 latches, 0 `check` problems.
- A Flit byte may wait up to 3 clk for the next byte slot: A6 measures 10,418 clk for 40 Flits versus 10,416, so the drain rate is still 260 clk per Flit.
- Wire-level counters such as `bridge_top/tb/perf_monitor.v`'s PAM4-symbol stage (`hit(4)`, `tx_level_valid`) now include idle bytes and read as line rate throughout L0.

**For others (not applied, outside `protocol_PHY_layer/`)**
- **`ci/protected_lines.txt`:** add the K6 rules in `integration/K6_protected_lines.patch` (`git apply` from the repo root).
- **Aditya, F7b in `tb_bridge_rand`:** pull the cable by masking the loopback valid (`.rx_level_valid(lvl_v & ~cut)`). Expect `link_trained` to fall within about 70 clk and `datapath_rst_n` to pulse (the same as F7, so the F7 recovery path applies). Keep `cut` until you want the link back. Training resumes on its own, so re-configure after `link_trained` returns. The header line "Not modelled ... link down without hot reset" can go.
- **Aditya / Swaray, `tb_bridge_rand` far above CI noise (FYI, pre-existing, not K6):** at `+NOISE_PPM=3000` or `+NOISE_PPM=2000 +BURST=3`, F8 ("after unmask n of m MSI-X") and F9 (inbound BAR0 write not read back) can fail. **The original step3 RTL fails the same way:** baseline `+SEED=16 +NOISE_PPM=2000 +BURST=3 +SESS_MAX=3 +BEATS_MAX=15` gives F8 "after unmask 0 of 2 MSI-X". With K6 there were no link drops (`pcie_ltssm` logs every one) and no framing errors. In K6 seed 12 at 3000 ppm the late MSI-X did arrive (10/10). In K6 seed 14 (2000 ppm, burst 3) a probe showed the inbound MWr to BAR0+0x28 never reached the RX link layer within F9's 2M-clk wait, while other TLPs kept flowing. The ch1 session then ran on the old base, so 13 TLPs show as "mismatched" by address with correct data. That points at link-layer replay or flow control under heavy burst errors, not the PHY (every Flit is CRC-64 checked). All CI noise settings pass.
- **`perf_monitor.v` stage 4 (PAM4 symbols)** now counts logical idle, so it reads line rate throughout L0. If you want Flit-only wire bytes, count only cycles outside idle (the TX `byte_is_idl` flag) or use stage 3.

## Design decisions to confirm

1. **SOF once per 256-byte Flit (the PCIe meaning).** TX inserts SOF before each word that arrives with `flit_sof=1`, and RX expects `WORDS_PER_FLIT=8` words after each SOF. `flit_sof_strip` gained `PAYLOAD_CYCLES` (8 = old per-word behaviour, which is still the default for its unit test; 64 in the top). It also gained `RESYNC_MIDFLIT=0` in the top, so payload containing 0xFB on all 4 lanes no longer resyncs. That was the old "known limitation", and the benches now push all-0xFB words through to prove it.
2. **Seam C uses one clock and a ready signal**, not a separate symbol-clock domain.
3. **`pcie_link_top` is replaced by `pcie_phy_top`.** Every old port keeps its name, so the 4 original cocotb tests run unchanged. Only the BFM's `reset()` drives the new inputs.
4. **`mwr_ready` is a new required input.** Tie it to 1 if the link layer can always take the MSI-X write.

## What other people need to do

**Swaray (link layer)**
- `link_layer_tx` must **stall its output word counter while `phy_tx_ready` is low**. This is the second half of Seam C. The PHY drains a Flit in 260 clocks; the link writes one in 8.
- Drive `phy_tx_sof` on word 0 and `phy_tx_eof` on word 7 of each Flit. `phy_framing_err` latches if not.
- `link_layer_rx` gets `phy_rx_data/valid/sof/eof` and must accept a word on every `phy_rx_valid`. There is no RX backpressure.
- Base addresses: `cfg_base_addr_ch0/1` plus a `cfg_base_addr_wr[1:0]` pulse when a session arms.
- MSI-X (L10): take `mwr_valid/addr/data` with `mwr_ready`, and drive `session_done[1:0]` once the tlast TLP of a session has been issued.

**Aditya (interconnect)**
- `session_start_aclk[1:0]`: a 1-cycle pulse on `aclk` per channel.
- Return `session_overflow_aclk[1:0]` (sticky levels). `session_end_aclk[1:0]` (tlast accepted or overflow abort, a pulse on `aclk`) is still an input but **debug only** since K1: sessions end on the link layer's `session_done`.
- Weights and limit are parameters `WW=8`, `LW=24` on `pcie_phy_top`. **Please confirm they match `interconnect_top`.** They are quasi-static: written before the arm bit, never changed during a session.

## Not done / known gaps

- **The LTSSM is simplified.** Not modelled: a real Recovery state (K6 leaves L0 through DETECT with a datapath reset, i.e. DL_Down, instead of a TS1/TS2 exchange that keeps the link up), L0s/L1/L2 power states, speed change / equalization, and link/lane-number negotiation (lane numbers are fixed per physical lane).
- **Lane skew must stay under 32 clk (8 bytes) between the earliest and latest lane.** COM repeats every 16 bytes, so larger skew aliases onto the wrong ordered set and can't be detected from TS1 alone. Real PCIe skew limits are far smaller.

## Files deleted (now in `rtl/`, `tb/` or `formal/`)

`pcie_phy_datapath.v`, `pcie_phy_datapath_v0.v`, `end_pipeline.v`, `pci_stuff.v`, `flit_stripper.v`, `gray_enc_dec.v`, `pam4.v`, `reset.v`, `pam_specy.v`, `pcie_tag_man.v`, `pcie_config_msix.v`, `pref_mon.v`, `tb_config_msix.v`, `pcie_link_top.v` and `cocotb/pcie_link_top.v` (both replaced by `rtl/pcie_phy_top.v`), `verify_reset.sby` (replaced by `verify_sof_strip.sby`), plus `cocotb/sim_build/`, `results.xml` and `__pycache__/`.

**K2/K3/K5 (Update 8):** these really are gone now. `legacy/pre_restructure/` (the files above, which had been left in the repo root and then parked there) and `legacy/cocotb_old/` (`pcie_link_top.v`, `cocotb.runner.py`) are deleted. Run `git rm -r protocol_PHY_layer/legacy/pre_restructure protocol_PHY_layer/legacy/cocotb_old` and check `git status` before pushing, so the deletions reach the repo.

## How to run

```sh
python run_all.py                  # all 24 Verilog benches (Icarus); Windows/macOS/Linux
python run_all.py tb_pcie_phy_top  # one bench
./run_all.sh                       # same thing, wrapper for bash users
cd cocotb && python3 run_tests.py  # 10 cocotb tests (or: make)
cd formal && sby -f gray.sby       # also pam4.sby, verify_sof_strip.sby
```

Other simulators can use `files.f`. Benches add `tb/models/*.v` and `tb/pcie_reset_controller_checker.v`.
