# Phase 2 · Step 4 — Final optimisation cycle

> **Status (implementation):** Wave A + B done and verified (tag `waveAB`); Wave C in progress (CRC-64 split, TX merge-unit restructure already in). Register map changes and as-built notes are in section 7.

Branch: `phase2/step4`, created from tag **`step4-base`** (= commit `49a467b`, the merged main + Vivado summary script).
This is the last cycle of RTL changes before the GDSII flow. Whatever is green at the freeze becomes tag `step4-final`, the GDSII input.

---

## 1. Starting point (`step4-base`, Vivado 2026.1, xcu250, place & route)

| Metric | Value |
|---|---|
| LUT / FF | 56,043 / 24,482 (3.2% / 0.7%) |
| BRAM / URAM / DSP | 0 / 0 / 0 (shallow memories correctly in LUT RAM) |
| clk WNS (target 4.0 ns = 250 MHz) | **−4.44 ns** → slowest path 8.44 ns → Fmax ≈ 118 MHz |
| clk failing endpoints / TNS | 19,046 / −15,965 ns |
| aclk (185 MHz) | +1.01 ns, 0 failing |

| Module | Failing endpoints | Worst slack |
|---|---:|---:|
| `link_layer_rx` | 3,087 | −4.44 |
| `link_layer_tx` | 435 | −3.27 |
| `rx_tlp_delimiter` | 3,954 | −2.71 |
| `u_msix` | 1,626 | −1.27 |
| `tlp32_to_axis_packer` | 2,053 | −1.27 |
| `write_data_buffer` / `replay_buffer` | ~220 | −1.0 to −1.6 |
| `stream_tx_path_top` glue | 522 | −0.98 |

Throughput today (simulation): about 20 clocks per 64 B beat in the TX packetizer, 1 symbol per lane per clock in the PHY, 1 DW per clock in the RX decode, one 16 B header per 64 B beat (80% payload).

---

## 2. Requirements

### Wave A — independent, start immediately, in parallel

| # | Requirement | Specification | Done when | Owner |
|---|---|---|---|---|
| **T1** | Pipeline the RX CRC-64 / FEC check | Split the check in `link_layer_rx` into 2–3 register stages (mirror of the TX pipelining). `rx_flit_ok`, `crc_err`, `fec_*`, ACK/NAK generation and replay follow by the same number of clocks. | The −4.44 ns path is gone; link, errors, bridge, rand, long pass | Swaray |
| **6** | Reliability: sticky error counters in BAR0 | Add `perf_counters.v` to the build. BAR0 0x30–0x5C: CRC fail, FEC corrected, FEC uncorrectable, replay, NAK sent, malformed TLP, link-down (K6), hot reset, overflow ch0, overflow ch1. 16-bit saturating; a write clears the register. XLAT_MISS stays at 0x1C. Hot-reset and link-down recovery are already done (F7, F7b, K6). | Directed bench for every counter; `tb_bridge_rand` reads them at the end and checks them against its own counts; guard rules added | Kshitij (PHY counters, BAR0 map) + Swaray (link counters) |
| **7** | Interconnect upgrade | (a) Per-channel session limit: BAR0 0x0C = ch0 (unchanged), new 0x34 = ch1. (b) `session_end[1:0]` 1-clock pulse per channel, out of `interconnect_top` to `bridge_top`. (c) Interleaved weighted round-robin: one beat per grant, a weight is the number of turns per round, instead of a burst of `weight` beats. | Interconnect unit benches extended; rand passes; Step 5 mixed workload shows the bandwidth split follows the weights within 5 points and NIC p99 latency falls vs today at 1:7 | Aditya |

### Wave B — datapath widening (the long pole; order matters)

| # | Requirement | Specification | Done when | Owner |
|---|---|---|---|---|
| **3** | Widen the PHY lane interface | Parameter `SYM_PER_CLK` (1 = today, 4 = target): PAM4 map/demap, Gray, lane TX/RX, striper/destriper, deskew, ordered-set detector, LTSSM counts (in symbols). Default stays 1 until every PHY bench passes at 4. | PHY 24/24 and cocotb 10/10 at `SYM_PER_CLK=4`; bridge benches pass with it; 4× link bytes per clock | Kshitij |
| **1** | 512-bit TLP packetizer | Header + payload formed directly on 512-bit beats. The 512→32 serialiser and `tlp32_to_axis_packer` are removed. One beat per clock into `flit_payload_builder`. 4 KB splitting, MSI-X ordering and credits unchanged in behaviour. | `tb_stream_path` (errors on), bridge, rand, long pass; TX accepts one beat per clock when credits allow | Swaray |
| **8** | Widen the RX decode (absorbs **T2**) | 512-bit unpacker, header parser working on 16 DW, multiple DW per clock into `pcie_rx_engine`; the `rx_tlp_delimiter` pointer is registered as part of the rewrite. Required, not optional: in loopback RX carries our own TX traffic, so once 1 + 3 land a 1 DW/clock RX becomes the bottleneck and causes NAKs. | Same suites pass; no NAKs on a clean link at full rate | Swaray |
| **2** | Combine beats into max-payload TLPs | Merge consecutive beats of one session into a TLP up to Max Payload (256 B, `MAX_PAYLOAD_DW`), never across a 4 KB boundary, never past the session limit. Flush on `tlast`, a channel switch, or an idle timeout (parameter). MSI-X still after the session's last data. | Reference model in the bridge benches rewritten for merged TLPs; all suites pass; Flit payload efficiency ≈ 94% (from ≈ 80%) | Swaray (RTL) + Aditya/Claude (reference model) |

Order: **3** and **1** start with Wave A. **8** follows **1**. **2** follows **1** and **8**.

### Wave C — after the widening

| # | Requirement | Done when |
|---|---|---|
| **T3–T7** | Vivado on the widened design; fix in histogram order: TX FEC, packing, MSI-X table/arbiter, LUT RAM write enable/address, reset fan-out on datapath registers | WNS at the agreed target, or the target set from data with the reason recorded here |
| **Step 5 report** | Before/after workload table (NIC, ML, mixed) for every item above | Table complete in `step5_results/` and in the submission |
| **Freeze** | Full regression + nightly sweep + coverage + final Vivado; tag `step4-final` | GDSII flow starts from that tag |

---

## 3. Rules for every Step 4 commit

1. One requirement (or one sub-step of it) per commit. Message starts with the ID: `S4-1: 512-bit packetizer, remove serialiser`.
2. Work on a feature branch off `phase2/step4`, merge by PR.
3. Must be green before merging:
   - `python3 ci/run_regression.py` (guards, link, interconnect, phy, cocotb, bridge, errors, asic, rand + coverage)
   - Step 5 workloads (quick set) — no metric worse than its tolerance unless the PR explains why.
4. The PR description states any added latency: `latency: +N clk (where)`.
5. A fix that must never be undone gets a `ci/protected_lines.txt` rule in the same commit.
6. Benches never get a longer fixed wait to pass; they wait on the event.
7. **Freeze rule:** at the cutoff time agreed by the team, anything not green is reverted (one revert per item) and `step4-final` is tagged on the last green commit.

---

## 4. Vivado runs (manual)

Run after Wave A, after **3 + 1 + 8**, after **2**, and at the freeze:

```powershell
cd C:\work\rk_synth3                 # no spaces or brackets in the path
git checkout phase2/step4; git pull
git log -1 --oneline
$env:BASELINE_SIM=0; $env:BASELINE_JOBS=4
& "C:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat" -mode batch -source bridge_top/vivado_baseline.tcl -log s4_run.log
copy vivado_bridge\baseline\baseline_summary_*.txt step4_results\
git add step4_results; git commit -m "vivado results for <hash>"; git push
```

---

## 5. Tracking

| Item | Commit | clk WNS | Fmax | Failing EPs | LUT | FF | Latency change | NIC p99 | ML GB/s @250 MHz |
|---|---|---:|---:|---:|---:|---:|---|---:|---:|
| base | 49a467b | −4.439 | 118 | 19,046 | 56,043 | 24,482 | — | | |
| T1 | | | | | | | | | |
| 6 | | | | | | | | | |
| 7 | | | | | | | | | |
| 3 | | | | | | | | | |
| 1 | | | | | | | | | |
| 8 (+T2) | | | | | | | | | |
| 2 | | | | | | | | | |
| T3–T7 | | | | | | | | | |
| final | | | | | | | | | |

---

## 6. Done when

- Every requirement above is merged green, or reverted at the freeze and listed under "Future work" in the submission with its reason.
- clk meets the agreed target with hold met; aclk still met.
- Full regression, nightly sweep and coverage green on `step4-final`.
- Step 5 before/after table complete for every merged item.
- Final Vivado summary in `step4_results/` and in the Phase 2 doc.

---

## 7. As built (differences from the plan above)

| Item | As built |
|---|---|
| 1 | `src-link_layer/hdl/tlp_tx_wide.v`: translation, merging, 4 KB split, header and 512-bit beats in one module. `address_translation`, `ar_request_splitter`, `tlp_packetizer`, `tlp32_to_axis_packer` stay in the repo for their unit benches but are no longer on the bridge TX path. |
| 2 | Merging in `tlp_tx_wide` (`MERGE_EN`, `MERGE_IDLE`); bridge benches check a DW stream plus legality rules (length <= 64 DW, no 4 KB crossing, one session per TLP). `tb_stream_path` keeps the exact one-TLP-per-beat model with `MERGE_EN=0`. |
| 3 | `pcie_phy_tx_w` / `pcie_phy_rx_w` / `ts_os_detector_w`, selected by `PHY_WIDE=1` (default in `bridge_top` and `link_phy_bridge_top`; `pcie_phy_top` default stays 0 so the narrow PHY benches are unchanged). 32 PAM4 symbols per lane per clock; level bus 384 bits. Lane-to-lane skew budget < 1 lane word (32 symbols = 8 bytes), same as the narrow PHY. New bench `tb_pcie_phy_top_wide` (symbol skew + crossed lanes). |
| 8 + T2 | `rx_tlp_delimiter` moves up to 16 DW per clock from a shifted Flit register; `link_layer_top` decodes headers on the wide beats (no 1-DW unpacker / `tlp_header_parser` / `pcie_rx_engine` on the bridge path; `pcie_rx_engine` keeps its unit tests incl. S12/S13). |
| T1 | `link_layer_rx`: syndrome / correction / CRC parts / CRC combine (4 stages). |
| TX framer | `link_layer_tx`: load / CRC parts + FEC body syndromes / combine + FEC check bits / send; 8 clocks per Flit (was 18). |
| 6 | `err_counters` (PERST#-only reset), BAR0 **0x040-0x064** (the plan said 0x30-0x5C; moved so 0x34 is free for item 7): 0x40 CRC, 0x44 FEC corrected, 0x48 FEC uncorrectable, 0x4C replays, 0x50 NAKs sent, 0x54 malformed TLPs, 0x58 link downs, 0x5C hot resets, 0x60 / 0x64 overflow ch0 / ch1. 16-bit saturating, write clears. `perf_counters.v` is not used. |
| 7 | BAR0 0x0C sets both channel limits (backward compatible), 0x34 sets ch1 only; `session_end[1:0]` is an `interconnect_top` / `bridge_top` output. The arbiter already alternated grants while both channels hold credit (the "burst" note in its header was wrong), so 7c is a documentation fix, measured by the Step 5 mixed workload. |
| Step 5 | Workloads run in `tb_bridge_rand` (`+WL=1..5`), same checker as the regression; `ci/load/run_workloads.py`, `compare.py`, `thresholds.json`, `.github/workflows/load.yml`. The cocotb wrapper from the Step 5 plan is not built: the Verilog bench reuses the regression checker directly and runs about 10x faster. |
