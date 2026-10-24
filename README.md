## Branch: phase2/X

This is the primary development branch for Phase 2. It contains all features from:

- **phase2/step3** — CI pipeline (regression.yml, main-guard.yml), protected-lines guard,
  testbench suites (tb_bridge_rand F1–F9+F7b), functional coverage
- **phase2/step4** — Wide TLP engine (512-bit, merge up to 64 DW, 4 KB split),
  pipelined CRC-64 (flit_crc64_parts), FEC syndrome generation (flit_fec_syn),
  err_counters at BAR0 0x40–0x64, per-channel session limit 0x34, replay timer 2048,
  Step 5 load analyser workloads (WL=1..5). FPGA result: **248 MHz, WNS −0.027 ns,
  LUT 61k / FF 39k** on xcu250.
- **X patches (GDS fixes)** — X.1: TREADY ingest fix; X.2: GDS build config (bridge_top,
  gds/bridge, gds/interconnect, asic sim profiles); X.3: link_layer_rx synth checks
  (no latches/loops); X.4: gate-level FEC correction rewrite.

GDSII flow is on branch **phase3/GDSII** (SKY130A, LibreLane).
Interconnect layer GDS completed clean DRC/LVS/antenna (~1.12 mm²).
