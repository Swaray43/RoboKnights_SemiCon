# Protocol-to-PHY layer: review fixes

This change set covers Kshitij's items from the *AXI4–PCIe6 Bridge Review*: **P1–P5**, **I2**, **I3**, the PHY side of **Seams B, C and D**, and the testbench bugs. Every bench runs in Icarus Verilog 12.

## Results

||Before|After|
|-|-|-|
|Verilog benches (`python run\_all.py`)|19 pass, `tb\_full\_system\_integration` 5 FAIL, `tb\_pam4\_roundtrip` won't build|**24 / 24 pass** (incl. new `tb\_pcie\_phy\_top`, 35 checks)|
|cocotb (`cocotb/run\_tests.py`)|4 / 4 on `pcie\_link\_top`|**8 / 8** on `pcie\_phy\_top` (4 original unchanged + 4 new)|
|Synthesis (`yosys synth -top pcie\_phy\_top`)|no top existed|clean, 0 latches, 0 `check` problems|
|Formal|`gray.sby` OK; `pam4.sby` and `verify\_reset.sby` pointed at missing files|`gray.sby`, `pam4.sby`, `verify\_sof\_strip.sby` all PASS|
|PHY drain rate|n/a (no top)|**260 clk per 256-byte Flit** (floor 256 + 4 for the SOF byte)|

## Fixes by review ID

|ID|What changed|Where|
|-|-|-|
|**P1**|`lane\_symbol\_pipe` split into `lane\_tx` (byte → symbols → Gray → PAM4, registered level out) and `lane\_rx` (level in → slicer → Gray decode → byte). The top exposes `tx\_level\[11:0]` / `tx\_level\_valid\[3:0]` and `rx\_level` / `rx\_level\_valid` (lane *i* = `\[3\*i +: 3]`, signed −3/−1/+1/+3). The ideal channel now lives only in the testbench; `lane\_symbol\_pipe` survives as a sim model in `tb/models/` so the old benches still run.|`rtl/lane\_tx.v`, `rtl/lane\_rx.v`, `tb/models/lane\_symbol\_pipe.v`|
|**P2**|New synthesizable `pcie\_phy\_tx` / `pcie\_phy\_rx` / `pcie\_phy\_top`. TX inserts the 0xFB SOF itself (it used to come from the testbench's `sof\_phase`). RX rebuilds `sof`/`eof` per Flit for `link\_layer\_rx`.|`rtl/pcie\_phy\_\*.v`|
|**P3 / Seam C**|Flit FIFO (16 × 256 b) with a valid/ready handshake: `phy\_tx\_ready`. A lane scheduler feeds all four lanes in lockstep, one byte every 4 clocks, which is the full 2 bits/clk/lane. The deskew FIFO now reports `room\_ok`, so the striper can't overflow it. Before, `elastic\_fifo` dropped bytes silently when full.|`rtl/pcie\_phy\_tx.v`, `rtl/tx\_elastic\_deskew.v`|
|**P4 / Seam B**|New BAR0 register block holding `base\_addr\_ch0/1` (0x20/0x28), arm bits (0x00), weights (0x08), session limit (0x0C), MSI-X vector per channel (0x10), TC (0x18), status and debug. It's driven by a `bar0\_\*` port, from cocotb for now, until an inbound MWr path exists. Register map is in the file header.|`rtl/bar0\_dma\_regs.v`|
|**P5**|One module per file in `rtl/`, a single list in `files.f`, testbenches in `tb/`, proofs in `formal/`. Duplicates removed (copies came from 7 files). Gray ports standardized to `(a, g)` / `(g, a)`. The `ifdef FORMAL` block from `pci\_stuff.v`'s copy of `flit\_sof\_strip` was kept, moved below the declarations so it elaborates.|whole tree|
|**I2**|Kshitij's `axi4s\_ingest\_port` (16-bit offsets) and its `session\_overflow\_ctrl` were renamed `legacy\_\*` and moved to `legacy/`, off the file list. Aditya's 24-bit `axi4s\_ingest\_port` is the one that stays. All three layers can now compile together.|`legacy/`|
|**I3**|`session\_valid` (a level on `clk`) → rising edge → toggle synchroniser → `session\_start\_aclk`, a 1-cycle pulse on `aclk`. The reverse direction is covered too: `session\_end\_aclk` → `clk` → `session\_arm\_ctrl`. The overflow flags get a 2-flop sync into BAR0 STATUS.|`rtl/pulse\_sync.v`, `rtl/pcie\_phy\_top.v`|
|**Seam D** (PHY side)|`msix\_controller` now uses valid/ready (`mwr\_ready`), so the MWr is held until the link layer takes it. A completion that arrives while busy goes to PBA instead of being lost. Pending vectors are sent when unmasked and their PBA bit clears, per the PCIe rule. `session\_done\[1:0]` from the link layer picks that channel's vector from BAR0.|`rtl/msix\_controller.v`, `rtl/pcie\_phy\_top.v`|
|**Tests**|`tb\_full\_system\_integration`: `sext\_rst\_n` → `ext\_rst\_n`; drives `DEAD\_BEEF…` to match the check; loopback moved into the testbench (`lane\_tx` → wires → `lane\_rx`); `default\_nettype none` added to catch the next typo; leftover `$monitor` removed. Result: 14 pass, 0 fail. `tb\_pam4\_roundtrip` now builds.|`tb/`|

## Design decisions to confirm

1. **SOF once per 256-byte Flit (the PCIe meaning).** TX inserts SOF before each word that arrives with `flit\_sof=1`, and RX expects `WORDS\_PER\_FLIT=8` words after each SOF. `flit\_sof\_strip` gained `PAYLOAD\_CYCLES` (8 = old per-word behaviour, which is still the default for its unit test; 64 in the top). It also gained `RESYNC\_MIDFLIT=0` in the top, so payload containing 0xFB on all 4 lanes no longer resyncs. That was the old "known limitation", and the benches now push all-0xFB words through to prove it.
2. **Seam C uses one clock and a ready signal**, not a separate symbol-clock domain.
3. **`pcie\_link\_top` is replaced by `pcie\_phy\_top`.** Every old port keeps its name, so the 4 original cocotb tests run unchanged. Only the BFM's `reset()` drives the new inputs.
4. **`mwr\_ready` is a new required input.** Tie it to 1 if the link layer can always take the MSI-X write.

## What other people need to do

**Swaray (link layer)**

* `link\_layer\_tx` must **stall its output word counter while `phy\_tx\_ready` is low**. This is the second half of Seam C. The PHY drains a Flit in 260 clocks; the link writes one in 8.
* Drive `phy\_tx\_sof` on word 0 and `phy\_tx\_eof` on word 7 of each Flit. `phy\_framing\_err` latches if not.
* `link\_layer\_rx` gets `phy\_rx\_data/valid/sof/eof` and must accept a word on every `phy\_rx\_valid`. There is no RX backpressure.
* Base addresses: `cfg\_base\_addr\_ch0/1` plus a `cfg\_base\_addr\_wr\[1:0]` pulse when a session arms.
* MSI-X (L10): take `mwr\_valid/addr/data` with `mwr\_ready`, and drive `session\_done\[1:0]` once the tlast TLP of a session has been issued.

**Aditya (interconnect)**

* `session\_start\_aclk\[1:0]`: a 1-cycle pulse on `aclk` per channel.
* Return `session\_end\_aclk\[1:0]` (tlast accepted or overflow abort, a pulse on `aclk`) and `session\_overflow\_aclk\[1:0]` (sticky levels).
* Weights and limit are parameters `WW=8`, `LW=24` on `pcie\_phy\_top`. **Please confirm they match `interconnect\_top`.** They are quasi-static: written before the arm bit, never changed during a session.

## Not done / known gaps(I'M WORKING ON IT)

* **TS1/TS2 training is still fed from the `ts\_byte\_\*` ports**, not from the received lane bytes. There's no TX-side TS generator either.
* **No RX lane deskew.** RX assumes the lanes arrive in lockstep; `phy\_lane\_err` latches if they don't.
* **Two config-space details are open.** BAR0 writes aren't gated by `mem\_space\_enable`, and MSI-X isn't gated by `msix\_enable` / function mask. MSI-X table BIR/offset in config space is still 0.

## Files deleted (now in `rtl/`, `tb/` or `formal/`)

`pcie\_phy\_datapath.v`, `pcie\_phy\_datapath\_v0.v`, `end\_pipeline.v`, `pci\_stuff.v`, `flit\_stripper.v`, `gray\_enc\_dec.v`, `pam4.v`, `reset.v`, `pam\_specy.v`, `pcie\_tag\_man.v`, `pcie\_config\_msix.v`, `pref\_mon.v`, `tb\_config\_msix.v`, `pcie\_link\_top.v` and `cocotb/pcie\_link\_top.v` (both replaced by `rtl/pcie\_phy\_top.v`), `verify\_reset.sby` (replaced by `verify\_sof\_strip.sby`), plus `cocotb/sim\_build/`, `results.xml` and `\_\_pycache\_\_/`.

## How to run

```sh
python run\_all.py                  # all 24 Verilog benches (Icarus); Windows/macOS/Linux
python run\_all.py tb\_pcie\_phy\_top  # one bench
./run\_all.sh                       # same thing, wrapper for bash users
cd cocotb \&\& python3 run\_tests.py  # 8 cocotb tests (or: make)
cd formal \&\& sby -f gray.sby       # also pam4.sby, verify\_sof\_strip.sby
```

Other simulators can use `files.f`. Benches add `tb/models/\*.v` and `tb/pcie\_reset\_controller\_checker.v`.

