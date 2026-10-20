# Physical design with LibreLane (OpenLane 2)

LibreLane is the current name of OpenLane 2. Same flow (Yosys → OpenROAD → Magic/KLayout → GDSII),
same YAML/JSON configs, new package name `librelane`. These configs were written against the
LibreLane docs but **have not been run yet**: the workspace they were written in can't reach the
Nix cache, Docker Hub or GitHub. Expect to fix small things on the first run.

## 1. Install (pick one)

### Windows (your laptop): WSL2 + Docker Desktop + pip

```powershell
wsl --install -d Ubuntu          # PowerShell, then reboot and finish the Ubuntu setup
```
Install **Docker Desktop**, tick "Use WSL 2 instead of Hyper-V", and in Settings → Resources →
WSL integration enable your Ubuntu. Then, inside Ubuntu:

```bash
sudo apt-get update
sudo apt-get install -y build-essential python3 python3-venv python3-pip python3-tk curl make git
docker run hello-world                         # must work before going on
python3 -m venv ~/ll && source ~/ll/bin/activate
python3 -m pip install librelane
python3 -m librelane --dockerized --smoke-test # downloads the tool image and the sky130 PDK
```
Work inside the WSL filesystem (`~/...`), not `/mnt/c/...`: the flow is many times slower on
Windows-mounted folders.

### Linux machine or lab server: Nix (recommended by the LibreLane docs)

```bash
curl --proto '=https' --tlsv1.2 -fsSL https://artifacts.nixos.org/nix-installer | sh -s -- install --no-confirm --extra-conf "
    extra-substituters = https://nix-cache.fossi-foundation.org
    extra-trusted-public-keys = nix-cache.fossi-foundation.org:3+K59iFwXqKsL7BNu6Guy0v+uTlwsxYQxjspXzqLYQs=
    extra-experimental-features = nix-command flakes
"
# close and reopen the terminal
git clone https://github.com/librelane/librelane && cd librelane
nix-shell                     # first time ~10 min
librelane --smoke-test
```

**Machine size:** 16 GB RAM minimum for the link layer; 32 GB is more comfortable. Disk: ~20 GB for
the PDK and runs.

## 2. Run the blocks (one at a time, smallest first)

From the repo root, with LibreLane available (`librelane ...`, or `python3 -m librelane --dockerized ...`):

```bash
cd asic/interconnect && librelane --pdk sky130A config.yaml   # smallest, ~30k cells: proves the setup
cd asic/phy          && librelane --pdk sky130A config.yaml
cd asic/link         && librelane --pdk sky130A config.yaml   # largest: CRC-64/FEC + replay buffer
```

Results land in `runs/<timestamp>/final/` (`gds/`, `lef/`, `nl/`, `spef/`, `lib/`, and the
`metrics.json` with area, timing, DRC/LVS counts). View with:

```bash
librelane --pdk sky130A config.yaml --last-run --flow OpenInKLayout
librelane --pdk sky130A config.yaml --last-run --flow OpenInOpenROAD
```

**PDKs:** `sky130A` (default, best documented), `ihp-sg13g2` (130 nm, IHP runs free academic
shuttles), `gf180mcuD` (180 nm, slower and larger). Change only `--pdk`; the SDCs read the
driving cell and load from LibreLane, so they're PDK-neutral.

## 3. What's in each folder

| Folder | Top | Clocks | Size-reducing overrides (`SYNTH_PARAMETERS`) |
|---|---|---|---|
| `interconnect/` | `interconnect_top` | `aclk`, `pcie_clk` | CDC FIFO 16 → 8 deep |
| `link/` | `link_layer_top` | `clk` | Replay buffer 32 → 8 Flits; write buffer 16 → 8 |
| `phy/` | `pcie_phy_top` | `clk`, `aclk` | PHY Flit FIFO 16 → 8 words |
| `bridge_flat/` | `bridge_top` (all 56 files) | `clk`, `aclk` | none yet; try only after the three blocks close |

Each folder has its own SDC, with every port on the right clock, the two clocks declared
asynchronous, and the quasi-static BAR0 outputs (weights, limit) as false paths.

**Before taping out with the smaller buffers, re-run the regression with the same parameters**
(`tb_stream_path` with bit errors, `tb_bridge_top`, `tb_bridge_ovf`). A smaller replay buffer
changes behaviour under errors.

## 4. Known issues to expect

1. **Clock target.** `CLOCK_PERIOD: 10` (100 MHz). 250 MHz isn't realistic in a 130/180 nm open PDK
   with 2048-bit CRC/FEC trees. Close at 10 ns first, then tighten.
2. **Memories as flip-flops.** There are no SRAM macros in this flow yet, so every FIFO and buffer
   becomes flops plus muxes. That's the main area cost. Next step: DFFRAM or OpenRAM macros for the
   replay buffer and the FIFOs.
3. **Pin count.** `bridge_top` has ~1,900 signal pins (two 512-bit inputs, a 512-bit output...).
   That's fine for a **hard macro / IP block** (pins on the block edge), which is what "generate the
   final GDSII" needs. A padded, fabricable chip needs a wrapper with narrow I/O: on-chip traffic
   generators and checkers, SPI for BAR0, a few status pins.
4. **Run time.** The link layer's CRC/FEC logic is large; detailed routing can take hours. Use
   `FP_CORE_UTIL` 30–35 if routing congests.
5. **Variable names.** These files use LibreLane's names (`PNR_SDC_FILE`, `SIGNOFF_SDC_FILE`,
   `PL_TARGET_DENSITY_PCT`, `SYNTH_PARAMETERS`, `VERILOG_INCLUDE_DIRS`). If a run reports an unknown
   variable, check it against the configuration reference for your installed version.

## 5. Order of work

1. Smoke test → interconnect block → read `metrics.json` (area, worst slack, DRC, LVS).
2. PHY block, then the link block. Fix timing per block before integrating.
3. Integrate: either `bridge_flat` (simplest), or harden the three blocks as macros and place them
   in a top config with `MACROS:` (better run time and timing, more setup).
4. Signoff: zero DRC and LVS errors, no setup/hold violations at all corners, antenna check clean.
5. Final GDSII + a short report: area, utilisation, Fmax, power, layout screenshots per block.
