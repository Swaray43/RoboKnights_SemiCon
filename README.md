# RoboKnights SemiCon — FPGA Build (RTL Freeze)

## Target
Xilinx Alveo U250 (xcu250-figd2104-2L-e)

## Result
- Frequency: 248 MHz (WNS −0.027 ns)
- LUT: 61k | FF: 39k | Failing EP: 82
- Commit: 58731cd (Step 4 Final freeze)

## Patches applied
- X.1: axi4s_ingest_port TREADY fix
- X.3: link_layer_rx synth checks (integer scope)
- X.4: link_layer_rx gate-level FEC correction

## Run Vivado
powershell:
  $env:BASELINE_SIM=0; $env:BASELINE_JOBS=4
  & "C:\AMDDesignTools\2026.1\Vivado\bin\vivado.bat" -mode batch -source bridge_top/vivado_baseline.tcl