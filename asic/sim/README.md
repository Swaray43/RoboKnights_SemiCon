# Simulating the ASIC buffer sizes

The `SYNTH_PARAMETERS` in `../*/config.yaml` shrink buffers. These defparam files apply the same
values in simulation. Run from the repo root:

```bash
iverilog -g2012 -DSIMULATION -s tb_bridge_top -s asic_params -o a1.vvp -c bridge_top/bridge_top.f -c bridge_top/bridge_top_tb.f asic/sim/asic_params.v && vvp a1.vvp +REARM_FAST=1
iverilog -g2012 -DSIMULATION -s tb_bridge_ovf -s asic_params_ovf -o a2.vvp -c bridge_top/bridge_top.f -c bridge_top/bridge_top_tb.f asic/sim/asic_params_ovf.v && vvp a2.vvp
iverilog -g2012 -DSIMULATION -Isrc-link_layer/hdl -s tb_stream_path -s asic_params_sp -o a3.vvp src-link_layer/hdl/testbench/tb_stream_path.v src-link_layer/hdl/*.v asic/sim/asic_params_sp.v && vvp a3.vvp +NBEATS=1500 +ERR1=3000 +ERR2=1000 +READY_PCT=40 +RXREADY_PCT=40
```

All three pass with the current overrides (replay 8, write buffer 8, PHY FIFO 8, CDC FIFO 8).
Keep these files in step with the configs.
