// ASIC profile: same overrides as asic/*/config.yaml SYNTH_PARAMETERS
module asic_params_ovf;
  defparam tb_bridge_ovf.dut.u_ic.FIFO_ADDR_WIDTH = 3;
  defparam tb_bridge_ovf.dut.u_lpb.u_link_layer_top.REPLAY_DEPTH = 8;
  defparam tb_bridge_ovf.dut.u_lpb.u_link_layer_top.FIFO_D = 8;
  defparam tb_bridge_ovf.dut.u_lpb.u_link_layer_top.HWM = 6;
  defparam tb_bridge_ovf.dut.u_lpb.u_pcie_phy_top.TX_FIFO_DEPTH = 8;
endmodule
