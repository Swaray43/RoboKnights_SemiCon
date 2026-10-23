// X.2 GDS profile: exactly the parameters gds/bridge/config.json passes to
// bridge_top (narrow PHY, small buffers). tb_bridge_top's 384-bit level wires
// connect to the 12-bit narrow-PHY ports through their low bits (loopback).
module asic_params_gds_ovf;
  defparam tb_bridge_ovf.dut.PHY_WIDE      = 0;
  defparam tb_bridge_ovf.dut.IC_FIFO_AW    = 2;
  defparam tb_bridge_ovf.dut.REPLAY_DEPTH  = 4;
  defparam tb_bridge_ovf.dut.WDB_FIFO_D    = 4;
  defparam tb_bridge_ovf.dut.WDB_HWM       = 3;
  defparam tb_bridge_ovf.dut.PHY_TX_FIFO_D = 8;
endmodule
