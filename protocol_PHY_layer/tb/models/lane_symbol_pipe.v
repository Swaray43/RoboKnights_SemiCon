// -----------------------------------------------------------------------------
// lane_symbol_pipe - SIMULATION MODEL ONLY (moved out of the RTL, see P1).
//
// lane_tx -> ideal zero-noise channel (a wire) -> lane_rx. Kept with its old
// ports so the existing unit/integration benches (tb_end_to_end_datapath,
// tb_startup_integration) still run unchanged. The ideal channel now lives
// here in the testbench tree, not inside a synthesizable lane module; the
// real design exposes the levels on pcie_phy_top's tx_level / rx_level ports.
// -----------------------------------------------------------------------------
module lane_symbol_pipe (
input  wire       clk,
input  wire       rst_n,
input  wire       byte_in_valid,
input  wire [7:0] byte_in,
output wire       byte_out_valid,
output wire [7:0] byte_out,
output wire       busy
);

wire              lvl_valid;
wire signed [2:0] lvl;

lane_tx u_tx (
  .clk(clk), .rst_n(rst_n),
  .byte_in_valid(byte_in_valid), .byte_in(byte_in), .busy(busy),
  .tx_level_valid(lvl_valid), .tx_level(lvl)
);

// ideal channel: no noise, no attenuation, no skew
lane_rx u_rx (
  .clk(clk), .rst_n(rst_n),
  .rx_level_valid(lvl_valid), .rx_level(lvl),
  .byte_out_valid(byte_out_valid), .byte_out(byte_out)
);
endmodule
