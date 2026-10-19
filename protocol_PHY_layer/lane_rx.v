// -----------------------------------------------------------------------------
// lane_rx - RX half of one PHY lane (the other half of the P1 split).
//
//   PAM4 level in (from SerDes) -> pam4_demap (threshold slicer) -> gray_dec
//        -> symbol_to_byte (4 symbols, MSB first) -> byte out.
//
// rx_level_valid must be high for each of the 4 symbols of a byte; symbols are
// grouped into bytes by count from reset, so TX and RX must agree on byte
// alignment (they do here: lane_tx only ever emits whole bytes).
// -----------------------------------------------------------------------------
module lane_rx (
input  wire              clk,
input  wire              rst_n,
input  wire              rx_level_valid,
input  wire signed [2:0] rx_level,
output wire              byte_out_valid,
output wire [7:0]        byte_out
);

wire [1:0] rx_g;
wire [1:0] rx_sym;

pam4_demap u_demap (.level_in(rx_level), .g_out(rx_g));
gray_dec   u_gdec  (.g(rx_g), .a(rx_sym));

symbol_to_byte u_deser (
  .clk(clk), .rst_n(rst_n),
  .symbol_valid(rx_level_valid), .symbol_in(rx_sym),
  .byte_valid(byte_out_valid), .byte_out(byte_out)
);
endmodule
