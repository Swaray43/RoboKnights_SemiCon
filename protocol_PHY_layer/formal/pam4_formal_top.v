// Formal harness for the PAM4 mapper/demapper: free inputs on both sides,
// pam4_spec_checker asserts the locked Gray->level table in both directions.
// (pam4.sby used to reference this file, but it was missing from the repo.)
module pam4_formal_top (
input wire               clk,
input wire [1:0]         tx_gray_in,
input wire signed [2:0]  rx_level_in
);
wire signed [2:0] tx_level_out;
wire [1:0]        rx_gray_out;

pam4_map   u_map   (.g(tx_gray_in), .level(tx_level_out));
pam4_demap u_demap (.level_in(rx_level_in), .g_out(rx_gray_out));

pam4_spec_checker u_chk (
  .clk(clk), .rst_n(1'b1),
  .tx_gray_in(tx_gray_in), .tx_level_out(tx_level_out),
  .rx_level_in(rx_level_in), .rx_gray_out(rx_gray_out)
);
endmodule
