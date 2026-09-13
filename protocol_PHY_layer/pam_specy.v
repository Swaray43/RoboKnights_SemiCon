module pam4_spec_checker (
input wire clk,
input wire rst_n,
// TX side
input wire [1:0] tx_gray_in,
input wire signed [2:0] tx_level_out,
// RX side
input wire signed [2:0] rx_level_in,
input wire [1:0] rx_gray_out
);

`ifdef FORMAL
always @(posedge clk) begin
if (rst_n) begin
// ---------------------------------------------------------
// TX MAPPER: Assertions against the hardcoded spec
// ---------------------------------------------------------
if (tx_gray_in == 2'b00) assert(tx_level_out == -3);
if (tx_gray_in == 2'b01) assert(tx_level_out == -1);
if (tx_gray_in == 2'b11) assert(tx_level_out ==1);
if (tx_gray_in == 2'b10) assert(tx_level_out ==3);

// ---------------------------------------------------------
// RX DEMAPPER: Assertions against the hardcoded spec
// ---------------------------------------------------------
if (rx_level_in == -3) assert(rx_gray_out == 2'b00);
if (rx_level_in == -1) assert(rx_gray_out == 2'b01);
if (rx_level_in ==1) assert(rx_gray_out == 2'b11);
if (rx_level_in ==3) assert(rx_gray_out == 2'b10);
end
end
`endif

endmodule
