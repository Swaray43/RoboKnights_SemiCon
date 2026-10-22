`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// lane_tx - TX half of one PHY lane (fixes P1: TX and RX used to be fused in
// lane_symbol_pipe through an ideal in-RTL channel).
//
//   byte -> byte_to_symbol (4 symbols, MSB first) -> gray_enc -> pam4_map
//        -> registered PAM4 level out, towards the SerDes.
//
// Timing: a byte presented with byte_in_valid while busy==0 is accepted. Its
// four levels appear on tx_level on the following 4 cycles (one register stage
// after byte_to_symbol), with tx_level_valid high on each. A new byte can be
// accepted every 4 cycles (byte_to_symbol accepts again the cycle after its
// 4th symbol), i.e. 2 bits per clock per lane.
//
// tx_level encoding: signed [2:0], one of -3, -1, +1, +3.
// -----------------------------------------------------------------------------
module lane_tx (
input  wire              clk,
input  wire              rst_n,
input  wire              byte_in_valid,
input  wire [7:0]        byte_in,
output wire              busy,
output reg               tx_level_valid,
output reg signed [2:0]  tx_level
);

wire       sym_valid;
wire [1:0] sym;
wire [1:0] gray_g;
wire signed [2:0] level_c;

byte_to_symbol u_ser (
  .clk(clk), .rst_n(rst_n),
  .byte_valid(byte_in_valid), .byte_in(byte_in),
  .symbol_valid(sym_valid), .symbol_out(sym), .busy(busy)
);

gray_enc u_genc (.a(sym), .g(gray_g));
pam4_map u_map  (.g(gray_g), .level(level_c));

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    tx_level_valid <= 1'b0;
    tx_level       <= -3'sd3;   // a legal PAM4 level, never X on the wire
  end else begin
    tx_level_valid <= sym_valid;
    if (sym_valid) tx_level <= level_c;
  end
end
endmodule
