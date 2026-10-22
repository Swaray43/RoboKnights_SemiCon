`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// RX deserializer: four sequential 2-bit symbols -> one reassembled byte, MSB-first
// Must match byte_to_symbol's bit ordering exactly (interface contract).
// -----------------------------------------------------------------------------
module symbol_to_byte (
input  wire       clk,
input  wire       rst_n,
input  wire       symbol_valid,
input  wire [1:0] symbol_in,
output reg        byte_valid,
output reg  [7:0] byte_out
);

reg [1:0] sym_cnt;
reg [7:0] shift_reg;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    sym_cnt    <= 2'd0;
    shift_reg  <= 8'd0;
    byte_valid <= 1'b0;
  end else begin
    byte_valid <= 1'b0; // default: pulse only when a byte completes

    if (symbol_valid) begin
      shift_reg <= {shift_reg[5:0], symbol_in};
      if (sym_cnt == 2'd3) begin
        byte_out   <= {shift_reg[5:0], symbol_in};
        byte_valid <= 1'b1;
        sym_cnt    <= 2'd0;
      end else begin
        sym_cnt <= sym_cnt + 2'd1;
      end
    end
  end
end
endmodule
