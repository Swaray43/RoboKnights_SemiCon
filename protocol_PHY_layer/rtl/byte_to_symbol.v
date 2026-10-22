`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// TX serializer: one byte -> four sequential 2-bit Gray-domain symbols, MSB-first
// -----------------------------------------------------------------------------
module byte_to_symbol (
input  wire       clk,
input  wire       rst_n,
input  wire       byte_valid,
input  wire [7:0] byte_in,
output reg        symbol_valid,
output reg  [1:0] symbol_out,
output wire       busy
);

reg [1:0] sym_cnt;
reg [7:0] shift_reg;
reg       active;

assign busy = active;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    sym_cnt      <= 2'd0;
    shift_reg    <= 8'd0;
    active       <= 1'b0;
    symbol_valid <= 1'b0;
    symbol_out   <= 2'd0;
  end else begin
    symbol_valid <= 1'b0; // default: pulse only

    if (byte_valid && !active) begin
      shift_reg    <= byte_in;
      symbol_out   <= byte_in[7:6];   // first symbol emitted same cycle - full byte already available
      symbol_valid <= 1'b1;
      sym_cnt      <= 2'd1;
      active       <= 1'b1;
    end else if (active) begin
      case (sym_cnt)
        2'd1: symbol_out <= shift_reg[5:4];
        2'd2: symbol_out <= shift_reg[3:2];
        2'd3: symbol_out <= shift_reg[1:0];
      endcase
      symbol_valid <= 1'b1;
      if (sym_cnt == 2'd3) begin
        sym_cnt <= 2'd0;
        active  <= 1'b0;
      end else begin
        sym_cnt <= sym_cnt + 2'd1;
      end
    end
  end
end
endmodule
