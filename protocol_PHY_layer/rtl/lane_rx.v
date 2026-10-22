`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// lane_rx - RX half of one PHY lane (the other half of the P1 split).
//
//   PAM4 level in (from SerDes) -> pam4_demap (threshold slicer) -> gray_dec
//        -> 4 symbols, MSB first -> byte out.
//
// Byte alignment (symbol lock):
//   Four 2-bit symbols make a byte, but the receiver does not know where a
//   byte starts if the channel came up part-way through one. While align_en
//   is high (training: link not yet up), every received symbol is checked
//   against a sliding 8-bit window; when the window equals COM (0xBC) the byte
//   boundary snaps to it. In the TS1 pattern 0xBC only appears on a true byte
//   boundary, so this locks within one ordered set. After training align_en
//   drops and the boundary is frozen, so 0xBC inside Flit data is harmless.
//   With align_en tied low this behaves exactly like the old symbol_to_byte
//   (bytes counted from reset).
// -----------------------------------------------------------------------------
module lane_rx (
input  wire              clk,
input  wire              rst_n,
input  wire              align_en,
input  wire              rx_level_valid,
input  wire signed [2:0] rx_level,
output reg               byte_out_valid,
output reg  [7:0]        byte_out
);

localparam [7:0] COM = 8'hBC;

wire [1:0] rx_g;
wire [1:0] rx_sym;

pam4_demap u_demap (.level_in(rx_level), .g_out(rx_g));
gray_dec   u_gdec  (.g(rx_g), .a(rx_sym));

reg  [5:0] hist;        // previous three symbols
reg  [1:0] sym_cnt;     // symbols already collected in the current byte
wire [7:0] window = {hist, rx_sym};

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    hist           <= 6'd0;
    sym_cnt        <= 2'd0;
    byte_out_valid <= 1'b0;
    byte_out       <= 8'd0;
  end else begin
    byte_out_valid <= 1'b0;
    if (rx_level_valid) begin
      hist <= {hist[3:0], rx_sym};
      if ((sym_cnt == 2'd3) || (align_en && window == COM)) begin
        byte_out       <= window;
        byte_out_valid <= 1'b1;
        sym_cnt        <= 2'd0;
      end else begin
        sym_cnt <= sym_cnt + 2'd1;
      end
    end
  end
end
endmodule
