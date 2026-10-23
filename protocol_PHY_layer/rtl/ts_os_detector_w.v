`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// ts_os_detector_w - wide (8 bytes per clock) version of ts_ordered_set_detector
// for pcie_phy_rx_w. After the aligner a TS1/TS2 ordered set occupies exactly
// two consecutive lane words: word A = bytes 0..7 (COM, link, lane, 0...),
// word B = bytes 8..15 (bytes 10..15 = identifier). Same outputs and rules as
// the narrow detector: os_valid on every well-formed TS1/TS2, lane_num_valid
// once the same lane number has been seen in two consecutive ordered sets.
// -----------------------------------------------------------------------------
module ts_os_detector_w (
input  wire        clk,
input  wire        rst_n,
input  wire        word_valid,
input  wire [63:0] word_in,          // byte j at [8*j +: 8]
output reg         lane_num_valid,
output reg  [7:0]  lane_num_out,
output reg         os_valid,
output reg         os_ts2
);
localparam [7:0] COM = 8'hBC, TS1_ID = 8'h4A, TS2_ID = 8'h45;

reg       have_a;
reg [7:0] lane_cand;
reg [7:0] prev_lane;
reg       prev_v;

wire [7:0] id    = word_in[8*2 +: 8];               // byte 10 of the OS
wire       id_ok = (id == TS1_ID || id == TS2_ID) &&
                   (word_in[8*3 +: 8] == id) && (word_in[8*4 +: 8] == id) &&
                   (word_in[8*5 +: 8] == id) && (word_in[8*6 +: 8] == id) &&
                   (word_in[8*7 +: 8] == id);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    have_a <= 1'b0; lane_cand <= 8'd0; prev_lane <= 8'd0; prev_v <= 1'b0;
    lane_num_valid <= 1'b0; lane_num_out <= 8'd0; os_valid <= 1'b0; os_ts2 <= 1'b0;
  end else begin
    lane_num_valid <= 1'b0;
    os_valid       <= 1'b0;
    if (word_valid) begin
      if (have_a) begin
        have_a <= 1'b0;
        if (id_ok) begin
          os_valid <= 1'b1;
          os_ts2   <= (id == TS2_ID);
          if (prev_v && prev_lane == lane_cand) begin
            lane_num_valid <= 1'b1;
            lane_num_out   <= lane_cand;
          end
          prev_lane <= lane_cand;
          prev_v    <= 1'b1;
        end else begin
          prev_v <= 1'b0;
        end
      end else if (word_in[7:0] == COM) begin
        have_a    <= 1'b1;
        lane_cand <= word_in[8*2 +: 8];
      end
    end
  end
end
endmodule
