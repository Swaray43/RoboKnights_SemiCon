// -----------------------------------------------------------------------------
// RX lane de-striper: inverse of flit_lane_striper. Reassembles 4 lanes'
// worth of bytes per cycle back into the 256-bit FLIT over 8 cycles.
// Completion word is built directly from {lane_data_in, flit_reg[223:0]}
// rather than reading flit_reg post-update, to avoid a same-cycle NBA race
// on the final chunk.
// -----------------------------------------------------------------------------
module flit_lane_destriper (
input wire clk,
input wire rst_n,
input wire [3:0] lane_valid_in,
input wire [31:0] lane_data_in,
output reg flit_valid_out,
output reg [255:0] flit_out
);

reg [255:0] flit_reg;
reg [2:0] cycle_cnt;
reg active;

wire all_lanes_valid = &lane_valid_in;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    active         <= 1'b0;
    cycle_cnt      <= 3'd0;
    flit_reg       <= 256'd0;
    flit_valid_out <= 1'b0;
    flit_out       <= 256'd0;
  end else begin
    flit_valid_out <= 1'b0;

    if (all_lanes_valid) begin
      flit_reg[cycle_cnt * 32 +: 32] <= lane_data_in;
      active <= 1'b1;

      if (cycle_cnt == 3'd7) begin
        flit_out       <= {lane_data_in, flit_reg[223:0]};
        flit_valid_out <= 1'b1;
        cycle_cnt      <= 3'd0;
        active         <= 1'b0;
      end else begin
        cycle_cnt <= cycle_cnt + 3'd1;
      end
    end
  end
end
endmodule
