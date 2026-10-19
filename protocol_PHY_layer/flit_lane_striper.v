// -----------------------------------------------------------------------------
// TX lane striper: 256-bit FLIT -> 4 lanes, byte-interleaved round-robin.
// Byte-index-is-lane-index falls out of the indexed part-select directly:
// cycle c covers bytes {4c,4c+1,4c+2,4c+3} -> lane0 gets 0,4,8...; lane1 1,5,9...
// -----------------------------------------------------------------------------
module flit_lane_striper (
input wire clk,
input wire rst_n,
input wire valid_in,
input wire [255:0] flit_in,
output wire ready_out,
output reg [3:0] lane_valid_out,
output reg [31:0] lane_data_out
);

reg [255:0] flit_reg;
reg [2:0] cycle_cnt;
reg active;

assign ready_out = !active || (active && (cycle_cnt == 3'd7));

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    active    <= 1'b0;
    cycle_cnt <= 3'd0;
    flit_reg  <= 256'd0;
  end else begin
    if (valid_in && ready_out) begin
      flit_reg  <= flit_in;
      active    <= 1'b1;
      cycle_cnt <= 3'd0;
    end else if (active) begin
      if (cycle_cnt == 3'd7) begin
        active <= 1'b0;
      end else begin
        cycle_cnt <= cycle_cnt + 3'd1;
      end
    end
  end
end

always @(*) begin
  lane_valid_out = 4'b0000;
  lane_data_out  = 32'd0;
  if (active) begin
    lane_valid_out = 4'b1111;
    lane_data_out  = flit_reg[cycle_cnt * 32 +: 32];
  end
end
endmodule
