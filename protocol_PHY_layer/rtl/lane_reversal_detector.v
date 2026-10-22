`timescale 1ns/1ps
module lane_reversal_detector (
input  wire       clk,
input  wire       rst_n,
input  wire [3:0] lane_num_valid,
input  wire [7:0] lane_num_l0,
input  wire [7:0] lane_num_l1,
input  wire [7:0] lane_num_l2,
input  wire [7:0] lane_num_l3,
output reg        reversal_detected_valid,
output reg        lane_reversed
);

wire all_valid = &lane_num_valid;

wire identity_match = (lane_num_l0==8'd0) && (lane_num_l1==8'd1) &&
                       (lane_num_l2==8'd2) && (lane_num_l3==8'd3);
wire reversed_match = (lane_num_l0==8'd3) && (lane_num_l1==8'd2) &&
                       (lane_num_l2==8'd1) && (lane_num_l3==8'd0);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    reversal_detected_valid <= 1'b0;
    lane_reversed           <= 1'b0;
  end else begin
    reversal_detected_valid <= 1'b0;
    if (all_valid && (identity_match || reversed_match)) begin
      reversal_detected_valid <= 1'b1;
      lane_reversed           <= reversed_match;
    end
    // NOTE (known limitation): all_valid true but neither pattern matches ->
    // holds previous state silently. Partial reversal / wiring fault not flagged.
  end
end
endmodule
