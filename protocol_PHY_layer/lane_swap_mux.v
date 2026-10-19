module lane_swap_mux (
input  wire        lane_reversed,
input  wire [7:0]  lane_in_0,
input  wire [7:0]  lane_in_1,
input  wire [7:0]  lane_in_2,
input  wire [7:0]  lane_in_3,
output wire [7:0]  lane_out_0,
output wire [7:0]  lane_out_1,
output wire [7:0]  lane_out_2,
output wire [7:0]  lane_out_3
);

assign lane_out_0 = lane_reversed ? lane_in_3 : lane_in_0;
assign lane_out_1 = lane_reversed ? lane_in_2 : lane_in_1;
assign lane_out_2 = lane_reversed ? lane_in_1 : lane_in_2;
assign lane_out_3 = lane_reversed ? lane_in_0 : lane_in_3;
endmodule
