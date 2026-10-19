module tb_lane_swap_mux();
reg lane_reversed;
reg [7:0] lane_in_0, lane_in_1, lane_in_2, lane_in_3;
wire [7:0] lane_out_0, lane_out_1, lane_out_2, lane_out_3;

integer errors;

lane_swap_mux dut(.lane_reversed(lane_reversed),
  .lane_in_0(lane_in_0), .lane_in_1(lane_in_1), .lane_in_2(lane_in_2), .lane_in_3(lane_in_3),
  .lane_out_0(lane_out_0), .lane_out_1(lane_out_1), .lane_out_2(lane_out_2), .lane_out_3(lane_out_3));

initial begin
  errors = 0;
  lane_in_0 = 8'h10; lane_in_1 = 8'h11; lane_in_2 = 8'h12; lane_in_3 = 8'h13;

  lane_reversed = 0;
  #1;
  if (lane_out_0 !== lane_in_0 || lane_out_1 !== lane_in_1 ||
      lane_out_2 !== lane_in_2 || lane_out_3 !== lane_in_3) begin
    $display("FAIL: pass-through case incorrect");
    errors = errors + 1;
  end else $display("PASS: pass-through (not reversed) correct");

  lane_reversed = 1;
  #1;
  if (lane_out_0 !== lane_in_3 || lane_out_1 !== lane_in_2 ||
      lane_out_2 !== lane_in_1 || lane_out_3 !== lane_in_0) begin
    $display("FAIL: reversed case incorrect: out0=%h out1=%h out2=%h out3=%h",
              lane_out_0, lane_out_1, lane_out_2, lane_out_3);
    errors = errors + 1;
  end else $display("PASS: reversed swap correct");

  if (errors == 0) $display("ALL PASS: lane_swap_mux verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule
