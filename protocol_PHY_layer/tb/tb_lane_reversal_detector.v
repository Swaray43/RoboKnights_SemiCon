module tb_lane_reversal_detector();
reg clk, rst_n;
reg [3:0] lane_num_valid;
reg [7:0] lane_num_l0, lane_num_l1, lane_num_l2, lane_num_l3;
wire reversal_detected_valid;
wire lane_reversed;

integer errors;

lane_reversal_detector dut(.clk(clk), .rst_n(rst_n), .lane_num_valid(lane_num_valid),
  .lane_num_l0(lane_num_l0), .lane_num_l1(lane_num_l1),
  .lane_num_l2(lane_num_l2), .lane_num_l3(lane_num_l3),
  .reversal_detected_valid(reversal_detected_valid), .lane_reversed(lane_reversed));

initial clk = 0;
always #5 clk = ~clk;

initial begin
  errors = 0;
  rst_n = 0; lane_num_valid = 4'b0000;
  lane_num_l0 = 0; lane_num_l1 = 0; lane_num_l2 = 0; lane_num_l3 = 0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;
  @(negedge clk);

  // Test 1: identity mapping
  lane_num_l0=0; lane_num_l1=1; lane_num_l2=2; lane_num_l3=3;
  lane_num_valid = 4'b1111;
  @(negedge clk);
  if (reversal_detected_valid !== 1'b1 || lane_reversed !== 1'b0) begin
    $display("FAIL identity case: valid=%b reversed=%b", reversal_detected_valid, lane_reversed);
    errors = errors + 1;
  end else $display("PASS identity case: reversed=%b", lane_reversed);
  lane_num_valid = 4'b0000;
  @(negedge clk);

  // Test 2: reversed mapping
  lane_num_l0=3; lane_num_l1=2; lane_num_l2=1; lane_num_l3=0;
  lane_num_valid = 4'b1111;
  @(negedge clk);
  if (reversal_detected_valid !== 1'b1 || lane_reversed !== 1'b1) begin
    $display("FAIL reversed case: valid=%b reversed=%b", reversal_detected_valid, lane_reversed);
    errors = errors + 1;
  end else $display("PASS reversed case: reversed=%b", lane_reversed);
  lane_num_valid = 4'b0000;
  @(negedge clk);

  // Test 3: not all lanes valid yet -> should not assert
  lane_num_l0=0; lane_num_l1=1; lane_num_l2=2; lane_num_l3=3;
  lane_num_valid = 4'b0111; // lane 3 not valid yet
  @(negedge clk);
  if (reversal_detected_valid !== 1'b0) begin
    $display("FAIL: asserted with incomplete lane_num_valid");
    errors = errors + 1;
  end else $display("PASS: correctly withheld with incomplete valid mask");

  if (errors == 0) $display("ALL PASS: lane_reversal_detector verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
end
endmodule
