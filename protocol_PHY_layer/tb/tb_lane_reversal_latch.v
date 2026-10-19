module tb_lane_reversal_latch();
reg clk, rst_n;
reg reversal_detected_valid;
reg lane_reversed_in;
wire lane_reversed_locked, link_trained;

integer errors;

lane_reversal_latch dut(.clk(clk), .rst_n(rst_n),
  .reversal_detected_valid(reversal_detected_valid), .lane_reversed_in(lane_reversed_in),
  .lane_reversed_locked(lane_reversed_locked), .link_trained(link_trained));

initial clk = 0;
always #5 clk = ~clk;

initial begin
  errors = 0;
  rst_n = 0; reversal_detected_valid = 0; lane_reversed_in = 0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;
  @(negedge clk);

  if (link_trained !== 1'b0) begin
    $display("FAIL: link_trained should be 0 before any detection pulse");
    errors = errors + 1;
  end else $display("PASS: link_trained correctly low pre-training");

  // First confirmed detection: reversed = 1
  reversal_detected_valid = 1; lane_reversed_in = 1;
  @(negedge clk);
  reversal_detected_valid = 0;
  if (link_trained !== 1'b1 || lane_reversed_locked !== 1'b1) begin
    $display("FAIL: expected latch to lock reversed=1, got locked=%b trained=%b",
              lane_reversed_locked, link_trained);
    errors = errors + 1;
  end else $display("PASS: latched reversed=1 correctly");

  // Second pulse with a DIFFERENT value should be ignored (already trained)
  reversal_detected_valid = 1; lane_reversed_in = 0;
  @(negedge clk);
  reversal_detected_valid = 0;
  if (lane_reversed_locked !== 1'b1) begin
    $display("FAIL: latch value changed after already trained (got %b, expected still 1)",
              lane_reversed_locked);
    errors = errors + 1;
  end else $display("PASS: latch correctly held value, ignored post-training pulse");

  if (errors == 0) $display("ALL PASS: lane_reversal_latch verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule
