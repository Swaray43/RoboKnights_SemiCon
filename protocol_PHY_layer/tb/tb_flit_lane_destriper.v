module tb_flit_lane_destriper();

reg clk, rst_n;
reg [3:0] lane_valid_in;
reg [31:0] lane_data_in;
wire flit_valid_out;
wire [255:0] flit_out;

integer cycle, errors;
reg [255:0] expected_flit;
integer i;

flit_lane_destriper dut (
  .clk(clk), .rst_n(rst_n),
  .lane_valid_in(lane_valid_in),
  .lane_data_in(lane_data_in),
  .flit_valid_out(flit_valid_out),
  .flit_out(flit_out)
);

initial clk = 0;
always #5 clk = ~clk;

initial begin
  expected_flit = 256'd0;
  for (i = 0; i < 32; i = i + 1)
    expected_flit[i*8 +: 8] = i[7:0];
end

initial begin
  errors = 0;
  rst_n = 0;
  lane_valid_in = 4'b0000;
  lane_data_in = 32'd0;

  @(negedge clk);
  @(negedge clk);
  rst_n = 1;

  // Drive 8 cycles of lane data, byte N = index N, same pattern as the striper TB
  for (cycle = 0; cycle < 8; cycle = cycle + 1) begin
    lane_valid_in = 4'b1111;
//    lane_data_in  = {(cycle*4+3), (cycle*4+2), (cycle*4+1), (cycle*4+0)};
    lane_data_in = {8'((cycle*4+3)), 8'((cycle*4+2)), 8'((cycle*4+1)), 8'((cycle*4+0))};//{Lane 3,Lane 2,Lane 1,Lane 0}
    @(negedge clk);
  end
  lane_valid_in = 4'b0000;

  if (flit_valid_out !== 1'b1) begin
    $display("FAIL: flit_valid_out not asserted after 8 cycles");
    errors = errors + 1;
  end

  if (flit_out !== expected_flit) begin
    $display("FAIL: flit_out mismatch");
    $display("  got:      %h", flit_out);
    $display("  expected: %h", expected_flit);
    errors = errors + 1;
  end else begin
    $display("PASS: flit_out reconstructed correctly = %h", flit_out);
  end

  if (errors == 0)
    $display("ALL PASS: de-striper round-trip verified");
  else
    $display("TOTAL FAILURES: %0d", errors);

  $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
end

endmodule
