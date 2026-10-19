module tb_flit_sof_strip();
reg clk, rst_n;
reg [7:0] lane_in_0, lane_in_1, lane_in_2, lane_in_3;
reg lane_in_valid;
wire [7:0] lane_out_0, lane_out_1, lane_out_2, lane_out_3;
wire [3:0] lane_valid_out;

integer errors, i;

flit_sof_strip dut(.clk(clk), .rst_n(rst_n),
  .lane_in_0(lane_in_0), .lane_in_1(lane_in_1), .lane_in_2(lane_in_2), .lane_in_3(lane_in_3),
  .lane_in_valid(lane_in_valid),
  .lane_out_0(lane_out_0), .lane_out_1(lane_out_1), .lane_out_2(lane_out_2), .lane_out_3(lane_out_3),
  .lane_valid_out(lane_valid_out));

initial clk = 0;
always #5 clk = ~clk;

task send_cycle(input [7:0] b0, input [7:0] b1, input [7:0] b2, input [7:0] b3, input v);
  begin
    lane_in_0 = b0; lane_in_1 = b1; lane_in_2 = b2; lane_in_3 = b3; lane_in_valid = v;
    @(negedge clk);
  end
endtask

initial begin
  errors = 0;
  rst_n = 0; lane_in_valid = 0;
  lane_in_0=0; lane_in_1=0; lane_in_2=0; lane_in_3=0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;
  @(negedge clk);

  // Test 1: idle bytes before SOF should never appear at output
  send_cycle(8'h00, 8'h00, 8'h00, 8'h00, 1);
  send_cycle(8'hAA, 8'hAA, 8'hAA, 8'hAA, 1); // garbage, not SOF
  if (lane_valid_out !== 4'b0000) begin
    $display("FAIL: output valid asserted before SOF seen");
    errors = errors + 1;
  end else $display("PASS: idle bytes correctly suppressed pre-SOF");

  // Test 2: send SOF, then 8 payload cycles (byte value = cycle*4+lane, using *17 for variety)
  send_cycle(8'hFB, 8'hFB, 8'hFB, 8'hFB, 1); // SOF - should be consumed, not passed
  if (lane_valid_out !== 4'b0000) begin
    $display("FAIL: SOF cycle itself leaked to output");
    errors = errors + 1;
  end else $display("PASS: SOF cycle correctly consumed, not passed downstream");

  for (i = 0; i < 8; i = i + 1) begin
    send_cycle(i*4+0, i*4+1, i*4+2, i*4+3, 1);
    if (lane_valid_out !== 4'b1111 ||
        lane_out_0 !== i*4+0 || lane_out_1 !== i*4+1 ||
        lane_out_2 !== i*4+2 || lane_out_3 !== i*4+3) begin
      $display("FAIL payload cycle=%0d: valid=%b out=%h,%h,%h,%h",
                i, lane_valid_out, lane_out_0, lane_out_1, lane_out_2, lane_out_3);
      errors = errors + 1;
    end else begin
      $display("PASS payload cycle=%0d", i);
    end
  end

  // Test 3: after 8 payload cycles, state should return to WAIT_SOF -
  // sending non-SOF garbage now should NOT produce valid output
  send_cycle(8'h55, 8'h55, 8'h55, 8'h55, 1);
  if (lane_valid_out !== 4'b0000) begin
    $display("FAIL: output valid after FLIT should have completed and returned to WAIT_SOF");
    errors = errors + 1;
  end else $display("PASS: correctly returned to idle-wait after 8 payload cycles");

  // Test 4: second FLIT, preceded by a fresh SOF - proves the block can frame multiple FLITs
  send_cycle(8'hFB, 8'hFB, 8'hFB, 8'hFB, 1);
  send_cycle(8'h20, 8'h21, 8'h22, 8'h23, 1);
  if (lane_valid_out !== 4'b1111 || lane_out_0 !== 8'h20) begin
    $display("FAIL: second FLIT not framed correctly after first FLIT completed");
    errors = errors + 1;
  end else $display("PASS: second FLIT correctly framed");

  if (errors == 0) $display("ALL PASS: flit_sof_strip verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule
