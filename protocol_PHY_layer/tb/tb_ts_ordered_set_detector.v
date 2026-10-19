module tb_ts_ordered_set_detector();
reg clk, rst_n;
reg byte_valid;
reg [7:0] byte_in;
wire lane_num_valid;
wire [7:0] lane_num_out;

integer errors;
integer i;

ts_ordered_set_detector dut(.clk(clk), .rst_n(rst_n), .byte_valid(byte_valid),
                             .byte_in(byte_in), .lane_num_valid(lane_num_valid),
                             .lane_num_out(lane_num_out));

initial clk = 0;
always #5 clk = ~clk;

// Sends one full 16-byte ordered set. lane_num = the Lane Number field (byte 2).
// corrupt_id: if 1, flips the last ID byte to break the TS1/TS2 match.
task send_ordered_set(input [7:0] lane_num, input corrupt_id);
  begin
    byte_valid = 1;
    byte_in = 8'hBC; @(negedge clk);            // byte 0: COM
    byte_in = 8'h00; @(negedge clk);            // byte 1: Link Number (don't care)
    byte_in = lane_num; @(negedge clk);         // byte 2: Lane Number
    for (i = 3; i <= 9; i = i + 1) begin
      byte_in = 8'h00; @(negedge clk);          // bytes 3-9: don't care
    end
    for (i = 10; i <= 15; i = i + 1) begin
      if (corrupt_id && i == 15)
        byte_in = 8'h00;                        // break the match on the last ID byte
      else
        byte_in = 8'h4A;                        // TS1 identifier
      @(negedge clk);
    end
    byte_valid = 0;
  end
endtask

initial begin
  errors = 0;
  rst_n = 0; byte_valid = 0; byte_in = 0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;
  @(negedge clk);

  // Test 1: single clean ordered set -> should NOT confirm yet (needs 2 consecutive)
  send_ordered_set(8'd2, 0);
  if (lane_num_valid !== 1'b0) begin
    $display("FAIL: confirmed after only 1 ordered set (expected 2 consecutive required)");
    errors = errors + 1;
  end else $display("PASS: correctly withheld confirmation after 1st ordered set");

  // Test 2: second clean, matching ordered set -> SHOULD confirm now
  send_ordered_set(8'd2, 0);
  if (lane_num_valid !== 1'b1 || lane_num_out !== 8'd2) begin
    $display("FAIL: expected confirmation after 2nd matching set, got valid=%b lane_num=%0d",
              lane_num_valid, lane_num_out);
    errors = errors + 1;
  end else $display("PASS: confirmed lane_num=%0d after 2 consecutive matches", lane_num_out);

  // Test 3: corrupted ordered set should reset the consecutive-match streak
  send_ordered_set(8'd2, 0);   // 1st good one after reset state
  send_ordered_set(8'd2, 1);   // corrupted -> should NOT confirm, and should reset streak
  if (lane_num_valid !== 1'b0) begin
    $display("FAIL: corrupted ordered set incorrectly confirmed");
    errors = errors + 1;
  end else $display("PASS: corrupted ordered set correctly rejected");

  // Test 4: after the corruption, need 2 fresh consecutive matches again
  send_ordered_set(8'd2, 0);
  send_ordered_set(8'd2, 0);
  if (lane_num_valid !== 1'b1 || lane_num_out !== 8'd2) begin
    $display("FAIL: failed to re-confirm after corruption reset the streak");
    errors = errors + 1;
  end else $display("PASS: re-confirmed correctly after streak reset");

  if (errors == 0) $display("ALL PASS: ts_ordered_set_detector verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule
