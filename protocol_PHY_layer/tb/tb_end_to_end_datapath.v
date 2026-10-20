// =============================================================================
// TOP-LEVEL INTEGRATION TESTBENCH
// striper -> 4x lane_symbol_pipe -> lane_swap_mux (reversal=0) -> flit_sof_strip
// -> flit_lane_destriper -> compare reconstructed FLIT to original.
//
// Elastic buffer and TS1/TS2 training are deliberately NOT included in this
// test - those are startup/training-time concerns and are verified separately
// per-block. This test isolates the core data-path claim only.
//
// KNOWN RISK, stated up front: the SOF-then-payload cycle alignment below is
// a best-effort estimate, not a hand-traced guarantee, given the striper's
// own load latency plus each lane pipe's multi-cycle serialize/deserialize
// latency. A timeout guard is included so a misalignment fails loudly
// instead of hanging the simulation. If it fails, check the waveform for
// when striper_lane_valid first pulses vs. when pipe_valid0 first pulses.
// =============================================================================
module tb_end_to_end_datapath();

reg clk, rst_n;
reg valid_in;
reg [255:0] flit_in;
wire striper_ready;
wire [3:0] striper_lane_valid;
wire [31:0] striper_lane_data;

wire [7:0] pipe_out0, pipe_out1, pipe_out2, pipe_out3;
wire pipe_valid0, pipe_valid1, pipe_valid2, pipe_valid3;
wire pipe_busy0, pipe_busy1, pipe_busy2, pipe_busy3;

// tx_elastic_deskew sits between the striper (which emits one byte/lane every
// cycle, back-to-back) and the lane pipes (which take 4 cycles per byte to
// serialize). Without this, most striper bytes get silently dropped because
// byte_to_symbol ignores new input while busy - this was the actual cause of
// the "timed out" failure, not a cycle-count off-by-one.
wire [7:0] deskew_out0, deskew_out1, deskew_out2, deskew_out3;
wire deskew_valid;

// GUARD: pipe_busyN lags one full cycle behind the byte actually being
// accepted by byte_to_symbol (active<=1 only takes effect on the NEXT edge).
// Without this guard, lanes_ready stays high for that one extra cycle and
// the deskew buffer issues a second read before busy can block it - the
// extra byte gets silently dropped since the pipe isn't ready to accept it
// yet. read_pending blocks exactly that one-cycle gap.
reg read_pending;
wire lanes_ready = !pipe_busy0 && !pipe_busy1 && !pipe_busy2 && !pipe_busy3 && !read_pending;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) read_pending <= 1'b0;
  else        read_pending <= lanes_ready; // block the cycle immediately after any read we issue
end

tx_elastic_deskew #(.DEPTH(8), .THRESHOLD(4)) deskew (
  .clk(clk), .rst_n(rst_n),
  .wr_en(striper_lane_valid),
  .wr_data_0(striper_lane_data[7:0]),   .wr_data_1(striper_lane_data[15:8]),
  .wr_data_2(striper_lane_data[23:16]), .wr_data_3(striper_lane_data[31:24]),
  .rd_en(lanes_ready),
  .rd_data_0(deskew_out0), .rd_data_1(deskew_out1),
  .rd_data_2(deskew_out2), .rd_data_3(deskew_out3),
  .lanes_valid_out(deskew_valid)
);

// elastic_fifo's rd_data is a registered output - valid one cycle after the
// read that produced it - so byte_in_valid to the lane pipes must lag rd_en
// by exactly one cycle to line up with the data it's meant to describe.
reg deskew_valid_d;
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) deskew_valid_d <= 1'b0;
  else        deskew_valid_d <= deskew_valid && lanes_ready;
end

reg  sof_phase;
wire [7:0] strip_in0, strip_in1, strip_in2, strip_in3;
wire strip_in_valid;

wire [7:0] swap_out0, swap_out1, swap_out2, swap_out3;
wire [7:0] strip_out0, strip_out1, strip_out2, strip_out3;
wire [3:0] strip_valid_out;

wire destriper_flit_valid;
wire [255:0] destriper_flit_out;

integer errors, cyc;

flit_lane_striper striper (
  .clk(clk), .rst_n(rst_n),
  .valid_in(valid_in), .flit_in(flit_in), .ready_out(striper_ready),
  .lane_valid_out(striper_lane_valid), .lane_data_out(striper_lane_data)
);

lane_symbol_pipe lane0 (.clk(clk), .rst_n(rst_n),
  .byte_in_valid(deskew_valid_d), .byte_in(deskew_out0),
  .byte_out_valid(pipe_valid0), .byte_out(pipe_out0), .busy(pipe_busy0));
lane_symbol_pipe lane1 (.clk(clk), .rst_n(rst_n),
  .byte_in_valid(deskew_valid_d), .byte_in(deskew_out1),
  .byte_out_valid(pipe_valid1), .byte_out(pipe_out1), .busy(pipe_busy1));
lane_symbol_pipe lane2 (.clk(clk), .rst_n(rst_n),
  .byte_in_valid(deskew_valid_d), .byte_in(deskew_out2),
  .byte_out_valid(pipe_valid2), .byte_out(pipe_out2), .busy(pipe_busy2));
lane_symbol_pipe lane3 (.clk(clk), .rst_n(rst_n),
  .byte_in_valid(deskew_valid_d), .byte_in(deskew_out3),
  .byte_out_valid(pipe_valid3), .byte_out(pipe_out3), .busy(pipe_busy3));

// SOF injection mux: swap between the injected SOF byte and live pipe output
assign strip_in0 = sof_phase ? 8'hFB : pipe_out0;
assign strip_in1 = sof_phase ? 8'hFB : pipe_out1;
assign strip_in2 = sof_phase ? 8'hFB : pipe_out2;
assign strip_in3 = sof_phase ? 8'hFB : pipe_out3;
assign strip_in_valid = sof_phase ? 1'b1 : pipe_valid0; // assumes all 4 lanes run in lockstep

lane_swap_mux swap (
  .lane_reversed(1'b0),
  .lane_in_0(strip_in0), .lane_in_1(strip_in1),
  .lane_in_2(strip_in2), .lane_in_3(strip_in3),
  .lane_out_0(swap_out0), .lane_out_1(swap_out1),
  .lane_out_2(swap_out2), .lane_out_3(swap_out3)
);

flit_sof_strip sof_strip (
  .clk(clk), .rst_n(rst_n),
  .lane_in_0(swap_out0), .lane_in_1(swap_out1),
  .lane_in_2(swap_out2), .lane_in_3(swap_out3),
  .lane_in_valid(strip_in_valid),
  .lane_out_0(strip_out0), .lane_out_1(strip_out1),
  .lane_out_2(strip_out2), .lane_out_3(strip_out3),
  .lane_valid_out(strip_valid_out)
);

flit_lane_destriper destriper (
  .clk(clk), .rst_n(rst_n),
  .lane_valid_in(strip_valid_out),
  .lane_data_in({strip_out3, strip_out2, strip_out1, strip_out0}),
  .flit_valid_out(destriper_flit_valid),
  .flit_out(destriper_flit_out)
);

initial clk = 0;
always #5 clk = ~clk;

initial begin
  errors = 0;
  rst_n = 0; valid_in = 0; flit_in = 256'd0; sof_phase = 0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;

  // Known test pattern: byte N = value N, makes any mismatch immediately readable
  for (cyc = 0; cyc < 32; cyc = cyc + 1)
    flit_in[cyc*8 +: 8] = cyc[7:0];

  // Inject one SOF cycle first
  sof_phase = 1;
  @(negedge clk);
  sof_phase = 0;

  // Push the real FLIT into the striper
  @(negedge clk);
  if (!striper_ready) begin
    $display("FAIL: striper not ready to accept FLIT");
    errors = errors + 1;
  end
  valid_in = 1;
  @(negedge clk);
  valid_in = 0;

  // Wait for completion with a timeout guard, so a misaligned SOF/payload
  // sequencing fails loudly instead of hanging the simulation forever.
  begin : wait_for_completion
    integer timeout_cnt;
    timeout_cnt = 0;
    while (!destriper_flit_valid && timeout_cnt < 400) begin
      @(negedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (!destriper_flit_valid) begin
      $display("FAIL: timed out waiting for destriper_flit_valid - chain stalled somewhere");
      errors = errors + 1;
      $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
    end
  end

  if (destriper_flit_out !== flit_in) begin
    $display("FAIL: reconstructed FLIT mismatch");
    $display("  got:      %h", destriper_flit_out);
    $display("  expected: %h", flit_in);
    errors = errors + 1;
  end else begin
    $display("PASS: end-to-end FLIT round-trip bit-exact");
  end

  if (errors == 0) $display("ALL PASS: full data-path integration verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
end
endmodule
