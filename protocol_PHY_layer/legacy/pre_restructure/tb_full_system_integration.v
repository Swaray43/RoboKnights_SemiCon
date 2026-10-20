// =============================================================================
// tb_full_system_integration
//
// Purpose: prove rst_sync, fifo_scoreboard, and session_arm_ctrl actually
// coexist with the full reset-controller + TS1/TS2 training + steady-state
// datapath chain in ONE simulation, sharing the same clk/reset domains they'll
// share for real - not just passing in isolation the way each did in its own
// standalone testbench (tb_rst_sync, and whatever ad-hoc bench built
// fifo_scoreboard/session_arm_ctrl before this).
//
// Reused verbatim from tb_startup_integration (pcie_phy_datapath.v):
//   - TS1/TS2 detector -> lane_reversal_detector -> lane_reversal_latch chain
//   - pcie_reset_controller instantiation and its clk/perst_n stimulus style
//   - the steady-state striper -> deskew -> lane pipes -> swap -> sof-strip ->
//     destriper chain, to confirm the new blocks don't disturb it
//
// NEW in this file:
//   - rst_sync, sitting between datapath_rst_n and a synthetic "ext domain"
//   - fifo_scoreboard, driven by testbench-modeled fifo_occupancy/fifo_full/
//     fifo_hwm/err_tkeep_gap (the actual FIFO RTL isn't in our repo, so
//     these are hand-driven stimulus standing in for his module's outputs -
//     this validates OUR wiring and counting logic, not his FIFO itself)
//   - session_arm_ctrl, sharing datapath_rst_n (this is a register in OUR
//     boundary, so it uses our sync reset directly, no rst_sync
//     needed for this one)
// =============================================================================

module tb_full_system_integration();

wire datapath_rst_n_probe;
reg clk, perst_n;
integer errors, cyc, k;

// ---- TS1/TS2 stimulus, 4 lanes in parallel (verbatim reuse) ----
reg [7:0] ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3;
reg       ts_byte_valid;

wire        lnv0, lnv1, lnv2, lnv3;
wire [7:0]  lnout0, lnout1, lnout2, lnout3;

ts_ordered_set_detector u_ts0 (.clk(clk), .rst_n(datapath_rst_n_probe), .byte_valid(ts_byte_valid), .byte_in(ts_byte_l0), .lane_num_valid(lnv0), .lane_num_out(lnout0));
ts_ordered_set_detector u_ts1 (.clk(clk), .rst_n(datapath_rst_n_probe), .byte_valid(ts_byte_valid), .byte_in(ts_byte_l1), .lane_num_valid(lnv1), .lane_num_out(lnout1));
ts_ordered_set_detector u_ts2 (.clk(clk), .rst_n(datapath_rst_n_probe), .byte_valid(ts_byte_valid), .byte_in(ts_byte_l2), .lane_num_valid(lnv2), .lane_num_out(lnout2));
ts_ordered_set_detector u_ts3 (.clk(clk), .rst_n(datapath_rst_n_probe), .byte_valid(ts_byte_valid), .byte_in(ts_byte_l3), .lane_num_valid(lnv3), .lane_num_out(lnout3));

wire reversal_detected_valid, lane_reversed;
lane_reversal_detector u_revdet (
  .clk(clk), .rst_n(datapath_rst_n_probe),
  .lane_num_valid({lnv3, lnv2, lnv1, lnv0}),
  .lane_num_l0(lnout0), .lane_num_l1(lnout1), .lane_num_l2(lnout2), .lane_num_l3(lnout3),
  .reversal_detected_valid(reversal_detected_valid), .lane_reversed(lane_reversed)
);

wire lane_reversed_locked, link_trained;
lane_reversal_latch u_latch (
  .clk(clk), .rst_n(datapath_rst_n_probe),
  .reversal_detected_valid(reversal_detected_valid), .lane_reversed_in(lane_reversed),
  .lane_reversed_locked(lane_reversed_locked), .link_trained(link_trained)
);

// ---- Reset controller (verbatim reuse) ----
reg hot_reset_request, flr_request, updatefc_received;
wire datapath_rst_n, axi_rst_n, flr_active;

pcie_reset_controller u_rst (
  .clk(clk), .perst_n(perst_n),
  .hot_reset_request(hot_reset_request), .flr_request(flr_request),
  .link_trained(link_trained), .updatefc_received(updatefc_received),
  .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n), .flr_active(flr_active)
);

assign datapath_rst_n_probe = perst_n;

// =============================================================================
// NEW BLOCK 1: rst_sync + fifo_scoreboard, sitting in other reset domain
// =============================================================================
wire ext_rst_n;
rst_sync u_rst_sync (
  .clk(clk),
  .async_rst_n(datapath_rst_n),
  .sync_rst_n(ext_rst_n)
);

reg [4:0] fake_fifo_occupancy;
reg       fake_fifo_full, fake_fifo_hwm, fake_err_tkeep_gap;
wire [31:0] err_tkeep_gap_count, fifo_hwm_cycles, fifo_full_cycles;
wire [4:0]  fifo_occupancy_peak;

fifo_scoreboard u_scoreboard (
  .clk(clk), .rst_n(sext_rst_n),
  .fifo_occupancy(fake_fifo_occupancy),
  .fifo_full(fake_fifo_full),
  .fifo_hwm(fake_fifo_hwm),
  .err_tkeep_gap(fake_err_tkeep_gap),
  .err_tkeep_gap_count(err_tkeep_gap_count),
  .fifo_occupancy_peak(fifo_occupancy_peak),
  .fifo_hwm_cycles(fifo_hwm_cycles),
  .fifo_full_cycles(fifo_full_cycles),
  .clear_err_count(1'b0), .clear_occ_peak(1'b0),
  .clear_hwm_cycles(1'b0), .clear_full_cycles(1'b0)
);

// =============================================================================
// NEW BLOCK 2: session_arm_ctrl, sharing OUR sync reset (not external domain -
// this is our own register-file boundary, no rst_sync needed here)
// =============================================================================
reg  [63:0] session_base_addr;
reg  [2:0]  session_tc;
reg         session_arm, session_end;
wire [63:0] session_base_addr_latched;
wire [2:0]  session_tc_latched;
wire        session_valid;
wire [7:0]  session_arm_while_active_cnt;

session_arm_ctrl u_session (
  .clk(clk), .rst_n(datapath_rst_n),
  .base_addr_in(session_base_addr), .tc_in(session_tc),
  .session_arm(session_arm), .session_end(session_end),
  .base_addr_latched(session_base_addr_latched), .tc_latched(session_tc_latched),
  .session_valid(session_valid),
  .session_arm_while_active_cnt(session_arm_while_active_cnt)
);

// ---- Steady-state datapath (verbatim reuse from tb_startup_integration) ----
reg valid_in;
reg [255:0] flit_in;
wire striper_ready;
wire [3:0] striper_lane_valid;
wire [31:0] striper_lane_data;

wire [7:0] deskew_out0, deskew_out1, deskew_out2, deskew_out3;
wire deskew_valid;
wire pipe_busy0, pipe_busy1, pipe_busy2, pipe_busy3;
reg read_pending;
wire lanes_ready = !pipe_busy0 && !pipe_busy1 && !pipe_busy2 && !pipe_busy3 && !read_pending;

always @(posedge clk or negedge datapath_rst_n) begin
  if (!datapath_rst_n) read_pending <= 1'b0;
  else                 read_pending <= lanes_ready;
end

tx_elastic_deskew #(.DEPTH(8), .THRESHOLD(4)) u_deskew (
  .clk(clk), .rst_n(datapath_rst_n),
  .wr_en(striper_lane_valid),
  .wr_data_0(striper_lane_data[7:0]),   .wr_data_1(striper_lane_data[15:8]),
  .wr_data_2(striper_lane_data[23:16]), .wr_data_3(striper_lane_data[31:24]),
  .rd_en(lanes_ready),
  .rd_data_0(deskew_out0), .rd_data_1(deskew_out1),
  .rd_data_2(deskew_out2), .rd_data_3(deskew_out3),
  .lanes_valid_out(deskew_valid)
);

reg deskew_valid_d;
always @(posedge clk or negedge datapath_rst_n) begin
  if (!datapath_rst_n) deskew_valid_d <= 1'b0;
  else                 deskew_valid_d <= deskew_valid && lanes_ready;
end

wire [7:0] pipe_out0, pipe_out1, pipe_out2, pipe_out3;
wire pipe_valid0, pipe_valid1, pipe_valid2, pipe_valid3;

flit_lane_striper u_striper (
  .clk(clk), .rst_n(datapath_rst_n),
  .valid_in(valid_in), .flit_in(flit_in), .ready_out(striper_ready),
  .lane_valid_out(striper_lane_valid), .lane_data_out(striper_lane_data)
);

lane_symbol_pipe u_lane0 (.clk(clk), .rst_n(datapath_rst_n), .byte_in_valid(deskew_valid_d), .byte_in(deskew_out0), .byte_out_valid(pipe_valid0), .byte_out(pipe_out0), .busy(pipe_busy0));
lane_symbol_pipe u_lane1 (.clk(clk), .rst_n(datapath_rst_n), .byte_in_valid(deskew_valid_d), .byte_in(deskew_out1), .byte_out_valid(pipe_valid1), .byte_out(pipe_out1), .busy(pipe_busy1));
lane_symbol_pipe u_lane2 (.clk(clk), .rst_n(datapath_rst_n), .byte_in_valid(deskew_valid_d), .byte_in(deskew_out2), .byte_out_valid(pipe_valid2), .byte_out(pipe_out2), .busy(pipe_busy2));
lane_symbol_pipe u_lane3 (.clk(clk), .rst_n(datapath_rst_n), .byte_in_valid(deskew_valid_d), .byte_in(deskew_out3), .byte_out_valid(pipe_valid3), .byte_out(pipe_out3), .busy(pipe_busy3));

reg sof_phase;
wire [7:0] strip_in0 = sof_phase ? 8'hFB : pipe_out0;
wire [7:0] strip_in1 = sof_phase ? 8'hFB : pipe_out1;
wire [7:0] strip_in2 = sof_phase ? 8'hFB : pipe_out2;
wire [7:0] strip_in3 = sof_phase ? 8'hFB : pipe_out3;
wire strip_in_valid  = sof_phase ? 1'b1  : pipe_valid0;

wire [7:0] swap_out0, swap_out1, swap_out2, swap_out3;
lane_swap_mux u_swap (
  .lane_reversed(lane_reversed_locked),
  .lane_in_0(strip_in0), .lane_in_1(strip_in1), .lane_in_2(strip_in2), .lane_in_3(strip_in3),
  .lane_out_0(swap_out0), .lane_out_1(swap_out1), .lane_out_2(swap_out2), .lane_out_3(swap_out3)
);

wire [7:0] strip_out0, strip_out1, strip_out2, strip_out3;
wire [3:0] strip_valid_out;
flit_sof_strip u_sofstrip (
  .clk(clk), .rst_n(datapath_rst_n),
  .lane_in_0(swap_out0), .lane_in_1(swap_out1), .lane_in_2(swap_out2), .lane_in_3(swap_out3),
  .lane_in_valid(strip_in_valid),
  .lane_out_0(strip_out0), .lane_out_1(strip_out1), .lane_out_2(strip_out2), .lane_out_3(strip_out3),
  .lane_valid_out(strip_valid_out)
);

wire destriper_flit_valid;
wire [255:0] destriper_flit_out;
flit_lane_destriper u_destriper (
  .clk(clk), .rst_n(datapath_rst_n),
  .lane_valid_in(strip_valid_out),
  .lane_data_in({strip_out3, strip_out2, strip_out1, strip_out0}),
  .flit_valid_out(destriper_flit_valid), .flit_out(destriper_flit_out)
);

initial clk = 0;
always #5 clk = ~clk;

task send_ts1_all_lanes;
  begin
    ts_byte_valid = 1;
    ts_byte_l0 = 8'hBC; ts_byte_l1 = 8'hBC; ts_byte_l2 = 8'hBC; ts_byte_l3 = 8'hBC;
    @(negedge clk);
    ts_byte_l0 = 8'h00; ts_byte_l1 = 8'h00; ts_byte_l2 = 8'h00; ts_byte_l3 = 8'h00;
    @(negedge clk);
    ts_byte_l0 = 8'd0; ts_byte_l1 = 8'd1; ts_byte_l2 = 8'd2; ts_byte_l3 = 8'd3;
    @(negedge clk);
    for (k = 3; k <= 9; k = k + 1) begin
      ts_byte_l0 = 8'h00; ts_byte_l1 = 8'h00; ts_byte_l2 = 8'h00; ts_byte_l3 = 8'h00;
      @(negedge clk);
    end
    for (k = 10; k <= 15; k = k + 1) begin
      ts_byte_l0 = 8'h4A; ts_byte_l1 = 8'h4A; ts_byte_l2 = 8'h4A; ts_byte_l3 = 8'h4A;
      @(negedge clk);
    end
    ts_byte_valid = 0;
  end
endtask

initial begin
  errors = 0;
  perst_n = 0; ts_byte_valid = 0;
  ts_byte_l0 = 0; ts_byte_l1 = 0; ts_byte_l2 = 0; ts_byte_l3 = 0;
  hot_reset_request = 0; flr_request = 0; updatefc_received = 0;
  valid_in = 0; flit_in = 256'd0; sof_phase = 0;
  fake_fifo_occupancy = 0; fake_fifo_full = 0; fake_fifo_hwm = 0; fake_err_tkeep_gap = 0;
  session_base_addr = 64'h0; session_tc = 3'd0; session_arm = 0; session_end = 0;
  $monitor("t=%0t perst_n=%b probe=%b u_ts0.rst_n=%b u_ts0.state=%b u_ts0.byte_idx=%0d ts_byte_valid=%b ts_byte_l0=%h lnv0=%b link_trained=%b",
          $time, perst_n, datapath_rst_n_probe, u_ts0.rst_n, u_ts0.state, u_ts0.byte_idx,
          ts_byte_valid, ts_byte_l0, lnv0, link_trained);
  // ---- Phase 0: under PERST#, every new block must sit quietly in reset,
  // sharing the domain correctly rather than free-running ----
  @(negedge clk); @(negedge clk);

  if (ext_rst_n !== 1'b0) begin
    $display("FAIL: ext_rst_n not held low under PERST#");
    errors = errors + 1;
  end else $display("PASS: ext_rst_n correctly held low under PERST#");

  if (session_valid !== 1'b0) begin
    $display("FAIL: session_valid not held low under PERST#");
    errors = errors + 1;
  end else $display("PASS: session_valid correctly held low under PERST#");

  perst_n = 1;
  @(negedge clk);

  // ---- Phase 1: training (verbatim reuse) ----
  send_ts1_all_lanes;
  send_ts1_all_lanes;
  @(negedge clk);
  @(negedge clk);

  if (link_trained !== 1'b1) begin
    $display("FAIL: link_trained not asserted after training sequence");
    errors = errors + 1;
  end else $display("PASS: link_trained asserted after training sequence");

  // ---- Phase 2: rst_sync deassertion timing ----
  // datapath_rst_n rose synchronously (perst_n was already high, no async
  // edge here), so sync_rst_n should track it within rst_sync's 2-stage
  // pipeline - check it eventually rises and never rises BEFORE datapath_rst_n.
  if (datapath_rst_n === 1'b1 && ext_rst_n === 1'b0) begin
    $display("INFO: ext_rst_n correctly still settling behind datapath_rst_n (2-FF sync stage)");
  end
  repeat (3) @(negedge clk);
  if (ext_rst_n !== 1'b1) begin
    $display("FAIL: ext_rst_n never deasserted after datapath_rst_n went high");
    errors = errors + 1;
  end else $display("PASS: ext_rst_n correctly followed datapath_rst_n through rst_sync");

  // ---- Phase 3: release AXI reset (verbatim reuse) ----
  updatefc_received = 1;
  @(negedge clk);
  updatefc_received = 0;
  @(negedge clk);

  if (axi_rst_n !== 1'b1) begin
    $display("FAIL: axi_rst_n did not release in full integration");
    errors = errors + 1;
  end else $display("PASS: axi_rst_n released correctly in full integration");

  // ---- Phase 4: exercise fifo_scoreboard against modeled ext FIFO
  // behaviour - ramp occupancy up past HWM, pulse err_tkeep_gap twice,
  // never assert fifo_full (per his note: should stay zero in normal op) ----
  fake_fifo_occupancy = 5'd3;  @(negedge clk);
  fake_fifo_occupancy = 5'd8;  @(negedge clk);
  fake_fifo_occupancy = 5'd12; fake_fifo_hwm = 1; @(negedge clk); // HWM cycle 1
  fake_err_tkeep_gap = 1;      @(negedge clk); // err pulse 1, HWM cycle 2
  fake_err_tkeep_gap = 0;
  fake_fifo_occupancy = 5'd14; @(negedge clk); // HWM cycle 3
  fake_err_tkeep_gap = 1;      @(negedge clk); // err pulse 2, HWM cycle 4
  fake_err_tkeep_gap = 0;
  fake_fifo_occupancy = 5'd10; fake_fifo_hwm = 0; @(negedge clk);
  @(negedge clk); // let counters settle

  if (err_tkeep_gap_count !== 32'd2) begin
    $display("FAIL: err_tkeep_gap_count = %0d, expected 2", err_tkeep_gap_count);
    errors = errors + 1;
  end else $display("PASS: err_tkeep_gap_count correctly counted 2 pulses");

  if (fifo_occupancy_peak !== 5'd14) begin
    $display("FAIL: fifo_occupancy_peak = %0d, expected 14", fifo_occupancy_peak);
    errors = errors + 1;
  end else $display("PASS: fifo_occupancy_peak correctly tracked 14");

  if (fifo_hwm_cycles !== 32'd4) begin
    $display("FAIL: fifo_hwm_cycles = %0d, expected 4", fifo_hwm_cycles);
    errors = errors + 1;
  end else $display("PASS: fifo_hwm_cycles correctly counted 4");

  if (fifo_full_cycles !== 32'd0) begin
    $display("FAIL: fifo_full_cycles = %0d, expected 0 (protocol violation if nonzero)", fifo_full_cycles);
    errors = errors + 1;
  end else $display("PASS: fifo_full_cycles correctly stayed at 0");

  // ---- Phase 5: exercise session_arm_ctrl now that the link is up ----
  session_base_addr = 64'hBEEE_FEED_0000_1234;
  session_tc = 3'd5;
  session_arm = 1;
  @(negedge clk);
  session_arm = 0;
  @(negedge clk); // LATCH -> ACTIVE edge

  if (session_valid !== 1'b1) begin
    $display("FAIL: session_valid did not assert after arm sequence");
    errors = errors + 1;
  end else $display("PASS: session_valid asserted after arm sequence");

  if (session_base_addr_latched !== 64'hDEAD_BEEF_0000_1234 || session_tc_latched !== 3'd5) begin
    $display("FAIL: session_base_addr_latched/tc_latched mismatch");
    $display("  expected base_addr_latched: %h", 64'hDEAD_BEEF_0000_1234);
    $display("  got      base_addr_latched: %h", session_base_addr_latched);
    $display("  expected tc_latched: %b", 3'd5);
    $display("  got      tc_latched: %b", session_tc_latched);
    $display("  driven   base_addr_in (live): %h", session_base_addr);
    $display("  driven   tc_in       (live): %b", session_tc);
    $display("  u_session.state at check time: %b (00=IDLE 01=LATCH 10=ACTIVE)", u_session.state);
    $display("  session_valid at check time: %b", session_valid);
    errors = errors + 1;
  end else $display("PASS: base_addr/tc latched correctly");

  session_end = 1;
  @(negedge clk);
  session_end = 0;
  @(negedge clk);

  if (session_valid !== 1'b0) begin
    $display("FAIL: session_valid did not clear after session_end");
    errors = errors + 1;
  end else $display("PASS: session_valid correctly cleared after session_end");

  // ---- Phase 6: push one real FLIT through the datapath, confirm the new
  // blocks (scoreboard, session-arm, rst_sync) sharing the same clk domain
  // haven't disturbed the existing verified datapath ----
  for (cyc = 0; cyc < 32; cyc = cyc + 1)
    flit_in[cyc*8 +: 8] = cyc[7:0];

  sof_phase = 1;
  @(negedge clk);
  sof_phase = 0;

  @(negedge clk);
  valid_in = 1;
  @(negedge clk);
  valid_in = 0;

  begin : wait_for_completion
    integer timeout_cnt;
    timeout_cnt = 0;
    while (!destriper_flit_valid && timeout_cnt < 400) begin
      @(negedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (!destriper_flit_valid) begin
      $display("FAIL: timed out waiting for destriper_flit_valid in full integration");
      errors = errors + 1;
      $stop;
    end
  end

  if (destriper_flit_out !== flit_in) begin
    $display("FAIL: reconstructed FLIT mismatch in full integration");
    errors = errors + 1;
  end else begin
    $display("PASS: FLIT round-trip bit-exact with all new blocks live in the same simulation");
  end

  if (errors == 0) $display("ALL PASS: rst_sync + fifo_scoreboard + session_arm_ctrl integrated cleanly");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule
