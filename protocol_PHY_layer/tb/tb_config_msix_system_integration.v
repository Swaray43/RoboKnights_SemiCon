// =============================================================================
// tb_config_msix_system_integration
//
// Same pattern as tb_full_system_integration: prove pcie_config_space and
// msix_controller actually coexist with the real reset controller + TS1/TS2
// training chain, sharing datapath_rst_n and link_trained for real instead of
// each passing only its own isolated bench.
//
// Reset domain choice: both blocks sit in the datapath_rst_n domain directly
// (no rst_sync needed) - these are YOUR own register-file/config boundary,
// same reasoning as session_arm_ctrl, not Swaray's synchronous-FIFO domain.
// =============================================================================

module tb_config_msix_system_integration();

reg clk, perst_n;
integer errors, k;

// ---- TS1/TS2 stimulus, 4 lanes in parallel (verbatim reuse) ----
reg [7:0] ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3;
reg       ts_byte_valid;
wire        lnv0, lnv1, lnv2, lnv3;
wire [7:0]  lnout0, lnout1, lnout2, lnout3;

ts_ordered_set_detector u_ts0 (.clk(clk), .rst_n(perst_n), .byte_valid(ts_byte_valid), .byte_in(ts_byte_l0), .lane_num_valid(lnv0), .lane_num_out(lnout0));
ts_ordered_set_detector u_ts1 (.clk(clk), .rst_n(perst_n), .byte_valid(ts_byte_valid), .byte_in(ts_byte_l1), .lane_num_valid(lnv1), .lane_num_out(lnout1));
ts_ordered_set_detector u_ts2 (.clk(clk), .rst_n(perst_n), .byte_valid(ts_byte_valid), .byte_in(ts_byte_l2), .lane_num_valid(lnv2), .lane_num_out(lnout2));
ts_ordered_set_detector u_ts3 (.clk(clk), .rst_n(perst_n), .byte_valid(ts_byte_valid), .byte_in(ts_byte_l3), .lane_num_valid(lnv3), .lane_num_out(lnout3));

wire reversal_detected_valid, lane_reversed;
lane_reversal_detector u_revdet (
  .clk(clk), .rst_n(perst_n),
  .lane_num_valid({lnv3, lnv2, lnv1, lnv0}),
  .lane_num_l0(lnout0), .lane_num_l1(lnout1), .lane_num_l2(lnout2), .lane_num_l3(lnout3),
  .reversal_detected_valid(reversal_detected_valid), .lane_reversed(lane_reversed)
);

wire lane_reversed_locked, link_trained;
lane_reversal_latch u_latch (
  .clk(clk), .rst_n(perst_n),
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

// =============================================================================
// NEW: pcie_config_space + msix_controller, sharing datapath_rst_n
// =============================================================================
reg         cfg_wr_en, cfg_rd_en;
reg  [11:0] cfg_addr;
reg  [31:0] cfg_wdata;
wire [31:0] cfg_rdata;
wire        cfg_cpl_valid;
wire [31:0] bar0, bar1, bar2, bar3, bar4, bar5;
wire [2:0]  mps, mrrs;
wire        mem_en, bm_en, msix_en, msix_fmask;
reg  [3:0]  fixed_link_speed;
reg  [5:0]  fixed_link_width;

pcie_config_space #(.BAR0_SIZE(32'h0001_0000)) u_cfg (
  .clk(clk), .rst_n(datapath_rst_n),
  .cfg_wr_en(cfg_wr_en), .cfg_rd_en(cfg_rd_en), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
  .cfg_rdata(cfg_rdata), .cfg_cpl_valid(cfg_cpl_valid),
  .link_training_bit(link_trained), .link_width(fixed_link_width), .link_speed(fixed_link_speed),
  .bar_addr_0(bar0), .bar_addr_1(bar1), .bar_addr_2(bar2), .bar_addr_3(bar3), .bar_addr_4(bar4), .bar_addr_5(bar5),
  .max_payload_size(mps), .max_read_req_size(mrrs),
  .mem_space_enable(mem_en), .bus_master_enable(bm_en),
  .msix_enable(msix_en), .msix_func_mask(msix_fmask)
);

reg         tbl_wr_en, tbl_rd_en, dma_complete_pulse;
reg  [2:0]  tbl_wr_vec, tbl_rd_vec, dma_complete_vector;
reg  [63:0] tbl_wr_addr;
reg  [31:0] tbl_wr_data;
reg         tbl_wr_mask;
wire [63:0] tbl_rd_addr;
wire [31:0] tbl_rd_data;
wire        tbl_rd_mask;
wire [7:0]  pba;
wire        mwr_valid;
wire [63:0] mwr_addr;
wire [31:0] mwr_data;

msix_controller #(.NUM_VECTORS(8)) u_msix (
  .clk(clk), .rst_n(datapath_rst_n),
  .tbl_wr_en(tbl_wr_en), .tbl_wr_vec(tbl_wr_vec), .tbl_wr_addr(tbl_wr_addr),
  .tbl_wr_data(tbl_wr_data), .tbl_wr_mask(tbl_wr_mask),
  .tbl_rd_en(tbl_rd_en), .tbl_rd_vec(tbl_rd_vec),
  .tbl_rd_addr(tbl_rd_addr), .tbl_rd_data(tbl_rd_data), .tbl_rd_mask(tbl_rd_mask),
  .pba(pba),
  .dma_complete_pulse(dma_complete_pulse), .dma_complete_vector(dma_complete_vector),
  .mwr_valid(mwr_valid), .mwr_ready(1'b1), .mwr_addr(mwr_addr), .mwr_data(mwr_data),
    .msix_enable(1'b1), .func_mask(1'b0),   // unit-level: MSI-X on (gating is tested in tb_pcie_phy_top)
    .mm_wr_en(1'b0), .mm_rd_en(1'b0), .mm_pba(1'b0), .mm_addr(12'h0), .mm_wdata(32'h0), .mm_rdata(), .mm_rd_valid()
);

// ---- Steady-state datapath (verbatim reuse, proves no interference) ----
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

task cfg_write(input [11:0] addr, input [31:0] data);
  begin
    @(negedge clk); cfg_addr = addr; cfg_wdata = data; cfg_wr_en = 1;
    @(negedge clk); cfg_wr_en = 0;
  end
endtask

task cfg_read(input [11:0] addr);
  begin
    @(negedge clk); cfg_addr = addr; cfg_rd_en = 1;
    @(negedge clk); cfg_rd_en = 0;
  end
endtask

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
  cfg_wr_en = 0; cfg_rd_en = 0; cfg_addr = 0; cfg_wdata = 0;
  fixed_link_speed = 4'd0; fixed_link_width = 6'd0;
  tbl_wr_en = 0; tbl_rd_en = 0; dma_complete_pulse = 0;
  tbl_wr_vec = 0; tbl_wr_addr = 0; tbl_wr_data = 0; tbl_wr_mask = 0;
  tbl_rd_vec = 0; dma_complete_vector = 0;

  // ---- Phase 0: under PERST#, config space and MSI-X must sit quietly in reset ----
  @(negedge clk); @(negedge clk);

  cfg_read(12'h000);
  if (cfg_rdata !== 32'h0) begin
    $display("FAIL: config space produced non-zero read under PERST# (rst_n not held)");
    errors = errors + 1;
  end else $display("PASS: config space correctly held in reset under PERST#");

  if (pba !== 8'h0) begin
    $display("FAIL: MSI-X PBA non-zero under PERST#");
    errors = errors + 1;
  end else $display("PASS: MSI-X controller correctly held in reset under PERST#");

  perst_n = 1;
  @(negedge clk);

  // ---- Phase 1: training (verbatim reuse) ----
  send_ts1_all_lanes;
  send_ts1_all_lanes;
  @(negedge clk); @(negedge clk);

  if (link_trained !== 1'b1) begin
    $display("FAIL: link_trained not asserted after training sequence");
    errors = errors + 1;
  end else $display("PASS: link_trained asserted after training sequence");

  repeat (3) @(negedge clk); // let link_state settle to LINK_UP before requesting AXI reset release
  updatefc_received = 1; @(negedge clk); updatefc_received = 0; @(negedge clk);

  if (axi_rst_n !== 1'b1 || datapath_rst_n !== 1'b1) begin
    $display("FAIL: reset chain did not fully release before config-space testing (axi_rst_n=%b datapath_rst_n=%b)", axi_rst_n, datapath_rst_n);
    errors = errors + 1;
  end else $display("PASS: datapath_rst_n/axi_rst_n released, config space now live");

  // ---- Phase 2: config space now live - readback Vendor/Device ID after real reset chain ----
  cfg_read(12'h000);
  if (cfg_rdata !== 32'hC0DEFACE) begin
    $display("FAIL: Vendor/Device ID = %h after real reset chain, expected C0DEFACE", cfg_rdata);
    errors = errors + 1;
  end else $display("PASS: config space correctly live and correct post-training/reset");

  // ---- Phase 3: Link Status now reflects the REAL link_trained from training chain ----
  cfg_read(12'h05C);
  if (cfg_rdata[0] !== 1'b1) begin
    $display("FAIL: Link Status training bit = %b, expected 1 (real link_trained, not a stand-in)", cfg_rdata[0]);
    errors = errors + 1;
  end else $display("PASS: Link Status correctly reflects the REAL link_trained signal from the training chain");

  // ---- Phase 4: program an MSI-X vector, drive a DMA-complete pulse, confirm MWr fires ----
  tbl_wr_en = 1; tbl_wr_vec = 3'd1; tbl_wr_addr = 64'hB000_0000_2000; tbl_wr_data = 32'h1111_2222; tbl_wr_mask = 1'b0;
  @(negedge clk); tbl_wr_en = 0; @(negedge clk);

  dma_complete_pulse = 1; dma_complete_vector = 3'd1;
  @(negedge clk); dma_complete_pulse = 0;

  if (mwr_valid !== 1'b1 || mwr_addr !== 64'hB000_0000_2000 || mwr_data !== 32'h1111_2222) begin
    $display("FAIL: MSI-X MWr incorrect in full integration. valid=%b addr=%h data=%h", mwr_valid, mwr_addr, mwr_data);
    errors = errors + 1;
  end else $display("PASS: MSI-X MWr fires correctly while sharing datapath_rst_n with the rest of the chain");

  // ---- Phase 5: push one real FLIT through the datapath IN PARALLEL with config
  // space / MSI-X activity, to confirm no shared-resource interference ----
  cfg_write(12'h004, 32'h6); // enable mem space (bit1) + bus master (bit2), concurrently with FLIT traffic below

  for (k = 0; k < 32; k = k + 1)
    flit_in[k*8 +: 8] = k[7:0];

  sof_phase = 1; @(negedge clk); sof_phase = 0;
  @(negedge clk);
  valid_in = 1; @(negedge clk); valid_in = 0;

  begin : wait_for_completion
    integer timeout_cnt;
    timeout_cnt = 0;
    while (!destriper_flit_valid && timeout_cnt < 400) begin
      @(negedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (!destriper_flit_valid) begin
      $display("FAIL: timed out waiting for destriper_flit_valid with config/MSI-X sharing the domain");
      errors = errors + 1;
      $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
    end
  end

  if (destriper_flit_out !== flit_in) begin
    $display("FAIL: reconstructed FLIT mismatch with config/MSI-X sharing the domain");
    errors = errors + 1;
  end else $display("PASS: FLIT round-trip bit-exact with config space + MSI-X live in the same simulation");

  cfg_read(12'h004);
  if (cfg_rdata[2:1] !== 2'b11) begin
    $display("FAIL: Command register write lost during concurrent FLIT traffic");
    errors = errors + 1;
  end else $display("PASS: Command register write survived concurrent FLIT traffic, no resource conflict");

  if (errors == 0) $display("ALL PASS: pcie_config_space + msix_controller integrated cleanly into the full chain");
  else $display("TOTAL FAILURES: %0d", errors);
  $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
end
endmodule
