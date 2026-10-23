`timescale 1ns/1ps
`default_nettype none
// =============================================================================
// tb_pcie_phy_top_wide - Step 4 item 3: pcie_phy_top with PHY_WIDE=1
// (32 PAM4 symbols per lane per clock) through a channel with per-lane
// SYMBOL skew (0..SKEW_MAX symbols, i.e. the byte boundary moves inside the
// 32-symbol lane word, and lanes land up to a word apart), optionally with
// lanes crossed. Checks:
//   - link trains over the lanes (LTSSM reaches L0), lane reversal detected
//   - every Flit word sent on phy_tx_* comes out of phy_rx_* once, in order,
//     bit-exact, with sof on word 0 and eof on word 7 of each Flit
//   - no framing / lane errors
// Lane-to-lane skew must stay below one lane word (32 symbols = 8 bytes,
// the narrow PHY allows 8 bytes too): TS ordered sets repeat every 2 words,
// so a larger skew cannot be told apart from the next ordered set.
// plusargs: +SEED=n +SKEW_MAX=n (31) +REV=0/1 +FLITS=n (60)
// =============================================================================
module tb_pcie_phy_top_wide;

reg clk = 0, aclk = 0;
always #5   clk  = ~clk;
always #3.5 aclk = ~aclk;

integer SEED, SKEW_MAX, REV, FLITS;
reg perst_n = 0, aclk_rst_n = 0;
reg [255:0] phy_tx_data = 0; reg phy_tx_valid = 0, phy_tx_sof = 0, phy_tx_eof = 0;

wire link_trained, lane_reversed_locked, datapath_rst_n, axi_rst_n, flr_active;
wire phy_tx_ready;
wire [383:0] tx_level; wire [3:0] tx_level_valid;
reg  [383:0] rx_level; reg  [3:0] rx_level_valid;
wire [255:0] phy_rx_data; wire phy_rx_valid, phy_rx_sof, phy_rx_eof;
wire phy_tx_idle, phy_framing_err, phy_lane_err, phy_ts_sending, phy_deskew_locked;
wire [1:0] ltssm_state;

pcie_phy_top #(.PHY_WIDE(1)) dut (
  .clk(clk), .perst_n(perst_n),
  .ts_byte_valid(1'b0), .ts_byte_l0(8'd0), .ts_byte_l1(8'd0), .ts_byte_l2(8'd0), .ts_byte_l3(8'd0),
  .link_trained(link_trained), .lane_reversed_locked(lane_reversed_locked),
  .hot_reset_request(1'b0), .flr_request(1'b0), .updatefc_received(1'b0),
  .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n), .flr_active(flr_active),
  .cfg_wr_en(1'b0), .cfg_rd_en(1'b0), .cfg_addr(12'd0), .cfg_wdata(32'd0), .cfg_rdata(), .cfg_cpl_valid(),
  .tbl_wr_en(1'b0), .tbl_wr_vec(3'd0), .tbl_wr_addr(64'd0), .tbl_wr_data(32'd0), .tbl_wr_mask(1'b0),
  .tbl_rd_en(1'b0), .tbl_rd_vec(3'd0), .tbl_rd_addr(), .tbl_rd_data(), .tbl_rd_mask(), .pba(),
  .dma_complete_pulse(1'b0), .dma_complete_vector(3'd0),
  .mwr_valid(), .mwr_ready(1'b1), .mwr_addr(), .mwr_data(),
  .phy_tx_data(phy_tx_data), .phy_tx_valid(phy_tx_valid), .phy_tx_sof(phy_tx_sof), .phy_tx_eof(phy_tx_eof),
  .phy_tx_ready(phy_tx_ready),
  .tx_level(tx_level), .tx_level_valid(tx_level_valid), .rx_level(rx_level), .rx_level_valid(rx_level_valid),
  .phy_rx_data(phy_rx_data), .phy_rx_valid(phy_rx_valid), .phy_rx_sof(phy_rx_sof), .phy_rx_eof(phy_rx_eof),
  .phy_tx_idle(phy_tx_idle), .phy_framing_err(phy_framing_err), .phy_lane_err(phy_lane_err),
  .phy_ts_sending(phy_ts_sending), .phy_deskew_locked(phy_deskew_locked), .ltssm_state(ltssm_state),
  .bar0_wr_en(1'b0), .bar0_rd_en(1'b0), .bar0_addr(16'd0), .bar0_wdata(32'd0), .bar0_rdata(), .bar0_rd_valid(),
  .cfg_base_addr_ch0(), .cfg_base_addr_ch1(), .cfg_base_addr_wr(), .cfg_tc_ch0(), .cfg_tc_ch1(),
  .session_valid(), .session_done(2'b00),
  .aclk(aclk), .aclk_rst_n(aclk_rst_n),
  .session_start_aclk(), .session_end_aclk(2'b00), .session_overflow_aclk(2'b00),
  .arb_weight_ch0(), .arb_weight_ch1(), .session_limit(), .session_limit_ch1(), .link_err_evt(6'd0)
);

// ---------------- channel: lane crossing + per-lane symbol skew ----------------
integer skew [0:3];
reg [287:0] hist [0:3];        // 96 symbols x 3 bits per lane: [0 +: 96] = 2 words ago ... [192 +: 96] = this word
reg [3:0]   vh   [0:2];
integer l, s;
always @(posedge clk) begin
  for (l = 0; l < 4; l = l + 1)
    hist[l] <= {(REV ? tx_level[96*(3-l) +: 96] : tx_level[96*l +: 96]), hist[l][287:96]};
  vh[2] <= vh[1]; vh[1] <= vh[0]; vh[0] <= tx_level_valid;
end
always @(*) begin
  for (l = 0; l < 4; l = l + 1) begin
    // newest 32 symbols end at index 95; this lane is skew[l] symbols late
    for (s = 0; s < 32; s = s + 1)
      rx_level[96*l + 3*s +: 3] = hist[l][3*(64 - skew[l] + s) +: 3];
    rx_level_valid[l] = (vh[1][l] === 1'b1) && (vh[0][l] === 1'b1);
  end
end

// ---------------- scoreboard ----------------
reg [255:0] q [0:4095];
integer qw = 0, qr = 0, ok = 0, bad = 0, sof_bad = 0, k;
always @(posedge clk) if (phy_rx_valid) begin
  if (qr >= qw) begin bad = bad + 1; if (bad < 4) $display("[%0t] EXTRA word %h", $time, phy_rx_data[63:0]); end
  else begin
    if (phy_rx_data !== q[qr % 4096]) begin bad = bad + 1; if (bad < 4) $display("[%0t] word %0d MISMATCH", $time, qr); end
    else ok = ok + 1;
    if (phy_rx_sof !== ((qr % 8) == 0) || phy_rx_eof !== ((qr % 8) == 7)) sof_bad = sof_bad + 1;
    qr = qr + 1;
  end
end

integer f, w, t;
initial begin
  if (!$value$plusargs("SEED=%d", SEED)) SEED = 1;
  if (!$value$plusargs("SKEW_MAX=%d", SKEW_MAX)) SKEW_MAX = 31;   // < 1 lane word: same budget as the narrow PHY (MAX_SKEW 8 bytes)
  if (!$value$plusargs("REV=%d", REV)) REV = 0;
  if (!$value$plusargs("FLITS=%d", FLITS)) FLITS = 60;
  for (l = 0; l < 4; l = l + 1) begin
    t = $random(SEED); if (t < 0) t = -t;
    skew[l] = t % (SKEW_MAX + 1);
    hist[l] = 288'd0;
  end
  vh[0] = 0; vh[1] = 0; vh[2] = 0;
  $display("tb_pcie_phy_top_wide: SEED=%0d skew %0d/%0d/%0d/%0d symbols, lanes %s", SEED, skew[0], skew[1], skew[2], skew[3], REV ? "crossed" : "straight");
  repeat (5) @(posedge clk); perst_n = 1; aclk_rst_n = 1;
  t = 0; while (!link_trained && t < 40000) begin @(posedge clk); t = t + 1; end
  if (!link_trained) begin $display("FAIL: link did not train (ltssm %0d, deskew_locked %b)", ltssm_state, phy_deskew_locked); $display("RESULT: FAIL"); $finish; end
  $display("[%0t] link up after %0d clk, lane_reversed_locked=%b", $time, t, lane_reversed_locked);
  for (f = 0; f < FLITS; f = f + 1)
    for (w = 0; w < 8; w = w + 1) begin
      @(negedge clk);
      phy_tx_data = {$random(SEED), $random(SEED), $random(SEED), $random(SEED), $random(SEED), $random(SEED), $random(SEED), $random(SEED)};
      phy_tx_valid = 1; phy_tx_sof = (w == 0); phy_tx_eof = (w == 7);
      @(posedge clk); while (!phy_tx_ready) @(posedge clk);
      q[qw % 4096] = phy_tx_data; qw = qw + 1;
      if (($random(SEED) & 7) == 0 && w == 7) begin @(negedge clk); phy_tx_valid = 0; repeat ($random(SEED) & 31) @(posedge clk); end
    end
  @(negedge clk); phy_tx_valid = 0;
  t = 0; while (qr < qw && t < 20000) begin @(posedge clk); t = t + 1; end
  repeat (50) @(posedge clk);
  $display(" words sent %0d received ok %0d bad %0d sof/eof errors %0d  framing_err %b lane_err %b  reversed %b",
           qw, ok, bad, sof_bad, phy_framing_err, phy_lane_err, lane_reversed_locked);
  if (bad == 0 && sof_bad == 0 && ok == qw && !phy_framing_err && !phy_lane_err && lane_reversed_locked == (REV != 0))
    $display("RESULT: PASS");
  else
    $display("RESULT: FAIL");
  $finish;
end
initial begin #20_000_000; $display("RESULT: FAIL (timeout)"); $finish; end
endmodule
