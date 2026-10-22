// =============================================================================
// tb_pcie_phy_top - self-checking bench for the new synthesizable top.
//
// The SerDes channel is modelled HERE (tx_level -> 1 register stage -> rx_level),
// optionally with lanes crossed (lane i -> lane 3-i) to exercise lane reversal.
//
//   Part A  identity lanes
//     A1  link trains from TS1 ordered sets sent over the lanes, reset release
//     A2  BAR0 register write/readback
//     A3  arm ch0 -> session_valid, base latched, start pulse arrives on aclk
//     A4  session_done -> MSI-X MWr, held until mwr_ready
//     A5  (K1) session ends on session_done, not session_end_aclk; an arm
//         while draining is ignored and counted; STATUS shows draining
//     A6  40 Flits (320 words) of random data, random producer stalls,
//         includes an all-0xFB word; checks data, sof/eof, ready backpressure,
//         and the PHY drain rate (~260 clk per 256-byte Flit)
//   Part F  config-space gating: MSI-X table/PBA in BAR0, function mask,
//           MSI-X enable, Memory Space Enable
//   Part B  crossed lanes (lane reversal)
//     B1  crossed channel -> detectors see 3,2,1,0 -> lane_reversed_locked = 1
//     B2  10 Flits bit-exact through the reversed channel
//   Part C  framing check: a word with sof in the wrong place sets framing_err
//   Part D  Flits offered before link-up wait behind TS1 and arrive intact
//   Part E  channel connected mid-byte: lane_rx locks on COM, then trains
//   Part H  hot reset: link drops, TS1 resent, retrains, fresh UpdateFC, data
//   Part I  no partner: Polling times out to Detect, retries, then trains
//   Part G  per-lane channel skew (up to 30 clk, odd amounts, also with
//           crossed lanes): deskew on COM, train, data bit-exact
//   Part K  (K6) link loss in L0 - "Recovery-lite":
//     K0  lanes carry logical idle in L0 (never quiet), RX checks arm
//     K1  a 32-clk dropout (< LOSS_CLKS, whole bytes) is tolerated
//     K2  a 7-clk dropout slips the symbol alignment: lost framing between
//         Flits is detected, link drops and retrains
//     K3  cable pull mid-session (all lanes, data in flight): L0 exit on
//         signal loss, datapath reset (sessions, config, AXI), LTSSM keeps
//         retrying while pulled; cable back -> retrain, fresh UpdateFC,
//         host re-configures, clean session and data bit-exact
//     K4  one lane lost: link drops, retrains after it returns
//     K5  partner retraining: TS1s received in L0 -> link drops; trailing
//         TS2s in L0 do NOT drop it
// =============================================================================
`timescale 1ns/1ps
`default_nettype none
module tb_pcie_phy_top;

reg clk = 0, aclk = 0;
always #5   clk  = ~clk;     // 100 MHz pcie_clk
always #3.5 aclk = ~aclk;    // ~143 MHz aclk, unrelated to clk

reg perst_n, aclk_rst_n;
reg ts_byte_valid;
reg [7:0] ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3;
reg hot_reset_request, flr_request, updatefc_received;
reg cfg_wr_en, cfg_rd_en; reg [11:0] cfg_addr; reg [31:0] cfg_wdata;
reg tbl_wr_en, tbl_wr_mask, tbl_rd_en; reg [2:0] tbl_wr_vec, tbl_rd_vec;
reg [63:0] tbl_wr_addr; reg [31:0] tbl_wr_data;
reg dma_complete_pulse; reg [2:0] dma_complete_vector;
reg mwr_ready;
reg [255:0] phy_tx_data; reg phy_tx_valid, phy_tx_sof, phy_tx_eof;
reg bar0_wr_en, bar0_rd_en; reg [15:0] bar0_addr; reg [31:0] bar0_wdata;
reg [1:0] session_done, session_end_aclk, session_overflow_aclk;

wire link_trained, lane_reversed_locked, datapath_rst_n, axi_rst_n, flr_active;
wire [31:0] cfg_rdata; wire cfg_cpl_valid;
wire [63:0] tbl_rd_addr; wire [31:0] tbl_rd_data; wire tbl_rd_mask; wire [7:0] pba;
wire mwr_valid; wire [63:0] mwr_addr; wire [31:0] mwr_data;
wire phy_tx_ready;
wire [11:0] tx_level; wire [3:0] tx_level_valid;
reg  [11:0] rx_level; reg  [3:0] rx_level_valid;   // driven by the channel model
wire [255:0] phy_rx_data; wire phy_rx_valid, phy_rx_sof, phy_rx_eof;
wire phy_tx_idle, phy_framing_err, phy_lane_err, phy_ts_sending, phy_deskew_locked;
wire [1:0] ltssm_state;
wire [31:0] bar0_rdata; wire bar0_rd_valid;
wire [63:0] cfg_base_addr_ch0, cfg_base_addr_ch1; wire [1:0] cfg_base_addr_wr;
wire [2:0] cfg_tc_ch0, cfg_tc_ch1; wire [1:0] session_valid;
wire [1:0] session_start_aclk;
wire [7:0] arb_weight_ch0, arb_weight_ch1; wire [23:0] session_limit;

pcie_phy_top dut (
  .clk(clk), .perst_n(perst_n),
  .ts_byte_valid(ts_byte_valid), .ts_byte_l0(ts_byte_l0), .ts_byte_l1(ts_byte_l1),
  .ts_byte_l2(ts_byte_l2), .ts_byte_l3(ts_byte_l3),
  .link_trained(link_trained), .lane_reversed_locked(lane_reversed_locked),
  .hot_reset_request(hot_reset_request), .flr_request(flr_request), .updatefc_received(updatefc_received),
  .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n), .flr_active(flr_active),
  .cfg_wr_en(cfg_wr_en), .cfg_rd_en(cfg_rd_en), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
  .cfg_rdata(cfg_rdata), .cfg_cpl_valid(cfg_cpl_valid),
  .tbl_wr_en(tbl_wr_en), .tbl_wr_vec(tbl_wr_vec), .tbl_wr_addr(tbl_wr_addr), .tbl_wr_data(tbl_wr_data),
  .tbl_wr_mask(tbl_wr_mask), .tbl_rd_en(tbl_rd_en), .tbl_rd_vec(tbl_rd_vec),
  .tbl_rd_addr(tbl_rd_addr), .tbl_rd_data(tbl_rd_data), .tbl_rd_mask(tbl_rd_mask), .pba(pba),
  .dma_complete_pulse(dma_complete_pulse), .dma_complete_vector(dma_complete_vector),
  .mwr_valid(mwr_valid), .mwr_ready(mwr_ready), .mwr_addr(mwr_addr), .mwr_data(mwr_data),
  .phy_tx_data(phy_tx_data), .phy_tx_valid(phy_tx_valid), .phy_tx_sof(phy_tx_sof), .phy_tx_eof(phy_tx_eof),
  .phy_tx_ready(phy_tx_ready),
  .tx_level(tx_level), .tx_level_valid(tx_level_valid), .rx_level(rx_level), .rx_level_valid(rx_level_valid),
  .phy_rx_data(phy_rx_data), .phy_rx_valid(phy_rx_valid), .phy_rx_sof(phy_rx_sof), .phy_rx_eof(phy_rx_eof),
  .phy_tx_idle(phy_tx_idle), .phy_framing_err(phy_framing_err), .phy_lane_err(phy_lane_err),
  .phy_ts_sending(phy_ts_sending), .phy_deskew_locked(phy_deskew_locked), .ltssm_state(ltssm_state),
  .bar0_wr_en(bar0_wr_en), .bar0_rd_en(bar0_rd_en), .bar0_addr(bar0_addr), .bar0_wdata(bar0_wdata),
  .bar0_rdata(bar0_rdata), .bar0_rd_valid(bar0_rd_valid),
  .cfg_base_addr_ch0(cfg_base_addr_ch0), .cfg_base_addr_ch1(cfg_base_addr_ch1),
  .cfg_base_addr_wr(cfg_base_addr_wr), .cfg_tc_ch0(cfg_tc_ch0), .cfg_tc_ch1(cfg_tc_ch1),
  .session_valid(session_valid), .session_done(session_done),
  .aclk(aclk), .aclk_rst_n(aclk_rst_n),
  .session_start_aclk(session_start_aclk), .session_end_aclk(session_end_aclk),
  .session_overflow_aclk(session_overflow_aclk),
  .arb_weight_ch0(arb_weight_ch0), .arb_weight_ch1(arb_weight_ch1), .session_limit(session_limit)
);

// ---------------------------------------------------------------------------
// Channel model: one register stage, optional lane crossing, and a separate
// delay per lane (skew) of skew_l[i] clk, 0..63.
// ---------------------------------------------------------------------------
reg cross_lanes;
reg chan_on;             // 0 = channel not connected yet (receiver sees nothing)
reg [3:0] lane_kill = 4'b0000;   // K6: per-lane signal loss (receiver lane i sees nothing)
reg [5:0] skew_l0, skew_l1, skew_l2, skew_l3;
integer h;
// lane i of the wire after crossing, before skew
wire [3:0] w0 = cross_lanes ? {tx_level_valid[3], tx_level[11:9]} : {tx_level_valid[0], tx_level[2:0]};
wire [3:0] w1 = cross_lanes ? {tx_level_valid[2], tx_level[8:6]}  : {tx_level_valid[1], tx_level[5:3]};
wire [3:0] w2 = cross_lanes ? {tx_level_valid[1], tx_level[5:3]}  : {tx_level_valid[2], tx_level[8:6]};
wire [3:0] w3 = cross_lanes ? {tx_level_valid[0], tx_level[2:0]}  : {tx_level_valid[3], tx_level[11:9]};
reg [15:0] hw [0:63];    // 4 lanes x 4 bits, hw[0] = this cycle's wire
always @(posedge clk) begin
  for (h = 63; h > 0; h = h - 1) hw[h] <= hw[h-1];
  hw[0] <= {w3, w2, w1, w0};
end
wire [3:0] r0 = hw[skew_l0][3:0];
wire [3:0] r1 = hw[skew_l1][7:4];
wire [3:0] r2 = hw[skew_l2][11:8];
wire [3:0] r3 = hw[skew_l3][15:12];
always @(*) begin
  if (!chan_on) begin
    rx_level       = 12'd0;
    rx_level_valid = 4'b0000;
  end else begin
    rx_level       = {r3[2:0], r2[2:0], r1[2:0], r0[2:0]};
    rx_level_valid = {r3[3] === 1'b1, r2[3] === 1'b1, r1[3] === 1'b1, r0[3] === 1'b1} & ~lane_kill;
  end
end

// ---------------------------------------------------------------------------
// LTSSM monitor: record the order of states and which ordered sets the RX saw
// ---------------------------------------------------------------------------
reg [1:0] ltssm_prev = 0;
reg       ltssm_bad_jump = 0;
integer   seen_ts1 = 0, seen_ts2 = 0;
always @(posedge clk) begin
  if (ltssm_state != ltssm_prev) begin
    // legal: 0->1, 1->2, 2->3, and any -> 0 (timeout / reset)
    if (!((ltssm_state == ltssm_prev + 2'd1) || (ltssm_state == 2'd0))) ltssm_bad_jump <= 1'b1;
    ltssm_prev <= ltssm_state;
  end
  if (&dut.osv && !(&dut.ost2)) seen_ts1 = seen_ts1 + 1;
  if (&dut.osv &&  (&dut.ost2)) seen_ts2 = seen_ts2 + 1;
end

// ---------------------------------------------------------------------------
// K6 monitors: L0 exits (with reason), datapath reset pulses, quiet TX lanes in L0
// ---------------------------------------------------------------------------
integer   l0_exits = 0, dp_resets = 0, l0_quiet_clks = 0;
reg [1:0] last_exit_reason = 0;
reg       dp_rst_d = 1;
always @(posedge clk) begin
  if (dut.ltssm_l0_exit) begin
    l0_exits = l0_exits + 1;
    last_exit_reason = dut.ltssm_exit_reason;
  end
  dp_rst_d <= datapath_rst_n;
  if (dp_rst_d && !datapath_rst_n && perst_n) dp_resets = dp_resets + 1;
  // TX side of L0: every clock must carry a symbol on all 4 lanes
  if (perst_n && datapath_rst_n && ltssm_state == 2'd3 && tx_level_valid != 4'b1111)
    l0_quiet_clks = l0_quiet_clks + 1;
end

integer errors = 0;
localparam LOSS_WAIT = 2000;
task check(input cond, input [8*96-1:0] msg);
  begin
    if (cond) $display("PASS: %0s", msg);
    else begin $display("FAIL: %0s", msg); errors = errors + 1; end
  end
endtask

// ---------------------------------------------------------------------------
// aclk-side monitor: count start pulses and check they are single-cycle
// ---------------------------------------------------------------------------
integer start_cnt0 = 0, start_cnt1 = 0;
reg [1:0] start_d = 0;
always @(posedge aclk) begin
  start_d <= session_start_aclk;
  if (session_start_aclk[0]) start_cnt0 = start_cnt0 + 1;
  if (session_start_aclk[1]) start_cnt1 = start_cnt1 + 1;
  if (session_start_aclk[0] && start_d[0]) begin
    $display("FAIL: session_start_aclk[0] wider than one aclk cycle"); errors = errors + 1;
  end
end

// ---------------------------------------------------------------------------
// Scoreboard: compares every RX word against the expected queue
// ---------------------------------------------------------------------------
reg [255:0] exp_q [0:1023];
integer exp_wr = 0, exp_rd = 0, rx_bad = 0;
always @(posedge clk) begin
  if (phy_rx_valid) begin
    if (exp_rd >= exp_wr) begin
      $display("FAIL: unexpected RX word"); rx_bad = rx_bad + 1;
    end else begin
      if (phy_rx_data !== exp_q[exp_rd] ||
          phy_rx_sof  !== ((exp_rd % 8) == 0) || phy_rx_eof !== ((exp_rd % 8) == 7)) begin
        if (rx_bad < 5)
          $display("MISMATCH word %0d: sof=%b eof=%b\n  got %h\n  exp %h",
                   exp_rd, phy_rx_sof, phy_rx_eof, phy_rx_data, exp_q[exp_rd]);
        rx_bad = rx_bad + 1;
      end
      exp_rd = exp_rd + 1;
    end
  end
end

// ---------------------------------------------------------------------------
// Tasks
// ---------------------------------------------------------------------------
task idle_inputs;
  begin
    ts_byte_valid = 0; ts_byte_l0 = 0; ts_byte_l1 = 0; ts_byte_l2 = 0; ts_byte_l3 = 0;
    hot_reset_request = 0; flr_request = 0; updatefc_received = 0;
    cfg_wr_en = 0; cfg_rd_en = 0; cfg_addr = 0; cfg_wdata = 0;
    tbl_wr_en = 0; tbl_wr_mask = 0; tbl_rd_en = 0; tbl_wr_vec = 0; tbl_rd_vec = 0;
    tbl_wr_addr = 0; tbl_wr_data = 0;
    dma_complete_pulse = 0; dma_complete_vector = 0;
    mwr_ready = 1;
    phy_tx_data = 0; phy_tx_valid = 0; phy_tx_sof = 0; phy_tx_eof = 0;
    bar0_wr_en = 0; bar0_rd_en = 0; bar0_addr = 0; bar0_wdata = 0;
    session_done = 0; session_overflow_aclk = 0;
  end
endtask

// Training over the lanes: no ts_byte_* injection. TX sends TS1 through the
// channel model; the far end's detectors read them off the received lanes.
integer train_clk;
task bring_up;
  begin
    idle_inputs;
    perst_n = 0; aclk_rst_n = 0;
    repeat (5) @(negedge clk);
    perst_n = 1;
    @(negedge clk);
    aclk_rst_n = 1;
    train_clk = 0;
    while (!link_trained && train_clk < 5000) begin @(negedge clk); train_clk = train_clk + 1; end
    $display("INFO: link trained over the lanes after %0d clk", train_clk);
    repeat (3) @(negedge clk);
    updatefc_received = 1; @(negedge clk); updatefc_received = 0; @(negedge clk);
  end
endtask

task cfg_write(input [11:0] a, input [31:0] d);
  begin
    cfg_addr = a; cfg_wdata = d; cfg_wr_en = 1; @(negedge clk); cfg_wr_en = 0;
  end
endtask

task cfg_read(input [11:0] a, output [31:0] d);
  begin
    cfg_addr = a; cfg_rd_en = 1; @(negedge clk); cfg_rd_en = 0; d = cfg_rdata;
  end
endtask

task bar0_write(input [15:0] a, input [31:0] d);
  begin
    bar0_addr = a; bar0_wdata = d; bar0_wr_en = 1; @(negedge clk); bar0_wr_en = 0;
  end
endtask

task bar0_read(input [15:0] a, output [31:0] d);
  begin
    bar0_addr = a; bar0_rd_en = 1; @(negedge clk); bar0_rd_en = 0; d = bar0_rdata;
  end
endtask

// Push n_flits Flits of random data with random producer gaps.
// Stops pushing a word while phy_tx_ready is low, exactly as the link layer must.
integer stall_cycles;
task push_flits(input integer n_flits, input integer with_fb_word);
  integer w, total;
  reg [255:0] wd;
  begin
    total = n_flits * 8;
    w = 0;
    while (w < total) begin
      wd = {$random, $random, $random, $random, $random, $random, $random, $random};
      if (with_fb_word && w == 11) wd = {32{8'hFB}};   // payload that looks like SOF on every lane
      if (with_fb_word && w == 12) wd = {8{32'hFBFBFBFB}};
      exp_q[exp_wr] = wd; exp_wr = exp_wr + 1;
      phy_tx_data = wd; phy_tx_sof = (w % 8 == 0); phy_tx_eof = (w % 8 == 7);
      // random idle gap before offering the word
      if (($random & 7) == 0) begin phy_tx_valid = 0; @(negedge clk); end
      phy_tx_valid = 1;
      @(posedge clk);
      while (!phy_tx_ready) begin stall_cycles = stall_cycles + 1; @(posedge clk); end
      @(negedge clk);
      w = w + 1;
    end
    phy_tx_valid = 0; phy_tx_sof = 0; phy_tx_eof = 0;
  end
endtask

task wait_rx_drain(input integer max_cycles);
  integer c;
  begin
    c = 0;
    while (exp_rd < exp_wr && c < max_cycles) begin @(negedge clk); c = c + 1; end
  end
endtask

task skew_test(input [5:0] s0, input [5:0] s1, input [5:0] s2, input [5:0] s3,
               input crossed, input [8*48-1:0] name);
  begin
    exp_wr = 0; exp_rd = 0; rx_bad = 0;
    idle_inputs;
    cross_lanes = crossed;
    skew_l0 = s0; skew_l1 = s1; skew_l2 = s2; skew_l3 = s3;
    perst_n = 0; aclk_rst_n = 0;
    repeat (5) @(negedge clk);
    perst_n = 1; aclk_rst_n = 1;
    train_clk = 0;
    while (!link_trained && train_clk < 5000) begin @(negedge clk); train_clk = train_clk + 1; end
    $display("INFO: %0s: trained after %0d clk", name, train_clk);
    check(link_trained === 1'b1 && phy_deskew_locked === 1'b1, {name, ": deskew locks and link trains"});
    check(lane_reversed_locked === crossed, {name, ": reversal detected correctly"});
    push_flits(4, 1);
    wait_rx_drain(6000);
    check(exp_rd == 32 && rx_bad == 0 && phy_lane_err === 1'b0, {name, ": 4 Flits bit-exact"});
  end
endtask

// K6: play a link partner that is retraining: inject one TS1 or TS2 ordered
// set on all 4 lanes through the ts_byte_* port (1 byte per clk per lane).
reg  [3:0] inj_idx = 0;
reg        inj_ts2 = 0;
wire [7:0] inj_b0, inj_b1, inj_b2, inj_b3;
ts_os_gen u_inj (.idx(inj_idx), .ts2(inj_ts2),
                 .byte_l0(inj_b0), .byte_l1(inj_b1), .byte_l2(inj_b2), .byte_l3(inj_b3));
task inject_os(input is_ts2);
  integer k;
  begin
    inj_ts2 = is_ts2;
    for (k = 0; k < 16; k = k + 1) begin
      inj_idx = k;
      #1;
      ts_byte_l0 = inj_b0; ts_byte_l1 = inj_b1; ts_byte_l2 = inj_b2; ts_byte_l3 = inj_b3;
      ts_byte_valid = 1;
      @(negedge clk);
    end
    ts_byte_valid = 0;
  end
endtask

task wait_trained(input integer max_clk);
  begin
    train_clk = 0;
    while (!link_trained && train_clk < max_clk) begin @(negedge clk); train_clk = train_clk + 1; end
  end
endtask

task wait_l0_exit(input integer max_clk, output integer took);
  integer e0;
  begin
    e0 = l0_exits; took = 0;
    while (l0_exits == e0 && took < max_clk) begin @(negedge clk); took = took + 1; end
  end
endtask

// UpdateFC + the host's minimum re-configuration after a (re)train
task host_config;
  begin
    repeat (3) @(negedge clk);
    updatefc_received = 1; @(negedge clk); updatefc_received = 0; @(negedge clk);
    cfg_write(12'h004, 32'h0000_0006);          // Memory Space + Bus Master
    cfg_write(12'h050, 32'h8000_0000);          // MSI-X enable
  end
endtask

// ---------------------------------------------------------------------------
reg [31:0] rd;
integer t_start, t_end, i, base_wr_seen, took, quiet0, starts0;
initial begin
  cross_lanes = 0; stall_cycles = 0; chan_on = 1;
  skew_l0 = 0; skew_l1 = 0; skew_l2 = 0; skew_l3 = 0;
  // =================== Part A: identity lanes ===================
  bring_up;
  check(link_trained === 1'b1,          "A1 link_trained from TS1 sent over the lanes (no injection)");
  check(lane_reversed_locked === 1'b0,  "A1 straight channel -> not reversed");
  check(ltssm_state === 2'd3 && !ltssm_bad_jump, "A1 LTSSM went Detect -> Polling -> Configuration -> L0");
  check(seen_ts1 >= 8 && seen_ts2 >= 8, "A1 at least 8 TS1 and 8 TS2 received on all lanes before L0");
  repeat (200) @(negedge clk);
  check(phy_ts_sending === 1'b0,        "A1 TX stopped sending ordered sets in L0");
  check(axi_rst_n === 1'b1,             "A1 axi_rst_n released after UpdateFC");
  check(phy_tx_ready === 1'b1,          "A1 phy_tx_ready high once datapath is out of reset");

  // K6: from here to the end of A6 the TX lanes must never be quiet in L0
  quiet0 = l0_quiet_clks;

  // ---- A2: BAR0 ----
  // A real host enables Memory Space and MSI-X before using either.
  cfg_write(12'h004, 32'h0000_0006);          // Command: Memory Space (bit1) + Bus Master (bit2)
  cfg_write(12'h050, 32'h8000_0000);          // MSI-X Message Control: Enable
  bar0_write(12'h020, 32'h1111_2000);  bar0_write(12'h024, 32'h0000_00AB);
  bar0_write(12'h028, 32'h3333_4000);  bar0_write(12'h02C, 32'h0000_00CD);
  bar0_write(12'h008, 32'h0000_0503);  // weights ch0=3 ch1=5
  bar0_write(12'h00C, 32'h0012_3456);  // limit
  bar0_write(12'h010, 32'h0000_0602);  // MSI-X vectors ch0=2 ch1=6
  bar0_write(12'h018, 32'h0000_0105);  // TC ch0=5 ch1=1
  bar0_read(12'h020, rd); check(rd == 32'h1111_2000, "A2 BAR0 BASE_CH0_LO readback");
  bar0_read(12'h02C, rd); check(rd == 32'h0000_00CD, "A2 BAR0 BASE_CH1_HI readback");
  bar0_read(12'h008, rd); check(rd == 32'h0000_0503, "A2 BAR0 ARB_WEIGHTS readback");
  check(arb_weight_ch0 == 8'd3 && arb_weight_ch1 == 8'd5, "A2 weights reach interconnect ports");
  check(session_limit == 24'h12_3456,   "A2 session limit reaches interconnect port");
  check(session_valid == 2'b00,         "A2 no session before arm");

  // ---- A3: arm ch0 ----
  base_wr_seen = 0;
  fork
    begin bar0_write(12'h000, 32'h1); end
    begin repeat (6) begin @(posedge clk); if (cfg_base_addr_wr[0]) base_wr_seen = 1; end end
  join
  repeat (2) @(negedge clk);
  check(session_valid[0] === 1'b1 && session_valid[1] === 1'b0, "A3 arm bit -> session_valid ch0 only");
  check(cfg_base_addr_ch0 == 64'h0000_00AB_1111_2000, "A3 base_addr_ch0 latched from BAR0");
  check(cfg_tc_ch0 == 3'd5,                "A3 tc ch0 latched");
  check(base_wr_seen == 1,                 "A3 cfg_base_addr_wr pulse for ch0");
  repeat (6) @(negedge clk);
  check(start_cnt0 == 1 && start_cnt1 == 0, "A3 exactly one session_start pulse on aclk (I3)");
  bar0_read(12'h004, rd); check(rd[1:0] == 2'b01, "A3 BAR0 STATUS shows ch0 active");
  bar0_write(12'h000, 32'h1);            // re-arm while active must be ignored, counted
  repeat (2) @(negedge clk);             // arm pulse is registered, counter one cycle later
  bar0_read(12'h014, rd); check(rd[7:0] == 8'd1, "A3 arm-while-active counted, not restarted");
  check(start_cnt0 == 1, "A3 re-arm while active did not produce a second start");

  // ---- A4: MSI-X on session_done ----
  tbl_wr_vec = 3'd2; tbl_wr_addr = 64'hFEE0_0000_0000_1000; tbl_wr_data = 32'h0000_4242; tbl_wr_mask = 0;
  tbl_wr_en = 1; @(negedge clk); tbl_wr_en = 0;
  mwr_ready = 0;
  session_done = 2'b01; @(negedge clk); session_done = 2'b00;
  @(negedge clk);
  check(mwr_valid === 1'b1 && mwr_addr == 64'hFEE0_0000_0000_1000 && mwr_data == 32'h4242,
        "A4 session_done ch0 -> MWr with vector 2's address/data");
  repeat (3) @(negedge clk);
  check(mwr_valid === 1'b1, "A4 MWr held while mwr_ready low (no longer a lost pulse)");
  mwr_ready = 1; @(negedge clk);
  check(mwr_valid === 1'b0, "A4 MWr retired after mwr_ready");
  // masked-then-unmasked: event pends, then drains on unmask
  tbl_wr_vec = 3'd6; tbl_wr_addr = 64'hFEE0_0000_0000_2000; tbl_wr_data = 32'h6666; tbl_wr_mask = 1;
  tbl_wr_en = 1; @(negedge clk); tbl_wr_en = 0;
  session_done = 2'b10; @(negedge clk); session_done = 2'b00; @(negedge clk);
  check(pba[6] === 1'b1 && mwr_valid === 1'b0, "A4 masked ch1 vector -> PBA bit, no MWr");
  tbl_wr_mask = 0; tbl_wr_en = 1; @(negedge clk); tbl_wr_en = 0; @(negedge clk);
  check(mwr_valid === 1'b1 && mwr_data == 32'h6666 && pba[6] === 1'b0,
        "A4 unmask -> pending vector sent, PBA bit cleared");
  @(negedge clk);

  // ---- A5 (K1): sessions end on the link layer's session_done ----
  // A4's session_done on ch0 already ended the ch0 session.
  check(session_valid[0] === 1'b0, "A5 session_done from the link layer retires the ch0 session");
  bar0_write(12'h000, 32'h1);
  repeat (8) @(negedge clk);
  check(session_valid[0] === 1'b1 && start_cnt0 == 2, "A5 re-arm after session_done accepted (second start pulse)");
  // interconnect finishes accepting, but the data is still in flight
  @(negedge aclk); session_end_aclk = 2'b01; @(negedge aclk); session_end_aclk = 2'b00;
  repeat (6) @(negedge clk);
  check(session_valid[0] === 1'b1, "A5 interconnect end alone does NOT end the session (data still draining)");
  bar0_read(12'h004, rd); check(rd[4] == 1'b1 && rd[0] == 1'b1, "A5 BAR0 STATUS[4] shows ch0 draining");
  bar0_write(12'h000, 32'h1);              // early re-arm while draining
  repeat (8) @(negedge clk);
  bar0_read(12'h014, rd);
  check(rd[7:0] == 8'd2 && start_cnt0 == 2, "A5 arm while draining is ignored and counted in ARM_DEBUG");
  session_done = 2'b01; @(negedge clk); session_done = 2'b00;
  repeat (2) @(negedge clk);
  check(session_valid[0] === 1'b0, "A5 session_done ends the drained session -> safe to re-arm");
  bar0_read(12'h004, rd); check(rd[4] == 1'b0, "A5 draining bit clears with the session");
  @(negedge aclk); session_overflow_aclk = 2'b10;
  repeat (4) @(negedge clk);
  bar0_read(12'h004, rd); check(rd[9:8] == 2'b10, "A5 aclk overflow flag visible in BAR0 STATUS");

  // ---- A6: datapath ----
  t_start = $time;
  push_flits(40, 1);
  wait_rx_drain(20000);
  t_end = $time;
  check(exp_rd == exp_wr && exp_wr == 320, "A6 all 320 words (40 Flits) came back");
  check(rx_bad == 0,                    "A6 every word bit-exact with correct sof/eof (incl. all-0xFB payload)");
  check(stall_cycles > 0,               "A6 phy_tx_ready applied backpressure (link must stall)");
  check(phy_framing_err === 1'b0 && phy_lane_err === 1'b0, "A6 no framing or lane errors");
  $display("INFO: %0d Flits in %0d clk = %0d clk per 256-byte Flit (floor is 256 + 4 SOF)",
           40, (t_end - t_start) / 10, (t_end - t_start) / 10 / 40);
  check(((t_end - t_start) / 10 / 40) <= 266, "A6 drain rate within 4% of the 260-clk floor");
  repeat (20) @(negedge clk);
  check(phy_tx_idle === 1'b1,           "A6 TX path idle after drain");
  check(l0_quiet_clks == quiet0 && tx_level_valid === 4'b1111,
        "A6 (K6) TX lanes never quiet in L0: logical idle between Flits, incl. stalls");
  check(dut.ltssm_rx_armed === 1'b1 && l0_exits == 0, "A6 (K6) RX link-loss checks armed, no spurious L0 exit");

  // =================== Part F: config-space gating (#3) ===================
  // F1: MSI-X table/PBA location advertised in config space
  cfg_read(12'h054, rd); check(rd == 32'h0000_1000, "F1 config 0x54: MSI-X table at BAR0+0x1000, BIR 0");
  cfg_read(12'h058, rd); check(rd == 32'h0000_1800, "F1 config 0x58: MSI-X PBA at BAR0+0x1800, BIR 0");

  // F2: program vector 5 through BAR0 (standard 16-byte entry), read it back
  bar0_write(16'h1050, 32'h0000_3000);        // entry 5 +0 msg addr lo
  bar0_write(16'h1054, 32'h0000_00FE);        // entry 5 +4 msg addr hi
  bar0_write(16'h1058, 32'h0000_5555);        // entry 5 +8 msg data
  bar0_write(16'h105C, 32'h0000_0000);        // entry 5 +12 vector control: unmasked
  bar0_read(16'h1058, rd); check(rd == 32'h5555, "F2 MSI-X table entry readable through BAR0");
  dma_complete_vector = 3'd5; dma_complete_pulse = 1; @(negedge clk); dma_complete_pulse = 0;
  check(mwr_valid === 1'b1 && mwr_addr == 64'h0000_00FE_0000_3000 && mwr_data == 32'h5555,
        "F2 vector programmed via BAR0 sends the right MWr");
  @(negedge clk);

  // F3: Function Mask -> pend; clearing it drains the pending vector
  cfg_write(12'h050, 32'hC000_0000);          // Enable + Function Mask
  dma_complete_vector = 3'd5; dma_complete_pulse = 1; @(negedge clk); dma_complete_pulse = 0;
  @(negedge clk);
  check(mwr_valid === 1'b0 && pba[5] === 1'b1, "F3 function mask: no MWr, PBA bit set");
  bar0_read(16'h1800, rd); check(rd[5] == 1'b1, "F3 PBA readable through BAR0");
  cfg_write(12'h050, 32'h8000_0000);          // clear Function Mask
  @(negedge clk);                             // mask register updates, then the drain issues
  check(mwr_valid === 1'b1 && mwr_data == 32'h5555 && pba[5] === 1'b0, "F3 clearing function mask sends the pending MWr");
  @(negedge clk);

  // F4: MSI-X disabled -> completion dropped (no MWr, no PBA)
  cfg_write(12'h050, 32'h0000_0000);
  dma_complete_vector = 3'd5; dma_complete_pulse = 1; @(negedge clk); dma_complete_pulse = 0;
  repeat (2) @(negedge clk);
  check(mwr_valid === 1'b0 && pba[5] === 1'b0, "F4 MSI-X disabled: completion dropped");
  cfg_write(12'h050, 32'h8000_0000);

  // F5: Memory Space disabled -> BAR0 writes ignored, reads get no completion
  cfg_write(12'h004, 32'h0000_0004);          // Bus Master only
  bar0_write(16'h0020, 32'hBAD0_BAD0);
  bar0_addr = 16'h0020; bar0_rd_en = 1; @(negedge clk); bar0_rd_en = 0;
  check(bar0_rd_valid === 1'b0, "F5 memory space off: BAR0 read not completed");
  cfg_write(12'h004, 32'h0000_0006);
  bar0_read(16'h0020, rd); check(rd == 32'h1111_2000, "F5 memory space off: BAR0 write was ignored");

  // =================== Part B: crossed lanes ===================
  cross_lanes = 1;
  exp_wr = 0; exp_rd = 0; rx_bad = 0;
  bring_up;
  check(link_trained === 1'b1 && lane_reversed_locked === 1'b1, "B1 crossed channel detected from TS1 lane numbers alone");
  push_flits(10, 0);
  wait_rx_drain(10000);
  check(exp_rd == 80 && rx_bad == 0, "B2 10 Flits bit-exact through a lane-reversed channel");

  // =================== Part C: framing check ===================
  phy_tx_data = 256'h1; phy_tx_sof = 0; phy_tx_eof = 0; phy_tx_valid = 1;   // word 0 without sof
  @(negedge clk); phy_tx_valid = 0;
  repeat (2) @(negedge clk);
  check(phy_framing_err === 1'b1, "C  word 0 without sof flags framing_err");

  // =================== Part D: data offered before link-up ===================
  // The link layer may push Flits before training finishes; they must wait in
  // the FIFO behind the TS1 ordered sets and come out intact afterwards.
  cross_lanes = 0;
  exp_wr = 0; exp_rd = 0; rx_bad = 0;
  idle_inputs;
  perst_n = 0; aclk_rst_n = 0;
  repeat (5) @(negedge clk);
  perst_n = 1; aclk_rst_n = 1;
  @(negedge clk);
  check(ltssm_state === 2'd0 && phy_ts_sending === 1'b0, "D  LTSSM starts in Detect, lanes quiet");
  push_flits(4, 0);
  wait_rx_drain(5000);
  check(link_trained === 1'b1, "D  link trained while data was queued");
  check(exp_rd == 32 && rx_bad == 0, "D  4 Flits queued during training delivered bit-exact after link-up");

  // =================== Part E: channel connects mid-byte ===================
  // The receiver starts listening 37 clk after reset (not a multiple of the
  // 4-symbol byte), so its first symbol is mid-byte. lane_rx must lock onto
  // the COM symbol before training can succeed and data can pass.
  exp_wr = 0; exp_rd = 0; rx_bad = 0;
  idle_inputs;
  chan_on = 0;
  perst_n = 0; aclk_rst_n = 0;
  repeat (5) @(negedge clk);
  perst_n = 1; aclk_rst_n = 1;
  repeat (37) @(negedge clk);
  chan_on = 1;
  train_clk = 0;
  while (!link_trained && train_clk < 5000) begin @(negedge clk); train_clk = train_clk + 1; end
  check(link_trained === 1'b1, "E  link trains when the channel comes up mid-byte (COM symbol lock)");
  push_flits(3, 1);
  wait_rx_drain(5000);
  check(exp_rd == 24 && rx_bad == 0, "E  3 Flits bit-exact after mid-byte lock");

  // =================== Part G: lane-to-lane skew (#1) ===================
  // Each lane gets a different channel delay, including odd, sub-byte
  // amounts. RX must lock each lane on COM, deskew them on the shared COM,
  // train, and deliver Flits bit-exact.
  skew_test(6'd0, 6'd5, 6'd13, 6'd22, 1'b0, "G1 skew 0/5/13/22 clk");
  skew_test(6'd30, 6'd0, 6'd17, 6'd3, 1'b0, "G2 skew 30/0/17/3 clk (31 clk max)");
  skew_test(6'd9, 6'd27, 6'd1, 6'd14, 1'b1, "G3 skew 9/27/1/14 clk + crossed lanes");
  skew_l0 = 0; skew_l1 = 0; skew_l2 = 0; skew_l3 = 0;

  // =================== Part H: hot reset retrains the link ===================
  // Bring the link up (with skew, so deskew must redo its work too), pass
  // data, then hot reset. link_trained must drop, TX must resend TS1, the link
  // must retrain over the lanes, AXI reset must wait for a fresh UpdateFC, and
  // data must pass again afterwards.
  skew_test(6'd4, 6'd11, 6'd0, 6'd19, 1'b0, "H0 link up before hot reset");
  updatefc_received = 1; @(negedge clk); updatefc_received = 0; @(negedge clk);
  check(axi_rst_n === 1'b1, "H0 AXI out of reset before the hot reset");
  hot_reset_request = 1; @(negedge clk); hot_reset_request = 0;
  repeat (4) @(negedge clk);
  check(link_trained === 1'b0, "H1 hot reset clears link_trained");
  check(ltssm_state === 2'd0, "H1 LTSSM back in Detect");
  repeat (40) @(negedge clk);
  check(ltssm_state === 2'd1 && phy_ts_sending === 1'b1, "H1 then Polling: TX resumes sending TS1");
  check(axi_rst_n === 1'b0, "H1 AXI held in reset across the hot reset");
  train_clk = 0;
  while (!link_trained && train_clk < 5000) begin @(negedge clk); train_clk = train_clk + 1; end
  $display("INFO: retrained %0d clk after hot reset", train_clk);
  check(link_trained === 1'b1 && phy_deskew_locked === 1'b1, "H2 link retrains over the lanes after hot reset");
  repeat (3) @(negedge clk);
  check(axi_rst_n === 1'b0, "H2 AXI still in reset until a fresh UpdateFC");
  updatefc_received = 1; @(negedge clk); updatefc_received = 0; @(negedge clk);
  check(axi_rst_n === 1'b1, "H2 AXI released after fresh UpdateFC");
  exp_wr = 0; exp_rd = 0; rx_bad = 0;
  push_flits(3, 1);
  wait_rx_drain(5000);
  check(exp_rd == 24 && rx_bad == 0, "H3 3 Flits bit-exact after the retrain");
  skew_l0 = 0; skew_l1 = 0; skew_l2 = 0; skew_l3 = 0;

  // =================== Part I: LTSSM timeout with no partner ===================
  // Nothing on the other end: Polling must time out back to Detect (and keep
  // cycling), and the link must still train once a partner appears.
  idle_inputs; chan_on = 0; cross_lanes = 0;
  perst_n = 0; aclk_rst_n = 0; repeat (5) @(negedge clk); perst_n = 1; aclk_rst_n = 1;
  repeat (100) @(negedge clk);
  check(ltssm_state === 2'd1, "I1 no partner: LTSSM sits in Polling sending TS1");
  begin : wait_timeout
    integer c; c = 0;
    while (ltssm_state != 2'd0 && c < 20000) begin @(negedge clk); c = c + 1; end
    $display("INFO: Polling timed out to Detect after %0d clk", c + 100);
  end
  check(ltssm_state === 2'd0 && link_trained === 1'b0, "I2 Polling times out back to Detect");
  repeat (50) @(negedge clk);
  check(ltssm_state === 2'd1, "I2 and retries Polling");
  chan_on = 1;
  train_clk = 0;
  while (!link_trained && train_clk < 6000) begin @(negedge clk); train_clk = train_clk + 1; end
  check(link_trained === 1'b1 && !ltssm_bad_jump, "I3 partner appears -> link trains, only legal LTSSM transitions");

  // =================== Part K (K6): link loss in L0 ===================
  // K0: fresh link with channel skew, host configures, a session is armed.
  skew_test(6'd3, 6'd9, 6'd0, 6'd14, 1'b0, "K0 link up with skew");
  host_config;
  bar0_write(12'h020, 32'h5555_0000); bar0_write(12'h024, 32'h0000_0001);
  bar0_write(12'h000, 32'h1);
  repeat (8) @(negedge clk);
  check(axi_rst_n === 1'b1 && session_valid[0] === 1'b1, "K0 AXI out of reset, session armed on ch0");
  quiet0 = l0_quiet_clks;
  repeat (400) @(negedge clk);
  check(l0_quiet_clks == quiet0 && dut.ltssm_rx_armed === 1'b1,
        "K0 idle L0: lanes carry logical idle (never quiet), RX checks armed");

  // K1: a 32-clk dropout on all lanes (8 whole bytes lost, < LOSS_CLKS)
  l0_exits = 0; dp_resets = 0;
  chan_on = 0; repeat (32) @(negedge clk); chan_on = 1;
  repeat (300) @(negedge clk);
  check(link_trained === 1'b1 && l0_exits == 0 && dp_resets == 0 && session_valid[0] === 1'b1,
        "K1 32-clk dropout (< LOSS_CLKS) tolerated: still L0, no reset, session kept");
  exp_wr = 0; exp_rd = 0; rx_bad = 0;
  push_flits(2, 1);
  wait_rx_drain(3000);
  check(exp_rd == 16 && rx_bad == 0, "K1 2 Flits bit-exact after the short dropout");

  // K2: a 7-clk dropout - too short for signal loss, but 7 symbols is not a
  // whole byte, so the frozen byte boundary has slipped. Nothing between
  // Flits decodes as IDL/SOF any more -> lost-framing exit, then retrain.
  l0_exits = 0; dp_resets = 0;
  chan_on = 0; repeat (7) @(negedge clk); chan_on = 1;
  repeat (100) @(negedge clk);
  check(link_trained === 1'b1 && l0_exits == 0, "K2 7-clk dropout: no signal-loss exit (shorter than LOSS_CLKS)");
  wait_l0_exit(4000, took);
  $display("INFO: K2 lost framing detected %0d clk after the slip", took + 107);
  check(l0_exits == 1 && last_exit_reason == 2'd2, "K2 slipped alignment -> L0 left for lost framing (reason 2)");
  repeat (5) @(negedge clk);
  check(dp_resets == 1 && axi_rst_n === 1'b0 && session_valid === 2'b00,
        "K2 link down -> datapath reset pulse, AXI reset, session cleared");
  wait_trained(5000);
  check(link_trained === 1'b1 && phy_deskew_locked === 1'b1, "K2 link retrains on its own (alignment recovered)");
  host_config;
  exp_wr = 0; exp_rd = 0; rx_bad = 0;
  push_flits(2, 1);
  wait_rx_drain(3000);
  check(exp_rd == 16 && rx_bad == 0, "K2 2 Flits bit-exact after the retrain");

  // K3: cable pull in the middle of a session with Flits in flight.
  bar0_write(12'h020, 32'h7777_0000); bar0_write(12'h024, 32'h0000_0002);
  bar0_write(12'h000, 32'h1);
  repeat (8) @(negedge clk);
  check(session_valid[0] === 1'b1 && axi_rst_n === 1'b1, "K3 session active before the cable pull");
  l0_exits = 0; dp_resets = 0;
  exp_wr = 0; exp_rd = 0; rx_bad = 0;
  fork
    push_flits(4, 0);                           // producer keeps going; data is lost by design
    begin
      repeat (300) @(negedge clk);              // mid-Flit
      chan_on = 0;
      wait_l0_exit(LOSS_WAIT, took);
    end
  join
  $display("INFO: K3 L0 left %0d clk after the cable pull", took);
  check(l0_exits == 1 && last_exit_reason == 2'd1 && took <= 64 + 8,
        "K3 cable pull -> L0 left on signal loss within LOSS_CLKS (reason 1)");
  repeat (5) @(negedge clk);
  check(link_trained === 1'b0 && dp_resets == 1, "K3 link_trained drops, datapath_rst_n pulsed (like hot reset)");
  check(axi_rst_n === 1'b0 && session_valid === 2'b00, "K3 AXI held in reset, sessions cleared");
  cfg_read(12'h004, rd);
  check(rd[2:1] == 2'b00, "K3 function state reset: Command register cleared, host must re-configure");
  repeat (3000) @(negedge clk);                 // cable stays out
  check(link_trained === 1'b0 && ltssm_state === 2'd1 && phy_ts_sending === 1'b1 && dp_resets == 1,
        "K3 while pulled: LTSSM stays out of L0, Polling sends TS1, no reset storm");
  check(axi_rst_n === 1'b0, "K3 AXI still in reset while the cable is out");
  exp_wr = 0; exp_rd = 0; rx_bad = 0;           // everything sent before the pull is gone
  chan_on = 1;
  wait_trained(17000);                          // may have to wait out one Polling timeout
  $display("INFO: K3 retrained %0d clk after the cable came back", train_clk);
  check(link_trained === 1'b1 && phy_deskew_locked === 1'b1 && !ltssm_bad_jump,
        "K3 cable back -> link retrains to L0 over the lanes");
  repeat (3) @(negedge clk);
  check(axi_rst_n === 1'b0, "K3 AXI stays in reset until a fresh UpdateFC");
  starts0 = start_cnt0;
  host_config;
  check(axi_rst_n === 1'b1, "K3 fresh UpdateFC releases AXI");
  bar0_write(12'h020, 32'h9999_0000); bar0_write(12'h024, 32'h0000_0003);
  bar0_write(12'h000, 32'h1);
  repeat (8) @(negedge clk);
  check(session_valid[0] === 1'b1 && cfg_base_addr_ch0 == 64'h0000_0003_9999_0000 && start_cnt0 == starts0 + 1,
        "K3 host re-configures: clean new session (new base, one start pulse)");
  push_flits(3, 1);
  wait_rx_drain(4000);
  check(exp_rd == 24 && rx_bad == 0 && phy_lane_err === 1'b0, "K3 3 Flits bit-exact after the re-train, nothing stale");

  // K4: one lane lost (lane 2), others fine
  l0_exits = 0; dp_resets = 0;
  lane_kill = 4'b0100;
  wait_l0_exit(LOSS_WAIT, took);
  check(l0_exits == 1 && last_exit_reason == 2'd1, "K4 single lane lost -> L0 left on signal loss");
  repeat (500) @(negedge clk);
  check(link_trained === 1'b0, "K4 no retrain while a lane is missing");
  lane_kill = 4'b0000;
  wait_trained(17000);
  check(link_trained === 1'b1 && !ltssm_bad_jump, "K4 lane back -> link retrains");
  host_config;
  exp_wr = 0; exp_rd = 0; rx_bad = 0;
  push_flits(2, 0);
  wait_rx_drain(3000);
  check(exp_rd == 16 && rx_bad == 0, "K4 data bit-exact after the retrain");

  // K5: the partner went back to Polling (sends TS1). TS2s alone (as at the
  // end of Configuration) must NOT drop the link; TS1s must.
  repeat (100) @(negedge clk);
  l0_exits = 0; dp_resets = 0;
  inject_os(1); inject_os(1); inject_os(1);
  repeat (50) @(negedge clk);
  check(link_trained === 1'b1 && l0_exits == 0, "K5 three TS2s received in L0: link stays up");
  inject_os(0); inject_os(0);
  repeat (5) @(negedge clk);
  check(l0_exits == 1 && last_exit_reason == 2'd3 && link_trained === 1'b0,
        "K5 two TS1s received in L0 (partner retraining) -> L0 left (reason 3)");
  wait_trained(17000);
  check(link_trained === 1'b1 && !ltssm_bad_jump, "K5 link retrains with the partner");

  if (errors == 0) $display("ALL PASS: pcie_phy_top");
  else             $display("TOTAL FAILURES: %0d", errors);
  $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
end

initial begin
  session_end_aclk = 0;
  #5_000_000;
  $display("FAIL: global timeout"); $display("TOTAL FAILURES: timeout");
  errors = errors + 1;
  $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
end

endmodule
`default_nettype wire
