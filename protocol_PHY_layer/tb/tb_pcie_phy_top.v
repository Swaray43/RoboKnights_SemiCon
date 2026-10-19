// =============================================================================
// tb_pcie_phy_top - self-checking bench for the new synthesizable top.
//
// The SerDes channel is modelled HERE (tx_level -> 1 register stage -> rx_level),
// optionally with lanes crossed (lane i -> lane 3-i) to exercise lane reversal.
//
//   Part A  identity lanes
//     A1  training + reset release through the real chain
//     A2  BAR0 register write/readback
//     A3  arm ch0 -> session_valid, base latched, start pulse arrives on aclk
//     A4  session_done -> MSI-X MWr, held until mwr_ready
//     A5  session_end on aclk -> session_valid drops
//     A6  40 Flits (320 words) of random data, random producer stalls,
//         includes an all-0xFB word; checks data, sof/eof, ready backpressure,
//         and the PHY drain rate (~260 clk per 256-byte Flit)
//   Part B  crossed lanes (lane reversal)
//     B1  train with lane numbers 3,2,1,0 -> lane_reversed_locked = 1
//     B2  10 Flits bit-exact through the reversed channel
//   Part C  framing check: a word with sof in the wrong place sets framing_err
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
reg bar0_wr_en, bar0_rd_en; reg [11:0] bar0_addr; reg [31:0] bar0_wdata;
reg [1:0] session_done, session_end_aclk, session_overflow_aclk;

wire link_trained, lane_reversed_locked, datapath_rst_n, axi_rst_n, flr_active;
wire [31:0] cfg_rdata; wire cfg_cpl_valid;
wire [63:0] tbl_rd_addr; wire [31:0] tbl_rd_data; wire tbl_rd_mask; wire [7:0] pba;
wire mwr_valid; wire [63:0] mwr_addr; wire [31:0] mwr_data;
wire phy_tx_ready;
wire [11:0] tx_level; wire [3:0] tx_level_valid;
reg  [11:0] rx_level; reg  [3:0] rx_level_valid;
wire [255:0] phy_rx_data; wire phy_rx_valid, phy_rx_sof, phy_rx_eof;
wire phy_tx_idle, phy_framing_err, phy_lane_err;
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
// Channel model: one register stage, optional lane crossing
// ---------------------------------------------------------------------------
reg cross_lanes;
always @(posedge clk) begin
  if (!cross_lanes) begin
    rx_level       <= tx_level;
    rx_level_valid <= tx_level_valid;
  end else begin
    rx_level       <= {tx_level[2:0], tx_level[5:3], tx_level[8:6], tx_level[11:9]};
    rx_level_valid <= {tx_level_valid[0], tx_level_valid[1], tx_level_valid[2], tx_level_valid[3]};
  end
end

integer errors = 0;
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

task send_ts1(input [7:0] n0, input [7:0] n1, input [7:0] n2, input [7:0] n3);
  integer k;
  begin
    ts_byte_valid = 1;
    {ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3} = {4{8'hBC}}; @(negedge clk);
    {ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3} = 32'h0;     @(negedge clk);
    ts_byte_l0 = n0; ts_byte_l1 = n1; ts_byte_l2 = n2; ts_byte_l3 = n3; @(negedge clk);
    for (k = 3; k <= 9; k = k + 1) begin {ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3} = 32'h0; @(negedge clk); end
    for (k = 10; k <= 15; k = k + 1) begin {ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3} = {4{8'h4A}}; @(negedge clk); end
    ts_byte_valid = 0;
  end
endtask

task bring_up(input [7:0] n0, input [7:0] n1, input [7:0] n2, input [7:0] n3);
  begin
    idle_inputs;
    perst_n = 0; aclk_rst_n = 0;
    repeat (5) @(negedge clk);
    perst_n = 1;
    @(negedge clk);
    aclk_rst_n = 1;
    send_ts1(n0, n1, n2, n3);
    send_ts1(n0, n1, n2, n3);
    repeat (3) @(negedge clk);
    updatefc_received = 1; @(negedge clk); updatefc_received = 0; @(negedge clk);
  end
endtask

task bar0_write(input [11:0] a, input [31:0] d);
  begin
    bar0_addr = a; bar0_wdata = d; bar0_wr_en = 1; @(negedge clk); bar0_wr_en = 0;
  end
endtask

task bar0_read(input [11:0] a, output [31:0] d);
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

// ---------------------------------------------------------------------------
reg [31:0] rd;
integer t_start, t_end, i, base_wr_seen;
initial begin
  cross_lanes = 0; stall_cycles = 0;
  // =================== Part A: identity lanes ===================
  bring_up(8'd0, 8'd1, 8'd2, 8'd3);
  check(link_trained === 1'b1,          "A1 link_trained after two TS1 sets");
  check(lane_reversed_locked === 1'b0,  "A1 identity lane numbers -> not reversed");
  check(axi_rst_n === 1'b1,             "A1 axi_rst_n released after UpdateFC");
  check(phy_tx_ready === 1'b1,          "A1 phy_tx_ready high once datapath is out of reset");

  // ---- A2: BAR0 ----
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

  // ---- A5: session end from aclk ----
  @(negedge aclk); session_end_aclk = 2'b01; @(negedge aclk); session_end_aclk = 2'b00;
  repeat (6) @(negedge clk);
  check(session_valid[0] === 1'b0, "A5 session_end on aclk crosses and retires ch0 session");
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

  // =================== Part B: crossed lanes ===================
  cross_lanes = 1;
  exp_wr = 0; exp_rd = 0; rx_bad = 0;
  bring_up(8'd3, 8'd2, 8'd1, 8'd0);
  check(link_trained === 1'b1 && lane_reversed_locked === 1'b1, "B1 reversed lane numbers -> lane_reversed_locked");
  push_flits(10, 0);
  wait_rx_drain(10000);
  check(exp_rd == 80 && rx_bad == 0, "B2 10 Flits bit-exact through a lane-reversed channel");

  // =================== Part C: framing check ===================
  phy_tx_data = 256'h1; phy_tx_sof = 0; phy_tx_eof = 0; phy_tx_valid = 1;   // word 0 without sof
  @(negedge clk); phy_tx_valid = 0;
  repeat (2) @(negedge clk);
  check(phy_framing_err === 1'b1, "C  word 0 without sof flags framing_err");

  if (errors == 0) $display("ALL PASS: pcie_phy_top");
  else             $display("TOTAL FAILURES: %0d", errors);
  $finish;
end

initial begin
  session_end_aclk = 0;
  #5_000_000;
  $display("FAIL: global timeout"); $display("TOTAL FAILURES: timeout");
  $finish;
end

endmodule
`default_nettype wire
