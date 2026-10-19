// =============================================================================
// pcie_phy_top - Protocol-to-PHY layer top (Kshitij's layer), synthesizable.
// Replaces pcie_link_top (fixes P1, P2, P3, P4, I3 and the PHY side of Seam D).
//
// Before: the datapath only existed inside testbenches, with an ideal channel
// inside each lane module, and pcie_link_top had sideband only. Now this one
// top holds the whole layer and exposes the 4-lane PAM4 symbol bus the PS asks
// for. Loopback (tx_level -> rx_level) is done in the testbench, not here.
//
//            link_layer_tx                         link_layer_rx
//                 | phy_tx_* (valid/ready)               ^ phy_rx_* (sof/eof)
//          +------v-----------------------------------------+------+
//          |  pcie_phy_tx                     pcie_phy_rx           |
//          |   Flit FIFO, striper, SOF,        lanes, swap, SOF     |
//          |   deskew, 4x lane_tx              strip, destriper     |
//          +------|----------------------------------------^--------+
//                 v tx_level[11:0]          rx_level[11:0] |
//                          ======  SerDes / channel  ======
//
//  Sideband (all in this top):
//    TS1/TS2 x4 -> lane_reversal_detector -> latch -> link_trained, lane_reversed
//    pcie_reset_controller  (perst_n -> datapath_rst_n, axi_rst_n)
//    pcie_config_space      (Type-0 header, BAR sizing, MSI-X cap)
//    bar0_dma_regs          (base_addr_ch0/1, arm, weights, limit, MSI-X vectors)
//    2x session_arm_ctrl    (snapshot base_addr/tc on arm -> session_valid)
//    pulse_sync             (session_start -> aclk, session_end <- aclk)
//    msix_controller        (session_done -> 1-DW MWr, valid/ready to link layer)
//
// Clocks: clk is the PCIe/link clock (pcie_clk) shared with the link layer.
// aclk is the interconnect's AXI clock; only session_start/end/overflow cross.
//
// The port list keeps every pcie_link_top port with the same name and
// meaning (the existing cocotb tests run unchanged), plus the new ones.
// =============================================================================
module pcie_phy_top #(
    parameter WORDS_PER_FLIT = 8,
    parameter TX_FIFO_DEPTH  = 16,
    parameter WW             = 8,     // arbiter weight width   (interconnect_top)
    parameter LW             = 24     // session limit width    (interconnect_top)
)(
    input  wire         clk,
    input  wire         perst_n,

    // ---- TS1/TS2 training stimulus, 4 lanes ----
    input  wire         ts_byte_valid,
    input  wire [7:0]   ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3,
    output wire         link_trained,
    output wire         lane_reversed_locked,

    // ---- Reset controller ----
    input  wire         hot_reset_request,
    input  wire         flr_request,
    input  wire         updatefc_received,
    output wire         datapath_rst_n,
    output wire         axi_rst_n,
    output wire         flr_active,

    // ---- Config space CfgRd/CfgWr access ----
    input  wire         cfg_wr_en,
    input  wire         cfg_rd_en,
    input  wire [11:0]  cfg_addr,
    input  wire [31:0]  cfg_wdata,
    output wire [31:0]  cfg_rdata,
    output wire         cfg_cpl_valid,

    // ---- MSI-X table/PBA access ----
    input  wire         tbl_wr_en,
    input  wire [2:0]   tbl_wr_vec,
    input  wire [63:0]  tbl_wr_addr,
    input  wire [31:0]  tbl_wr_data,
    input  wire         tbl_wr_mask,
    input  wire         tbl_rd_en,
    input  wire [2:0]   tbl_rd_vec,
    output wire [63:0]  tbl_rd_addr,
    output wire [31:0]  tbl_rd_data,
    output wire         tbl_rd_mask,
    output wire [7:0]   pba,
    input  wire         dma_complete_pulse,      // direct trigger (test/debug); session_done below is the real source
    input  wire [2:0]   dma_complete_vector,

    // ---- MSI-X message to link layer (Seam D / L10): 1-DW MWr, valid/ready ----
    output wire         mwr_valid,
    input  wire         mwr_ready,               // NEW: tie 1 if the link layer can always take it
    output wire [63:0]  mwr_addr,
    output wire [31:0]  mwr_data,

    // ==== NEW: datapath ====
    // from link_layer_tx
    input  wire [255:0] phy_tx_data,
    input  wire         phy_tx_valid,
    input  wire         phy_tx_sof,
    input  wire         phy_tx_eof,
    output wire         phy_tx_ready,            // link layer must stall when low (Seam C)
    // 4-lane PAM4 symbol bus (lane i = [3*i +: 3], signed -3/-1/+1/+3)
    output wire [11:0]  tx_level,
    output wire [3:0]   tx_level_valid,
    input  wire [11:0]  rx_level,
    input  wire [3:0]   rx_level_valid,
    // to link_layer_rx
    output wire [255:0] phy_rx_data,
    output wire         phy_rx_valid,
    output wire         phy_rx_sof,
    output wire         phy_rx_eof,
    // status
    output wire         phy_tx_idle,
    output wire         phy_framing_err,
    output wire         phy_lane_err,

    // ==== NEW: BAR0 DMA registers (driven by cocotb until the inbound MWr path exists) ====
    input  wire         bar0_wr_en,
    input  wire         bar0_rd_en,
    input  wire [11:0]  bar0_addr,
    input  wire [31:0]  bar0_wdata,
    output wire [31:0]  bar0_rdata,
    output wire         bar0_rd_valid,

    // ==== NEW: to link layer (clk domain) ====
    output wire [63:0]  cfg_base_addr_ch0,
    output wire [63:0]  cfg_base_addr_ch1,
    output wire [1:0]   cfg_base_addr_wr,        // 1-cycle pulse when a channel's base is (re)latched
    output wire [2:0]   cfg_tc_ch0,
    output wire [2:0]   cfg_tc_ch1,
    output wire [1:0]   session_valid,
    input  wire [1:0]   session_done,            // link layer: last TLP of the session issued -> MSI-X

    // ==== NEW: to / from interconnect (aclk domain) ====
    input  wire         aclk,
    input  wire         aclk_rst_n,
    output wire [1:0]   session_start_aclk,      // 1-cycle pulse per channel on aclk (I3)
    input  wire [1:0]   session_end_aclk,        // tlast accepted / overflow abort, pulse on aclk
    input  wire [1:0]   session_overflow_aclk,   // sticky overflow flags (level), shown in BAR0 STATUS
    output wire [WW-1:0] arb_weight_ch0,         // quasi-static: only change between sessions
    output wire [WW-1:0] arb_weight_ch1,
    output wire [LW-1:0] session_limit
);

// -----------------------------------------------------------------------------
// Training chain + reset controller (unchanged from pcie_link_top)
// -----------------------------------------------------------------------------
wire lnv0, lnv1, lnv2, lnv3;
wire [7:0] lnout0, lnout1, lnout2, lnout3;

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

lane_reversal_latch u_latch (
  .clk(clk), .rst_n(perst_n),
  .reversal_detected_valid(reversal_detected_valid), .lane_reversed_in(lane_reversed),
  .lane_reversed_locked(lane_reversed_locked), .link_trained(link_trained)
);

pcie_reset_controller u_rst (
  .clk(clk), .perst_n(perst_n),
  .hot_reset_request(hot_reset_request), .flr_request(flr_request),
  .link_trained(link_trained), .updatefc_received(updatefc_received),
  .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n), .flr_active(flr_active)
);

// -----------------------------------------------------------------------------
// Datapath
// -----------------------------------------------------------------------------
pcie_phy_tx #(.FIFO_DEPTH(TX_FIFO_DEPTH), .WORDS_PER_FLIT(WORDS_PER_FLIT)) u_tx (
  .clk(clk), .rst_n(datapath_rst_n),
  .flit_data(phy_tx_data), .flit_valid(phy_tx_valid), .flit_sof(phy_tx_sof), .flit_eof(phy_tx_eof),
  .flit_ready(phy_tx_ready),
  .tx_level(tx_level), .tx_level_valid(tx_level_valid),
  .tx_idle(phy_tx_idle), .framing_err(phy_framing_err)
);

pcie_phy_rx #(.WORDS_PER_FLIT(WORDS_PER_FLIT)) u_rx (
  .clk(clk), .rst_n(datapath_rst_n),
  .rx_level(rx_level), .rx_level_valid(rx_level_valid),
  .lane_reversed(lane_reversed_locked),
  .flit_out_data(phy_rx_data), .flit_out_valid(phy_rx_valid),
  .flit_out_sof(phy_rx_sof), .flit_out_eof(phy_rx_eof),
  .lane_err(phy_lane_err)
);

// -----------------------------------------------------------------------------
// Config space (unchanged block)
// -----------------------------------------------------------------------------
pcie_config_space #(.BAR0_SIZE(32'h0001_0000)) u_cfg (
  .clk(clk), .rst_n(datapath_rst_n),
  .cfg_wr_en(cfg_wr_en), .cfg_rd_en(cfg_rd_en), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
  .cfg_rdata(cfg_rdata), .cfg_cpl_valid(cfg_cpl_valid),
  .link_training_bit(link_trained), .link_width(6'd4), .link_speed(4'd6),
  .bar_addr_0(), .bar_addr_1(), .bar_addr_2(), .bar_addr_3(), .bar_addr_4(), .bar_addr_5(),
  .max_payload_size(), .max_read_req_size(),
  .mem_space_enable(), .bus_master_enable(),
  .msix_enable(), .msix_func_mask()
);

// -----------------------------------------------------------------------------
// BAR0 DMA registers + per-channel session control
// -----------------------------------------------------------------------------
wire [1:0]  arm_pulse;
wire [63:0] base_ch0, base_ch1;
wire [2:0]  msix_vec_ch0, msix_vec_ch1, tc_ch0, tc_ch1;
wire [7:0]  arm_dbg0, arm_dbg1;
wire [1:0]  session_end_clk;

// overflow flags are levels in aclk: plain 2-flop synchroniser into clk
(* ASYNC_REG = "TRUE" *) reg [1:0] ovf_s1, ovf_s2;
always @(posedge clk or negedge datapath_rst_n) begin
  if (!datapath_rst_n) begin ovf_s1 <= 2'b00; ovf_s2 <= 2'b00; end
  else                 begin ovf_s1 <= session_overflow_aclk; ovf_s2 <= ovf_s1; end
end

bar0_dma_regs #(.WW(WW), .LW(LW)) u_bar0 (
  .clk(clk), .rst_n(datapath_rst_n),
  .bar0_wr_en(bar0_wr_en), .bar0_rd_en(bar0_rd_en), .bar0_addr(bar0_addr),
  .bar0_wdata(bar0_wdata), .bar0_rdata(bar0_rdata), .bar0_rd_valid(bar0_rd_valid),
  .arm_pulse(arm_pulse),
  .base_addr_ch0(base_ch0), .base_addr_ch1(base_ch1),
  .weight_ch0(arb_weight_ch0), .weight_ch1(arb_weight_ch1),
  .session_limit(session_limit),
  .msix_vec_ch0(msix_vec_ch0), .msix_vec_ch1(msix_vec_ch1),
  .tc_ch0(tc_ch0), .tc_ch1(tc_ch1),
  .session_valid(session_valid), .status_in(ovf_s2),
  .arm_dbg_ch0(arm_dbg0), .arm_dbg_ch1(arm_dbg1)
);

session_arm_ctrl u_sess0 (
  .clk(clk), .rst_n(datapath_rst_n),
  .base_addr_in(base_ch0), .tc_in(tc_ch0),
  .session_arm(arm_pulse[0]), .session_end(session_end_clk[0]),
  .base_addr_latched(cfg_base_addr_ch0), .tc_latched(cfg_tc_ch0),
  .session_valid(session_valid[0]), .session_arm_while_active_cnt(arm_dbg0)
);

session_arm_ctrl u_sess1 (
  .clk(clk), .rst_n(datapath_rst_n),
  .base_addr_in(base_ch1), .tc_in(tc_ch1),
  .session_arm(arm_pulse[1]), .session_end(session_end_clk[1]),
  .base_addr_latched(cfg_base_addr_ch1), .tc_latched(cfg_tc_ch1),
  .session_valid(session_valid[1]), .session_arm_while_active_cnt(arm_dbg1)
);

// session_valid is a LEVEL; the interconnect needs a start PULSE on aclk (I3).
// Rising edge in clk -> toggle synchroniser -> 1-cycle pulse on aclk.
reg  [1:0] session_valid_d;
always @(posedge clk or negedge datapath_rst_n) begin
  if (!datapath_rst_n) session_valid_d <= 2'b00;
  else                 session_valid_d <= session_valid;
end
wire [1:0] session_start_clk = session_valid & ~session_valid_d;
assign cfg_base_addr_wr = session_start_clk;   // base_addr_latched is stable from here on

pulse_sync u_start_cdc0 (.src_clk(clk),  .src_rst_n(datapath_rst_n), .src_pulse(session_start_clk[0]),
                         .dst_clk(aclk), .dst_rst_n(aclk_rst_n),     .dst_pulse(session_start_aclk[0]));
pulse_sync u_start_cdc1 (.src_clk(clk),  .src_rst_n(datapath_rst_n), .src_pulse(session_start_clk[1]),
                         .dst_clk(aclk), .dst_rst_n(aclk_rst_n),     .dst_pulse(session_start_aclk[1]));
pulse_sync u_end_cdc0   (.src_clk(aclk), .src_rst_n(aclk_rst_n),     .src_pulse(session_end_aclk[0]),
                         .dst_clk(clk),  .dst_rst_n(datapath_rst_n), .dst_pulse(session_end_clk[0]));
pulse_sync u_end_cdc1   (.src_clk(aclk), .src_rst_n(aclk_rst_n),     .src_pulse(session_end_aclk[1]),
                         .dst_clk(clk),  .dst_rst_n(datapath_rst_n), .dst_pulse(session_end_clk[1]));

// -----------------------------------------------------------------------------
// MSI-X: session_done from the link layer -> completion event with the
// channel's vector from BAR0. A direct dma_complete_pulse has priority; done
// events wait one cycle in done_pend if they collide (ch0 before ch1).
// -----------------------------------------------------------------------------
reg  [1:0] done_pend;
wire [1:0] done_all  = done_pend | session_done;
wire       serve     = !dma_complete_pulse && (done_all != 2'b00);
wire       serve_ch1 = !done_all[0];
always @(posedge clk) begin
  if (!datapath_rst_n) done_pend <= 2'b00;
  else if (serve)      done_pend <= done_all & (serve_ch1 ? 2'b01 : 2'b10);
  else                 done_pend <= done_all;
end

wire       msix_evt = dma_complete_pulse || serve;
wire [2:0] msix_vec = dma_complete_pulse ? dma_complete_vector :
                      (serve_ch1 ? msix_vec_ch1 : msix_vec_ch0);

msix_controller #(.NUM_VECTORS(8)) u_msix (
  .clk(clk), .rst_n(datapath_rst_n),
  .tbl_wr_en(tbl_wr_en), .tbl_wr_vec(tbl_wr_vec), .tbl_wr_addr(tbl_wr_addr),
  .tbl_wr_data(tbl_wr_data), .tbl_wr_mask(tbl_wr_mask),
  .tbl_rd_en(tbl_rd_en), .tbl_rd_vec(tbl_rd_vec),
  .tbl_rd_addr(tbl_rd_addr), .tbl_rd_data(tbl_rd_data), .tbl_rd_mask(tbl_rd_mask),
  .pba(pba),
  .dma_complete_pulse(msix_evt), .dma_complete_vector(msix_vec),
  .mwr_valid(mwr_valid), .mwr_ready(mwr_ready), .mwr_addr(mwr_addr), .mwr_data(mwr_data)
);

endmodule
