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
//    pcie_ltssm (Detect -> Polling/TS1 -> Configuration/TS2 -> L0) drives the
//    TX ordered-set generator; received lanes -> rx_lane_deskew -> TS1/TS2
//    detectors x4 -> LTSSM, and -> lane_reversal_detector -> latch -> lane_reversed.
//    link_trained = LTSSM in L0.
//    pcie_reset_controller  (perst_n -> datapath_rst_n, axi_rst_n)
//    pcie_config_space      (Type-0 header, BAR sizing, MSI-X cap)
//    bar0_dma_regs          (base_addr_ch0/1, arm, weights, limit, MSI-X vectors)
//    2x session_arm_ctrl    (snapshot base_addr/tc on arm -> session_valid)
//    pulse_sync             (session_start -> aclk; session_end_aclk -> clk, debug only)
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
    parameter LW             = 24,    // session limit width    (interconnect_top)
    parameter LTSSM_DETECT_CLKS  = 32,
    parameter LTSSM_RX_TS_MIN    = 8,     // TS received to leave Polling / Configuration
    parameter LTSSM_TX_TS2_MIN   = 16,    // TS2 sent after the first one received
    parameter LTSSM_TIMEOUT_CLKS = 16384  // Polling / Configuration timeout -> Detect
)(
    input  wire         clk,
    input  wire         perst_n,

    // ---- TS1/TS2 training ----
    // Training now normally happens over the lanes: TX sends TS1 ordered sets
    // until link_trained, and the detectors read the received lane bytes.
    // ts_byte_* is kept as an injection port (the original cocotb tests use
    // it); while ts_byte_valid is high it overrides the lane bytes.
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
    output wire         phy_ts_sending,          // TX lanes are carrying TS1/TS2 (training)
    output wire [1:0]   ltssm_state,             // 0 Detect, 1 Polling, 2 Configuration, 3 L0
    output wire         phy_deskew_locked,       // RX lanes aligned on the TS1 COM symbol

    // ==== NEW: BAR0 access (driven by cocotb until the inbound MWr path exists) ====
    // BAR0 map (64 KB): 0x0000-0x0FFF DMA/session regs (bar0_dma_regs)
    //                   0x1000-0x17FF MSI-X table   (config 0x54 points here)
    //                   0x1800-0x1FFF MSI-X PBA     (config 0x58 points here)
    // Accesses are ignored (no write, no rd_valid = unsupported request)
    // while Command.Memory Space Enable is 0, as for a real PCIe function.
    input  wire         bar0_wr_en,
    input  wire         bar0_rd_en,
    input  wire [15:0]  bar0_addr,
    input  wire [31:0]  bar0_wdata,
    output wire [31:0]  bar0_rdata,
    output wire         bar0_rd_valid,
    // G1 (P2-1): BAR0 base as programmed by the host (config 0x10, 32-bit
    // memory BAR, 64 KB). The link side uses it to decide whether a decoded
    // inbound MWr hits BAR0 before driving bar0_wr_en/addr/wdata above.
    output wire [31:0]  cfg_bar0_addr,

    // ==== NEW: to link layer (clk domain) ====
    // G3 (P2-3): inbound address-translation miss status (BAR0 XLAT_MISS)
    input  wire [7:0]   xlat_miss_cnt,
    output wire          xlat_miss_clr,

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
    input  wire [1:0]   session_end_aclk,        // interconnect stopped accepting (tlast / overflow abort),
                                                 // pulse on aclk. DEBUG ONLY since K1: shown in BAR0
                                                 // STATUS[5:4]; sessions end on session_done instead
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

// Training-chain reset (hot-reset retrain): the detectors, reversal detector
// and latch now reset on datapath_rst_n, not only PERST#. A hot reset pulses
// datapath_rst_n, which clears link_trained, so TX sends TS1 again, RX
// re-locks/deskews, and the link retrains over the lanes. No deadlock:
// datapath_rst_n is released by the reset controller on its own (PERST# high,
// hot-reset pulse done) and never waits for link_trained.
wire train_rst_n = datapath_rst_n;

// Detector input: injected bytes when ts_byte_valid, else the received lane
// bytes (pre-swap, so physical lane numbers reach the reversal detector).
wire [31:0] rx_lane_bytes;
wire [3:0]  rx_lane_bytes_valid;
wire [3:0]  det_valid = ts_byte_valid ? 4'b1111 : rx_lane_bytes_valid;
wire [3:0]  osv, ost2;
wire [7:0]  det_b0    = ts_byte_valid ? ts_byte_l0 : rx_lane_bytes[7:0];
wire [7:0]  det_b1    = ts_byte_valid ? ts_byte_l1 : rx_lane_bytes[15:8];
wire [7:0]  det_b2    = ts_byte_valid ? ts_byte_l2 : rx_lane_bytes[23:16];
wire [7:0]  det_b3    = ts_byte_valid ? ts_byte_l3 : rx_lane_bytes[31:24];

ts_ordered_set_detector u_ts0 (.clk(clk), .rst_n(train_rst_n), .byte_valid(det_valid[0]), .byte_in(det_b0), .lane_num_valid(lnv0), .lane_num_out(lnout0),
                              .os_valid(osv[0]), .os_ts2(ost2[0]));
ts_ordered_set_detector u_ts1 (.clk(clk), .rst_n(train_rst_n), .byte_valid(det_valid[1]), .byte_in(det_b1), .lane_num_valid(lnv1), .lane_num_out(lnout1),
                              .os_valid(osv[1]), .os_ts2(ost2[1]));
ts_ordered_set_detector u_ts2 (.clk(clk), .rst_n(train_rst_n), .byte_valid(det_valid[2]), .byte_in(det_b2), .lane_num_valid(lnv2), .lane_num_out(lnout2),
                              .os_valid(osv[2]), .os_ts2(ost2[2]));
ts_ordered_set_detector u_ts3 (.clk(clk), .rst_n(train_rst_n), .byte_valid(det_valid[3]), .byte_in(det_b3), .lane_num_valid(lnv3), .lane_num_out(lnout3),
                              .os_valid(osv[3]), .os_ts2(ost2[3]));

wire reversal_detected_valid, lane_reversed;
wire lanes_identified;   // lane order decided (lane_reversal_latch)
lane_reversal_detector u_revdet (
  .clk(clk), .rst_n(train_rst_n),
  .lane_num_valid({lnv3, lnv2, lnv1, lnv0}),
  .lane_num_l0(lnout0), .lane_num_l1(lnout1), .lane_num_l2(lnout2), .lane_num_l3(lnout3),
  .reversal_detected_valid(reversal_detected_valid), .lane_reversed(lane_reversed)
);

lane_reversal_latch u_latch (
  .clk(clk), .rst_n(train_rst_n),
  .reversal_detected_valid(reversal_detected_valid), .lane_reversed_in(lane_reversed),
  .lane_reversed_locked(lane_reversed_locked), .link_trained(lanes_identified)
);

// ---- LTSSM: Detect -> Polling (TS1) -> Configuration (TS2) -> L0 ----
// link_trained (used everywhere downstream) now means "LTSSM is in L0".
wire [1:0] tx_os_mode;
wire       tx_os_done;
pcie_ltssm #(.DETECT_CLKS(LTSSM_DETECT_CLKS), .RX_TS_MIN(LTSSM_RX_TS_MIN),
             .TX_TS2_MIN(LTSSM_TX_TS2_MIN), .TIMEOUT_CLKS(LTSSM_TIMEOUT_CLKS)) u_ltssm (
  .clk(clk), .rst_n(train_rst_n),
  .os_valid(osv), .os_ts2(ost2), .lanes_identified(lanes_identified),
  .tx_os_done(tx_os_done), .tx_os_mode(tx_os_mode),
  .state(ltssm_state), .link_up(link_trained)
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
  .os_mode(tx_os_mode), .data_en(link_trained), .ts_sending(phy_ts_sending), .os_done(tx_os_done),
  .flit_data(phy_tx_data), .flit_valid(phy_tx_valid), .flit_sof(phy_tx_sof), .flit_eof(phy_tx_eof),
  .flit_ready(phy_tx_ready),
  .tx_level(tx_level), .tx_level_valid(tx_level_valid),
  .tx_idle(phy_tx_idle), .framing_err(phy_framing_err)
);

pcie_phy_rx #(.WORDS_PER_FLIT(WORDS_PER_FLIT)) u_rx (
  .clk(clk), .rst_n(datapath_rst_n),
  .rx_level(rx_level), .rx_level_valid(rx_level_valid),
  .lane_reversed(lane_reversed_locked),
  .align_en(!link_trained),
  .flit_out_data(phy_rx_data), .flit_out_valid(phy_rx_valid),
  .flit_out_sof(phy_rx_sof), .flit_out_eof(phy_rx_eof),
  .lane_err(phy_lane_err),
  .lane_bytes(rx_lane_bytes), .lane_bytes_valid(rx_lane_bytes_valid),
  .deskew_locked(phy_deskew_locked)
);

// -----------------------------------------------------------------------------
// Config space (unchanged block)
// -----------------------------------------------------------------------------
wire mem_space_en, msix_en, msix_fmask;

pcie_config_space #(.BAR0_SIZE(32'h0001_0000)) u_cfg (
  .clk(clk), .rst_n(datapath_rst_n),
  .cfg_wr_en(cfg_wr_en), .cfg_rd_en(cfg_rd_en), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
  .cfg_rdata(cfg_rdata), .cfg_cpl_valid(cfg_cpl_valid),
  .link_training_bit(link_trained), .link_width(6'd4), .link_speed(4'd6),
  .bar_addr_0(cfg_bar0_addr), .bar_addr_1(), .bar_addr_2(), .bar_addr_3(), .bar_addr_4(), .bar_addr_5(),
  .max_payload_size(), .max_read_req_size(),
  .mem_space_enable(mem_space_en), .bus_master_enable(),
  .msix_enable(msix_en), .msix_func_mask(msix_fmask)
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

// ---- BAR0 decode, gated by Memory Space Enable ----
wire        bar0_ok     = mem_space_en;
wire        sel_regs    = (bar0_addr[15:12] == 4'h0);
wire        sel_msix    = (bar0_addr[15:12] == 4'h1);   // bit 11: 0 = table, 1 = PBA
wire        regs_wr     = bar0_ok && bar0_wr_en && sel_regs;
wire        regs_rd     = bar0_ok && bar0_rd_en && sel_regs;
wire        msix_mm_wr  = bar0_ok && bar0_wr_en && sel_msix;
wire        msix_mm_rd  = bar0_ok && bar0_rd_en && sel_msix;
wire [31:0] regs_rdata, msix_mm_rdata;
wire        regs_rd_valid, msix_mm_rd_valid;
assign bar0_rdata    = msix_mm_rd_valid ? msix_mm_rdata : regs_rdata;
assign bar0_rd_valid = regs_rd_valid | msix_mm_rd_valid;

bar0_dma_regs #(.WW(WW), .LW(LW)) u_bar0 (
  .clk(clk), .rst_n(datapath_rst_n),
  .bar0_wr_en(regs_wr), .bar0_rd_en(regs_rd), .bar0_addr(bar0_addr[11:0]),
  .bar0_wdata(bar0_wdata), .bar0_rdata(regs_rdata), .bar0_rd_valid(regs_rd_valid),
  .arm_pulse(arm_pulse),
  .base_addr_ch0(base_ch0), .base_addr_ch1(base_ch1),
  .weight_ch0(arb_weight_ch0), .weight_ch1(arb_weight_ch1),
  .session_limit(session_limit),
  .msix_vec_ch0(msix_vec_ch0), .msix_vec_ch1(msix_vec_ch1),
  .tc_ch0(tc_ch0), .tc_ch1(tc_ch1),
  .session_valid(session_valid), .status_in(ovf_s2), .ic_end_seen(ic_end_seen),
  .arm_dbg_ch0(arm_dbg0), .arm_dbg_ch1(arm_dbg1),
  .xlat_miss_cnt(xlat_miss_cnt), .xlat_miss_clr(xlat_miss_clr)
);

session_arm_ctrl u_sess0 (
  .clk(clk), .rst_n(datapath_rst_n),
  .base_addr_in(base_ch0), .tc_in(tc_ch0),
  .session_arm(arm_pulse[0]), .session_end(session_done[0]),      // K1: end when the data has left the link layer
  .base_addr_latched(cfg_base_addr_ch0), .tc_latched(cfg_tc_ch0),
  .session_valid(session_valid[0]), .session_arm_while_active_cnt(arm_dbg0)
);

session_arm_ctrl u_sess1 (
  .clk(clk), .rst_n(datapath_rst_n),
  .base_addr_in(base_ch1), .tc_in(tc_ch1),
  .session_arm(arm_pulse[1]), .session_end(session_done[1]),
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

// ---- K1: session end ----------------------------------------------------------
// A session ends on session_done from the link layer (its last TLP has been
// issued), not on the interconnect's session_end_aclk. The interconnect stops
// accepting while up to ~32 beats are still queued in its CDC FIFO and the link
// layer; ending the session there let an early re-arm send those beats to the
// NEXT session's base address (46 of 144 TLPs misrouted in bridge sim).
// session_end_aclk is kept as a debug signal: ic_end_seen = "the interconnect
// has finished accepting this session, its data is still draining". It is
// shown in BAR0 STATUS[5:4] and clears when the session ends (or on a new arm).
reg [1:0] ic_end_seen;
always @(posedge clk or negedge datapath_rst_n) begin
  if (!datapath_rst_n) ic_end_seen <= 2'b00;
  else                 ic_end_seen <= (ic_end_seen | session_end_clk) & session_valid & ~arm_pulse;
end

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
  .mwr_valid(mwr_valid), .mwr_ready(mwr_ready), .mwr_addr(mwr_addr), .mwr_data(mwr_data),
  .msix_enable(msix_en), .func_mask(msix_fmask),
  .mm_wr_en(msix_mm_wr), .mm_rd_en(msix_mm_rd), .mm_pba(bar0_addr[11]), .mm_addr({1'b0, bar0_addr[10:0]}),
  .mm_wdata(bar0_wdata), .mm_rdata(msix_mm_rdata), .mm_rd_valid(msix_mm_rd_valid)
);

endmodule
