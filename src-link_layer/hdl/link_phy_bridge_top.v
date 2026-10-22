`timescale 1ns/1ps
 
// ============================================================
// Module : link_phy_bridge_top
// Layer  : glue between the link layer (Swaray) and the protocol-to-PHY
//          layer (Kshitij) -- the module bridge_top.v has always expected
//          to instantiate under this exact name/port list, but which
//          never actually existed as a project doc until now.
//
// What this wires together
//
//   interconnect_top --s_axis_*--> [ this module ] --tx_level/rx_level--> SerDes
//                                        |
//                        link_layer_top (Swaray: TX chain + Flit DLL +
//                        the new RX decode -- see link_layer_top.v) on one
//                        side, pcie_phy_top (Kshitij: training, reset,
//                        config space, BAR0/MSI-X regs, PHY datapath) on
//                        the other, joined at:
//                          * phy_tx_*/phy_rx_*        (Flit <-> PAM4 PHY)
//                          * cfg_base_addr_ch0/ch1/wr (session arm -> DMA base)
//                          * msix: mwr_* <-> msix_req_*, session_done
//
// cfg_base_addr arbitration
//   pcie_phy_top exposes TWO base-address write pulses (one per stream),
//   but link_layer_top's config port is a single {id, data, wr} write
//   (it forwards straight into stream_tx_path_top -> address_translation,
//   which only has one such port, shared by both streams via the id
//   field). If both channels arm in the same cycle, channel 0 is served
//   immediately and channel 1 is deferred one cycle (pend1), the same
//   collision-handling shape pcie_phy_top's own MSI-X arbiter uses
//   (done_pend). base_addr_latched is already stable on pcie_phy_top's
//   side by the time this pulses, so a one-cycle defer loses nothing.
//
// Known follow-up (flagged, not fixed here)
//   link_layer_top's inbound-MWr decode window (cfg_window_base/mask,
//   cfg_axi_base) has no config-space source yet -- pcie_config_space /
//   bar0_dma_regs don't expose a BAR0 window register today. Until they
//   do, the window is fixed by the WINDOW_* / AXI_BASE_DEFAULT parameters
//   below rather than a port, so this module's own port list doesn't
//   have to change later -- override the parameters at this instance if
//   a different window is needed.
//   m_axi_awready is tied high (no AXI slave behind the window yet).
//
// G1 (P2-1): host-driven BAR0 writes
//   link_layer_top reports every inbound MWr's first data DW on inb_mwr_*
//   (raw PCIe address, data, first-DW BE). This module checks it against
//   the BAR0 the host programmed in config space (pcie_phy_top
//   cfg_bar0_addr, 32-bit memory BAR, 64 KB) and, on a hit with all four
//   byte enables set, drives pcie_phy_top's existing BAR0 write port with
//   offset = addr[15:0]. That port already decodes 0x0000 DMA/session regs
//   (bar0_dma_regs), 0x1000 MSI-X table, 0x1800 PBA and gates on Memory
//   Space Enable -- so host MWrs now reach all three.
//   * The external bar0_* ports stay (cocotb / benches). A test-port
//     access (read or write) in the same cycle wins; the inbound write
//     waits one cycle in a 1-deep holding register (ib_pend).
//   * BAR0 base 0 is treated as "not assigned": no inbound hits. Keeps
//     loopback benches (which never program BAR0) from having their own
//     DMA MWrs land in the registers.
//   * Partial-BE writes are dropped and flagged (err_bar0_partial_wr):
//     the BAR0 blocks are whole-DW registers with no byte merge.
//   * Only the first DW of a multi-DW MWr is written.
//   * CfgWr is not covered here (still the cfg_* test port).
//
// G2 (P2-2): pcie_phy_top's updatefc_received (gates axi_rst_n) is now
//   driven by the link layer's FC init (InitFC recorded for P/NP/Cpl),
//   OR'd with the old external input so existing benches still work.
// ============================================================
 
module link_phy_bridge_top #(
    parameter DATA_W     = 512,
    parameter KEEP_W     = DATA_W / 8,
    parameter SOFF_W     = 24,
    parameter TUSER_W    = SOFF_W + 2,     // {tlast_flag, session_offset, stream_id}
    parameter NUM_STREAMS = 2,
    parameter SYS_BUS_W  = 256,
    parameter AXI_ADDR_W = 32,
    parameter WW         = 8,              // arbiter weight width  (interconnect_top)
    parameter LW         = 24,             // session limit width   (interconnect_top)
 
    // Inbound-MWr decode window (see "Known follow-up" above)
    parameter [63:0]        WINDOW_BASE      = 64'h0000_0001_0000_0000,
    parameter [63:0]        WINDOW_MASK      = 64'hFFFF_FFFF_0000_0000,
    parameter [AXI_ADDR_W-1:0] AXI_BASE_DEFAULT = 32'h2000_0000
)(
    input  wire              clk,
    input  wire              perst_n,
    input  wire              aclk,
    input  wire              aclk_rst_n,
 
    // ---- Upstream AXI4-Stream (from interconnect_top) ----
    input  wire [DATA_W-1:0]   s_axis_tdata,
    input  wire [KEEP_W-1:0]   s_axis_tkeep,
    input  wire [TUSER_W-1:0]  s_axis_tuser,    // {tlast_flag, session_offset, stream_id} -- emitted natively by interconnect_top
    input  wire                s_axis_tlast,
    input  wire                s_axis_tvalid,
    output wire                s_axis_tready,
 
    // ---- Inbound TLPs (host -> device), decoded raw bytes, pre-BAR0 ----
    output wire                     m_rx_axis_tvalid,
    output wire [DATA_W-1:0]        m_rx_axis_tdata,
    output wire [KEEP_W-1:0]        m_rx_axis_tkeep,
    output wire                     m_rx_axis_tlast,
    input  wire                     m_rx_axis_tready,
 
    // ---- Training / reset sideband (straight to pcie_phy_top) ----
    input  wire         ts_byte_valid,
    input  wire [7:0]   ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3,
    output wire         link_trained,
    output wire         lane_reversed_locked,
    input  wire         hot_reset_request,
    input  wire         flr_request,
    input  wire         updatefc_received,
    output wire         datapath_rst_n,
    output wire         axi_rst_n,
    output wire         flr_active,
 
    // ---- Config-space CfgRd/CfgWr (straight to pcie_phy_top) ----
    input  wire         cfg_wr_en,
    input  wire         cfg_rd_en,
    input  wire [11:0]  cfg_addr,
    input  wire [31:0]  cfg_wdata,
    output wire [31:0]  cfg_rdata,
    output wire         cfg_cpl_valid,
 
    // ---- MSI-X table/PBA (straight to pcie_phy_top) ----
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
    input  wire         dma_complete_pulse,
    input  wire [2:0]   dma_complete_vector,
 
    // ---- BAR0 DMA registers, external/test access (straight to pcie_phy_top) ----
    input  wire         bar0_wr_en,
    input  wire         bar0_rd_en,
    input  wire [15:0]  bar0_addr,       // 0x0000 DMA regs, 0x1000 MSI-X table, 0x1800 PBA
    input  wire [31:0]  bar0_wdata,
    output wire [31:0]  bar0_rdata,
    output wire         bar0_rd_valid,
 
    // ---- SerDes symbol bus ----
    output wire [11:0]  tx_level,
    output wire [3:0]   tx_level_valid,
    input  wire [11:0]  rx_level,
    input  wire [3:0]   rx_level_valid,
 
    // ---- Session handshake with the interconnect (aclk domain) ----
    output wire [NUM_STREAMS-1:0] session_start_aclk,
    input  wire [NUM_STREAMS-1:0] session_end_aclk,
    input  wire [NUM_STREAMS-1:0] session_overflow_aclk,
    output wire [WW-1:0]          arb_weight_ch0,
    output wire [WW-1:0]          arb_weight_ch1,
    output wire [LW-1:0]          session_limit,
 
    // ---- Status ----
    output wire  err_xlat_miss,
    output wire  err_addr_ovf,
    output wire  stat_split_occurred,
    output wire  err_tkeep_gap,
    output wire  crc_err,
    output wire  fec_err,
    output wire  fec_corrected,
    output wire  replay_event,
    output wire  phy_tx_idle,
    output wire  phy_framing_err,
    output wire  phy_lane_err,
 
    // ---- Decoded inbound MWr -> AXI BAR0 write (new; not yet consumed by
    //      bridge_top.v -- see "Known follow-up" above) ----
    output wire                   m_axi_awvalid,
    output wire [AXI_ADDR_W-1:0]  m_axi_awaddr,
    output wire                   m_axi_awerr,
    output wire                   malformed_tlp_err,

    // ---- G1: inbound MWr -> BAR0 status (1-cycle pulses) ----
    output wire                   inb_bar0_wr,          // an inbound MWr was written into BAR0
    output wire                   err_bar0_partial_wr,  // BAR0 hit with first-DW BE != 4'hF, dropped

    // ---- G2: flow control ----
    output wire                   fc_init_done          // link-layer FC_INIT2 complete
);
 
    // synthesis translate_off
    initial begin
        if (TUSER_W != SOFF_W + 2)
            $fatal(1, "[link_phy_bridge_top] TUSER_W (%0d) must equal SOFF_W+2 (%0d)",
                   TUSER_W, SOFF_W + 2);
    end
    // synthesis translate_on
 
    // ================================================================
    // 1. pcie_phy_top -- Kshitij's layer (training, reset, config space,
    //    BAR0/MSI-X regs, PHY datapath)
    // ================================================================
    wire                   phy_tx_valid, phy_tx_sof, phy_tx_eof, phy_tx_ready;
    wire [SYS_BUS_W-1:0]   phy_tx_data;
    wire                   phy_rx_valid, phy_rx_sof, phy_rx_eof;
    wire [SYS_BUS_W-1:0]   phy_rx_data;
 
    wire [63:0] pt_base_ch0, pt_base_ch1;
    wire [1:0]  pt_base_wr;
    wire [2:0]  pt_tc_ch0, pt_tc_ch1;           // not consumed by link_layer_top today
    wire [1:0]  pt_session_valid;               // not consumed by link_layer_top today
 
    wire        pt_mwr_valid;
    wire [63:0] pt_mwr_addr;
    wire [31:0] pt_mwr_data;
    wire        pt_mwr_ready;
 
    wire [1:0]  ll_session_done;
    wire        ll_fc_credits_valid;   // G2
    wire [7:0]  ll_xlat_miss_cnt;       // G3
    wire        ll_xlat_miss_clr;       // G3

    // G1: BAR0 base from config space, inbound MWr from link_layer_top
    wire [31:0] pt_bar0_base;
    wire        ll_inb_mwr_valid;
    wire [63:0] ll_inb_mwr_addr;
    wire [31:0] ll_inb_mwr_data;
    wire [3:0]  ll_inb_mwr_be;

    // G1: BAR0 write port as seen by pcie_phy_top (test port or inbound MWr)
    wire        pt_bar0_wr_en;
    wire [15:0] pt_bar0_addr;
    wire [31:0] pt_bar0_wdata;
 
    pcie_phy_top #(
        .WW(WW), .LW(LW)
    ) u_pcie_phy_top (
        .clk               (clk),
        .perst_n           (perst_n),
 
        .ts_byte_valid     (ts_byte_valid),
        .ts_byte_l0        (ts_byte_l0), .ts_byte_l1(ts_byte_l1),
        .ts_byte_l2        (ts_byte_l2), .ts_byte_l3(ts_byte_l3),
        .link_trained      (link_trained),
        .lane_reversed_locked(lane_reversed_locked),
 
        .hot_reset_request (hot_reset_request),
        .flr_request       (flr_request),
        .updatefc_received (updatefc_received | ll_fc_credits_valid),   // G2: real FC init, test input kept
        .datapath_rst_n    (datapath_rst_n),
        .axi_rst_n         (axi_rst_n),
        .flr_active        (flr_active),
 
        .cfg_wr_en         (cfg_wr_en), .cfg_rd_en(cfg_rd_en),
        .cfg_addr          (cfg_addr), .cfg_wdata(cfg_wdata),
        .cfg_rdata         (cfg_rdata), .cfg_cpl_valid(cfg_cpl_valid),
 
        .tbl_wr_en         (tbl_wr_en), .tbl_wr_vec(tbl_wr_vec),
        .tbl_wr_addr       (tbl_wr_addr), .tbl_wr_data(tbl_wr_data), .tbl_wr_mask(tbl_wr_mask),
        .tbl_rd_en         (tbl_rd_en), .tbl_rd_vec(tbl_rd_vec),
        .tbl_rd_addr       (tbl_rd_addr), .tbl_rd_data(tbl_rd_data), .tbl_rd_mask(tbl_rd_mask),
        .pba               (pba),
        .dma_complete_pulse(dma_complete_pulse), .dma_complete_vector(dma_complete_vector),
 
        .mwr_valid         (pt_mwr_valid), .mwr_ready(pt_mwr_ready),
        .mwr_addr          (pt_mwr_addr),  .mwr_data (pt_mwr_data),
 
        .phy_tx_data       (phy_tx_data), .phy_tx_valid(phy_tx_valid),
        .phy_tx_sof        (phy_tx_sof),  .phy_tx_eof(phy_tx_eof), .phy_tx_ready(phy_tx_ready),
        .tx_level          (tx_level), .tx_level_valid(tx_level_valid),
        .rx_level          (rx_level), .rx_level_valid(rx_level_valid),
        .phy_rx_data       (phy_rx_data), .phy_rx_valid(phy_rx_valid),
        .phy_rx_sof        (phy_rx_sof),  .phy_rx_eof(phy_rx_eof),
        .phy_tx_idle       (phy_tx_idle), .phy_framing_err(phy_framing_err), .phy_lane_err(phy_lane_err),
 
        .bar0_wr_en        (pt_bar0_wr_en), .bar0_rd_en(bar0_rd_en), .bar0_addr(pt_bar0_addr),
        .bar0_wdata        (pt_bar0_wdata), .bar0_rdata(bar0_rdata), .bar0_rd_valid(bar0_rd_valid),
        .cfg_bar0_addr     (pt_bar0_base),
        .xlat_miss_cnt     (ll_xlat_miss_cnt), .xlat_miss_clr(ll_xlat_miss_clr),
 
        .cfg_base_addr_ch0 (pt_base_ch0), .cfg_base_addr_ch1(pt_base_ch1),
        .cfg_base_addr_wr  (pt_base_wr),
        .cfg_tc_ch0        (pt_tc_ch0),   .cfg_tc_ch1(pt_tc_ch1),
        .session_valid     (pt_session_valid),
        .session_done      (ll_session_done),
 
        .aclk              (aclk), .aclk_rst_n(aclk_rst_n),
        .session_start_aclk(session_start_aclk),
        .session_end_aclk  (session_end_aclk),
        .session_overflow_aclk(session_overflow_aclk),
        .arb_weight_ch0    (arb_weight_ch0), .arb_weight_ch1(arb_weight_ch1),
        .session_limit     (session_limit)
    );
 
    // ================================================================
    // 2. cfg_base_addr arbiter -- two per-channel pulses from
    //    pcie_phy_top down to link_layer_top's single {id,data,wr} port
    //    (see header comment)
    // ================================================================
    reg pend1;
    wire serve0 = pt_base_wr[0];
    wire serve1 = !serve0 && (pt_base_wr[1] || pend1);
 
    always @(posedge clk or negedge datapath_rst_n) begin
        if (!datapath_rst_n)       pend1 <= 1'b0;
        else if (serve1)          pend1 <= 1'b0;
        else if (pt_base_wr[1])   pend1 <= 1'b1;
    end
 
    wire                                   ll_cfg_base_addr_wr   = serve0 || serve1;
    wire [$clog2(NUM_STREAMS)-1:0]         ll_cfg_base_addr_id   = serve1 ? 1'b1 : 1'b0;
    wire [63:0]                             ll_cfg_base_addr_data = serve1 ? pt_base_ch1 : pt_base_ch0;
 
    // ================================================================
    // G1/S13: BAR0 window (declared before use)
    localparam BAR0_AW = 16;                       // 64 KB BAR0 (pcie_config_space BAR0_SIZE)
    wire ib_bar0_assigned = (pt_bar0_base[31:BAR0_AW] != {(32-BAR0_AW){1'b0}});

    // 3. link_layer_top -- Swaray's layer: TX stream chain + Flit DLL +
    //    the new RX decode (unpacker -> tlp_header_parser -> pcie_rx_engine)
    // ================================================================
    // TUSER arrives already in the link's native {tlast_flag, session_offset,
    // stream_id} order from interconnect_top -- passed straight through, no remap.
 
    link_layer_top #(
        .DATA_W      (DATA_W),
        .KEEP_W      (KEEP_W),
        .TUSER_W     (TUSER_W),
        .SOFF_W      (SOFF_W),
        .NUM_STREAMS (NUM_STREAMS),
        .SYS_BUS_W   (SYS_BUS_W),
        .AXIS_DATA_W (DATA_W),
        .AXI_ADDR_W  (AXI_ADDR_W)
    ) u_link_layer_top (
        .clk                (clk),
        .rst_n              (datapath_rst_n),
 
        .s_axis_tdata       (s_axis_tdata),
        .s_axis_tkeep       (s_axis_tkeep),
        .s_axis_tuser       (s_axis_tuser),
        .s_axis_tlast       (s_axis_tlast),
        .s_axis_tvalid      (s_axis_tvalid),
        .s_axis_tready      (s_axis_tready),
 
        .cfg_base_addr_wr   (ll_cfg_base_addr_wr),
        .cfg_base_addr_id   (ll_cfg_base_addr_id),
        .cfg_base_addr_data (ll_cfg_base_addr_data),
        .cfg_base_addr_clr  (1'b0),
 
        .cfg_window_base    (WINDOW_BASE),
        .cfg_window_mask    (WINDOW_MASK),
        .cfg_axi_base       (AXI_BASE_DEFAULT),
        .cfg_excl_en        (ib_bar0_assigned),                              // S13: BAR0 hits
        .cfg_excl_base      ({32'h0, pt_bar0_base[31:BAR0_AW], {BAR0_AW{1'b0}}}),   //   are not
        .cfg_excl_mask      ({{(64-BAR0_AW){1'b1}}, {BAR0_AW{1'b0}}}),                           //      xlat misses
 
        .msix_req_valid     (pt_mwr_valid),
        .msix_req_addr      (pt_mwr_addr),
        .msix_req_data      (pt_mwr_data),
        .msix_req_ready     (pt_mwr_ready),
        .session_done       (ll_session_done),
 
        .phy_tx_valid       (phy_tx_valid),
        .phy_tx_data        (phy_tx_data),
        .phy_tx_sof         (phy_tx_sof),
        .phy_tx_eof         (phy_tx_eof),
        .phy_tx_ready       (phy_tx_ready),
 
        .phy_rx_valid       (phy_rx_valid),
        .phy_rx_data        (phy_rx_data),
        .phy_rx_sof         (phy_rx_sof),
        .phy_rx_eof         (phy_rx_eof),
 
        .m_axi_awvalid      (m_axi_awvalid),
        .m_axi_awaddr       (m_axi_awaddr),
        .m_axi_awerr        (m_axi_awerr),
        .m_axi_awready      (1'b1),           // no BAR0-write slave wired yet -- see header note
 
        .cpl_tvalid         (/* unused -- MWr-only path, never fires */),
        .cpl_tdata          (/* unused */),
        .cpl_tlast          (/* unused */),
        .cpl_tkeep          (/* unused */),
        .cpl_tuser          (/* unused */),
        .cpl_tready         (1'b1),
 
        .err_xlat_miss      (err_xlat_miss),
        .err_addr_ovf       (err_addr_ovf),
        .stat_split_occurred(stat_split_occurred),
        .err_tkeep_gap      (err_tkeep_gap),
        .crc_err            (crc_err),
        .fec_err            (fec_err),
        .fec_corrected      (fec_corrected),
        .replay_event       (replay_event),
        .malformed_tlp_err  (malformed_tlp_err),

        .fc_credits_valid   (ll_fc_credits_valid),
        .fc_init_done       (fc_init_done),

        .err_xlat_miss_cnt  (ll_xlat_miss_cnt),
        .err_xlat_miss_clr  (ll_xlat_miss_clr),

        .inb_mwr_valid      (ll_inb_mwr_valid),
        .inb_mwr_addr       (ll_inb_mwr_addr),
        .inb_mwr_data       (ll_inb_mwr_data),
        .inb_mwr_be         (ll_inb_mwr_be),

        .mon_rx_axis_tvalid (m_rx_axis_tvalid),
        .mon_rx_axis_tdata  (m_rx_axis_tdata),
        .mon_rx_axis_tkeep  (m_rx_axis_tkeep),
        .mon_rx_axis_tlast  (m_rx_axis_tlast)
    );
 
    // ================================================================
    // 4. G1: inbound MWr -> BAR0 write port (see header note)
    // ================================================================
    // (BAR0_AW / ib_bar0_assigned are declared above u_link_layer_top: S13 uses them)

    wire ib_bar0_hit      = ll_inb_mwr_valid && ib_bar0_assigned &&
                            (ll_inb_mwr_addr[63:BAR0_AW] == {32'h0, pt_bar0_base[31:BAR0_AW]});
    wire ib_wr_req        = ib_bar0_hit && (ll_inb_mwr_be == 4'hF);
    assign err_bar0_partial_wr = ib_bar0_hit && (ll_inb_mwr_be != 4'hF);

    // Test port has the shared addr/data bus this cycle -> inbound waits.
    wire tb_bar0_busy = bar0_wr_en || bar0_rd_en;

    reg         ib_pend;
    reg  [15:0] ib_pend_addr;
    reg  [31:0] ib_pend_data;

    wire        ib_fire     = !tb_bar0_busy && (ib_pend || ib_wr_req);
    wire [15:0] ib_fire_addr = ib_pend ? ib_pend_addr : ll_inb_mwr_addr[BAR0_AW-1:0];
    wire [31:0] ib_fire_data = ib_pend ? ib_pend_data : ll_inb_mwr_data;

    always @(posedge clk or negedge datapath_rst_n) begin
        if (!datapath_rst_n) begin
            ib_pend      <= 1'b0;
            ib_pend_addr <= 16'h0;
            ib_pend_data <= 32'h0;
        end else if (ib_wr_req && !(ib_fire && !ib_pend)) begin
            // new request not served directly: hold it (if a pending one
            // fires this cycle the slot frees as the new one enters)
            ib_pend      <= 1'b1;
            ib_pend_addr <= ll_inb_mwr_addr[BAR0_AW-1:0];
            ib_pend_data <= ll_inb_mwr_data;
        end else if (ib_fire) begin
            ib_pend      <= 1'b0;
        end
    end

    assign pt_bar0_wr_en = bar0_wr_en || ib_fire;
    assign pt_bar0_addr  = ib_fire ? ib_fire_addr : bar0_addr;
    assign pt_bar0_wdata = ib_fire ? ib_fire_data : bar0_wdata;
    assign inb_bar0_wr   = ib_fire;

`ifdef SIMULATION
    // Inbound MWrs are >= 4 DW apart on the 1-DW/clk RX unpacker, so the
    // 1-deep holding register only overflows if the test port is held
    // busy for several cycles while host writes stream in.
    always @(posedge clk)
        if (datapath_rst_n && ib_wr_req && ib_pend && !ib_fire)
            $error("[link_phy_bridge_top] G1: inbound BAR0 write lost (test port busy, holding register full)");
`endif

    // m_rx_axis_* is a MONITOR copy of the inbound TLP byte stream (every beat
    // the RX decode accepts). The decode inside link_layer_top owns flow
    // control, so m_rx_axis_tready is ignored. bridge_top's testbenches check
    // TLP contents/order on this tap.
    wire unused_rx_tready = m_rx_axis_tready;
 
endmodule