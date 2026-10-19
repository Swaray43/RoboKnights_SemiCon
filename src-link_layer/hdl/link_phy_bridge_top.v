`timescale 1ns/1ps

// ============================================================
// Module : link_phy_bridge_top
// Layer  : Link layer <-> Protocol-to-PHY seam -- NEW
//
// Joins Swaray's link layer (stream_tx_path_top / data_link_layer_top)
// to Kshitij's pcie_phy_top. This is the "yet-to-be-built bridge top"
// that ic2link_tuser_remap was written for, now that both layers' RTL
// exists. The interconnect side (aclk domain: session_start/end/overflow,
// arbiter weights, session limit) is only passed through here -- it is
// Aditya's interconnect_top that drives/consumes it, and that file is
// not in this change set.
//
// Wiring notes
//   * phy_tx_*/phy_rx_* and mwr_*/session_done already share the same
//     port names and directions on both sides (stream_tx_path_top was
//     written to match pcie_phy_top's seam exactly) -- straight wires,
//     no adapter needed.
//   * cfg_base_addr_ch0/ch1 + cfg_base_addr_wr[1:0] (pcie_phy_top: two
//     always-on 64-bit buses + a per-channel pulse) do NOT fit
//     address_translation's single-pulse {wr, id, data} register-file
//     port. cfg_base_addr_bridge.v adapts between them (see that file).
//   * stream_tx_path_top's rst_n is tied to pcie_phy_top's datapath_rst_n,
//     not to the raw perst_n: the whole link-layer datapath (replay
//     buffer, sequence counters, Flit FSMs) must drop together with the
//     PHY on PERST#/hot-reset, same as pcie_reset_controller's own header
//     comment requires for anything downstream of link training.
//   * ic2link_tuser_remap sits between the raw interconnect TUSER order
//     and stream_tx_path_top's s_axis_tuser (Seam A fix, now instantiated).
// ============================================================

module link_phy_bridge_top #(
    parameter DATA_W         = 512,
    parameter KEEP_W         = DATA_W / 8,
    parameter SOFF_W         = 24,
    parameter ADDR_W         = 64,
    parameter NUM_STREAMS    = 2,
    parameter SYS_BUS_W      = 256,
    parameter WW             = 8,
    parameter LW             = 24
)(
    input  wire                 clk,          // pcie_clk, shared by link layer + PHY
    input  wire                 perst_n,

    input  wire                 aclk,
    input  wire                 aclk_rst_n,

    // ---- Upstream AXI4-Stream from the interconnect (via its CDC) ----
    // ic_tuser is the interconnect's own order: {stream_id, session_offset}
    input  wire [DATA_W-1:0]    s_axis_tdata,
    input  wire [KEEP_W-1:0]    s_axis_tkeep,
    input  wire [SOFF_W:0]      ic_tuser,
    input  wire                 s_axis_tlast,
    input  wire                 s_axis_tvalid,
    output wire                 s_axis_tready,

    // ---- Inbound TLPs (host -> device) ----
    output wire                     m_rx_axis_tvalid,
    output wire [DATA_W-1:0]        m_rx_axis_tdata,
    output wire [KEEP_W-1:0]        m_rx_axis_tkeep,
    output wire                     m_rx_axis_tlast,
    input  wire                     m_rx_axis_tready,

    // ---- TS1/TS2 training stimulus ----
    input  wire         ts_byte_valid,
    input  wire [7:0]   ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3,
    output wire         link_trained,
    output wire         lane_reversed_locked,

    // ---- Reset controller sideband ----
    input  wire         hot_reset_request,
    input  wire         flr_request,
    input  wire         updatefc_received,
    output wire         datapath_rst_n,
    output wire         axi_rst_n,
    output wire         flr_active,

    // ---- Config space CfgRd/CfgWr ----
    input  wire         cfg_wr_en,
    input  wire         cfg_rd_en,
    input  wire [11:0]  cfg_addr,
    input  wire [31:0]  cfg_wdata,
    output wire [31:0]  cfg_rdata,
    output wire         cfg_cpl_valid,

    // ---- MSI-X table/PBA host access ----
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

    // ---- BAR0 DMA registers ----
    input  wire         bar0_wr_en,
    input  wire         bar0_rd_en,
    input  wire [11:0]  bar0_addr,
    input  wire [31:0]  bar0_wdata,
    output wire [31:0]  bar0_rdata,
    output wire         bar0_rd_valid,

    // ---- 4-lane PAM4 symbol bus to/from the SerDes ----
    output wire [11:0]  tx_level,
    output wire [3:0]   tx_level_valid,
    input  wire [11:0]  rx_level,
    input  wire [3:0]   rx_level_valid,

    // ---- Interconnect (aclk domain) -- pass-through for interconnect_top ----
    output wire [1:0]       session_start_aclk,
    input  wire [1:0]       session_end_aclk,
    input  wire [1:0]       session_overflow_aclk,
    output wire [WW-1:0]    arb_weight_ch0,
    output wire [WW-1:0]    arb_weight_ch1,
    output wire [LW-1:0]    session_limit,

    // ---- Status ----
    output wire          err_xlat_miss,
    output wire          err_addr_ovf,
    output wire          stat_split_occurred,
    output wire          err_tkeep_gap,
    output wire          crc_err,
    output wire          fec_err,
    output wire          fec_corrected,
    output wire          replay_event,
    output wire          phy_tx_idle,
    output wire          phy_framing_err,
    output wire          phy_lane_err
);

    localparam ADDR_ID_W = (NUM_STREAMS <= 1) ? 1 : $clog2(NUM_STREAMS);

    // ---- Seam A: interconnect TUSER order -> link-layer TUSER order ----
    wire [SOFF_W+1:0] link_tuser;
    ic2link_tuser_remap #(.SOFF_W(SOFF_W)) u_tuser_remap (
        .ic_tuser  (ic_tuser),
        .ic_tlast  (s_axis_tlast),
        .link_tuser(link_tuser)
    );

    // ---- PHY <-> link-layer datapath wires (names already match) ----
    wire                phy_tx_valid, phy_tx_sof, phy_tx_eof, phy_tx_ready;
    wire [SYS_BUS_W-1:0] phy_tx_data;
    wire                phy_rx_valid, phy_rx_sof, phy_rx_eof;
    wire [SYS_BUS_W-1:0] phy_rx_data;

    wire                mwr_valid, mwr_ready;
    wire [63:0]         mwr_addr;
    wire [31:0]         mwr_data;
    wire [NUM_STREAMS-1:0] session_done;

    // ---- Seam B adapter: BAR0 per-channel base regs -> muxed cfg bus ----
    wire [1:0]        cfg_base_addr_wr_ch;
    wire [ADDR_W-1:0] cfg_base_addr_ch0, cfg_base_addr_ch1;
    wire              cfg_base_addr_wr;
    wire [ADDR_ID_W-1:0] cfg_base_addr_id;
    wire [ADDR_W-1:0] cfg_base_addr_data;

    cfg_base_addr_bridge #(.ADDR_W(ADDR_W)) u_cfg_base_addr_bridge (
        .clk                (clk),
        .rst_n              (datapath_rst_n),
        .cfg_base_addr_wr_ch(cfg_base_addr_wr_ch),
        .cfg_base_addr_ch0  (cfg_base_addr_ch0),
        .cfg_base_addr_ch1  (cfg_base_addr_ch1),
        .cfg_base_addr_wr   (cfg_base_addr_wr),
        .cfg_base_addr_id   (cfg_base_addr_id),
        .cfg_base_addr_data (cfg_base_addr_data)
    );

    // ---- Link layer (Swaray) ----
    stream_tx_path_top #(
        .DATA_W     (DATA_W),
        .KEEP_W     (KEEP_W),
        .TUSER_W    (SOFF_W + 2),
        .SOFF_W     (SOFF_W),
        .ADDR_W     (ADDR_W),
        .NUM_STREAMS(NUM_STREAMS),
        .SYS_BUS_W  (SYS_BUS_W)
    ) u_stream_tx_path_top (
        .clk               (clk),
        .rst_n             (datapath_rst_n),     // whole link datapath resets with the PHY

        .s_axis_tdata      (s_axis_tdata),
        .s_axis_tkeep      (s_axis_tkeep),
        .s_axis_tuser      (link_tuser),
        .s_axis_tlast      (s_axis_tlast),
        .s_axis_tvalid     (s_axis_tvalid),
        .s_axis_tready     (s_axis_tready),

        .cfg_base_addr_wr  (cfg_base_addr_wr),
        .cfg_base_addr_id  (cfg_base_addr_id),
        .cfg_base_addr_data(cfg_base_addr_data),
        .cfg_base_addr_clr (1'b0),               // no PHY-side equivalent; see cfg_base_addr_bridge.v

        .msix_req_valid    (mwr_valid),
        .msix_req_addr     (mwr_addr),
        .msix_req_data     (mwr_data),
        .msix_req_ready    (mwr_ready),
        .session_done      (session_done),

        .phy_tx_valid      (phy_tx_valid),
        .phy_tx_data       (phy_tx_data),
        .phy_tx_sof        (phy_tx_sof),
        .phy_tx_eof        (phy_tx_eof),
        .phy_tx_ready      (phy_tx_ready),

        .phy_rx_valid      (phy_rx_valid),
        .phy_rx_data       (phy_rx_data),
        .phy_rx_sof        (phy_rx_sof),
        .phy_rx_eof        (phy_rx_eof),

        .m_rx_axis_tvalid  (m_rx_axis_tvalid),
        .m_rx_axis_tdata   (m_rx_axis_tdata),
        .m_rx_axis_tkeep   (m_rx_axis_tkeep),
        .m_rx_axis_tlast   (m_rx_axis_tlast),
        .m_rx_axis_tready  (m_rx_axis_tready),

        .err_xlat_miss       (err_xlat_miss),
        .err_addr_ovf        (err_addr_ovf),
        .stat_split_occurred (stat_split_occurred),
        .err_tkeep_gap       (err_tkeep_gap),
        .crc_err             (crc_err),
        .fec_err             (fec_err),
        .fec_corrected       (fec_corrected),
        .replay_event        (replay_event)
    );

    // ---- Protocol-to-PHY layer (Kshitij) ----
    pcie_phy_top #(
        .WW(WW), .LW(LW)
    ) u_pcie_phy_top (
        .clk(clk), .perst_n(perst_n),

        .ts_byte_valid(ts_byte_valid),
        .ts_byte_l0(ts_byte_l0), .ts_byte_l1(ts_byte_l1),
        .ts_byte_l2(ts_byte_l2), .ts_byte_l3(ts_byte_l3),
        .link_trained(link_trained), .lane_reversed_locked(lane_reversed_locked),

        .hot_reset_request(hot_reset_request), .flr_request(flr_request),
        .updatefc_received(updatefc_received),
        .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n), .flr_active(flr_active),

        .cfg_wr_en(cfg_wr_en), .cfg_rd_en(cfg_rd_en), .cfg_addr(cfg_addr),
        .cfg_wdata(cfg_wdata), .cfg_rdata(cfg_rdata), .cfg_cpl_valid(cfg_cpl_valid),

        .tbl_wr_en(tbl_wr_en), .tbl_wr_vec(tbl_wr_vec), .tbl_wr_addr(tbl_wr_addr),
        .tbl_wr_data(tbl_wr_data), .tbl_wr_mask(tbl_wr_mask),
        .tbl_rd_en(tbl_rd_en), .tbl_rd_vec(tbl_rd_vec),
        .tbl_rd_addr(tbl_rd_addr), .tbl_rd_data(tbl_rd_data), .tbl_rd_mask(tbl_rd_mask),
        .pba(pba),
        .dma_complete_pulse(dma_complete_pulse), .dma_complete_vector(dma_complete_vector),

        .mwr_valid(mwr_valid), .mwr_ready(mwr_ready),
        .mwr_addr(mwr_addr), .mwr_data(mwr_data),

        .phy_tx_data(phy_tx_data), .phy_tx_valid(phy_tx_valid),
        .phy_tx_sof(phy_tx_sof), .phy_tx_eof(phy_tx_eof), .phy_tx_ready(phy_tx_ready),
        .tx_level(tx_level), .tx_level_valid(tx_level_valid),
        .rx_level(rx_level), .rx_level_valid(rx_level_valid),
        .phy_rx_data(phy_rx_data), .phy_rx_valid(phy_rx_valid),
        .phy_rx_sof(phy_rx_sof), .phy_rx_eof(phy_rx_eof),
        .phy_tx_idle(phy_tx_idle), .phy_framing_err(phy_framing_err), .phy_lane_err(phy_lane_err),

        .bar0_wr_en(bar0_wr_en), .bar0_rd_en(bar0_rd_en), .bar0_addr(bar0_addr),
        .bar0_wdata(bar0_wdata), .bar0_rdata(bar0_rdata), .bar0_rd_valid(bar0_rd_valid),

        .cfg_base_addr_ch0(cfg_base_addr_ch0), .cfg_base_addr_ch1(cfg_base_addr_ch1),
        .cfg_base_addr_wr(cfg_base_addr_wr_ch),
        .cfg_tc_ch0(), .cfg_tc_ch1(),            // TC/ATTR not wired into the stream path (see address_translation header)
        .session_valid(),
        .session_done(session_done),

        .aclk(aclk), .aclk_rst_n(aclk_rst_n),
        .session_start_aclk(session_start_aclk),
        .session_end_aclk(session_end_aclk),
        .session_overflow_aclk(session_overflow_aclk),
        .arb_weight_ch0(arb_weight_ch0), .arb_weight_ch1(arb_weight_ch1),
        .session_limit(session_limit)
    );

endmodule
