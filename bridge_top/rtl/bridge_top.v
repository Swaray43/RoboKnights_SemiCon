`timescale 1ns/1ps
// ============================================================
// bridge_top -- full AXI4-Stream -> PCIe 6 stack
//
//   interconnect_top (Aditya)  --m_*-->  link_phy_bridge_top (Swaray + Kshitij)
//
// Glue this file adds (everything else is straight wires):
//   * session_end_aclk[n]  = falling edge of interconnect chN_session_active
//                            (TLAST accepted or overflow abort). interconnect_top
//                            has no end pulse of its own.
//   * aclk-domain reset    = rst_sync(aclk, axi_rst_n). axi_rst_n is produced on
//                            clk by pcie_reset_controller.
//   * interconnect pcie_rstn = datapath_rst_n (same clk domain, already synchronous)
// ============================================================
module bridge_top #(
    parameter DATA_W = 512,
    parameter KEEP_W = DATA_W/8,
    parameter SOFF_W = 24
)(
    input  wire              clk,       // pcie_clk: link + PHY + interconnect read side
    input  wire              perst_n,
    input  wire              aclk,      // fabric clock

    // ---- fabric AXI4-Stream slaves ----
    input  wire [DATA_W-1:0] s0_tdata,  input wire [KEEP_W-1:0] s0_tkeep,
    input  wire              s0_tlast,  input wire s0_tvalid, output wire s0_tready,
    input  wire [DATA_W-1:0] s1_tdata,  input wire [KEEP_W-1:0] s1_tkeep,
    input  wire              s1_tlast,  input wire s1_tvalid, output wire s1_tready,
    output wire [1:0]        session_active,
    output wire [1:0]        session_overflow,

    // ---- inbound TLPs (host -> device) ----
    output wire              m_rx_axis_tvalid,
    output wire [DATA_W-1:0] m_rx_axis_tdata,
    output wire [KEEP_W-1:0] m_rx_axis_tkeep,
    output wire              m_rx_axis_tlast,
    input  wire              m_rx_axis_tready,

    // ---- training / reset sideband ----
    input  wire              ts_byte_valid,
    input  wire [7:0]        ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3,
    output wire              link_trained,
    input  wire              hot_reset_request, flr_request, updatefc_received,
    output wire              datapath_rst_n, axi_rst_n,

    // ---- host register access (until an inbound MWr/MRd decoder exists) ----
    input  wire              bar0_wr_en, bar0_rd_en,
    input  wire [11:0]       bar0_addr,
    input  wire [31:0]       bar0_wdata,
    output wire [31:0]       bar0_rdata,
    output wire              bar0_rd_valid,
    input  wire              tbl_wr_en,
    input  wire [2:0]        tbl_wr_vec,
    input  wire [63:0]       tbl_wr_addr,
    input  wire [31:0]       tbl_wr_data,
    input  wire              tbl_wr_mask,

    // ---- SerDes symbol bus ----
    output wire [11:0]       tx_level,
    output wire [3:0]        tx_level_valid,
    input  wire [11:0]       rx_level,
    input  wire [3:0]        rx_level_valid,

    // ---- status ----
    output wire              err_xlat_miss, err_addr_ovf, crc_err, fec_err,
    output wire              phy_framing_err, phy_lane_err
);
    // ---------------- aclk reset ----------------
    wire arstn;
    rst_sync u_arst (.clk(aclk), .async_rst_n(axi_rst_n), .sync_rst_n(arstn));

    // ---------------- interconnect <-> link wires ----------------
    wire [DATA_W-1:0] ic_tdata;
    wire [KEEP_W-1:0] ic_tkeep;
    wire [SOFF_W:0]   ic_tuser;
    wire              ic_tlast, ic_tvalid, ic_tready;

    wire [1:0]  session_start_aclk;
    wire [7:0]  w0, w1;
    wire [23:0] lim;

    interconnect_top u_ic (
        .aclk(aclk), .arstn(arstn),
        .s0_tdata(s0_tdata), .s0_tkeep(s0_tkeep), .s0_tlast(s0_tlast), .s0_tvalid(s0_tvalid), .s0_tready(s0_tready),
        .ch0_session_start(session_start_aclk[0]), .ch0_session_limit(lim),
        .ch0_session_active(session_active[0]), .ch0_session_overflow(session_overflow[0]),
        .s1_tdata(s1_tdata), .s1_tkeep(s1_tkeep), .s1_tlast(s1_tlast), .s1_tvalid(s1_tvalid), .s1_tready(s1_tready),
        .ch1_session_start(session_start_aclk[1]), .ch1_session_limit(lim),
        .ch1_session_active(session_active[1]), .ch1_session_overflow(session_overflow[1]),
        .weight0(w0), .weight1(w1),
        .pcie_clk(clk), .pcie_rstn(datapath_rst_n),
        .m_tdata(ic_tdata), .m_tkeep(ic_tkeep), .m_tuser(ic_tuser), .m_tlast(ic_tlast),
        .m_tvalid(ic_tvalid), .m_tready(ic_tready)
    );

    // session end = falling edge of session_active (aclk)
    reg [1:0] active_d;
    always @(posedge aclk or negedge arstn)
        if (!arstn) active_d <= 2'b00; else active_d <= session_active;
    wire [1:0] session_end_aclk = active_d & ~session_active;

    link_phy_bridge_top u_lpb (
        .clk(clk), .perst_n(perst_n), .aclk(aclk), .aclk_rst_n(arstn),
        .s_axis_tdata(ic_tdata), .s_axis_tkeep(ic_tkeep), .ic_tuser(ic_tuser),
        .s_axis_tlast(ic_tlast), .s_axis_tvalid(ic_tvalid), .s_axis_tready(ic_tready),
        .m_rx_axis_tvalid(m_rx_axis_tvalid), .m_rx_axis_tdata(m_rx_axis_tdata),
        .m_rx_axis_tkeep(m_rx_axis_tkeep), .m_rx_axis_tlast(m_rx_axis_tlast),
        .m_rx_axis_tready(m_rx_axis_tready),
        .ts_byte_valid(ts_byte_valid), .ts_byte_l0(ts_byte_l0), .ts_byte_l1(ts_byte_l1),
        .ts_byte_l2(ts_byte_l2), .ts_byte_l3(ts_byte_l3),
        .link_trained(link_trained), .lane_reversed_locked(),
        .hot_reset_request(hot_reset_request), .flr_request(flr_request),
        .updatefc_received(updatefc_received),
        .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n), .flr_active(),
        .cfg_wr_en(1'b0), .cfg_rd_en(1'b0), .cfg_addr(12'd0), .cfg_wdata(32'd0),
        .cfg_rdata(), .cfg_cpl_valid(),
        .tbl_wr_en(tbl_wr_en), .tbl_wr_vec(tbl_wr_vec), .tbl_wr_addr(tbl_wr_addr),
        .tbl_wr_data(tbl_wr_data), .tbl_wr_mask(tbl_wr_mask),
        .tbl_rd_en(1'b0), .tbl_rd_vec(3'd0), .tbl_rd_addr(), .tbl_rd_data(), .tbl_rd_mask(), .pba(),
        .dma_complete_pulse(1'b0), .dma_complete_vector(3'd0),
        .bar0_wr_en(bar0_wr_en), .bar0_rd_en(bar0_rd_en), .bar0_addr(bar0_addr),
        .bar0_wdata(bar0_wdata), .bar0_rdata(bar0_rdata), .bar0_rd_valid(bar0_rd_valid),
        .tx_level(tx_level), .tx_level_valid(tx_level_valid),
        .rx_level(rx_level), .rx_level_valid(rx_level_valid),
        .session_start_aclk(session_start_aclk),
        .session_end_aclk(session_end_aclk),
        .session_overflow_aclk(session_overflow),
        .arb_weight_ch0(w0), .arb_weight_ch1(w1), .session_limit(lim),
        .err_xlat_miss(err_xlat_miss), .err_addr_ovf(err_addr_ovf),
        .stat_split_occurred(), .err_tkeep_gap(),
        .crc_err(crc_err), .fec_err(fec_err), .fec_corrected(), .replay_event(),
        .phy_tx_idle(), .phy_framing_err(phy_framing_err), .phy_lane_err(phy_lane_err)
    );
endmodule
