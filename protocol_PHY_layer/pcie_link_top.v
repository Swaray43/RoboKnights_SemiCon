// =============================================================================
// pcie_link_top
//
// Purpose: a plain top-level wrapper with exposed ports, so cocotb (a Python
// driver, not another Verilog testbench) can stimulate the reset controller,
// TS1/TS2 training chain, config space, and MSI-X controller directly.
//
// Scope for this first cocotb pass: reset + training + config space + MSI-X.
// The PHY datapath (striper/deskew/lane-pipes/destriper) is NOT in this top
// yet - it's already verified in tb_end_to_end_datapath and
// tb_full_system_integration, and folding it in here is a straightforward
// next pass once this skeleton is confirmed working, not done now to keep
// this first pass reviewable.
// =============================================================================

module pcie_link_top (
    input  wire        clk,
    input  wire        perst_n,

    // ---- TS1/TS2 training stimulus, 4 lanes ----
    input  wire         ts_byte_valid,
    input  wire [7:0]   ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3,
    output wire         link_trained,
    output wire         lane_reversed_locked,

    // ---- Reset controller inputs/outputs ----
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
    input  wire                       tbl_wr_en,
    input  wire [2:0]                 tbl_wr_vec,
    input  wire [63:0]                tbl_wr_addr,
    input  wire [31:0]                tbl_wr_data,
    input  wire                       tbl_wr_mask,
    input  wire                       tbl_rd_en,
    input  wire [2:0]                 tbl_rd_vec,
    output wire [63:0]                tbl_rd_addr,
    output wire [31:0]                tbl_rd_data,
    output wire                       tbl_rd_mask,
    output wire [7:0]                 pba,
    input  wire                       dma_complete_pulse,
    input  wire [2:0]                 dma_complete_vector,
    output wire                       mwr_valid,
    output wire [63:0]                mwr_addr,
    output wire [31:0]                mwr_data
);

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

msix_controller #(.NUM_VECTORS(8)) u_msix (
  .clk(clk), .rst_n(datapath_rst_n),
  .tbl_wr_en(tbl_wr_en), .tbl_wr_vec(tbl_wr_vec), .tbl_wr_addr(tbl_wr_addr),
  .tbl_wr_data(tbl_wr_data), .tbl_wr_mask(tbl_wr_mask),
  .tbl_rd_en(tbl_rd_en), .tbl_rd_vec(tbl_rd_vec),
  .tbl_rd_addr(tbl_rd_addr), .tbl_rd_data(tbl_rd_data), .tbl_rd_mask(tbl_rd_mask),
  .pba(pba),
  .dma_complete_pulse(dma_complete_pulse), .dma_complete_vector(dma_complete_vector),
  .mwr_valid(mwr_valid), .mwr_addr(mwr_addr), .mwr_data(mwr_data)
);

endmodule
