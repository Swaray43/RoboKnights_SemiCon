`timescale 1ns/1ps


module pcie_rx_engine #(
    parameter TAG_WIDTH      = 10,
    parameter MAX_DW_PER_REQ = 16,
    parameter PCIE_ADDR_W    = 64,
    parameter AXI_ADDR_W     = 32
)(
    input wire clk,
    input wire rst_n,

    input wire                     tlp_rx_valid,
    input wire                     tlp_rx_sof,
    input wire                     tlp_rx_eof,
    input wire [31:0]              tlp_rx_data,
    
    input wire [4:0]               tlp_type,
    input wire [9:0]               tlp_length_dw,
    input wire [TAG_WIDTH-1:0]     tlp_tag,
    input wire [6:0]               tlp_lower_addr,
    input wire [2:0]               tlp_cpl_status,
    input wire [PCIE_ADDR_W-1:0]   tlp_addr,

    input  wire                    req_alloc_valid,
    input  wire [TAG_WIDTH-1:0]    req_alloc_tag,
    input  wire [9:0]              req_alloc_len,
    output wire                    req_alloc_ready,
    
    output wire                    m_axis_cpl_tvalid,
    output wire [31:0]             m_axis_cpl_tdata,
    output wire                    m_axis_cpl_tlast,
    output wire [3:0]              m_axis_cpl_tkeep,
    output wire                    m_axis_cpl_tuser,
    input  wire                    m_axis_cpl_tready,

    output wire                    m_axi_awvalid,
    input  wire                    m_axi_awready, // AXI4 Handshake addition
    output wire [AXI_ADDR_W-1:0]   m_axi_awaddr,
    output wire                    m_axi_awerr,
    
    input  wire [PCIE_ADDR_W-1:0]  cfg_window_base,
    input  wire [PCIE_ADDR_W-1:0]  cfg_window_mask,
    input  wire [AXI_ADDR_W-1:0]   cfg_axi_base,

    // Gap fix X1: CfgRd/CfgWr routing (extension point -- no
    // config-space block exists in the project yet to connect this
    // to). Asserted one cycle after tlp_rx_sof, once tag/addr/length
    // are valid (Cfg requests are always 3DW headers).
    output wire                    cfg_req_valid,
    output wire                    cfg_req_type1,     // 0: Type0, 1: Type1
    output wire [TAG_WIDTH-1:0]    cfg_req_tag,
    output wire [9:0]              cfg_req_len_dw,
    output wire [PCIE_ADDR_W-1:0]  cfg_req_addr,

    // Gap fix X2: malformed TLP (unrecognised Type field) detection.
    output wire                    malformed_tlp_err
);

    wire is_cpld = (tlp_type == 5'b01010);
    wire is_mwr  = (tlp_type == 5'b00000) || (tlp_type == 5'b10000);

    // Gap fix X1: Cfg Type0 (0_0100) / Type1 (0_0101) detection.
    wire is_cfg      = (tlp_type == 5'b00100) || (tlp_type == 5'b00101);
    wire cfg_type1_w = tlp_type[0];

    // Gap fix X2: any TLP whose Type isn't one of the above is
    // outside what this bridge currently recognises.
    // NOTE: is_mwr's existing 5'b00000 case is ambiguous with a plain
    // MRd (same Type per spec; the two are only pre-existing distinct
    // here via the non-standard 5'b10000 encoding) -- that ambiguity
    // predates this fix and isn't resolved here, so a genuine MRd is
    // still classified alongside MWr rather than flagged malformed.
    wire malformed_type_err = tlp_rx_valid && tlp_rx_sof &&
                               !(is_mwr || is_cpld || is_cfg);
    assign malformed_tlp_err = malformed_type_err;

    // Gap fix X1: delay cfg_req_valid to one cycle after SOF so
    // tlp_tag/tlp_addr/tlp_length_dw are valid (see header note).
    reg cfg_req_valid_r;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) cfg_req_valid_r <= 1'b0;
        else        cfg_req_valid_r <= tlp_rx_valid && tlp_rx_sof && is_cfg;
    end

    assign cfg_req_valid  = cfg_req_valid_r;
    assign cfg_req_type1  = cfg_type1_w;
    assign cfg_req_tag    = tlp_tag;
    assign cfg_req_len_dw = tlp_length_dw;
    assign cfg_req_addr   = tlp_addr;

    reg in_cpld_pkt;
    reg in_mwr_pkt;
    
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in_cpld_pkt <= 0;
            in_mwr_pkt  <= 0;
        end else begin
            // Single-cycle TLP edge case fix
            if (tlp_rx_valid && tlp_rx_sof && !tlp_rx_eof) begin
                in_cpld_pkt <= is_cpld;
                in_mwr_pkt  <= is_mwr;
            end else if (tlp_rx_valid && tlp_rx_eof) begin
                in_cpld_pkt <= 0;
                in_mwr_pkt  <= 0;
            end
        end
    end

    wire active_cpld = (tlp_rx_valid && tlp_rx_sof && is_cpld) || (tlp_rx_valid && in_cpld_pkt);
    wire [9:0] cpld_dw_offset = tlp_lower_addr[6:2];

    axi_pcie_reorder_buffer #(
        .NUM_SLOTS(8),
        .TAG_WIDTH(TAG_WIDTH),
        .MAX_DW_PER_REQ(MAX_DW_PER_REQ)
    ) reorder_buf_inst (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(req_alloc_valid),
        .req_tag(req_alloc_tag),
        .req_len_dw(req_alloc_len),
        .req_ready(req_alloc_ready),
        .rx_cpld_valid(active_cpld),
        .rx_cpld_sof(tlp_rx_valid && tlp_rx_sof && is_cpld),
        .rx_cpld_offset(cpld_dw_offset),
        .rx_cpld_tag(tlp_tag),
        .rx_cpld_data(tlp_rx_data),
        .rx_cpld_dvalid(active_cpld),
        .rx_cpld_status(tlp_cpl_status),
        .rx_cpld_last_dw(tlp_rx_valid && tlp_rx_eof),
        .m_axis_tvalid(m_axis_cpl_tvalid),
        .m_axis_tdata(m_axis_cpl_tdata),
        .m_axis_tlast(m_axis_cpl_tlast),
        .m_axis_tkeep(m_axis_cpl_tkeep),
        .m_axis_tuser(m_axis_cpl_tuser),
        .m_axis_tready(m_axis_cpl_tready)
    );

    pcie_address_translator #(
        .PCIE_ADDR_WIDTH(PCIE_ADDR_W),
        .AXI_ADDR_WIDTH(AXI_ADDR_W)
    ) addr_trans_inst (
        .clk(clk),
        .rst_n(rst_n),
        .trans_window_base(cfg_window_base),
        .trans_window_mask(cfg_window_mask),
        .trans_axi_base(cfg_axi_base),
        .req_valid(tlp_rx_valid && tlp_rx_sof && is_mwr),
        .req_pcie_addr(tlp_addr),
        .axi_addr_ready(m_axi_awready), // Connected AXI4 ready
        .axi_addr_valid(m_axi_awvalid),
        .axi_addr(m_axi_awaddr),
        .translation_err(m_axi_awerr)
    );

endmodule