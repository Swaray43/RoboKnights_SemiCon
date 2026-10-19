`timescale 1ns/1ps

// ============================================================
// Module : pcie_core_top
// Function: Integrates the Transaction Layer TX engine,
//   Data Link Layer (reliability + framing), and the
//   Transaction Layer RX engine. Connects to the PHY SerDes.
//
// Change (bridge review, link-layer rework):
//   * data_link_layer_top's new ports are connected:
//       s_tx_axis_tkeep = one DW (4'hF) -- tx_engine sends one DW per beat,
//                         so the Flit builder packs DWs back to back
//                         instead of carrying 15 zero DWs per word
//       m_phy_tx_ready  = new top-level input phy_tx_ready (Seam C)
//       m_rx_axis_tkeep = used by the RX unpacker below
//   * The RX unpacker now stops at the last valid DW of each beat (from
//     TKEEP) instead of always emitting 16 words, so TLP ends line up
//     with rx_word_eof.
// ============================================================

module pcie_core_top #(
    parameter AXIS_DATA_W = 512,
    parameter SYS_BUS_W   = 256,
    parameter SEQ_WIDTH   = 12,
    parameter TAG_WIDTH   = 10,
    parameter PCIE_ADDR_W = 64,
    parameter AXI_ADDR_W  = 32,
    // TUSER: {tlast_flag(1), session_offset(SOFF_W=24), stream_id(1)} = 26 bits.
    parameter TUSER_W     = 26
)(
    input  wire clk,
    input  wire rst_n,

    // PHY Interface (to SerDes / System Bus)
    output wire                   phy_tx_valid,
    output wire [SYS_BUS_W-1:0]   phy_tx_data,
    output wire                   phy_tx_sof,
    output wire                   phy_tx_eof,
    input  wire                   phy_tx_ready,      // NEW: PHY back-pressure

    input  wire                   phy_rx_valid,
    input  wire [SYS_BUS_W-1:0]   phy_rx_data,
    input  wire                   phy_rx_sof,
    input  wire                   phy_rx_eof,

    // Application-level posted request interface (to TX engine)
    input  wire        p_req_valid,
    input  wire [63:0] p_req_addr,
    input  wire [9:0]  p_req_len,
    input  wire [9:0]  p_req_tag,
    input  wire [3:0]  p_req_fbe,
    input  wire [3:0]  p_req_lbe,
    input  wire [31:0] p_req_wdata,
    input  wire        p_req_wvalid,
    output wire        p_req_ready,
    output wire        p_req_wready,

    // Application-level non-posted request interface
    input  wire        np_req_valid,
    input  wire [63:0] np_req_addr,
    input  wire [9:0]  np_req_len,
    input  wire [9:0]  np_req_tag,
    input  wire [3:0]  np_req_fbe,
    input  wire [3:0]  np_req_lbe,
    output wire        np_req_ready,

    // UpdateFC interface (from DLLP / PHY side)
    input  wire        update_fc_valid,
    input  wire [1:0]  update_fc_type,
    input  wire [7:0]  update_fc_hdr_limit,
    input  wire [11:0] update_fc_data_limit,

    // Config window (for RX engine address translation)
    input  wire [PCIE_ADDR_W-1:0] cfg_window_base,
    input  wire [PCIE_ADDR_W-1:0] cfg_window_mask,
    input  wire [AXI_ADDR_W-1:0]  cfg_axi_base,

    // AXI write address channel (host->FPGA MWr)
    output wire                   m_axi_awvalid,
    output wire [AXI_ADDR_W-1:0]  m_axi_awaddr,
    output wire                   m_axi_awerr,
    input  wire                   m_axi_awready,

    // Completion data out (MRd completions back to application)
    output wire        cpl_tvalid,
    output wire [31:0] cpl_tdata,
    output wire        cpl_tlast,
    output wire [3:0]  cpl_tkeep,
    output wire        cpl_tuser,
    input  wire        cpl_tready,

    // Status
    output wire        crc_err,
    output wire        fec_err,
    output wire        malformed_tlp_err
);

    // ---- TLP stream: TX Engine -> Data Link Layer ----
    wire [31:0] tlp_tx_data;
    wire        tlp_tx_valid;
    wire        tlp_tx_last;
    wire        tlp_tx_ready;
    wire [1:0]  tlp_tx_tready_ch;

    wire        np_accept_valid;
    wire [9:0]  np_accept_tag;
    wire [9:0]  np_accept_len;

    // ---- TLP stream: Data Link Layer -> RX chain ----
    wire [AXIS_DATA_W-1:0]   tlp_rx_data_wide;
    wire [AXIS_DATA_W/8-1:0] tlp_rx_keep_wide;
    wire                     tlp_rx_valid_wide;
    wire                     tlp_rx_last_wide;
    wire                     tlp_rx_ready_wide;

    // ---- Parsed TLP signals ----
    wire        parsed_rx_valid;
    wire        parsed_rx_sof;
    wire        parsed_rx_eof;
    wire [31:0] parsed_rx_data;
    wire [4:0]  parsed_type;
    wire [9:0]  parsed_length_dw;
    wire [TAG_WIDTH-1:0] parsed_tag;
    wire [6:0]  parsed_lower_addr;
    wire [2:0]  parsed_cpl_status;
    wire [PCIE_ADDR_W-1:0] parsed_addr;

    // ============================================================
    // 1. TX Engine (Transaction Layer)
    // ============================================================
    tx_engine #(
        .MAX_PAYLOAD_SIZE_DW(4),
        .REQ_ID(16'h0200)
    ) u_tx_engine (
        .clk(clk),
        .rst_n(rst_n),

        .p_req_valid(p_req_valid),
        .p_req_ready(p_req_ready),
        .p_req_addr(p_req_addr),
        .p_req_len(p_req_len),
        .p_req_tag(p_req_tag),
        .p_req_fbe(p_req_fbe),
        .p_req_lbe(p_req_lbe),
        .p_s_tdata(p_req_wdata),
        .p_s_tvalid(p_req_wvalid),
        .p_s_tready(p_req_wready),

        .np_req_valid(np_req_valid),
        .np_req_ready(np_req_ready),
        .np_req_addr(np_req_addr),
        .np_req_len(np_req_len),
        .np_req_tag(np_req_tag),
        .np_req_fbe(np_req_fbe),
        .np_req_lbe(np_req_lbe),

        .update_fc_valid(update_fc_valid),
        .update_fc_type(update_fc_type),
        .update_fc_hdr_limit(update_fc_hdr_limit),
        .update_fc_data_limit(update_fc_data_limit),

        .m_tdata(tlp_tx_data),
        .m_tvalid(tlp_tx_valid),
        .m_tready(tlp_tx_ready),
        .m_tlast(tlp_tx_last),

        .credit_stall(/* unused at this level */),
        .np_accept_valid(np_accept_valid),
        .np_accept_tag(np_accept_tag),
        .np_accept_len(np_accept_len)
    );

    // ============================================================
    // 2. Data Link Layer (Flits, replay, ACK/NAK, CRC-64, FEC)
    // ============================================================
    data_link_layer_top #(
        .AXIS_DATA_W(AXIS_DATA_W),
        .SYS_BUS_W(SYS_BUS_W),
        .SEQ_WIDTH(SEQ_WIDTH),
        .TUSER_W(TUSER_W)
    ) u_link_layer (
        .clk(clk),
        .rst_n(rst_n),

        // One 32-bit DW per beat from tx_engine; TKEEP marks it as the
        // only valid DW so the Flit builder packs DWs contiguously.
        .s_tx_axis_tvalid(tlp_tx_valid),
        .s_tx_axis_tdata({{(AXIS_DATA_W-32){1'b0}}, tlp_tx_data}),
        .s_tx_axis_tkeep({{(AXIS_DATA_W/8-4){1'b0}}, 4'hF}),
        .s_tx_axis_tlast(tlp_tx_last),
        .s_tx_axis_tuser({TUSER_W{1'b0}}),
        .s_tx_axis_tready(tlp_tx_ready),
        .s_tx_axis_tready_ch(tlp_tx_tready_ch),

        .m_rx_axis_tvalid(tlp_rx_valid_wide),
        .m_rx_axis_tdata(tlp_rx_data_wide),
        .m_rx_axis_tkeep(tlp_rx_keep_wide),
        .m_rx_axis_tlast(tlp_rx_last_wide),
        .m_rx_axis_tready(tlp_rx_ready_wide),

        .m_phy_tx_valid(phy_tx_valid),
        .m_phy_tx_data(phy_tx_data),
        .m_phy_tx_sof(phy_tx_sof),
        .m_phy_tx_eof(phy_tx_eof),
        .m_phy_tx_ready(phy_tx_ready),

        .s_phy_rx_valid(phy_rx_valid),
        .s_phy_rx_data(phy_rx_data),
        .s_phy_rx_sof(phy_rx_sof),
        .s_phy_rx_eof(phy_rx_eof),

        .crc_err(crc_err),
        .fec_err(fec_err),
        .fec_corrected(),
        .replay_event()
    );

    // ============================================================
    // RX wide-to-word unpacker
    //   Emits the valid DWs of each beat (per TKEEP), one per clock.
    //   tlast from the link layer now marks a real TLP end.
    // ============================================================
    localparam RX_WORDS_PER_BEAT = AXIS_DATA_W / 32;
    localparam RX_WIDX_W = (RX_WORDS_PER_BEAT <= 1) ? 1 : $clog2(RX_WORDS_PER_BEAT);

    reg [AXIS_DATA_W-1:0] rx_unpack_shift;
    reg                   rx_unpack_beat_last;
    reg [RX_WIDX_W-1:0]   rx_unpack_idx;
    reg [RX_WIDX_W-1:0]   rx_unpack_last_idx;
    reg                   rx_unpack_busy;
    reg                   rx_next_is_new_tlp;

    // index of the highest valid DW in the incoming beat
    integer ki;
    reg [RX_WIDX_W-1:0] in_last_idx;
    always @(*) begin
        in_last_idx = {RX_WIDX_W{1'b0}};
        for (ki = 0; ki < RX_WORDS_PER_BEAT; ki = ki + 1)
            if (tlp_rx_keep_wide[ki*4]) in_last_idx = ki;
    end

    assign tlp_rx_ready_wide = !rx_unpack_busy;

    wire rx_word_valid = rx_unpack_busy;
    wire rx_word_sof   = rx_unpack_busy &&
                         (rx_unpack_idx == {RX_WIDX_W{1'b0}}) &&
                         rx_next_is_new_tlp;
    wire rx_word_eof   = rx_unpack_busy &&
                         (rx_unpack_idx == rx_unpack_last_idx) &&
                         rx_unpack_beat_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_unpack_shift     <= {AXIS_DATA_W{1'b0}};
            rx_unpack_beat_last <= 1'b0;
            rx_unpack_idx       <= {RX_WIDX_W{1'b0}};
            rx_unpack_last_idx  <= {RX_WIDX_W{1'b0}};
            rx_unpack_busy      <= 1'b0;
            rx_next_is_new_tlp  <= 1'b1;
        end else if (rx_unpack_busy) begin
            rx_unpack_shift <= rx_unpack_shift >> 32;
            if (rx_unpack_idx == rx_unpack_last_idx) begin
                rx_unpack_busy <= 1'b0;
                rx_unpack_idx  <= {RX_WIDX_W{1'b0}};
                rx_next_is_new_tlp <= rx_unpack_beat_last;
            end else begin
                rx_unpack_idx      <= rx_unpack_idx + 1'b1;
                rx_next_is_new_tlp <= 1'b0;
            end
        end else if (tlp_rx_valid_wide) begin
            rx_unpack_shift     <= tlp_rx_data_wide;
            rx_unpack_beat_last <= tlp_rx_last_wide;
            rx_unpack_idx       <= {RX_WIDX_W{1'b0}};
            rx_unpack_last_idx  <= in_last_idx;
            rx_unpack_busy      <= 1'b1;
        end
    end

    // ============================================================
    // 3. TLP Header Parser
    // ============================================================
    tlp_header_parser #(
        .TAG_WIDTH(TAG_WIDTH),
        .PCIE_ADDR_W(PCIE_ADDR_W)
    ) u_tlp_parser (
        .clk(clk),
        .rst_n(rst_n),

        .rx_tlp_valid(rx_word_valid),
        .rx_tlp_data(rx_unpack_shift[31:0]),
        .rx_tlp_sof(rx_word_sof),
        .rx_tlp_eof(rx_word_eof),

        .parsed_rx_valid(parsed_rx_valid),
        .parsed_rx_sof(parsed_rx_sof),
        .parsed_rx_eof(parsed_rx_eof),
        .parsed_rx_data(parsed_rx_data),
        .parsed_type(parsed_type),
        .parsed_length_dw(parsed_length_dw),
        .parsed_tag(parsed_tag),
        .parsed_lower_addr(parsed_lower_addr),
        .parsed_cpl_status(parsed_cpl_status),
        .parsed_addr(parsed_addr)
    );

    // ============================================================
    // 4. PCIe RX Engine (Transaction Layer RX)
    // ============================================================
    pcie_rx_engine #(
        .TAG_WIDTH(TAG_WIDTH),
        .MAX_DW_PER_REQ(16),
        .PCIE_ADDR_W(PCIE_ADDR_W),
        .AXI_ADDR_W(AXI_ADDR_W)
    ) u_rx_engine (
        .clk(clk),
        .rst_n(rst_n),

        .tlp_rx_valid(parsed_rx_valid),
        .tlp_rx_sof(parsed_rx_sof),
        .tlp_rx_eof(parsed_rx_eof),
        .tlp_rx_data(parsed_rx_data),

        .tlp_type(parsed_type),
        .tlp_length_dw(parsed_length_dw),
        .tlp_tag(parsed_tag),
        .tlp_lower_addr(parsed_lower_addr),
        .tlp_cpl_status(parsed_cpl_status),
        .tlp_addr(parsed_addr),

        .req_alloc_valid(np_accept_valid),
        .req_alloc_tag(np_accept_tag),
        .req_alloc_len(np_accept_len),
        .req_alloc_ready(/* unused */),

        .m_axis_cpl_tvalid(cpl_tvalid),
        .m_axis_cpl_tdata(cpl_tdata),
        .m_axis_cpl_tlast(cpl_tlast),
        .m_axis_cpl_tkeep(cpl_tkeep),
        .m_axis_cpl_tuser(cpl_tuser),
        .m_axis_cpl_tready(cpl_tready),

        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready),
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awerr(m_axi_awerr),

        .cfg_window_base(cfg_window_base),
        .cfg_window_mask(cfg_window_mask),
        .cfg_axi_base(cfg_axi_base),

        .cfg_req_valid(/* unused */),
        .cfg_req_type1(/* unused */),
        .cfg_req_tag(/* unused */),
        .cfg_req_len_dw(/* unused */),
        .cfg_req_addr(/* unused */),

        .malformed_tlp_err(malformed_tlp_err)
    );

endmodule
