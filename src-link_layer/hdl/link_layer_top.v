`timescale 1ns/1ps

// ============================================================
// Module : link_layer_top
// Layer  : Link layer (Swaray) -- single top-level module
//
// Wraps stream_tx_path_top (TX MWr chain + Flit-mode DLL: replay, CRC-64,
// FEC, ACK/NAK) and adds the RX decode: unpack the wide inbound TLP stream
// to 32-bit DWs, parse it with tlp_header_parser, and run it through
// pcie_rx_engine (reorder buffer + address translator inside) to produce
// AXI BAR0 writes (m_axi_aw*). G1: also reports every inbound MWr's first
// data DW on inb_mwr_* (raw address + data + BE) for the BAR0 register path.
//
// This module issues no non-posted (MRd) requests of its own -- the stream
// path is MWr-only -- so pcie_rx_engine's tag-allocation input is tied off
// and its completion stream is exposed but never fires.
//
// RX unpacker note
//   Sizes each beat from its own TKEEP (popcount/4), because rx_tlp_delimiter
//   emits real byte enables on the final beat of a TLP.
//   Throughput: one DW per clk (a full 512-bit beat drains in 16 clk). When
//   inbound traffic is faster than that, the DLL's RX FIFO fills and the
//   link NAKs -> replay. Fine for host->device register writes; a Phase 2
//   item if inbound bulk traffic is ever needed (widen the parser).
//
// mon_rx_axis_* : observe-only copy of each inbound beat the decode accepts
//   (used by bridge_top's testbenches as the TLP checker tap).
// ============================================================

module link_layer_top #(
    // -- stream_tx_path_top / DLL parameters (passed straight through) --
    parameter DATA_W          = 512,
    parameter KEEP_W          = DATA_W / 8,
    parameter TUSER_W         = 26,
    parameter SOFF_W          = 24,
    parameter ADDR_W          = 64,     // PCIe host address width
    parameter LEN_W           = 11,
    parameter NUM_STREAMS     = 2,
    parameter MAX_PAYLOAD_DW  = 64,
    parameter [15:0] REQ_ID   = 16'h0200,
    parameter FIFO_D          = 16,
    parameter HWM             = 12,
    parameter SYS_BUS_W       = 256,
    parameter SEQ_WIDTH       = 12,
    parameter AXIS_DATA_W     = 512,
    parameter REPLAY_DEPTH    = 32,
    parameter REPLAY_TIMEOUT  = 16384,
    parameter FLIT_IDLE_FLUSH = 8,

    // -- RX decode parameters (pcie_rx_engine / tlp_header_parser) --
    parameter TAG_WIDTH      = 10,
    parameter MAX_DW_PER_REQ = 16,
    parameter AXI_ADDR_W     = 32
)(
    input  wire clk,
    input  wire rst_n,

    // ---- Upstream AXI4-Stream (from the interconnect) ----
    input  wire [DATA_W-1:0]   s_axis_tdata,
    input  wire [KEEP_W-1:0]   s_axis_tkeep,
    input  wire [TUSER_W-1:0]  s_axis_tuser,   // {tlast_flag, session_offset, stream_id}
    input  wire                s_axis_tlast,
    input  wire                s_axis_tvalid,
    output wire                s_axis_tready,

    // ---- Config-space: per-stream DMA base address ----
    input  wire                            cfg_base_addr_wr,
    input  wire [$clog2(NUM_STREAMS)-1:0]  cfg_base_addr_id,
    input  wire [ADDR_W-1:0]               cfg_base_addr_data,
    input  wire                            cfg_base_addr_clr,

    // ---- Config-space: inbound MWr -> AXI BAR0 window ----
    input  wire [ADDR_W-1:0]     cfg_window_base,
    input  wire [ADDR_W-1:0]     cfg_window_mask,
    input  wire [AXI_ADDR_W-1:0] cfg_axi_base,
    // S13: BAR0 window (inbound MWrs here are register writes, not misses)
    input  wire                  cfg_excl_en,
    input  wire [ADDR_W-1:0]     cfg_excl_base,
    input  wire [ADDR_W-1:0]     cfg_excl_mask,

    // ---- MSI-X MWr injection ----
    input  wire                    msix_req_valid,
    input  wire [ADDR_W-1:0]       msix_req_addr,
    input  wire [31:0]             msix_req_data,
    output wire                    msix_req_ready,
    output wire [NUM_STREAMS-1:0]  session_done,

    // ---- PHY TX ----
    output wire                   phy_tx_valid,
    output wire [SYS_BUS_W-1:0]   phy_tx_data,
    output wire                   phy_tx_sof,
    output wire                   phy_tx_eof,
    input  wire                   phy_tx_ready,

    // ---- PHY RX ----
    input  wire                   phy_rx_valid,
    input  wire [SYS_BUS_W-1:0]   phy_rx_data,
    input  wire                   phy_rx_sof,
    input  wire                   phy_rx_eof,

    // ---- Decoded inbound MWr -> AXI write address channel (host -> BAR0) ----
    output wire                    m_axi_awvalid,
    output wire [AXI_ADDR_W-1:0]   m_axi_awaddr,
    output wire                    m_axi_awerr,
    input  wire                    m_axi_awready,

    // ---- Inbound completions (unused on this path -- see header note) ----
    output wire        cpl_tvalid,
    output wire [31:0] cpl_tdata,
    output wire        cpl_tlast,
    output wire [3:0]  cpl_tkeep,
    output wire        cpl_tuser,
    input  wire        cpl_tready,

    // ---- Status ----
    output wire  err_xlat_miss,
    output wire  err_addr_ovf,
    output wire  stat_split_occurred,
    output wire  err_tkeep_gap,
    output wire  crc_err,
    output wire  fec_err,
    output wire  fec_corrected,
    output wire  replay_event,
    output wire  malformed_tlp_err,

    // ---- G2: flow-control status ----
    output wire  fc_credits_valid,   // far-end InitFC recorded -> "UpdateFC received"
    output wire  fc_init_done,

    // ---- G3: inbound address-translation miss count + clear ----
    output wire [7:0] err_xlat_miss_cnt,
    input  wire        err_xlat_miss_clr,

    // ---- G1 (P2-1): decoded inbound MWr, first data DW (host -> BAR0) ----
    // One-cycle pulse per inbound Memory Write (Fmt[1]=1, Type=0_0000) when
    // its first payload DW arrives. Raw PCIe address (DW-aligned) + first DW
    // + first-DW byte enables. No window/BAR decode here -- the BAR hit check
    // lives in link_phy_bridge_top against the config-space BAR0. Only the
    // first DW of a multi-DW MWr is reported.
    output reg                     inb_mwr_valid,
    output reg  [ADDR_W-1:0]       inb_mwr_addr,
    output reg  [31:0]             inb_mwr_data,
    output reg  [3:0]              inb_mwr_be,

    // ---- Monitor tap: inbound TLP byte stream, one pulse per beat the RX
    //      decode accepts (observe-only, no backpressure) ----
    output wire                     mon_rx_axis_tvalid,
    output wire [AXIS_DATA_W-1:0]   mon_rx_axis_tdata,
    output wire [AXIS_DATA_W/8-1:0] mon_rx_axis_tkeep,
    output wire                     mon_rx_axis_tlast
);

    // ================================================================
    // 1. stream_tx_path_top -- TX (MWr) chain + Flit-mode DLL
    // ================================================================
    wire                     rx_wide_valid;
    wire [AXIS_DATA_W-1:0]   rx_wide_data;
    wire [AXIS_DATA_W/8-1:0] rx_wide_keep;
    wire                     rx_wide_last;
    wire                     rx_wide_ready;

    stream_tx_path_top #(
        .DATA_W         (DATA_W),
        .KEEP_W         (KEEP_W),
        .TUSER_W        (TUSER_W),
        .SOFF_W         (SOFF_W),
        .ADDR_W         (ADDR_W),
        .LEN_W          (LEN_W),
        .NUM_STREAMS    (NUM_STREAMS),
        .MAX_PAYLOAD_DW (MAX_PAYLOAD_DW),
        .REQ_ID         (REQ_ID),
        .FIFO_D         (FIFO_D),
        .HWM            (HWM),
        .SYS_BUS_W      (SYS_BUS_W),
        .SEQ_WIDTH      (SEQ_WIDTH),
        .AXIS_DATA_W    (AXIS_DATA_W),
        .REPLAY_DEPTH   (REPLAY_DEPTH),
        .REPLAY_TIMEOUT (REPLAY_TIMEOUT),
        .FLIT_IDLE_FLUSH(FLIT_IDLE_FLUSH)
    ) u_stream_tx_path_top (
        .clk               (clk),
        .rst_n             (rst_n),

        .s_axis_tdata      (s_axis_tdata),
        .s_axis_tkeep      (s_axis_tkeep),
        .s_axis_tuser      (s_axis_tuser),
        .s_axis_tlast      (s_axis_tlast),
        .s_axis_tvalid     (s_axis_tvalid),
        .s_axis_tready     (s_axis_tready),

        .cfg_base_addr_wr  (cfg_base_addr_wr),
        .cfg_base_addr_id  (cfg_base_addr_id),
        .cfg_base_addr_data(cfg_base_addr_data),
        .cfg_base_addr_clr (cfg_base_addr_clr),

        .msix_req_valid    (msix_req_valid),
        .msix_req_addr     (msix_req_addr),
        .msix_req_data     (msix_req_data),
        .msix_req_ready    (msix_req_ready),
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

        .m_rx_axis_tvalid  (rx_wide_valid),
        .m_rx_axis_tdata   (rx_wide_data),
        .m_rx_axis_tkeep   (rx_wide_keep),
        .m_rx_axis_tlast   (rx_wide_last),
        .m_rx_axis_tready  (rx_wide_ready),

        .err_xlat_miss     (err_xlat_miss),
        .err_addr_ovf      (err_addr_ovf),
        .stat_split_occurred(stat_split_occurred),
        .err_tkeep_gap     (err_tkeep_gap),
        .crc_err           (crc_err),
        .fec_err           (fec_err),
        .fec_corrected     (fec_corrected),
        .replay_event      (replay_event),
        .fc_credits_valid  (fc_credits_valid),
        .fc_init_done      (fc_init_done)
    );

    // ================================================================
    // 2. RX wide-to-32bit unpacker, sized from TKEEP (not a fixed 16 DW)
    // ================================================================
    localparam RX_WORDS_PER_BEAT = AXIS_DATA_W / 32;
    localparam RX_WIDX_W = $clog2(RX_WORDS_PER_BEAT) + 1;

    function automatic [RX_WIDX_W-1:0] dw_count_from_keep;
        input [AXIS_DATA_W/8-1:0] keep;
        integer i;
        reg [RX_WIDX_W+1:0] byte_cnt;
        begin
            byte_cnt = 0;
            for (i = 0; i < AXIS_DATA_W/8; i = i + 1)
                byte_cnt = byte_cnt + keep[i];
            dw_count_from_keep = byte_cnt[RX_WIDX_W+1:2] +
                                  (|byte_cnt[1:0] ? 1'b1 : 1'b0);
        end
    endfunction

    reg [AXIS_DATA_W-1:0] rx_shift;
    reg [RX_WIDX_W-1:0]   rx_dw_total;   // valid DWs in the beat being drained
    reg [RX_WIDX_W-1:0]   rx_dw_idx;     // DWs already emitted from this beat
    reg                   rx_beat_last;  // this beat is the TLP's tlast beat
    reg                   rx_busy;
    reg                   rx_next_is_new_tlp;

    assign rx_wide_ready = !rx_busy;

    assign mon_rx_axis_tvalid = rx_wide_valid && rx_wide_ready;
    assign mon_rx_axis_tdata  = rx_wide_data;
    assign mon_rx_axis_tkeep  = rx_wide_keep;
    assign mon_rx_axis_tlast  = rx_wide_last;

    wire rx_word_valid = rx_busy && (rx_dw_total != {RX_WIDX_W{1'b0}});
    wire rx_word_sof   = rx_word_valid && (rx_dw_idx == {RX_WIDX_W{1'b0}}) &&
                         rx_next_is_new_tlp;
    wire rx_word_eof   = rx_word_valid && (rx_dw_idx == rx_dw_total - 1'b1) &&
                         rx_beat_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_shift            <= {AXIS_DATA_W{1'b0}};
            rx_dw_total         <= {RX_WIDX_W{1'b0}};
            rx_dw_idx           <= {RX_WIDX_W{1'b0}};
            rx_beat_last        <= 1'b0;
            rx_busy             <= 1'b0;
            rx_next_is_new_tlp  <= 1'b1;
        end else if (rx_busy) begin
            if (rx_dw_total == {RX_WIDX_W{1'b0}}) begin
                rx_busy <= 1'b0;
                if (rx_beat_last) rx_next_is_new_tlp <= 1'b1;
            end else begin
                rx_shift <= rx_shift >> 32;
                if (rx_dw_idx == rx_dw_total - 1'b1) begin
                    rx_busy   <= 1'b0;
                    rx_dw_idx <= {RX_WIDX_W{1'b0}};
                    if (rx_beat_last) rx_next_is_new_tlp <= 1'b1;
                end else begin
                    rx_dw_idx          <= rx_dw_idx + 1'b1;
                    rx_next_is_new_tlp <= 1'b0;
                end
            end
        end else if (rx_wide_valid) begin
            rx_shift     <= rx_wide_data;
            rx_dw_total  <= dw_count_from_keep(rx_wide_keep);
            rx_dw_idx    <= {RX_WIDX_W{1'b0}};
            rx_beat_last <= rx_wide_last;
            rx_busy      <= 1'b1;
        end
    end

    // ================================================================
    // 2b. G1: inbound MWr first-DW tracker (BAR0 write source)
    //     Runs on the unpacked DW stream (rx_word_*), independent of
    //     tlp_header_parser / pcie_address_translator timing.
    //       DW0: Fmt/Type -> is it an MWr (with data)? 3DW or 4DW?
    //       DW1: first-DW byte enables
    //       DW2/DW3: address
    //       DW at index hdr_len: first payload DW -> pulse inb_mwr_*
    // ================================================================
    reg  [2:0]        ib_idx;        // DW index inside the TLP, saturates at 7
    reg               ib_is_mwr;
    reg               ib_4dw;
    reg               ib_done;       // first payload DW already reported
    reg  [3:0]        ib_first_be;
    reg  [ADDR_W-1:0] ib_addr;
    wire [2:0]        ib_hdr_len = ib_4dw ? 3'd4 : 3'd3;
    wire [31:0]       ib_dw      = rx_shift[31:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ib_idx        <= 3'd0;
            ib_is_mwr     <= 1'b0;
            ib_4dw        <= 1'b0;
            ib_done       <= 1'b0;
            ib_first_be   <= 4'h0;
            ib_addr       <= {ADDR_W{1'b0}};
            inb_mwr_valid <= 1'b0;
            inb_mwr_addr  <= {ADDR_W{1'b0}};
            inb_mwr_data  <= 32'h0;
            inb_mwr_be    <= 4'h0;
        end else begin
            inb_mwr_valid <= 1'b0;
            if (rx_word_valid) begin
                if (rx_word_sof) begin                          // DW0
                    ib_idx    <= 3'd1;
                    ib_is_mwr <= ib_dw[30] && (ib_dw[28:24] == 5'b00000);
                    ib_4dw    <= ib_dw[29];
                    ib_done   <= 1'b0;
                    ib_addr   <= {ADDR_W{1'b0}};
                end else begin
                    if (ib_idx != 3'd7) ib_idx <= ib_idx + 3'd1;
                    if (ib_idx == 3'd1)
                        ib_first_be <= ib_dw[3:0];
                    if (ib_idx == 3'd2) begin
                        if (ib_4dw) ib_addr[ADDR_W-1:32] <= ib_dw[ADDR_W-33:0];
                        else        ib_addr <= {{(ADDR_W-32){1'b0}}, ib_dw[31:2], 2'b00};
                    end
                    if (ib_idx == 3'd3 && ib_4dw)
                        ib_addr[31:0] <= {ib_dw[31:2], 2'b00};
                    if (ib_is_mwr && !ib_done && ib_idx == ib_hdr_len) begin
                        inb_mwr_valid <= 1'b1;
                        inb_mwr_addr  <= ib_addr;
                        inb_mwr_data  <= ib_dw;
                        inb_mwr_be    <= ib_first_be;
                        ib_done       <= 1'b1;
                    end
                end
            end
        end
    end

    // ================================================================
    // 3. TLP header parser
    // ================================================================
    wire        parsed_rx_valid, parsed_rx_sof, parsed_rx_eof;
    wire [31:0] parsed_rx_data;
    wire [4:0]  parsed_type;
    wire [9:0]  parsed_length_dw;
    wire [TAG_WIDTH-1:0] parsed_tag;
    wire [6:0]  parsed_lower_addr;
    wire [2:0]  parsed_cpl_status;
    wire [ADDR_W-1:0] parsed_addr;
    wire        parsed_hdr_done;
    wire        parsed_has_data;   // G4

    tlp_header_parser #(
        .TAG_WIDTH  (TAG_WIDTH),
        .PCIE_ADDR_W(ADDR_W)
    ) u_tlp_header_parser (
        .clk              (clk),
        .rst_n            (rst_n),
        .rx_tlp_valid     (rx_word_valid),
        .rx_tlp_data      (rx_shift[31:0]),
        .rx_tlp_sof       (rx_word_sof),
        .rx_tlp_eof       (rx_word_eof),
        .parsed_rx_valid  (parsed_rx_valid),
        .parsed_rx_sof    (parsed_rx_sof),
        .parsed_rx_eof    (parsed_rx_eof),
        .parsed_rx_data   (parsed_rx_data),
        .parsed_type      (parsed_type),
        .parsed_length_dw (parsed_length_dw),
        .parsed_tag       (parsed_tag),
        .parsed_lower_addr(parsed_lower_addr),
        .parsed_cpl_status(parsed_cpl_status),
        .parsed_addr      (parsed_addr),
        .parsed_hdr_done  (parsed_hdr_done),
        .parsed_has_data  (parsed_has_data)
    );

    // ================================================================
    // 4. pcie_rx_engine -- MWr decode to AXI BAR0
    // ================================================================
    pcie_rx_engine #(
        .TAG_WIDTH     (TAG_WIDTH),
        .MAX_DW_PER_REQ(MAX_DW_PER_REQ),
        .PCIE_ADDR_W   (ADDR_W),
        .AXI_ADDR_W    (AXI_ADDR_W),
        .REQ_ON_HDR_DONE(1)
    ) u_pcie_rx_engine (
        .clk               (clk),
        .rst_n             (rst_n),

        .tlp_rx_valid      (parsed_rx_valid),
        .tlp_rx_sof        (parsed_rx_sof),
        .tlp_rx_eof        (parsed_rx_eof),
        .tlp_rx_data       (parsed_rx_data),

        .tlp_type          (parsed_type),
        .tlp_length_dw     (parsed_length_dw),
        .tlp_tag           (parsed_tag),
        .tlp_lower_addr    (parsed_lower_addr),
        .tlp_cpl_status    (parsed_cpl_status),
        .tlp_addr          (parsed_addr),
        .tlp_hdr_done      (parsed_hdr_done),
        .tlp_has_data      (parsed_has_data),

        // No MRd source on this (MWr-only) path
        .req_alloc_valid   (1'b0),
        .req_alloc_tag     ({TAG_WIDTH{1'b0}}),
        .req_alloc_len     (10'd0),
        .req_alloc_ready   (),

        .m_axis_cpl_tvalid (cpl_tvalid),
        .m_axis_cpl_tdata  (cpl_tdata),
        .m_axis_cpl_tlast  (cpl_tlast),
        .m_axis_cpl_tkeep  (cpl_tkeep),
        .m_axis_cpl_tuser  (cpl_tuser),
        .m_axis_cpl_tready (cpl_tready),

        .m_axi_awvalid     (m_axi_awvalid),
        .m_axi_awready     (m_axi_awready),
        .m_axi_awaddr      (m_axi_awaddr),
        .m_axi_awerr       (m_axi_awerr),

        .cfg_window_base   (cfg_window_base),
        .cfg_window_mask   (cfg_window_mask),
        .cfg_axi_base      (cfg_axi_base),
        .cfg_excl_en       (cfg_excl_en),          // S13
        .cfg_excl_base     (cfg_excl_base),
        .cfg_excl_mask     (cfg_excl_mask),

        .cfg_req_valid     (),
        .cfg_req_type1     (),
        .cfg_req_tag       (),
        .cfg_req_len_dw    (),
        .cfg_req_addr      (),

        .malformed_tlp_err (malformed_tlp_err),

        .err_xlat_miss_cnt (err_xlat_miss_cnt),
        .err_xlat_miss_clr (err_xlat_miss_clr)
    );

`ifdef SIMULATION
    always @(posedge clk) begin
        if (rx_word_sof && !rx_word_valid)
            $error("[link_layer_top] RX sof without valid");
        if (rx_word_eof && !rx_word_valid)
            $error("[link_layer_top] RX eof without valid");
    end
`endif

endmodule
