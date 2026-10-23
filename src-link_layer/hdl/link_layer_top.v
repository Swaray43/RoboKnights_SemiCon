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
    parameter MERGE_EN        = 1,      // Step 4 item 2

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
    output wire  nak_sent_evt,
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
        .FLIT_IDLE_FLUSH(FLIT_IDLE_FLUSH),
        .MERGE_EN       (MERGE_EN)
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
        .nak_sent_evt      (nak_sent_evt),
        .fc_credits_valid  (fc_credits_valid),
        .fc_init_done      (fc_init_done)
    );

    // ================================================================
    // 2. Wide RX decode (Step 4 item 8). Replaces the 1-DW unpacker +
    //    tlp_header_parser + pcie_rx_engine walk: TLPs arrive as up to 16 DWs
    //    per clock and are never back-pressured. The first 5 DWs of each TLP
    //    (header + first payload DW) are gathered, possibly across beats (a
    //    TLP can straddle two Flits), then the TLP is decoded once:
    //      * Type check (MWr, MRd/MRdLk, Msg, Cpl/CplD incl. locked (F6), Cfg);
    //        anything else is a malformed TLP (X2).
    //      * MWr -> pcie_address_translator: AXI write address or a counted
    //        translation miss (G3), BAR0 hits excluded (S13).
    //      * MWr first payload DW -> inb_mwr_* (G1, host BAR0 writes).
    //    No MRd is ever issued on this MWr-only path, so no completion can
    //    arrive; cpl_* stays idle (pcie_rx_engine keeps its unit tests).
    // ================================================================
    assign rx_wide_ready = 1'b1;             // the decode keeps up at line rate

    assign mon_rx_axis_tvalid = rx_wide_valid && rx_wide_ready;
    assign mon_rx_axis_tdata  = rx_wide_data;
    assign mon_rx_axis_tkeep  = rx_wide_keep;
    assign mon_rx_axis_tlast  = rx_wide_last;

    reg  [31:0] hb [0:4];          // DW0..DW4 of the current TLP
    reg  [2:0]  hcnt;              // DWs gathered (saturates at 5)
    reg         hdone;             // this TLP already decoded
    reg  [31:0] hn [0:4];
    reg  [2:0]  hcnt_n;
    integer     gi;

    // Wave C timing: register the beat's first 5 DWs and their count first;
    // the gather is then a 5-way pick by (hcnt, r_n), not a 16-step
    // compaction chained from the delimiter's output register (was the
    // worst path, 28 levels). Inbound decode is one clock later.
    reg          r_v, r_last;
    reg  [159:0] r_d;
    reg  [2:0]   r_n;               // min(valid DWs in the beat, 5); TKEEP is contiguous from DW 0
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            r_v <= 1'b0; r_last <= 1'b0; r_d <= 160'd0; r_n <= 3'd0;
        end else begin
            r_v    <= rx_wide_valid;
            r_last <= rx_wide_last;
            r_d    <= rx_wide_data[159:0];
            r_n    <= rx_wide_keep[16] ? 3'd5 : rx_wide_keep[12] ? 3'd4 : rx_wide_keep[8] ? 3'd3 :
                      rx_wide_keep[4]  ? 3'd2 : rx_wide_keep[0]  ? 3'd1 : 3'd0;
        end
    end

    reg  [3:0] hsum;
    always @(*) begin
        hsum   = {1'b0, hcnt} + {1'b0, r_n};
        hcnt_n = (hsum > 4'd5) ? 3'd5 : hsum[2:0];
        for (gi = 0; gi < 5; gi = gi + 1)
            hn[gi] = (gi >= hcnt && gi < hsum) ? r_d[(gi - hcnt)*32 +: 32] : hb[gi];
    end

    wire        dc_has_data = hn[0][30];
    wire        dc_4dw      = hn[0][29];
    wire [4:0]  dc_type     = hn[0][28:24];
    wire [2:0]  dc_need     = dc_has_data ? (dc_4dw ? 3'd5 : 3'd4) : (dc_4dw ? 3'd4 : 3'd3);
    wire        dc_fire     = r_v && !hdone && ((hcnt_n >= dc_need) || r_last);
    wire        dc_is_mwr   = (dc_type == 5'b00000) && dc_has_data;
    wire        dc_is_mrd   = ((dc_type == 5'b00000) || (dc_type == 5'b00001)) && !dc_has_data;
    wire        dc_is_msg   = (dc_type[4:3] == 2'b10);
    wire        dc_is_cpl   = (dc_type[4:1] == 4'b0101);                // F6: Cpl/CplD and CplLk/CplDLk
    wire        dc_is_cfg   = (dc_type == 5'b00100) || (dc_type == 5'b00101);
    wire [ADDR_W-1:0] dc_addr = dc_4dw ? {hn[2][ADDR_W-33:0], hn[3][31:2], 2'b00}
                                       : {{(ADDR_W-32){1'b0}}, hn[2][31:2], 2'b00};
    wire [31:0] dc_data     = dc_4dw ? hn[4] : hn[3];

    reg              xl_req;
    reg [ADDR_W-1:0] xl_addr;
    reg              malf_r;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            hb[0] <= 32'd0; hb[1] <= 32'd0; hb[2] <= 32'd0; hb[3] <= 32'd0; hb[4] <= 32'd0;
            hcnt <= 3'd0; hdone <= 1'b0;
            xl_req <= 1'b0; xl_addr <= {ADDR_W{1'b0}}; malf_r <= 1'b0;
            inb_mwr_valid <= 1'b0; inb_mwr_addr <= {ADDR_W{1'b0}};
            inb_mwr_data  <= 32'h0; inb_mwr_be <= 4'h0;
        end else begin
            xl_req        <= 1'b0;
            malf_r        <= 1'b0;
            inb_mwr_valid <= 1'b0;
            if (r_v) begin
                if (r_last) begin
                    hcnt  <= 3'd0;
                    hdone <= 1'b0;
                end else begin
                    hcnt  <= hcnt_n;
                    hb[0] <= hn[0]; hb[1] <= hn[1]; hb[2] <= hn[2]; hb[3] <= hn[3]; hb[4] <= hn[4];
                    if (dc_fire) hdone <= 1'b1;
                end
            end
            if (dc_fire) begin
                malf_r <= !(dc_is_mwr || dc_is_mrd || dc_is_msg || dc_is_cpl || dc_is_cfg);
                if (dc_is_mwr) begin
                    xl_req        <= 1'b1;               // request on header done (P1)
                    xl_addr       <= dc_addr;
                    inb_mwr_valid <= (hcnt_n >= dc_need);
                    inb_mwr_addr  <= dc_addr;
                    inb_mwr_data  <= dc_data;
                    inb_mwr_be    <= hn[1][3:0];
                end
            end
        end
    end

    assign malformed_tlp_err = malf_r;

    // ================================================================
    // 3. Inbound address translation (window -> AXI write address), G3 miss
    //    counter, S13 BAR0 exclusion
    // ================================================================
    pcie_address_translator #(
        .PCIE_ADDR_WIDTH(ADDR_W),
        .AXI_ADDR_WIDTH (AXI_ADDR_W)
    ) u_rx_xlat (
        .clk              (clk),
        .rst_n            (rst_n),
        .trans_window_base(cfg_window_base),
        .trans_window_mask(cfg_window_mask),
        .trans_axi_base   (cfg_axi_base),
        .excl_en          (cfg_excl_en),
        .excl_base        (cfg_excl_base),
        .excl_mask        (cfg_excl_mask),
        .req_valid        (xl_req),
        .req_pcie_addr    (xl_addr),
        .axi_addr_ready   (m_axi_awready),
        .axi_addr_valid   (m_axi_awvalid),
        .axi_addr         (m_axi_awaddr),
        .translation_err  (m_axi_awerr),
        .err_xlat_miss_cnt(err_xlat_miss_cnt),
        .err_xlat_miss_clr(err_xlat_miss_clr)
    );

    // no completion source on this MWr-only path
    assign cpl_tvalid = 1'b0;
    assign cpl_tdata  = 32'd0;
    assign cpl_tlast  = 1'b0;
    assign cpl_tkeep  = 4'h0;
    assign cpl_tuser  = 1'b0;

endmodule
