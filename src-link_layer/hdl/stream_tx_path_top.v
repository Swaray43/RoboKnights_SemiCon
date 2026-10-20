// ============================================================
// Module : stream_tx_path_top
// Layer  : Link layer (Engineer B -- Top-level stream TX path)
//
// Function:
//   Wires the complete MWr stream TX chain:
//
//     write_data_buffer
//          |  512-bit beat + TUSER {tlast_flag, session_offset, stream_id}
//          v
//     beat source mux  <-- MSI-X request (1-DW MWr, injected in order)
//          |                         |
//          | descriptor              | 512 -> 32 data serialiser
//          v                         |
//     address_translation           |
//          v                         |
//     ar_request_splitter (4KB)     |
//          v                         v
//     tlp_packetizer  <-------------+
//          |  32-bit TLP DW stream
//          v
//     tlp32_to_axis_packer  (512-bit beats + TKEEP)
//          v
//     data_link_layer_top   (Flits, replay, ACK/NAK, CRC-64, FEC)
//          v
//     PHY (256-bit, sof/eof, ready)
//
// ---- Changes for the AXI4-PCIe6 bridge review ----------------
//
//   L1  The serialiser used to load DWS_PER_BEAT-1 and stop at 1, so it
//       emitted 15 DWs while the packetizer waited for 16. It now loads
//       the beat's real DW count (popcount(TKEEP)/4, the same number the
//       descriptor carries) and emits exactly that many. Emitting a fixed
//       16 was also wrong for partial last beats: the leftover DWs were
//       handed to the NEXT TLP.
//
//   L2  stream_descriptor_gen saw wdb_tvalid unconditionally, so address
//       translation accepted the FIFO head again and again while the
//       serialiser was busy and the FIFO never popped (3 descriptors from
//       2 beats). The descriptor side now only sees a beat in the same
//       cycle the serialiser can take its data: one beat = one handshake
//       = one descriptor + one data load.
//
//   L10 MSI-X injection: msix_req_* takes a 1-DW MWr (address + data from
//       Kshitij's msix_controller). It is inserted at the beat-source mux,
//       i.e. in front of the descriptor path, so it is strictly ordered
//       after every TLP of the beats already accepted. It uses the
//       split_flag=1 path of address_translation, which passes s_addr
//       through untouched (MSI-X addresses are absolute).
//       session_done[stream] pulses when the TLAST beat of a stream is
//       accepted; feed it to msix_controller to raise that vector.
//
//   Seam C  phy_tx_ready from the PHY now back-pressures the link layer.
//
//   RX  The receive TLP stream is now brought out (m_rx_axis_*) instead of
//       being discarded, for the BAR0 register path (review P4/I5).
//       Tie m_rx_axis_tready high if unused.
//
//   Also: if address_translation drops a descriptor (base address not
//   programmed / overflow), the matching data beat is flushed from the
//   serialiser instead of being handed to the next TLP. Beats with zero
//   valid DWs (TLAST with TKEEP=0) are consumed without making a TLP.
//
// Throughput note (review L9, not addressed here): the 32-bit packetizer
// still needs ~20 clocks per 64-byte beat. Widening it is the next step.
// ============================================================

`timescale 1ns/1ps

module stream_tx_path_top #(
    parameter DATA_W         = 512,
    parameter KEEP_W         = DATA_W / 8,
    parameter TUSER_W        = 26,
    parameter SOFF_W         = 24,
    parameter ADDR_W         = 64,
    parameter LEN_W          = 11,
    parameter NUM_STREAMS    = 2,
    parameter MAX_PAYLOAD_DW = 64,
    parameter [15:0] REQ_ID  = 16'h0200,
    parameter FIFO_D         = 16,
    parameter HWM            = 12,
    parameter SYS_BUS_W      = 256,
    parameter SEQ_WIDTH      = 12,
    parameter AXIS_DATA_W    = 512,
    parameter REPLAY_DEPTH   = 32,
    parameter REPLAY_TIMEOUT = 16384,
    parameter FLIT_IDLE_FLUSH = 8      // idle clocks before a part-filled Flit is sent
)(
    input  wire clk,
    input  wire rst_n,

    // Upstream AXI4-Stream (interconnect CDC output)
    // TUSER = {tlast_flag, session_offset[23:0], stream_id}
    // (interconnect_top emits exactly this order since Phase 2; no remap needed)
    input  wire [DATA_W-1:0]   s_axis_tdata,
    input  wire [KEEP_W-1:0]   s_axis_tkeep,
    input  wire [TUSER_W-1:0]  s_axis_tuser,
    input  wire                s_axis_tlast,
    input  wire                s_axis_tvalid,
    output wire                s_axis_tready,

    // Config-space base address registers
    input  wire                            cfg_base_addr_wr,
    input  wire [$clog2(NUM_STREAMS)-1:0]  cfg_base_addr_id,
    input  wire [ADDR_W-1:0]               cfg_base_addr_data,
    input  wire                            cfg_base_addr_clr,

    // MSI-X MWr injection (L10). Hold valid until ready.
    input  wire                msix_req_valid,
    input  wire [ADDR_W-1:0]   msix_req_addr,
    input  wire [31:0]         msix_req_data,
    output wire                msix_req_ready,
    output reg  [NUM_STREAMS-1:0] session_done,   // pulse per stream on TLAST beat

    // PHY TX
    output wire                   phy_tx_valid,
    output wire [SYS_BUS_W-1:0]   phy_tx_data,
    output wire                   phy_tx_sof,
    output wire                   phy_tx_eof,
    input  wire                   phy_tx_ready,

    // PHY RX (carries the far end's ACK/NAK and inbound TLPs)
    input  wire                   phy_rx_valid,
    input  wire [SYS_BUS_W-1:0]   phy_rx_data,
    input  wire                   phy_rx_sof,
    input  wire                   phy_rx_eof,

    // Inbound TLPs (host -> device), one TLP per tlast burst
    output wire                     m_rx_axis_tvalid,
    output wire [AXIS_DATA_W-1:0]   m_rx_axis_tdata,
    output wire [AXIS_DATA_W/8-1:0] m_rx_axis_tkeep,
    output wire                     m_rx_axis_tlast,
    input  wire                     m_rx_axis_tready,

    // Status and error outputs
    output wire  err_xlat_miss,
    output wire  err_addr_ovf,
    output wire  stat_split_occurred,
    output wire  err_tkeep_gap,
    output wire  crc_err,
    output wire  fec_err,
    output wire  fec_corrected,
    output wire  replay_event,

    // G2: flow-control status (from data_link_layer_top)
    output wire  fc_credits_valid,   // far-end InitFC recorded (FC_INIT1 done)
    output wire  fc_init_done        // FC_INIT2 done
);

    // ==================================================================
    // Wires
    // ==================================================================
    wire [DATA_W-1:0]  wdb_tdata;
    wire [KEEP_W-1:0]  wdb_tkeep;
    wire [TUSER_W-1:0] wdb_tuser;
    wire               wdb_tlast;
    wire               wdb_tvalid;
    wire               wdb_tready;

    wire [ADDR_W-1:0]  sdg_addr;
    wire [LEN_W-1:0]   sdg_len_dw;
    wire [TUSER_W-1:0] sdg_tuser;
    wire               sdg_split_flag;
    wire [1:0]         sdg_sub_index;
    wire               sdg_valid_unused;
    wire               sdg_ready_unused;

    wire [ADDR_W-1:0]  xlat_addr;
    wire [LEN_W-1:0]   xlat_len_dw;
    wire               xlat_stream_id;
    wire               xlat_split_flag;
    wire [1:0]         xlat_sub_index;
    wire               xlat_tlast_flag;
    wire               xlat_valid;
    wire               xlat_ready;
    wire               xlat_ready_from_splitter;

    wire [ADDR_W-1:0]  split_addr;
    wire [LEN_W-1:0]   split_len_dw;
    wire [TUSER_W-1:0] split_tuser;
    wire               split_split_flag;
    wire [1:0]         split_sub_index;
    wire               split_valid;
    wire               split_ready;

    wire               tlp_out_valid;
    wire [31:0]        tlp_out_data;
    wire               tlp_out_sof;
    wire               tlp_out_eof;
    wire               tlp_out_ready;

    wire               wdata_tvalid;
    wire [31:0]        wdata_tdata;
    wire               wdata_tready;

    wire                     pack_tvalid;
    wire [AXIS_DATA_W-1:0]   pack_tdata;
    wire [AXIS_DATA_W/8-1:0] pack_tkeep;
    wire                     pack_tlast;
    wire                     pack_tready;

    wire [$clog2(FIFO_D):0] fifo_occupancy;
    wire fifo_full;
    wire fifo_hwm;
    wire err_burst_interleave_unused;
    wire xlat_busy_unused;

    // ---- S6: credit_manager (Posted pool) gating ---------------------
    // Inserted between ar_request_splitter's output and tlp_packetizer's
    // request input, on the existing split_valid/split_ready handshake.
    // Every MWr sub-TLP (post-split, one TLP each) must be granted a
    // Posted header+data credit before it is allowed to fire into the
    // packetizer; the grant is what actually drives req_valid, and
    // split_ready (fed back to the splitter) is gated the same way so
    // back-pressure matches reality instead of just hiding it.
    wire        pkt_req_ready;
    wire        credit_alloc_valid = split_valid && pkt_req_ready;
    wire        credit_grant;
    wire        credit_stall_unused;
    wire        gated_req_valid    = split_valid && credit_grant;

    // G2 (P2-2): credits come from the link partner's FC DLLPs.
    // data_link_layer_top runs FC_INIT1/FC_INIT2 over control Flits and
    // forwards each type's InitFC (first value) and later UpdateFCs here.
    // 0 = infinite is decided inside credit_manager; the old one-shot
    // 0x7F/0xFFF "infinite" bootstrap is gone. Until the far end's InitFC-P
    // arrives the Posted pool grants nothing, so no MWr leaves before FC init.
    wire        fc_upd_valid;
    wire [1:0]  fc_upd_type;
    wire [7:0]  fc_upd_hdr;
    wire [11:0] fc_upd_data;

    credit_manager u_credit_manager (
        .clk                 (clk),
        .rst_n               (rst_n),
        .update_fc_valid     (fc_upd_valid),
        .update_fc_type      (fc_upd_type),
        .update_fc_hdr_limit (fc_upd_hdr),
        .update_fc_data_limit(fc_upd_data),
        .alloc_req_valid     (credit_alloc_valid),
        .alloc_req_type      (2'b00),            // Posted (MWr-only TX path)
        .alloc_req_dw_len    (split_len_dw[9:0]),
        .alloc_req_grant     (credit_grant),
        .stall               (credit_stall_unused)
    );

    // ==================================================================
    // 1. write_data_buffer
    // ==================================================================
    write_data_buffer #(
        .DATA_W  (DATA_W),
        .KEEP_W  (KEEP_W),
        .TUSER_W (TUSER_W),
        .SOFF_W  (SOFF_W),
        .FIFO_D  (FIFO_D),
        .HWM     (HWM)
    ) u_write_data_buffer (
        .clk                 (clk),
        .rst_n               (rst_n),
        .s_tdata             (s_axis_tdata),
        .s_tkeep             (s_axis_tkeep),
        .s_tvalid            (s_axis_tvalid),
        .s_tlast             (s_axis_tlast),
        .s_tuser             (s_axis_tuser),
        .s_tready            (s_axis_tready),
        .m_tdata             (wdb_tdata),
        .m_tkeep             (wdb_tkeep),
        .m_tvalid            (wdb_tvalid),
        .m_tlast             (wdb_tlast),
        .m_tuser             (wdb_tuser),
        .m_tready            (wdb_tready),
        .err_tkeep_gap       (err_tkeep_gap),
        .err_burst_interleave(err_burst_interleave_unused),
        .fifo_occupancy      (fifo_occupancy),
        .fifo_full           (fifo_full),
        .fifo_hwm            (fifo_hwm)
    );

    // ==================================================================
    // 2. stream_descriptor_gen -- used purely for its combinational
    //    TKEEP -> len_dw conversion. Valid/ready are handled by the
    //    beat-source mux below (L2).
    // ==================================================================
    stream_descriptor_gen #(
        .DATA_W  (DATA_W),
        .KEEP_W  (KEEP_W),
        .TUSER_W (TUSER_W),
        .LEN_W   (LEN_W),
        .ADDR_W  (ADDR_W)
    ) u_stream_descriptor_gen (
        .clk         (clk),
        .rst_n       (rst_n),
        .s_tdata     (wdb_tdata),
        .s_tkeep     (wdb_tkeep),
        .s_tuser     (wdb_tuser),
        .s_tlast     (wdb_tlast),
        .s_tvalid    (wdb_tvalid),
        .s_tready    (sdg_ready_unused),
        .m_addr      (sdg_addr),
        .m_len_dw    (sdg_len_dw),
        .m_tuser     (sdg_tuser),
        .m_split_flag(sdg_split_flag),
        .m_sub_index (sdg_sub_index),
        .m_valid     (sdg_valid_unused),
        .m_ready     (xlat_ready)
    );

    // ==================================================================
    // 3. Beat-source mux: WDB beat or MSI-X request (L2, L10)
    //
    //    One handshake ("src_fire") does both halves at once:
    //      - the descriptor goes into address_translation, and
    //      - the data goes into the 512->32 serialiser.
    //    It can only happen when the serialiser is empty AND address
    //    translation is ready, so the two halves can never drift.
    // ==================================================================
    localparam DWS_PER_BEAT = DATA_W / 32;
    localparam DW_CNT_W     = $clog2(DWS_PER_BEAT) + 1;

    reg                 msix_pend;
    reg  [ADDR_W-1:0]   msix_addr_r;
    reg  [31:0]         msix_data_r;

    reg  [DATA_W-1:0]   wdata_shift_reg;
    reg  [DW_CNT_W-1:0] wdata_dw_cnt;       // DWs still to emit
    reg                 wdata_ser_busy;

    assign msix_req_ready = !msix_pend;

    wire                beat_zero_len = (sdg_len_dw == {LEN_W{1'b0}});
    wire                src_is_msix   = msix_pend;
    wire                src_valid     = msix_pend || wdb_tvalid;
    wire                src_zero_len  = !src_is_msix && beat_zero_len;

    // descriptor presented to address_translation
    wire [ADDR_W-1:0]   src_addr       = src_is_msix ? msix_addr_r : sdg_addr;
    wire [LEN_W-1:0]    src_len_dw     = src_is_msix ? {{(LEN_W-1){1'b0}}, 1'b1} : sdg_len_dw;
    wire [TUSER_W-1:0]  src_tuser      = src_is_msix ? {TUSER_W{1'b0}} : sdg_tuser;
    wire                src_split_flag = src_is_msix ? 1'b1 : sdg_split_flag;   // 1 = use s_addr as-is
    wire [1:0]          src_sub_index  = src_is_msix ? 2'd0 : sdg_sub_index;
    wire [DATA_W-1:0]   src_data       = src_is_msix ? {{(DATA_W-32){1'b0}}, msix_data_r} : wdb_tdata;

    wire xlat_s_valid = src_valid && !wdata_ser_busy && !src_zero_len;
    wire src_fire     = src_valid && !wdata_ser_busy && (src_zero_len || xlat_ready);

    assign wdb_tready = src_fire && !src_is_msix;

    // ==================================================================
    // 4. address_translation
    // ==================================================================
    address_translation #(
        .ADDR_W      (ADDR_W),
        .LEN_W       (LEN_W),
        .TUSER_W     (TUSER_W),
        .SOFF_W      (SOFF_W),
        .NUM_STREAMS (NUM_STREAMS)
    ) u_address_translation (
        .clk               (clk),
        .rst_n             (rst_n),
        .s_addr            (src_addr),
        .s_len_dw          (src_len_dw),
        .s_tuser           (src_tuser),
        .s_split_flag      (src_split_flag),
        .s_sub_index       (src_sub_index),
        .s_valid           (xlat_s_valid),
        .s_ready           (xlat_ready),
        .m_pcie_addr       (xlat_addr),
        .m_len_dw          (xlat_len_dw),
        .m_stream_id       (xlat_stream_id),
        .m_split_flag      (xlat_split_flag),
        .m_sub_index       (xlat_sub_index),
        .m_tlast_flag      (xlat_tlast_flag),
        .m_valid           (xlat_valid),
        .m_ready           (xlat_ready_from_splitter),
        .cfg_base_addr_wr   (cfg_base_addr_wr),
        .cfg_base_addr_id   (cfg_base_addr_id),
        .cfg_base_addr_data (cfg_base_addr_data),
        .cfg_base_addr_clr  (cfg_base_addr_clr),
        .err_xlat_miss     (err_xlat_miss),
        .err_addr_ovf      (err_addr_ovf),
        .xlat_busy         (xlat_busy_unused)
    );

    // Downstream of translation only stream_id and tlast_flag matter.
    wire [TUSER_W-1:0] xlat_tuser_reconstructed = {
        xlat_tlast_flag,
        {(SOFF_W){1'b0}},
        xlat_stream_id
    };

    // ==================================================================
    // 5. ar_request_splitter (4KB)
    // ==================================================================
    ar_request_splitter #(
        .ADDR_W  (ADDR_W),
        .LEN_W   (LEN_W),
        .TUSER_W (TUSER_W),
        .SOFF_W  (SOFF_W)
    ) u_ar_request_splitter (
        .clk                (clk),
        .rst_n              (rst_n),
        .s_addr             (xlat_addr),
        .s_len_dw           (xlat_len_dw),
        .s_tuser            (xlat_tuser_reconstructed),
        .s_valid            (xlat_valid),
        .s_ready            (xlat_ready_from_splitter),
        .m_addr             (split_addr),
        .m_len_dw           (split_len_dw),
        .m_tuser            (split_tuser),
        .m_split_flag       (split_split_flag),
        .m_sub_index        (split_sub_index),
        .m_valid            (split_valid),
        .m_ready            (split_ready),           // gated by credit_grant below
        .stat_split_occurred(stat_split_occurred)
    );

    // Posted credit must be held before a sub-TLP is allowed to advance
    // past the splitter: back-pressure the splitter exactly when the
    // packetizer would otherwise have accepted but no credit is granted.
    assign split_ready = pkt_req_ready && credit_grant;

    // ==================================================================
    // 6. 512 -> 32 write-data serialiser (L1)
    //    Loads exactly the beat's DW count and emits that many DWs, in
    //    order. Split sub-TLPs of one beat consume them back to back.
    // ==================================================================
    wire [DW_CNT_W-1:0] src_dw_cnt = src_is_msix ? {{(DW_CNT_W-1){1'b0}}, 1'b1}
                                                 : sdg_len_dw[DW_CNT_W-1:0];

    assign wdata_tvalid = wdata_ser_busy;
    assign wdata_tdata  = wdata_shift_reg[31:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wdata_shift_reg <= {DATA_W{1'b0}};
            wdata_dw_cnt    <= {DW_CNT_W{1'b0}};
            wdata_ser_busy  <= 1'b0;
        end else if (wdata_ser_busy) begin
            if (err_xlat_miss || err_addr_ovf) begin
                // address_translation dropped this beat's descriptor
                // (the error pulses the cycle after it took it, and the
                // serialiser cannot have reloaded in between): drop the
                // data too so it isn't handed to the next TLP.
                wdata_ser_busy <= 1'b0;
                wdata_dw_cnt   <= {DW_CNT_W{1'b0}};
            end else if (wdata_tready) begin
                wdata_shift_reg <= wdata_shift_reg >> 32;
                if (wdata_dw_cnt == {{(DW_CNT_W-1){1'b0}}, 1'b1}) begin
                    wdata_ser_busy <= 1'b0;
                    wdata_dw_cnt   <= {DW_CNT_W{1'b0}};
                end else begin
                    wdata_dw_cnt <= wdata_dw_cnt - 1'b1;
                end
            end
        end else if (src_fire && !src_zero_len) begin
            wdata_shift_reg <= src_data;
            wdata_dw_cnt    <= src_dw_cnt;       // L1: real count, not 16-1
            wdata_ser_busy  <= 1'b1;
        end
    end

    // ==================================================================
    // MSI-X holding register + session_done
    // ==================================================================
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            msix_pend    <= 1'b0;
            msix_addr_r  <= {ADDR_W{1'b0}};
            msix_data_r  <= 32'd0;
            session_done <= {NUM_STREAMS{1'b0}};
        end else begin
            session_done <= {NUM_STREAMS{1'b0}};

            if (src_fire && src_is_msix)
                msix_pend <= 1'b0;
            else if (msix_req_valid && !msix_pend) begin
                msix_pend   <= 1'b1;
                msix_addr_r <= msix_req_addr;
                msix_data_r <= msix_req_data;
            end

            // TUSER[0] is the stream id (NUM_STREAMS = 2)
            if (src_fire && !src_is_msix && wdb_tlast)
                session_done[wdb_tuser[0]] <= 1'b1;
        end
    end

    // ==================================================================
    // 7. tlp_packetizer
    // ==================================================================
    tlp_packetizer #(
        .MAX_PAYLOAD_DW(MAX_PAYLOAD_DW)
    ) u_tlp_packetizer (
        .clk          (clk),
        .rst_n        (rst_n),
        .req_valid    (gated_req_valid),
        .req_is_write (1'b1),
        .req_addr     (split_addr),
        .req_length   (split_len_dw[9:0]),
        .req_rid      (REQ_ID),
        .req_tag      (8'h00),
        .req_ready    (pkt_req_ready),
        .wdata_tdata  (wdata_tdata),
        .wdata_tvalid (wdata_tvalid),
        .wdata_tready (wdata_tready),
        .tlp_out_valid(tlp_out_valid),
        .tlp_out_data (tlp_out_data),
        .tlp_out_sof  (tlp_out_sof),
        .tlp_out_eof  (tlp_out_eof),
        .tlp_out_ready(tlp_out_ready)
    );

    // ==================================================================
    // 8. tlp32_to_axis_packer (now with TKEEP)
    // ==================================================================
    tlp32_to_axis_packer #(
        .DW_WIDTH   (32),
        .AXIS_WIDTH (AXIS_DATA_W)
    ) u_tlp32_to_axis_packer (
        .clk      (clk),
        .rst_n    (rst_n),
        .s_tvalid (tlp_out_valid),
        .s_tdata  (tlp_out_data),
        .s_tlast  (tlp_out_eof),
        .s_tready (tlp_out_ready),
        .m_tvalid (pack_tvalid),
        .m_tdata  (pack_tdata),
        .m_tkeep  (pack_tkeep),
        .m_tlast  (pack_tlast),
        .m_tready (pack_tready)
    );

    // ==================================================================
    // 9. data_link_layer_top
    // ==================================================================
    data_link_layer_top #(
        .AXIS_DATA_W   (AXIS_DATA_W),
        .SYS_BUS_W     (SYS_BUS_W),
        .SEQ_WIDTH     (SEQ_WIDTH),
        .TUSER_W       (TUSER_W),
        .REPLAY_DEPTH  (REPLAY_DEPTH),
        .REPLAY_TIMEOUT(REPLAY_TIMEOUT),
        .IDLE_FLUSH    (FLIT_IDLE_FLUSH)
    ) u_data_link_layer_top (
        .clk                (clk),
        .rst_n              (rst_n),
        .s_tx_axis_tvalid   (pack_tvalid),
        .s_tx_axis_tdata    (pack_tdata),
        .s_tx_axis_tkeep    (pack_tkeep),
        .s_tx_axis_tlast    (pack_tlast),
        .s_tx_axis_tuser    ({TUSER_W{1'b0}}),
        .s_tx_axis_tready   (pack_tready),
        .s_tx_axis_tready_ch(),
        .m_rx_axis_tvalid   (m_rx_axis_tvalid),
        .m_rx_axis_tdata    (m_rx_axis_tdata),
        .m_rx_axis_tkeep    (m_rx_axis_tkeep),
        .m_rx_axis_tlast    (m_rx_axis_tlast),
        .m_rx_axis_tready   (m_rx_axis_tready),
        .m_phy_tx_valid     (phy_tx_valid),
        .m_phy_tx_data      (phy_tx_data),
        .m_phy_tx_sof       (phy_tx_sof),
        .m_phy_tx_eof       (phy_tx_eof),
        .m_phy_tx_ready     (phy_tx_ready),
        .s_phy_rx_valid     (phy_rx_valid),
        .s_phy_rx_data      (phy_rx_data),
        .s_phy_rx_sof       (phy_rx_sof),
        .s_phy_rx_eof       (phy_rx_eof),
        .crc_err            (crc_err),
        .fec_err            (fec_err),
        .fec_corrected      (fec_corrected),
        .replay_event       (replay_event),
        .fc_upd_valid       (fc_upd_valid),
        .fc_upd_type        (fc_upd_type),
        .fc_upd_hdr         (fc_upd_hdr),
        .fc_upd_data        (fc_upd_data),
        .fc_credits_valid   (fc_credits_valid),
        .fc_init_done       (fc_init_done)
    );

`ifdef SIMULATION
    always @(posedge clk) begin
        if (src_fire && !src_zero_len && wdata_ser_busy)
            $error("[stream_tx_path_top] beat accepted while serialiser busy -- descriptor/data de-sync");
    end
`endif

endmodule