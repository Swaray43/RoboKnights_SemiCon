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
    parameter FLIT_IDLE_FLUSH = 8,     // idle clocks before a part-filled Flit is sent
    parameter MERGE_EN       = 1,      // Step 4 item 2: merge contiguous beats up to MAX_PAYLOAD_DW
    parameter MERGE_IDLE     = 8       // idle clocks before an open (merging) TLP is closed
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
    output wire  nak_sent_evt,

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

    wire                     pack_tvalid;
    wire [AXIS_DATA_W-1:0]   pack_tdata;
    wire [AXIS_DATA_W/8-1:0] pack_tkeep;
    wire                     pack_tlast;
    wire                     pack_tready;

    wire [$clog2(FIFO_D):0] fifo_occupancy;
    wire fifo_full;
    wire fifo_hwm;
    wire err_burst_interleave_unused;

    // ---- Posted credits (S6 / G2): one allocation per TLP -------------
    wire        fc_upd_valid;
    wire [1:0]  fc_upd_type;
    wire [7:0]  fc_upd_hdr;
    wire [11:0] fc_upd_data;
    wire        credit_alloc_valid;
    wire [9:0]  credit_alloc_len;
    wire        credit_grant;
    wire        credit_stall_unused;

    credit_manager u_credit_manager (
        .clk                 (clk),
        .rst_n               (rst_n),
        .update_fc_valid     (fc_upd_valid),
        .update_fc_type      (fc_upd_type),
        .update_fc_hdr_limit (fc_upd_hdr),
        .update_fc_data_limit(fc_upd_data),
        .alloc_req_valid     (credit_alloc_valid),
        .alloc_req_type      (2'b00),            // Posted (MWr-only TX path)
        .alloc_req_dw_len    (credit_alloc_len),
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
    // 2. Beat source: WDB beat or MSI-X (L10, strictly in order)
    // ==================================================================
    localparam DWS_PER_BEAT = DATA_W / 32;

    reg                 msix_pend;
    reg  [ADDR_W-1:0]   msix_addr_r;
    reg  [31:0]         msix_data_r;
    assign msix_req_ready = !msix_pend;

    reg  [4:0] beat_ndw;               // DWs in the WDB beat (TKEEP contiguous from DW 0)
    integer bi;
    always @(*) begin
        beat_ndw = 5'd0;
        for (bi = 0; bi < DWS_PER_BEAT; bi = bi + 1)
            if (wdb_tkeep[bi*4]) beat_ndw = beat_ndw + 5'd1;
    end

    wire        src_is_msix = msix_pend;
    wire        src_valid   = msix_pend || wdb_tvalid;
    wire        tw_ready;
    wire        src_fire    = src_valid && tw_ready;
    assign      wdb_tready  = src_fire && !src_is_msix;

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
    // 3. 512-bit TLP engine (Step 4 items 1 + 2): translate, merge up to
    //    Max Payload, split at 4 KB, 4-DW MWr header, one beat per clock.
    // ==================================================================
    wire tw_busy_unused;
    tlp_tx_wide #(
        .DATA_W        (DATA_W),
        .ADDR_W        (ADDR_W),
        .SOFF_W        (SOFF_W),
        .MAX_PAYLOAD_DW(MAX_PAYLOAD_DW),
        .MERGE_EN      (MERGE_EN),
        .MERGE_IDLE    (MERGE_IDLE),
        .REQ_ID        (REQ_ID)
    ) u_tlp_tx_wide (
        .clk               (clk),
        .rst_n             (rst_n),
        .s_valid           (src_valid),
        .s_ready           (tw_ready),
        .s_data            (src_is_msix ? {{(DATA_W-32){1'b0}}, msix_data_r} : wdb_tdata),
        .s_ndw             (src_is_msix ? 5'd1 : beat_ndw),
        .s_sid             (wdb_tuser[0]),
        .s_offset          (wdb_tuser[SOFF_W:1]),
        .s_tlast           (!src_is_msix && wdb_tlast),
        .s_msix            (src_is_msix),
        .s_msix_addr       (msix_addr_r),
        .cfg_base_addr_wr  (cfg_base_addr_wr),
        .cfg_base_addr_id  (cfg_base_addr_id[0]),
        .cfg_base_addr_data(cfg_base_addr_data),
        .cfg_base_addr_clr (cfg_base_addr_clr),
        .alloc_valid       (credit_alloc_valid),
        .alloc_dw_len      (credit_alloc_len),
        .alloc_grant       (credit_grant),
        .m_valid           (pack_tvalid),
        .m_data            (pack_tdata),
        .m_keep            (pack_tkeep),
        .m_last            (pack_tlast),
        .m_ready           (pack_tready),
        .err_xlat_miss     (err_xlat_miss),
        .err_addr_ovf      (err_addr_ovf),
        .stat_split_occurred(stat_split_occurred),
        .busy              (tw_busy_unused)
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
        .nak_sent_evt       (nak_sent_evt),
        .fc_upd_valid       (fc_upd_valid),
        .fc_upd_type        (fc_upd_type),
        .fc_upd_hdr         (fc_upd_hdr),
        .fc_upd_data        (fc_upd_data),
        .fc_credits_valid   (fc_credits_valid),
        .fc_init_done       (fc_init_done)
    );


endmodule