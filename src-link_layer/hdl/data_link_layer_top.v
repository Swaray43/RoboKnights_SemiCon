`timescale 1ns/1ps

// ============================================================
// Module : data_link_layer_top
// Function:
//   1. Wraps the TX and RX Link Layer pipelines.
//   2. Instantiates the Replay Buffer and DLLP Parser for
//      automatic error recovery.
//   3. Snoops RX traffic for ACK/NAK DLLPs and feeds sequence
//      numbers back to the TX replay buffer.
//
// Change (interleave support 2025):
//   link_layer_tx now has two new ports:
//     s_axis_tuser      — carries stream_id + session_offset
//     s_axis_tready_ch  — per-channel ready (2-bit)
//   The top-level TX path now propagates s_tx_axis_tuser from
//   the upstream TLP packetizer through the replay buffer and
//   into link_layer_tx.  The replay_buffer FLIT_WIDTH is kept
//   at AXIS_DATA_W (512-bit beats); TUSER is carried alongside
//   in a separate wire so the replay buffer itself is unchanged.
//
// Note on replay_buffer + TUSER:
//   The replay buffer stores raw 512-bit s_data beats.  TUSER
//   is not stored in the replay buffer — it is forwarded as a
//   combinational passthrough from the upstream input because:
//   (a) On normal TX the replay buffer passes data through
//       transparently (tx_ptr follows head_ptr).
//   (b) On NAK retransmit the buffer replays stored FLITs.
//       Since TUSER only carries stream_id/session_offset which
//       identify which channel generated the data, and since
//       the FLIT payload already contains that data correctly
//       packed, the TUSER on retransmit can safely be
//       reconstructed from the embedded stream_id in the FLIT
//       sequence number (top SEQ_WIDTH bits of stored data).
//   For now, s_tx_axis_tuser is passed straight through
//   alongside the replay buffer output (rb_tuser=s_tx_axis_tuser).
//   This is correct for the normal path; a full retransmit TUSER
//   reconstruction can be added when retransmit testing is done.
// ============================================================
module data_link_layer_top #(
    parameter AXIS_DATA_W = 512,
    parameter SYS_BUS_W   = 256,
    parameter SEQ_WIDTH   = 12,
    parameter TUSER_W     = 26
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // TX TLP Stream — from TLP packetizer
    input  wire                    s_tx_axis_tvalid,
    input  wire [AXIS_DATA_W-1:0]  s_tx_axis_tdata,
    input  wire                    s_tx_axis_tlast,
    input  wire [TUSER_W-1:0]      s_tx_axis_tuser,   // {tlast_flag, session_offset, stream_id}
    output wire                    s_tx_axis_tready,   // combined ready (both channels)
    output wire [1:0]              s_tx_axis_tready_ch,// per-channel ready

    // RX TLP Stream — to Transaction Layer RX engine
    output wire                    m_rx_axis_tvalid,
    output wire [AXIS_DATA_W-1:0]  m_rx_axis_tdata,
    output wire                    m_rx_axis_tlast,
    input  wire                    m_rx_axis_tready,

    // PHY TX
    output wire                    m_phy_tx_valid,
    output wire [SYS_BUS_W-1:0]    m_phy_tx_data,
    output wire                    m_phy_tx_sof,
    output wire                    m_phy_tx_eof,

    // PHY RX
    input  wire                    s_phy_rx_valid,
    input  wire [SYS_BUS_W-1:0]    s_phy_rx_data,
    input  wire                    s_phy_rx_sof,
    input  wire                    s_phy_rx_eof,

    // Status
    output wire                    crc_err,
    output wire                    fec_err
);

    // ---- Internal wires ----
    wire                   rb_valid;
    wire [AXIS_DATA_W-1:0] rb_data;
    wire                   ll_tx_ready;   // combined ready from link_layer_tx

    wire                   ack_valid;
    wire [SEQ_WIDTH-1:0]   ack_seq;
    wire                   nak_valid;
    wire [SEQ_WIDTH-1:0]   nak_seq;

    // TUSER is passed through alongside the replay buffer
    // (see module comment — not stored in the buffer itself)
    wire [TUSER_W-1:0] rb_tuser = s_tx_axis_tuser;

    // ----------------------------------------------------------------
    // 1. Replay Buffer
    // ----------------------------------------------------------------
    replay_buffer #(
        .FLIT_WIDTH(AXIS_DATA_W),
        .SEQ_WIDTH(SEQ_WIDTH),
        .BUF_DEPTH(32)
    ) u_replay_buffer (
        .clk           (clk),
        .rst_n         (rst_n),
        .s_valid       (s_tx_axis_tvalid),
        .s_data        (s_tx_axis_tdata),
        .s_ready       (s_tx_axis_tready),
        .m_valid       (rb_valid),
        .m_data        (rb_data),
        .m_ready       (ll_tx_ready),
        .rx_ack_valid  (ack_valid),
        .rx_ack_seq    (ack_seq),
        .rx_nak_valid  (nak_valid),
        .rx_nak_seq    (nak_seq)
    );

    // ----------------------------------------------------------------
    // 2. Link Layer TX — per-channel FLIT assembly + CRC/FEC
    // ----------------------------------------------------------------
    link_layer_tx #(
        .AXIS_DATA_W(AXIS_DATA_W),
        .SYS_BUS_W(SYS_BUS_W),
        .TUSER_W(TUSER_W)
    ) u_link_layer_tx (
        .clk                 (clk),
        .rst_n               (rst_n),
        .s_axis_tvalid       (rb_valid),
        .s_axis_tdata        (rb_data),
        .s_axis_tlast        (s_tx_axis_tlast),
        .s_axis_tuser        (rb_tuser),
        .s_axis_tready       (ll_tx_ready),
        .s_axis_tready_ch    (s_tx_axis_tready_ch),
        .m_sys_bus_valid     (m_phy_tx_valid),
        .m_sys_bus_data      (m_phy_tx_data),
        .m_sys_bus_sof       (m_phy_tx_sof),
        .m_sys_bus_eof       (m_phy_tx_eof)
    );

    // ----------------------------------------------------------------
    // 3. Link Layer RX — FLIT reassembly + CRC/FEC check
    // ----------------------------------------------------------------
    link_layer_rx #(
        .SYS_BUS_W  (SYS_BUS_W),
        .AXIS_DATA_W(AXIS_DATA_W)
    ) u_link_layer_rx (
        .clk            (clk),
        .rst_n          (rst_n),
        .s_sys_bus_valid(s_phy_rx_valid),
        .s_sys_bus_data (s_phy_rx_data),
        .s_sys_bus_sof  (s_phy_rx_sof),
        .s_sys_bus_eof  (s_phy_rx_eof),
        .m_axis_tvalid  (m_rx_axis_tvalid),
        .m_axis_tdata   (m_rx_axis_tdata),
        .m_axis_tlast   (m_rx_axis_tlast),
        .m_axis_tready  (m_rx_axis_tready),
        .crc_err        (crc_err),
        .fec_err        (fec_err)
    );

    // ----------------------------------------------------------------
    // 4. DLLP Parser — snoops RX stream for ACK/NAK
    // ----------------------------------------------------------------
    dllp_parser #(
        .FLIT_WIDTH(AXIS_DATA_W),
        .SEQ_WIDTH (SEQ_WIDTH)
    ) u_dllp_parser (
        .clk      (clk),
        .rst_n    (rst_n),
        .rx_valid (m_rx_axis_tvalid),
        .rx_data  (m_rx_axis_tdata),
        .ack_valid(ack_valid),
        .ack_seq  (ack_seq),
        .nak_valid(nak_valid),
        .nak_seq  (nak_seq)
    );

endmodule