`timescale 1ns/1ps

// ============================================================
// Module : data_link_layer_top
// Layer  : Link layer (Engineer B)
//
// Rewritten against the AXI4-PCIe6 bridge review (L3-L8, Seam C).
//
// TX:  TLP beats --> flit_payload_builder --> replay_buffer --> link_layer_tx --> PHY
//                     (59 DW per Flit)        (per-Flit seq,      (DLP, CRC-64,
//                                              ACK/NAK, replay)    FEC, ready)
//
// RX:  PHY --> link_layer_rx --> sequence check --> Flit FIFO --> rx_tlp_delimiter --> TLP beats
//              (FEC fix, CRC)    (ACK/NAK sched)
//
// ACK/NAK (L3). Each side ACKs the Flits it receives by piggy-backing on
// its own TX Flits (or a control-only Flit when it has nothing to send):
//   * good payload Flit with the expected seq, and room in the RX FIFO
//       -> accept, expected++, schedule ACK(seq)
//   * good payload Flit with an older seq (a replay we already have)
//       -> drop, re-ACK(expected-1)
//   * bad Flit (uncorrectable FEC / CRC), a seq gap, or no room
//       -> drop, schedule NAK(expected-1) once until the next good Flit
//   * ACK/NAK carried in any good incoming Flit drives our replay buffer.
// Connecting a link to itself (phy_tx -> phy_rx) is a valid loopback:
// it ACKs its own Flits and the replay buffer drains.
//
// Interface changes vs. the previous version
//   + s_tx_axis_tkeep    which bytes of a TX beat are valid (L7)
//   + m_rx_axis_tkeep    same on RX; tlast now marks real TLP ends
//   + m_phy_tx_ready     PHY back-pressure (Seam C)
//   + fec_corrected, replay_event   status pulses
//   s_tx_axis_tuser      kept for port compatibility, no longer used:
//                        Flits carry TLPs from both streams
//   s_tx_axis_tready_ch  kept; both bits equal s_tx_axis_tready
//   dllp_parser          no longer instantiated (ACK/NAK live in the DLP)
// ============================================================

module data_link_layer_top #(
    parameter AXIS_DATA_W    = 512,
    parameter SYS_BUS_W      = 256,
    parameter SEQ_WIDTH      = 12,
    parameter TUSER_W        = 26,
    parameter REPLAY_DEPTH   = 32,
    parameter REPLAY_TIMEOUT = 16384,
    parameter RX_FIFO_DEPTH  = 4,
    parameter IDLE_FLUSH     = 8
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // TX TLP stream -- from tlp32_to_axis_packer
    input  wire                     s_tx_axis_tvalid,
    input  wire [AXIS_DATA_W-1:0]   s_tx_axis_tdata,
    input  wire [AXIS_DATA_W/8-1:0] s_tx_axis_tkeep,
    input  wire                     s_tx_axis_tlast,
    input  wire [TUSER_W-1:0]       s_tx_axis_tuser,     // unused (see header)
    output wire                     s_tx_axis_tready,
    output wire [1:0]               s_tx_axis_tready_ch,

    // RX TLP stream -- one TLP per tlast-delimited burst
    output wire                     m_rx_axis_tvalid,
    output wire [AXIS_DATA_W-1:0]   m_rx_axis_tdata,
    output wire [AXIS_DATA_W/8-1:0] m_rx_axis_tkeep,
    output wire                     m_rx_axis_tlast,
    input  wire                     m_rx_axis_tready,

    // PHY TX
    output wire                     m_phy_tx_valid,
    output wire [SYS_BUS_W-1:0]     m_phy_tx_data,
    output wire                     m_phy_tx_sof,
    output wire                     m_phy_tx_eof,
    input  wire                     m_phy_tx_ready,

    // PHY RX
    input  wire                     s_phy_rx_valid,
    input  wire [SYS_BUS_W-1:0]     s_phy_rx_data,
    input  wire                     s_phy_rx_sof,
    input  wire                     s_phy_rx_eof,

    // Status
    output wire                     crc_err,
    output wire                     fec_err,
    output wire                     fec_corrected,
    output wire                     replay_event
);

    localparam FL_W = 1888;

    // ================================================================
    // TX
    // ================================================================
    wire            fb_valid, fb_ready;
    wire [FL_W-1:0] fb_payload;

    flit_payload_builder #(
        .AXIS_DATA_W(AXIS_DATA_W),
        .IDLE_FLUSH (IDLE_FLUSH)
    ) u_flit_payload_builder (
        .clk      (clk),
        .rst_n    (rst_n),
        .s_tvalid (s_tx_axis_tvalid),
        .s_tdata  (s_tx_axis_tdata),
        .s_tkeep  (s_tx_axis_tkeep),
        .s_tlast  (s_tx_axis_tlast),
        .s_tready (s_tx_axis_tready),
        .m_valid  (fb_valid),
        .m_payload(fb_payload),
        .m_ready  (fb_ready)
    );

    assign s_tx_axis_tready_ch = {s_tx_axis_tready, s_tx_axis_tready};

    wire                 rb_valid, rb_ready;
    wire [FL_W-1:0]      rb_payload;
    wire [SEQ_WIDTH-1:0] rb_seq;

    wire                 far_ack_valid, far_nak_valid;
    wire [SEQ_WIDTH-1:0] far_ackn_seq;

    replay_buffer #(
        .PAYLOAD_W     (FL_W),
        .SEQ_WIDTH     (SEQ_WIDTH),
        .BUF_DEPTH     (REPLAY_DEPTH),
        .REPLAY_TIMEOUT(REPLAY_TIMEOUT)
    ) u_replay_buffer (
        .clk          (clk),
        .rst_n        (rst_n),
        .s_valid      (fb_valid),
        .s_data       (fb_payload),
        .s_ready      (fb_ready),
        .m_valid      (rb_valid),
        .m_data       (rb_payload),
        .m_seq        (rb_seq),
        .m_ready      (rb_ready),
        .rx_ack_valid (far_ack_valid),
        .rx_ack_seq   (far_ackn_seq),
        .rx_nak_valid (far_nak_valid),
        .rx_nak_seq   (far_ackn_seq),
        .outstanding  (),
        .replay_event (replay_event)
    );

    reg                  ack_pend, nak_pend;
    reg [SEQ_WIDTH-1:0]  ackn_seq_r;
    wire                 ackn_taken;

    link_layer_tx #(
        .SYS_BUS_W(SYS_BUS_W),
        .SEQ_W    (SEQ_WIDTH)
    ) u_link_layer_tx (
        .clk            (clk),
        .rst_n          (rst_n),
        .s_valid        (rb_valid),
        .s_payload      (rb_payload),
        .s_seq          (rb_seq),
        .s_ready        (rb_ready),
        .ack_req        (ack_pend),
        .nak_req        (nak_pend),
        .ackn_seq       (ackn_seq_r),
        .ackn_taken     (ackn_taken),
        .m_sys_bus_valid(m_phy_tx_valid),
        .m_sys_bus_data (m_phy_tx_data),
        .m_sys_bus_sof  (m_phy_tx_sof),
        .m_sys_bus_eof  (m_phy_tx_eof),
        .m_sys_bus_ready(m_phy_tx_ready)
    );

    // ================================================================
    // RX
    // ================================================================
    wire                 rx_v, rx_ok;
    wire [1:0]           rx_kind;
    wire [SEQ_WIDTH-1:0] rx_seq;
    wire                 rx_ack_v, rx_nak_v;
    wire [SEQ_WIDTH-1:0] rx_ackn_seq;
    wire [FL_W-1:0]      rx_payload;

    link_layer_rx #(
        .SYS_BUS_W(SYS_BUS_W),
        .SEQ_W    (SEQ_WIDTH)
    ) u_link_layer_rx (
        .clk            (clk),
        .rst_n          (rst_n),
        .s_sys_bus_valid(s_phy_rx_valid),
        .s_sys_bus_data (s_phy_rx_data),
        .s_sys_bus_sof  (s_phy_rx_sof),
        .s_sys_bus_eof  (s_phy_rx_eof),
        .rx_flit_valid  (rx_v),
        .rx_flit_ok     (rx_ok),
        .rx_kind        (rx_kind),
        .rx_seq         (rx_seq),
        .rx_ack_valid   (rx_ack_v),
        .rx_nak_valid   (rx_nak_v),
        .rx_ackn_seq    (rx_ackn_seq),
        .rx_payload     (rx_payload),
        .crc_err        (crc_err),
        .fec_err        (fec_err),
        .fec_corrected  (fec_corrected)
    );

    // ACK/NAK the far end sent us -> our replay buffer (trusted only if the
    // Flit itself checked out)
    assign far_ack_valid = rx_v && rx_ok && rx_ack_v && !rx_nak_v;
    assign far_nak_valid = rx_v && rx_ok && rx_nak_v;
    assign far_ackn_seq  = rx_ackn_seq;

    // ---------------- RX Flit FIFO ----------------
    localparam RXF_AW = $clog2(RX_FIFO_DEPTH);

    reg [FL_W-1:0]   rxf_mem [0:RX_FIFO_DEPTH-1];
    reg [RXF_AW:0]   rxf_wp, rxf_rp;
    wire [RXF_AW:0]  rxf_cnt   = rxf_wp - rxf_rp;      // modulo pointer width
    wire             rxf_full  = (rxf_cnt == RX_FIFO_DEPTH);
    wire             rxf_empty = (rxf_wp == rxf_rp);
    wire             rxf_pop;

    // ---------------- sequence check / ACK-NAK scheduler ----------------
    reg  [SEQ_WIDTH-1:0] exp_seq;
    reg                  nak_sent;

    wire is_payload  = (rx_kind == 2'd0);
    wire in_order    = (rx_seq == exp_seq);
    // older = within the last half of the sequence space behind exp_seq
    wire [SEQ_WIDTH-1:0] behind = exp_seq - rx_seq;
    wire is_dup      = !in_order && (behind <= (1 << (SEQ_WIDTH - 1)));

    wire rx_accept   = rx_v && rx_ok && is_payload && in_order && !rxf_full;
    wire rx_dup      = rx_v && rx_ok && is_payload && is_dup;
    wire rx_need_nak = rx_v && (!rx_ok || (is_payload && !in_order && !is_dup) ||
                                (is_payload && in_order && rxf_full));

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rxf_wp     <= {(RXF_AW+1){1'b0}};
            exp_seq    <= {SEQ_WIDTH{1'b0}};
            nak_sent   <= 1'b0;
            ack_pend   <= 1'b0;
            nak_pend   <= 1'b0;
            ackn_seq_r <= {SEQ_WIDTH{1'b0}};
        end else begin
            // TX consumed the pending ACK/NAK (a new request below wins)
            if (ackn_taken) begin
                ack_pend <= 1'b0;
                nak_pend <= 1'b0;
            end

            if (rx_accept) begin
                rxf_mem[rxf_wp[RXF_AW-1:0]] <= rx_payload;
                rxf_wp     <= rxf_wp + 1'b1;
                exp_seq    <= exp_seq + 1'b1;
                nak_sent   <= 1'b0;
                ack_pend   <= 1'b1;
                nak_pend   <= 1'b0;
                ackn_seq_r <= rx_seq;
            end else if (rx_dup) begin
                ack_pend   <= 1'b1;
                nak_pend   <= 1'b0;
                ackn_seq_r <= exp_seq - 1'b1;
            end else if (rx_need_nak && !nak_sent) begin
                nak_sent   <= 1'b1;
                nak_pend   <= 1'b1;
                ack_pend   <= 1'b0;
                ackn_seq_r <= exp_seq - 1'b1;
            end
        end
    end

    // ---------------- RX TLP delimiter ----------------
    wire dl_ready;
    assign rxf_pop = !rxf_empty && dl_ready;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) rxf_rp <= {(RXF_AW+1){1'b0}};
        else if (rxf_pop) rxf_rp <= rxf_rp + 1'b1;
    end

    rx_tlp_delimiter #(
        .AXIS_DATA_W(AXIS_DATA_W)
    ) u_rx_tlp_delimiter (
        .clk          (clk),
        .rst_n        (rst_n),
        .s_valid      (!rxf_empty),
        .s_payload    (rxf_mem[rxf_rp[RXF_AW-1:0]]),
        .s_ready      (dl_ready),
        .m_axis_tvalid(m_rx_axis_tvalid),
        .m_axis_tdata (m_rx_axis_tdata),
        .m_axis_tkeep (m_rx_axis_tkeep),
        .m_axis_tlast (m_rx_axis_tlast),
        .m_axis_tready(m_rx_axis_tready)
    );

    // synthesis translate_off
    initial begin
        if ((RX_FIFO_DEPTH & (RX_FIFO_DEPTH - 1)) != 0)
            $fatal(1, "[data_link_layer_top] RX_FIFO_DEPTH must be a power of two");
    end
    // synthesis translate_on

endmodule
