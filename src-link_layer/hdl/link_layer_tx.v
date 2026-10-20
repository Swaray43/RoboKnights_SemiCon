`timescale 1ns/1ps

// ============================================================
// Module : link_layer_tx
// Layer  : Link layer (Engineer B) -- Flit framer + PHY serialiser
//
// Rewritten against the AXI4-PCIe6 bridge review (L4-L8, Seam C).
//
// What changed and why
//   * Input is now a finished 236-byte TLP area (59 DW) plus its Flit
//     sequence number, taken from the replay buffer. Packing TLP DWs into
//     that area is done upstream by flit_payload_builder, so nothing is
//     dropped (L6: the old 2008-bit payload threw away s_axis_tdata[511:472]
//     of every 4th beat) and short TLPs are not stranded in fixed slots
//     (L7: the old partial seal always wrote the last beat into slot 3).
//   * The sequence number travels in the DLP bytes, not over the top of
//     TLP data (L4).
//   * Every Flit carries a piggy-backed ACK/NAK for the reverse direction
//     in its DLP; if there is no payload to send but an ACK/NAK is pending,
//     a control-only Flit (kind=1, TLP area all NOPs) is sent so the far
//     end's replay buffer always drains (L3).
//   * Protection is CRC-64 over TLP+DLP and real 3-way interleaved SEC-DED
//     FEC (L8), laid out as 236/6/8/6 bytes like PCIe 6.0 (see flit_defs.vh).
//   * m_sys_bus_ready from the PHY now stalls the 8-word serialiser
//     (Seam C). The PHY can take as long as it needs per word.
//   * Per-channel Flit assembly is gone: a Flit is a link-level container
//     and TLPs from both streams share it. The TLP header carries the
//     address, which is all the far end needs.
//
// Timing: 1 cycle to latch, 1 cycle to compute CRC/FEC into a register,
// then 8 bus words. Back-to-back Flits cost 10 cycles each with ready=1.
// ============================================================

module link_layer_tx #(
    parameter SYS_BUS_W = 256,
    parameter SEQ_W     = 12
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // Payload Flit from replay buffer
    input  wire                 s_valid,
    input  wire [1887:0]        s_payload,       // 59 DW TLP area
    input  wire [SEQ_W-1:0]     s_seq,
    output wire                 s_ready,

    // ACK/NAK to piggy-back (from the receive-side scheduler)
    input  wire                 ack_req,
    input  wire                 nak_req,
    input  wire [SEQ_W-1:0]     ackn_seq,
    output wire                 ackn_taken,      // same-cycle: pending ACK/NAK is being loaded into a Flit

    // G2: Flow-Control DLLPs (DW0 = P, DW1 = NP, DW2 = Cpl), sent in a
    // control Flit with DLP[28] set. Takes priority over a payload Flit
    // (one Flit slot; requests are rate-limited by the FC timer).
    input  wire                 dllp_req,
    input  wire [95:0]          dllp_data,
    output wire                 dllp_taken,      // same-cycle: dllp_data is being loaded

    // System-synchronous 256-bit bus to PHY
    output wire                 m_sys_bus_valid,
    output wire [SYS_BUS_W-1:0] m_sys_bus_data,
    output wire                 m_sys_bus_sof,
    output wire                 m_sys_bus_eof,
    input  wire                 m_sys_bus_ready
);

    `include "flit_defs.vh"

    localparam ST_IDLE = 2'd0;
    localparam ST_CALC = 2'd1;
    localparam ST_SEND = 2'd2;

    reg [1:0]            state;
    reg [FL_BODY_W-1:0]  body_r;      // TLP area + DLP
    reg [FLIT_W-1:0]     flit_r;      // complete protected Flit
    reg [2:0]            word_idx;

    wire have_ackn = ack_req || nak_req;

    // Accept a payload Flit only when idle
    assign s_ready = (state == ST_IDLE);

    // DLP for whatever is loaded this cycle
    function [FL_DLP_W-1:0] make_dlp;
        input [1:0]       kind;
        input [SEQ_W-1:0] seq;
        input             ack_v;
        input             nak_v;
        input [SEQ_W-1:0] an_seq;
        begin
            make_dlp = {FL_DLP_W{1'b0}};
            make_dlp[11:0]  = seq;
            make_dlp[13:12] = kind;
            make_dlp[14]    = ack_v;
            make_dlp[15]    = nak_v;
            make_dlp[27:16] = an_seq;
        end
    endfunction

    // NAK wins over ACK if both are pending (NAK also acknowledges
    // everything up to its sequence number).
    wire send_nak = nak_req;
    wire send_ack = ack_req && !nak_req;

    // Combinational so the scheduler clears exactly the request that was
    // sampled; a newer request arriving on the same edge wins over the clear.
    assign ackn_taken = (state == ST_IDLE) && have_ackn;
    assign dllp_taken = (state == ST_IDLE) && dllp_req;

    // G2: DLP of an FC DLLP control Flit
    reg [FL_DLP_W-1:0] fc_dlp;
    always @(*) begin
        fc_dlp = make_dlp(FLIT_KIND_CONTROL, {SEQ_W{1'b0}}, send_ack, send_nak, ackn_seq);
        fc_dlp[FL_DLP_FC_BIT] = 1'b1;
    end

    // CRC + FEC on the latched body
    wire [63:0] body_crc = flit_crc64(body_r);
    wire [47:0] body_fec = flit_fec_encode({body_crc, body_r});

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            body_r     <= {FL_BODY_W{1'b0}};
            flit_r     <= {FLIT_W{1'b0}};
            word_idx   <= 3'd0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (dllp_req) begin
                        // G2: FC DLLP control Flit (also carries any pending ACK/NAK)
                        body_r     <= {fc_dlp, {(FL_TLP_W-96){1'b0}}, dllp_data};
                        state      <= ST_CALC;
                    end else if (s_valid) begin
                        body_r     <= {make_dlp(FLIT_KIND_PAYLOAD, s_seq, send_ack, send_nak, ackn_seq),
                                       s_payload};
                        state      <= ST_CALC;
                    end else if (have_ackn) begin
                        body_r     <= {make_dlp(FLIT_KIND_CONTROL, {SEQ_W{1'b0}}, send_ack, send_nak, ackn_seq),
                                       {FL_TLP_W{1'b0}}};
                        state      <= ST_CALC;
                    end
                end
                ST_CALC: begin
                    flit_r   <= {body_fec, body_crc, body_r};
                    word_idx <= 3'd0;
                    state    <= ST_SEND;
                end
                ST_SEND: begin
                    if (m_sys_bus_ready) begin
                        if (word_idx == 3'd7) state <= ST_IDLE;
                        word_idx <= word_idx + 3'd1;
                    end
                end
                default: state <= ST_IDLE;
            endcase
        end
    end

    assign m_sys_bus_valid = (state == ST_SEND);
    assign m_sys_bus_data  = flit_r[word_idx*SYS_BUS_W +: SYS_BUS_W];
    assign m_sys_bus_sof   = (state == ST_SEND) && (word_idx == 3'd0);
    assign m_sys_bus_eof   = (state == ST_SEND) && (word_idx == 3'd7);

    // synthesis translate_off
    initial begin
        if (SYS_BUS_W * 8 != FLIT_W)
            $fatal(1, "[link_layer_tx] SYS_BUS_W (%0d) * 8 must equal the 2048-bit Flit", SYS_BUS_W);
    end
    // synthesis translate_on

endmodule
