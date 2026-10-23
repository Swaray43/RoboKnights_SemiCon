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
// ---- OPT-4 (optimization pass, review item 4): pipelined CRC/FEC -------
//   Baseline: flit_crc64()/flit_fec_encode() each unrolled into a single
//   combinational call over the whole 1936/2000-bit body in ST_CALC's one
//   cycle. That one call was the worst path in the whole module: WNS
//   -21.45 ns (~39 MHz).
//
//   Fix: walk the same bits in 256-bit chunks (one SYS_BUS_W word each --
//   the same granularity ST_SEND already uses), one chunk per clock, using
//   incremental step functions that are bit-for-bit identical in algorithm
//   and processing ORDER to the originals in flit_defs.vh (same LFSR
//   update for CRC, same byte-interleaved Hamming walk for FEC) -- only
//   checkpointed across cycles instead of unrolled in one. 7 full 256-bit
//   body chunks (idx 0..6), 1 partial 144-bit body chunk that finishes the
//   CRC (idx 7), then 1 64-bit chunk of the now-known CRC bytes that
//   finishes the FEC, which also covers the CRC field (idx 8). Each
//   step's own combinational depth is now <=256 XOR levels instead of
//   <=2000, at the cost of 8 extra cycles of latency per Flit (ST_CALC
//   goes from 1 cycle to 9; back-to-back Flit cost 10 -> 18 cycles).
//
//   Each case arm below passes a LITERAL constant chunk length/byte
//   offset to the step functions (not calc_idx itself) so every call site
//   has a true elaboration-time-constant trip count for its internal
//   `for` loop -- required for this to synthesize as fixed combinational
//   logic per arm rather than a variable-bound loop.
//
//   flit_defs.vh's own flit_crc64()/flit_fec_encode() are left completely
//   untouched (still used as-is by the RX side), which doubles as the
//   reference model for verifying this pipeline bit-exact: feed the same
//   body_r through both and diff body_crc_r/body_fec_r against them.
//
//   STATUS: drafted, not yet simulated. Verify against tb_link_layer_loopback
//   (S5) and tb_stream_path before merging -- see claude/opt_fixes.patch.
//
// Timing: 1 cycle to latch, 9 cycles to compute CRC/FEC (OPT-4), then
// 8 bus words. Back-to-back Flits cost 18 cycles each with ready=1.
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

    // ---- Step 4 T3: full-width 3-stage pipeline (replaces OPT-4's 9-cycle
    // chunked walk). One Flit can be loaded every cycle; the 8-word send
    // is the only rate limit (8 clocks per Flit, was 18).
    //   L  (load)   body = DLP + TLP area, from the replay buffer / ACK / FC
    //   C1 (crc)    crc64 over the body, FEC syndromes of the body bytes
    //   C2 (fec)    FEC syndromes of the CRC bytes added, check bits formed
    //   S  (send)   8 x 256-bit words to the PHY, ready-stalled
    // Bit-exact with flit_defs.vh's flit_crc64 / flit_fec_encode (the FEC is
    // linear, so syndrome(body + crc) = syndrome(body) ^ syndrome(crc)).
    reg                  l_v;          // loaded body waiting for C1
    reg [FL_BODY_W-1:0]  l_body;
    reg                  c1_v;
    reg [FL_BODY_W-1:0]  c1_body;
    reg [511:0]          c1_parts;     // Wave C: CRC-64 per-chunk parts (combined in C2)
    reg [32:0]           c1_sp;        // {sp2, sp1, sp0}, 11 bits each, body bytes only
    reg                  c2_v;
    reg [FLIT_W-1:0]     c2_flit;
    reg                  snd;          // sending
    reg [FLIT_W-1:0]     flit_r;
    reg [2:0]            word_idx;

    wire snd_last  = snd && m_sys_bus_ready && (word_idx == 3'd7);
    wire snd_free  = !snd || snd_last;
    wire c2_take   = c2_v && snd_free;
    wire c2_free   = !c2_v || c2_take;
    wire c1_take   = c1_v && c2_free;
    wire c1_free   = !c1_v || c1_take;
    wire l_take    = l_v && c1_free;
    wire l_free    = !l_v || l_take;

    wire have_ackn = ack_req || nak_req;

    // A DLLP Flit wins the slot: do not also consume the payload
    assign s_ready = l_free && !dllp_req;

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
    assign ackn_taken = l_free && have_ackn;
    assign dllp_taken = l_free && dllp_req;

    reg [FL_DLP_W-1:0] fc_dlp;
    always @(*) begin
        fc_dlp = make_dlp(FLIT_KIND_CONTROL, {SEQ_W{1'b0}}, send_ack, send_nak, ackn_seq);
        fc_dlp[FL_DLP_FC_BIT] = 1'b1;
    end

    // FEC syndrome helpers on the full 2000-bit protected area
    function [32:0] fec_sp3;
        input [FL_PROT_W-1:0] d;
        begin
            fec_sp3 = {fl_cw_syn(d, 2), fl_cw_syn(d, 1), fl_cw_syn(d, 0)};
        end
    endfunction
    function [10:0] cw_bits;           // check bits + overall parity from {par, syn}
        input [10:0] sp;
        begin
            cw_bits = {sp[10] ^ (^sp[9:0]), sp[9:0]};
        end
    endfunction

    wire [63:0] c2_crc = flit_crc64_combine(c1_parts);
    wire [32:0] sp_crc = fec_sp3({c2_crc, {FL_BODY_W{1'b0}}});
    wire [32:0] sp_all = c1_sp ^ sp_crc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            l_v <= 1'b0; l_body <= {FL_BODY_W{1'b0}};
            c1_v <= 1'b0; c1_body <= {FL_BODY_W{1'b0}}; c1_parts <= 512'd0; c1_sp <= 33'd0;
            c2_v <= 1'b0; c2_flit <= {FLIT_W{1'b0}};
            snd <= 1'b0; flit_r <= {FLIT_W{1'b0}}; word_idx <= 3'd0;
        end else begin
            // ---- L: load (same priority as before: FC DLLP, payload, ACK/NAK) ----
            if (l_take) l_v <= 1'b0;
            if (l_free) begin
                if (dllp_req) begin
                    l_body <= {fc_dlp, {(FL_TLP_W-96){1'b0}}, dllp_data};
                    l_v    <= 1'b1;
                end else if (s_valid) begin
                    l_body <= {make_dlp(FLIT_KIND_PAYLOAD, s_seq, send_ack, send_nak, ackn_seq), s_payload};
                    l_v    <= 1'b1;
                end else if (have_ackn) begin
                    l_body <= {make_dlp(FLIT_KIND_CONTROL, {SEQ_W{1'b0}}, send_ack, send_nak, ackn_seq),
                               {FL_TLP_W{1'b0}}};
                    l_v    <= 1'b1;
                end
            end
            // ---- C1 ----
            if (c1_take) c1_v <= 1'b0;
            if (l_take) begin
                c1_v    <= 1'b1;
                c1_body <= l_body;
                c1_parts <= flit_crc64_parts(l_body);
                c1_sp   <= fec_sp3({64'd0, l_body});
            end
            // ---- C2 ----
            if (c2_take) c2_v <= 1'b0;
            if (c1_take) begin
                c2_v    <= 1'b1;
                c2_flit <= {15'd0, cw_bits(sp_all[32:22]), cw_bits(sp_all[21:11]), cw_bits(sp_all[10:0]),
                            c2_crc, c1_body};
            end
            // ---- S ----
            if (snd && m_sys_bus_ready) begin
                word_idx <= word_idx + 3'd1;
                if (word_idx == 3'd7) snd <= 1'b0;
            end
            if (c2_take) begin
                snd      <= 1'b1;
                flit_r   <= c2_flit;
                word_idx <= 3'd0;
            end
        end
    end

`ifndef SYNTHESIS
    always @(posedge clk) if (rst_n && c1_take && (c2_crc !== flit_crc64(c1_body)))
        $display("ERROR link_layer_tx @%0t: pipelined CRC-64 differs from flit_crc64", $time);
`endif

    assign m_sys_bus_valid = snd;
    assign m_sys_bus_data  = flit_r[word_idx*SYS_BUS_W +: SYS_BUS_W];
    assign m_sys_bus_sof   = snd && (word_idx == 3'd0);
    assign m_sys_bus_eof   = snd && (word_idx == 3'd7);

    // synthesis translate_off
    initial begin
        if (SYS_BUS_W * 8 != FLIT_W)
            $fatal(1, "[link_layer_tx] SYS_BUS_W (%0d) * 8 must equal the 2048-bit Flit", SYS_BUS_W);
    end
    // synthesis translate_on

endmodule
