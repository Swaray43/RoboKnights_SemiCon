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

    localparam ST_IDLE = 2'd0;
    localparam ST_CALC = 2'd1;
    localparam ST_SEND = 2'd2;

    reg [1:0]            state;
    reg [FL_BODY_W-1:0]  body_r;      // TLP area + DLP
    reg [FLIT_W-1:0]     flit_r;      // complete protected Flit
    reg [2:0]            word_idx;

    // ---- OPT-4 pipeline state ----
    reg [3:0]  calc_idx;              // 0..8 across ST_CALC
    reg [63:0] crc_acc;               // running CRC-64 state (pre-xorout)
    reg [9:0]  fec_s   [0:2];         // running Hamming syndrome, per interleave
    reg        fec_par [0:2];         // running overall parity, per interleave
    reg [63:0] body_crc_r;            // latched final CRC (post ~, i.e. xorout applied)

    // one incremental CRC-64/ECMA-182 step over the low `len` bits of
    // `chunk` -- identical per-bit update to flit_defs.vh's flit_crc64,
    // just resumable via c_in/returned state instead of a single 1936-bit
    // sweep. `len` must be a literal at the call site (see OPT-4 note above).
    function [63:0] flit_crc64_step;
        input [63:0]  c_in;
        input [255:0] chunk;
        input integer len;
        integer i;
        reg [63:0] c;
        begin
            c = c_in;
            for (i = 0; i < len; i = i + 1) begin
                if (c[63] ^ chunk[i])
                    c = (c << 1) ^ 64'h42F0E1EBA9EA3693;
                else
                    c = c << 1;
            end
            flit_crc64_step = c;
        end
    endfunction

    // Hamming position counter after `n` data bits of one interleave have
    // already been placed -- purely structural (same skip-the-powers-of-2
    // sequence as flit_defs.vh's fl_cw_syn, starting at p=2), so it can be
    // evaluated with constant inputs at each call site below; no running
    // position counter needs to be carried in hardware, only the
    // syndrome/parity accumulator (fec_s/fec_par above).
    function [10:0] calc_p_at;
        input integer n;
        integer i;
        reg [10:0] p;
        begin
            p = 11'd2;
            for (i = 0; i < n; i = i + 1) begin
                p = p + 11'd1;
                if (fl_is_pow2(p)) p = p + 11'd1;
            end
            calc_p_at = p;
        end
    endfunction

    // number of interleave-j bytes strictly before global byte `byte_lo`
    function integer calc_j_cnt;
        input integer byte_lo;
        input integer j;
        integer b, c;
        begin
            c = 0;
            for (b = 0; b < byte_lo; b = b + 1)
                if ((b % 3) == j) c = c + 1;
            calc_j_cnt = c;
        end
    endfunction

    // incremental partial Hamming syndrome/parity contribution of one
    // chunk's bytes belonging to interleave j. byte_lo/len_bits/j must be
    // literals at the call site (see OPT-4 note above); p_start is then a
    // genuine elaboration-time constant via calc_p_at/calc_j_cnt.
    function [10:0] calc_fec_chunk;
        input [255:0] chunk;
        input integer byte_lo;
        input integer len_bits;
        input integer j;
        integer b, k, bb, last_byte;
        reg [10:0] p;
        reg [9:0]  s;
        reg        par;
        begin
            p = calc_p_at(calc_j_cnt(byte_lo, j) * 8);
            s = 10'd0; par = 1'b0;
            last_byte = byte_lo + (len_bits / 8) - 1;
            for (b = byte_lo; b <= last_byte; b = b + 1) begin
                if ((b % 3) == j) begin
                    bb = (b - byte_lo) * 8;
                    for (k = 0; k < 8; k = k + 1) begin
                        p = p + 11'd1;
                        if (fl_is_pow2(p)) p = p + 11'd1;
                        if (chunk[bb+k]) begin
                            s   = s ^ p[9:0];
                            par = ~par;
                        end
                    end
                end
            end
            calc_fec_chunk = {par, s};
        end
    endfunction

    // thin wrappers so no call site ever part-selects a function call's
    // return value directly (kept portable across simulators)
    function [9:0] calc_fec_chunk_s;
        input [255:0] chunk;
        input integer byte_lo;
        input integer len_bits;
        input integer j;
        reg [10:0] r;
        begin
            r = calc_fec_chunk(chunk, byte_lo, len_bits, j);
            calc_fec_chunk_s = r[9:0];
        end
    endfunction

    function calc_fec_chunk_par;
        input [255:0] chunk;
        input integer byte_lo;
        input integer len_bits;
        input integer j;
        reg [10:0] r;
        begin
            r = calc_fec_chunk(chunk, byte_lo, len_bits, j);
            calc_fec_chunk_par = r[10];
        end
    endfunction

    // calc_idx==8's chunk is the (now-finalized) CRC field -- needs the
    // final per-interleave accumulate done combinationally in that same
    // cycle (there's no calc_idx==9 to read the updated fec_s/fec_par
    // registers back from), so compute it here rather than in the always
    // block below.
    wire [10:0] fec8_0 = calc_fec_chunk({192'd0, body_crc_r}, 242, 64, 0);
    wire [10:0] fec8_1 = calc_fec_chunk({192'd0, body_crc_r}, 242, 64, 1);
    wire [10:0] fec8_2 = calc_fec_chunk({192'd0, body_crc_r}, 242, 64, 2);
    wire [9:0]  fec_s0_final   = fec_s[0]   ^ fec8_0[9:0];
    wire        fec_par0_final = fec_par[0] ^ fec8_0[10];
    wire [9:0]  fec_s1_final   = fec_s[1]   ^ fec8_1[9:0];
    wire        fec_par1_final = fec_par[1] ^ fec8_1[10];
    wire [9:0]  fec_s2_final   = fec_s[2]   ^ fec8_2[9:0];
    wire        fec_par2_final = fec_par[2] ^ fec8_2[10];

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

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state      <= ST_IDLE;
            body_r     <= {FL_BODY_W{1'b0}};
            flit_r     <= {FLIT_W{1'b0}};
            word_idx   <= 3'd0;
            calc_idx   <= 4'd0;
            crc_acc    <= {64{1'b1}};
            body_crc_r <= 64'd0;
            fec_s[0]   <= 10'd0; fec_par[0] <= 1'b0;
            fec_s[1]   <= 10'd0; fec_par[1] <= 1'b0;
            fec_s[2]   <= 10'd0; fec_par[2] <= 1'b0;
        end else begin
            case (state)
                ST_IDLE: begin
                    if (dllp_req) begin
                        body_r   <= {fc_dlp, {(FL_TLP_W-96){1'b0}}, dllp_data};
                        state    <= ST_CALC;
                        calc_idx <= 4'd0;
                        crc_acc  <= {64{1'b1}};
                        fec_s[0] <= 10'd0; fec_par[0] <= 1'b0;
                        fec_s[1] <= 10'd0; fec_par[1] <= 1'b0;
                        fec_s[2] <= 10'd0; fec_par[2] <= 1'b0;
                    end else if (s_valid) begin
                        body_r   <= {make_dlp(FLIT_KIND_PAYLOAD, s_seq, send_ack, send_nak, ackn_seq),
                                     s_payload};
                        state    <= ST_CALC;
                        calc_idx <= 4'd0;
                        crc_acc  <= {64{1'b1}};
                        fec_s[0] <= 10'd0; fec_par[0] <= 1'b0;
                        fec_s[1] <= 10'd0; fec_par[1] <= 1'b0;
                        fec_s[2] <= 10'd0; fec_par[2] <= 1'b0;
                    end else if (have_ackn) begin
                        body_r   <= {make_dlp(FLIT_KIND_CONTROL, {SEQ_W{1'b0}}, send_ack, send_nak, ackn_seq),
                                     {FL_TLP_W{1'b0}}};
                        state    <= ST_CALC;
                        calc_idx <= 4'd0;
                        crc_acc  <= {64{1'b1}};
                        fec_s[0] <= 10'd0; fec_par[0] <= 1'b0;
                        fec_s[1] <= 10'd0; fec_par[1] <= 1'b0;
                        fec_s[2] <= 10'd0; fec_par[2] <= 1'b0;
                    end
                end
                // ---- OPT-4: one 256-bit (or shorter, final) chunk per cycle ----
                ST_CALC: begin
                    case (calc_idx)
                        4'd0: begin
                            crc_acc    <= flit_crc64_step(crc_acc, body_r[0*256 +: 256], 256);
                            fec_s[0]   <= fec_s[0]   ^ calc_fec_chunk_s(body_r[0*256 +: 256], 0*32, 256, 0);
                            fec_par[0] <= fec_par[0] ^ calc_fec_chunk_par(body_r[0*256 +: 256], 0*32, 256, 0);
                            fec_s[1]   <= fec_s[1]   ^ calc_fec_chunk_s(body_r[0*256 +: 256], 0*32, 256, 1);
                            fec_par[1] <= fec_par[1] ^ calc_fec_chunk_par(body_r[0*256 +: 256], 0*32, 256, 1);
                            fec_s[2]   <= fec_s[2]   ^ calc_fec_chunk_s(body_r[0*256 +: 256], 0*32, 256, 2);
                            fec_par[2] <= fec_par[2] ^ calc_fec_chunk_par(body_r[0*256 +: 256], 0*32, 256, 2);
                            calc_idx   <= 4'd1;
                        end
                        4'd1: begin
                            crc_acc    <= flit_crc64_step(crc_acc, body_r[1*256 +: 256], 256);
                            fec_s[0]   <= fec_s[0]   ^ calc_fec_chunk_s(body_r[1*256 +: 256], 1*32, 256, 0);
                            fec_par[0] <= fec_par[0] ^ calc_fec_chunk_par(body_r[1*256 +: 256], 1*32, 256, 0);
                            fec_s[1]   <= fec_s[1]   ^ calc_fec_chunk_s(body_r[1*256 +: 256], 1*32, 256, 1);
                            fec_par[1] <= fec_par[1] ^ calc_fec_chunk_par(body_r[1*256 +: 256], 1*32, 256, 1);
                            fec_s[2]   <= fec_s[2]   ^ calc_fec_chunk_s(body_r[1*256 +: 256], 1*32, 256, 2);
                            fec_par[2] <= fec_par[2] ^ calc_fec_chunk_par(body_r[1*256 +: 256], 1*32, 256, 2);
                            calc_idx   <= 4'd2;
                        end
                        4'd2: begin
                            crc_acc    <= flit_crc64_step(crc_acc, body_r[2*256 +: 256], 256);
                            fec_s[0]   <= fec_s[0]   ^ calc_fec_chunk_s(body_r[2*256 +: 256], 2*32, 256, 0);
                            fec_par[0] <= fec_par[0] ^ calc_fec_chunk_par(body_r[2*256 +: 256], 2*32, 256, 0);
                            fec_s[1]   <= fec_s[1]   ^ calc_fec_chunk_s(body_r[2*256 +: 256], 2*32, 256, 1);
                            fec_par[1] <= fec_par[1] ^ calc_fec_chunk_par(body_r[2*256 +: 256], 2*32, 256, 1);
                            fec_s[2]   <= fec_s[2]   ^ calc_fec_chunk_s(body_r[2*256 +: 256], 2*32, 256, 2);
                            fec_par[2] <= fec_par[2] ^ calc_fec_chunk_par(body_r[2*256 +: 256], 2*32, 256, 2);
                            calc_idx   <= 4'd3;
                        end
                        4'd3: begin
                            crc_acc    <= flit_crc64_step(crc_acc, body_r[3*256 +: 256], 256);
                            fec_s[0]   <= fec_s[0]   ^ calc_fec_chunk_s(body_r[3*256 +: 256], 3*32, 256, 0);
                            fec_par[0] <= fec_par[0] ^ calc_fec_chunk_par(body_r[3*256 +: 256], 3*32, 256, 0);
                            fec_s[1]   <= fec_s[1]   ^ calc_fec_chunk_s(body_r[3*256 +: 256], 3*32, 256, 1);
                            fec_par[1] <= fec_par[1] ^ calc_fec_chunk_par(body_r[3*256 +: 256], 3*32, 256, 1);
                            fec_s[2]   <= fec_s[2]   ^ calc_fec_chunk_s(body_r[3*256 +: 256], 3*32, 256, 2);
                            fec_par[2] <= fec_par[2] ^ calc_fec_chunk_par(body_r[3*256 +: 256], 3*32, 256, 2);
                            calc_idx   <= 4'd4;
                        end
                        4'd4: begin
                            crc_acc    <= flit_crc64_step(crc_acc, body_r[4*256 +: 256], 256);
                            fec_s[0]   <= fec_s[0]   ^ calc_fec_chunk_s(body_r[4*256 +: 256], 4*32, 256, 0);
                            fec_par[0] <= fec_par[0] ^ calc_fec_chunk_par(body_r[4*256 +: 256], 4*32, 256, 0);
                            fec_s[1]   <= fec_s[1]   ^ calc_fec_chunk_s(body_r[4*256 +: 256], 4*32, 256, 1);
                            fec_par[1] <= fec_par[1] ^ calc_fec_chunk_par(body_r[4*256 +: 256], 4*32, 256, 1);
                            fec_s[2]   <= fec_s[2]   ^ calc_fec_chunk_s(body_r[4*256 +: 256], 4*32, 256, 2);
                            fec_par[2] <= fec_par[2] ^ calc_fec_chunk_par(body_r[4*256 +: 256], 4*32, 256, 2);
                            calc_idx   <= 4'd5;
                        end
                        4'd5: begin
                            crc_acc    <= flit_crc64_step(crc_acc, body_r[5*256 +: 256], 256);
                            fec_s[0]   <= fec_s[0]   ^ calc_fec_chunk_s(body_r[5*256 +: 256], 5*32, 256, 0);
                            fec_par[0] <= fec_par[0] ^ calc_fec_chunk_par(body_r[5*256 +: 256], 5*32, 256, 0);
                            fec_s[1]   <= fec_s[1]   ^ calc_fec_chunk_s(body_r[5*256 +: 256], 5*32, 256, 1);
                            fec_par[1] <= fec_par[1] ^ calc_fec_chunk_par(body_r[5*256 +: 256], 5*32, 256, 1);
                            fec_s[2]   <= fec_s[2]   ^ calc_fec_chunk_s(body_r[5*256 +: 256], 5*32, 256, 2);
                            fec_par[2] <= fec_par[2] ^ calc_fec_chunk_par(body_r[5*256 +: 256], 5*32, 256, 2);
                            calc_idx   <= 4'd6;
                        end
                        4'd6: begin
                            crc_acc    <= flit_crc64_step(crc_acc, body_r[6*256 +: 256], 256);
                            fec_s[0]   <= fec_s[0]   ^ calc_fec_chunk_s(body_r[6*256 +: 256], 6*32, 256, 0);
                            fec_par[0] <= fec_par[0] ^ calc_fec_chunk_par(body_r[6*256 +: 256], 6*32, 256, 0);
                            fec_s[1]   <= fec_s[1]   ^ calc_fec_chunk_s(body_r[6*256 +: 256], 6*32, 256, 1);
                            fec_par[1] <= fec_par[1] ^ calc_fec_chunk_par(body_r[6*256 +: 256], 6*32, 256, 1);
                            fec_s[2]   <= fec_s[2]   ^ calc_fec_chunk_s(body_r[6*256 +: 256], 6*32, 256, 2);
                            fec_par[2] <= fec_par[2] ^ calc_fec_chunk_par(body_r[6*256 +: 256], 6*32, 256, 2);
                            calc_idx   <= 4'd7;
                        end
                        4'd7: begin
                            // last body chunk: 144 bits = bytes 224..241
                            // (TLP tail + DLP); finishes the CRC.
                            crc_acc    <= flit_crc64_step(crc_acc, {112'd0, body_r[1935:1792]}, 144);
                            body_crc_r <= ~flit_crc64_step(crc_acc, {112'd0, body_r[1935:1792]}, 144);
                            fec_s[0]   <= fec_s[0]   ^ calc_fec_chunk_s({112'd0, body_r[1935:1792]}, 224, 144, 0);
                            fec_par[0] <= fec_par[0] ^ calc_fec_chunk_par({112'd0, body_r[1935:1792]}, 224, 144, 0);
                            fec_s[1]   <= fec_s[1]   ^ calc_fec_chunk_s({112'd0, body_r[1935:1792]}, 224, 144, 1);
                            fec_par[1] <= fec_par[1] ^ calc_fec_chunk_par({112'd0, body_r[1935:1792]}, 224, 144, 1);
                            fec_s[2]   <= fec_s[2]   ^ calc_fec_chunk_s({112'd0, body_r[1935:1792]}, 224, 144, 2);
                            fec_par[2] <= fec_par[2] ^ calc_fec_chunk_par({112'd0, body_r[1935:1792]}, 224, 144, 2);
                            calc_idx   <= 4'd8;
                        end
                        4'd8: begin
                            // CRC-bytes chunk: finishes FEC (body_crc_r was
                            // latched by the 4'd7 arm above). fec*_final
                            // are the already-combined "next" values (see
                            // the wires above this always block).
                            flit_r   <= { 15'd0,
                                          {fec_par2_final ^ (^fec_s2_final), fec_s2_final},
                                          {fec_par1_final ^ (^fec_s1_final), fec_s1_final},
                                          {fec_par0_final ^ (^fec_s0_final), fec_s0_final},
                                          body_crc_r, body_r };
                            word_idx <= 3'd0;
                            state    <= ST_SEND;
                        end
                        default: calc_idx <= 4'd0;
                    endcase
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
