`timescale 1ns/1ps
//=============================================================================
// axi4s_ingest_port.v
// One AXI4-Stream ingest port for the interconnect layer (instantiated once
// per channel: CH_ID=0 SmartNIC, CH_ID=1 ML).
//
// Responsibilities (per interconnect_layer_spec.md):
//   - Standard AXI4-Stream slave handshake (TVALID/TREADY/TDATA/TKEEP/TLAST).
//   - Tracks session_offset: running byte position within the current
//     session, starting at 0 on session_start, incrementing by the number
//     of valid bytes (via TKEEP popcount) per accepted beat.
//   - Enforces session_limit: a beat that would push session_offset past
//     the bound sets a STICKY session_overflow flag and aborts the session
//     (session_active drops, TREADY deasserts) rather than silently
//     wrapping or corrupting the next channel's buffer slice. This is the
//     "stall on overflow" policy from the open design questions -- NOT
//     wrap-around. Confirm this is the intended overflow policy before
//     relying on it; wrap-around or drop-and-flag would need a different
//     implementation.
//   - Does NOT know or care about the real host address, BAR decode, or
//     FLIT framing -- those live in Kshitij's and Swaray's layers per the
//     project split. This module only ever produces a small local offset.
//   - session_start / session_limit are expected to come from Kshitij's
//     config-space block (the "session-armed handshake" flagged as an
//     open question -- this module assumes such a signal exists; if it
//     doesn't yet, session_start must be tied to a safe default upstream).
//=============================================================================

module axi4s_ingest_port #(
    parameter DATA_WIDTH   = 512,
    parameter KEEP_WIDTH   = DATA_WIDTH/8,
    parameter OFFSET_WIDTH = 24,
    parameter CH_ID        = 0
)(
    input  wire                       clk,
    input  wire                       rstn,

    // ---- AXI4-Stream slave (from fabric source) ----
    input  wire [DATA_WIDTH-1:0]      s_tdata,
    input  wire [KEEP_WIDTH-1:0]      s_tkeep,
    input  wire                       s_tlast,
    input  wire                       s_tvalid,
    output wire                       s_tready,

    // ---- Session control (from config-space block) ----
    input  wire                       session_start,     // pulse: new base_addr landed, arm this channel
    input  wire [OFFSET_WIDTH-1:0]    session_limit,      // max bytes for this session (slice bound)
    output reg                        session_active,
    output reg                        session_overflow,   // sticky; cleared only by session_start

    // ---- To crossbar/arbiter ----
    output wire                       req,                // beat ready to send, this channel wants the bus
    input  wire                       grant,               // arbiter granted this channel this cycle
    input  wire                       ds_backpressure,     // shared downstream-full signal (CDC FIFO almost-full)

    output wire [DATA_WIDTH-1:0]      out_tdata,
    output wire [KEEP_WIDTH-1:0]      out_tkeep,
    output wire                       out_tlast,
    output wire [OFFSET_WIDTH-1:0]    out_session_offset,
    output wire                       accepted            // pulses when a real handshake completed this cycle
);

    // ---- TKEEP popcount: valid-byte count for this beat ----
    function [$clog2(KEEP_WIDTH+1)-1:0] popcount;
        input [KEEP_WIDTH-1:0] keep;
        integer i;
        begin
            popcount = 0;
            for (i = 0; i < KEEP_WIDTH; i = i + 1)
                popcount = popcount + keep[i];
        end
    endfunction

    // ------------------------------------------------------------------
    // X.1 (GDSII timing): TREADY no longer depends on TKEEP.
    //
    // Before, every beat ran  s_tkeep pin -> 64-bit popcount -> offset add
    // -> limit compare -> s_tready -> FIFO write enable  in one aclk cycle
    // (27 ns at the SKY130 slow corner). Now:
    //   * The offset is kept as off_base + bytes_q, where bytes_q is the
    //     registered byte count of the beat accepted last cycle. off_cur is
    //     the exact offset before the current beat (same value the old
    //     session_offset register held), built from flops only.
    //   * Fast path: if even a full beat (KEEP_WIDTH bytes) fits under the
    //     limit, the beat is accepted without looking at TKEEP.
    //   * Exact path, only within the last KEEP_WIDTH bytes of the limit:
    //     the beat is held one cycle (TREADY low; AXI4-Stream requires the
    //     source to keep it stable), its byte count is registered, and the
    //     next cycle it is accepted or the session aborts on overflow, with
    //     the same result as before.
    // Behaviour seen by the fabric is unchanged except one extra stall
    // cycle for a beat that lands within 64 bytes of the session limit.
    // ------------------------------------------------------------------
    localparam BW = $clog2(KEEP_WIDTH+1);

    reg  [OFFSET_WIDTH-1:0] off_base;
    reg  [BW-1:0]           bytes_q;      // bytes of the beat accepted last cycle (0 if none)
    wire [OFFSET_WIDTH-1:0] off_cur = off_base + bytes_q;

    reg                     ex_valid;     // exact path: byte count of the held beat is in ex_bytes
    reg  [BW-1:0]           ex_bytes;

    wire near     = ({1'b0, off_cur} + KEEP_WIDTH) > {1'b0, session_limit};          // a full beat might not fit
    wire ex_ovf   = ({1'b0, off_cur} + ex_bytes)   > {1'b0, session_limit};          // F1: one bit wider, no wrap
    wire fits     = !near || (ex_valid && !ex_ovf);
    wire would_overflow = session_active && s_tvalid && ex_valid && ex_ovf;

    // FIX: on overflow abort, send one zero-length TLAST marker (tkeep=0) down
    // the same path so the link layer closes the session in order (session_done
    // -> MSI-X, session_arm_ctrl -> idle) after every beat already accepted.
    reg abort_pend;
    assign req      = (s_tvalid && session_active && !session_overflow && fits) || abort_pend;
    // FIX: the offending beat must never handshake (grant is registered and may still be high)
    assign s_tready  = grant && session_active && !session_overflow && fits && !ds_backpressure && !abort_pend;

    wire beat_hs = s_tvalid && s_tready;   // AXI4-Stream handshake fires
    wire marker_hs = abort_pend && grant && !ds_backpressure;
    assign accepted = beat_hs || marker_hs;             // downstream must gate on this, not on grant alone --
                                            // grant can be high while backpressure/overflow still
                                            // blocks the actual handshake, in which case the master
                                            // is legitimately holding the SAME beat steady and must
                                            // not be forwarded again

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            session_active   <= 1'b0;
            session_overflow <= 1'b0;
            off_base         <= {OFFSET_WIDTH{1'b0}};
            bytes_q          <= {BW{1'b0}};
            ex_valid         <= 1'b0;
            ex_bytes         <= {BW{1'b0}};
            abort_pend       <= 1'b0;
        end else begin
            if (marker_hs) abort_pend <= 1'b0;
            off_base <= off_cur;                         // absorb last cycle's beat
            bytes_q  <= beat_hs ? popcount(s_tkeep) : {BW{1'b0}};
            // exact path: sample the held beat's byte count, keep it until the beat moves
            if (beat_hs || session_start || !session_active || !s_tvalid)
                ex_valid <= 1'b0;
            else if (near && !ex_valid) begin
                ex_valid <= 1'b1;
                ex_bytes <= popcount(s_tkeep);
            end
            if (session_start) begin
                session_active   <= 1'b1;
                session_overflow <= 1'b0;
                off_base         <= {OFFSET_WIDTH{1'b0}};
                bytes_q          <= {BW{1'b0}};
                ex_valid         <= 1'b0;
            end else if (would_overflow) begin
                session_overflow <= 1'b1;
                abort_pend       <= 1'b1;
                session_active   <= 1'b0;   // abort: hold TREADY low until re-armed
                ex_valid         <= 1'b0;
            end else if (beat_hs) begin
                if (s_tlast)
                    session_active <= 1'b0; // normal end of session
            end
        end
    end

    // Pass-through data path; only forwarded when actually accepted (beat_hs)
    // is implied by the crossbar only sampling this on a grant cycle.
    assign out_tdata          = s_tdata;
    assign out_tkeep           = abort_pend ? {KEEP_WIDTH{1'b0}} : s_tkeep;
    assign out_tlast           = abort_pend ? 1'b1 : s_tlast;
    assign out_session_offset  = off_cur;

endmodule
