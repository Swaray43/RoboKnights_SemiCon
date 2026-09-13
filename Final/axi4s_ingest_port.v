//=============================================================================
// axi4s_ingest_port.v
// One AXI4-Stream ingest port for the interconnect layer (instantiated once
// per channel: CH_ID=0 SmartNIC, CH_ID=1 ML).
//
// INTEGRATION UPDATE (session-arm handshake finalized):
//   - session_start (pulse) + session_limit (runtime port) are REMOVED.
//   - Replaced by session_valid: a LEVEL input from Kshitij's
//     session_arm_ctrl (protocol_PHY_layer/pcie_tag_man.v), held high for
//     the full ACTIVE duration of a session and low between sessions.
//     There is no separate "start" edge to detect: session_offset simply
//     resets to 0 whenever session_valid is low, so the first beat after
//     session_valid rises naturally begins at offset 0.
//   - SESSION_LIMIT is now a compile-time parameter (was runtime
//     ch0_session_limit/ch1_session_limit ports), per Kshitij's confirmed
//     answer that SLICE_SIZE is a Stage-1 compile-time constant.
//   - New output session_end: pulses on a normal TLAST-accepted beat or
//     an overflow abort. Feeds session_arm_ctrl.session_end so its FSM can
//     retire ACTIVE -> IDLE and allow the next session_arm.
//   - session_active output is REMOVED: it was purely a mirror of the
//     session_valid input, which the caller (Kshitij's side) already has.
//
// Responsibilities (per interconnect_layer_spec.md):
//   - Standard AXI4-Stream slave handshake (TVALID/TREADY/TDATA/TKEEP/TLAST).
//   - Tracks session_offset: running byte position within the current
//     session, starting at 0 whenever session_valid is low, incrementing
//     by the number of valid bytes (via TKEEP popcount) per accepted beat.
//   - Enforces SESSION_LIMIT: a beat that would push session_offset past
//     the bound sets a STICKY session_overflow flag and stalls (TREADY
//     deasserts) rather than silently wrapping or corrupting the next
//     channel's buffer slice. "Stall on overflow" policy -- confirmed by
//     Kshitij's ticket reply, answer #3.
//   - Does NOT know or care about the real host address, BAR decode, or
//     FLIT framing -- those live in Kshitij's and Swaray's layers per the
//     project split. This module only ever produces a small local offset.
//=============================================================================

module axi4s_ingest_port #(
    parameter DATA_WIDTH    = 512,
    parameter KEEP_WIDTH    = DATA_WIDTH/8,
    parameter OFFSET_WIDTH  = 24,
    parameter CH_ID         = 0,
    parameter [OFFSET_WIDTH-1:0] SESSION_LIMIT = {OFFSET_WIDTH{1'b1}}  // compile-time SLICE_SIZE; override per instance
)(
    input  wire                       clk,
    input  wire                       rstn,

    // ---- AXI4-Stream slave (from fabric source) ----
    input  wire [DATA_WIDTH-1:0]      s_tdata,
    input  wire [KEEP_WIDTH-1:0]      s_tkeep,
    input  wire                       s_tlast,
    input  wire                       s_tvalid,
    output wire                       s_tready,

    // ---- Session control (from Kshitij's session_arm_ctrl, this channel's instance) ----
    input  wire                       session_valid,       // level: high through the whole ACTIVE session
    output reg                        session_end,         // pulses: TLAST-accepted or overflow abort -> feeds session_arm_ctrl.session_end
    output reg                        session_overflow,    // sticky; cleared automatically when session_valid drops (next arm)

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

    wire [$clog2(KEEP_WIDTH+1)-1:0] beat_bytes = popcount(s_tkeep);

    reg [OFFSET_WIDTH-1:0] session_offset;

    wire would_overflow = session_valid && s_tvalid &&
                           ((session_offset + beat_bytes) > SESSION_LIMIT);

    assign req      = s_tvalid && session_valid && !session_overflow && !would_overflow;
    assign s_tready  = grant && session_valid && !session_overflow && !ds_backpressure;

    wire beat_hs = s_tvalid && s_tready;   // AXI4-Stream handshake fires
    assign accepted = beat_hs;             // downstream must gate on this, not on grant alone --
                                            // grant can be high while backpressure/overflow still
                                            // blocks the actual handshake, in which case the master
                                            // is legitimately holding the SAME beat steady and must
                                            // not be forwarded again

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            session_overflow <= 1'b0;
            session_offset   <= {OFFSET_WIDTH{1'b0}};
            session_end       <= 1'b0;
        end else begin
            session_end <= 1'b0;  // default: pulse only

            if (!session_valid) begin
                // Between sessions: offset and overflow reset, ready for
                // the next session_valid rising edge (next arm).
                session_offset   <= {OFFSET_WIDTH{1'b0}};
                session_overflow <= 1'b0;
            end else if (would_overflow) begin
                session_overflow <= 1'b1;
                session_end       <= 1'b1;  // tell session_arm_ctrl to retire ACTIVE -> IDLE
            end else if (beat_hs) begin
                session_offset <= session_offset + beat_bytes;
                if (s_tlast)
                    session_end <= 1'b1;    // normal end of session
            end
        end
    end

    // Pass-through data path; only forwarded when actually accepted (beat_hs)
    // is implied by the crossbar only sampling this on a grant cycle.
    assign out_tdata          = s_tdata;
    assign out_tkeep           = s_tkeep;
    assign out_tlast           = s_tlast;
    assign out_session_offset  = session_offset;

endmodule
