// ============================================================
// Module : write_data_buffer
// Layer  : Link layer (Engineer B — Module 1 of 9)
//
// ---- Architectural changes (change-report 2025) -------------
//   OLD TUSER (96-bit): {attr[2:0], tc[2:0], reqtype[1:0],
//                        addr64[63:0], padding}
//   NEW TUSER layout (see TUSER_W breakdown below):
//     [0]           stream_id      — 0=SmartNIC, 1=ML
//     [SOFF_HI:1]   session_offset — byte offset in session buffer
//                                    (increments by 64 per beat;
//                                     per-beat, NOT stable across beats)
//     [TUSER_W-1]   tlast_flag     — REDUNDANT MIRROR of the TLAST wire.
//                                    Carried in TUSER so that FIFO
//                                    entries are self-contained.
//                                    Downstream consumers connecting
//                                    m_tuser to a 26-bit port will
//                                    receive this bit; those that
//                                    already use m_tlast may ignore
//                                    bit TUSER_W-1.
//   Dropped fields: addr64, TC, ATTR, reqtype
//   (reqtype is always MWr for both AXI4-Stream use-cases;
//    addr64 is now base_addr + session_offset, computed
//    downstream by the TLP packetizer using the base_addr
//    register in the config-space block.)
//
//   TUSER_W is now a parameter (default 26):
//     1  stream_id        bit [0]
//     24 session_offset   bits [SOFF_W:1]  (per-beat, not latched)
//     1  tlast_flag       bit [TUSER_W-1]  (mirror of TLAST)
//   Total = 26 bits.  Set TUSER_W and SOFF_W together.
//
//   INTERFACE NOTE for downstream integrators (Point 3):
//   m_tuser is TUSER_W=26 bits wide.  If your block expects
//   OFFSET_WIDTH+1 = 25 bits, bit 25 (tlast_flag) is a
//   harmless redundant copy of m_tlast — either connect the
//   full 26 bits or ignore/mask bit 25.  The stream_id and
//   session_offset fields are in bits [0] and [SOFF_W:1]
//   respectively, and are unaffected by this extra bit.
//
// ---- Bug fixes (bug report 2025) ----------------------------
//   Bug 1 — fifo_occupancy port width is now parameterised
//            ($clog2(FIFO_D)+1 bits) so it does not truncate
//            when FIFO_D > 16.
//   Bug 2 — tkeep_has_gap rewritten without reg-division.
//            Uses subtraction-based contiguous-run check that
//            is fully synthesisable on Vivado / OpenRoad.
//   Bug 3 — s_tready de-asserts at (HWM-1) not HWM, closing
//            the one-cycle registration window that could
//            push occupancy one above HWM.
//            An initial_value parameter guard ensures
//            HWM < FIFO_D at elaboration time.
//   Bug 4 — overflow assertion now checks wr_en (the actual
//            write-enable), not s_tvalid && s_tready, so it
//            does not false-fire during the HWM lag cycle.
//   Bug 5 — gap_detected is gated by s_tready so error pulses
//            are only generated on beats that are actually
//            accepted (valid + ready). Spurious repeated pulses
//            during back-pressure are suppressed.
//   Bug 6 — all-zero TKEEP on a non-TLAST beat is now
//            explicitly flagged as err_tkeep_gap and the beat
//            is dropped.  The FSM no longer silently passes
//            zero-TKEEP beats to the TLP packetizer.
//
// Function:
//   Accepts AXI4-Stream 512-bit write bursts (MWr only) from
//   the CDC output (Engineer A). Stores beats in a synchronous
//   FIFO of depth FIFO_D. Checks TKEEP for contiguous byte
//   enables and for zero-TKEEP on non-last beats, flags and
//   drops any malformed beat. Applies back-pressure
//   (s_tready=0) when occupancy reaches the high-watermark.
//   Outputs beats to the TLP packetizer with full mid-burst
//   stall support via m_tready.
//
// Parameters:
//   DATA_W  : AXI data width in bits        (default 512)
//   KEEP_W  : TKEEP width = DATA_W/8        (default 64)
//   TUSER_W : TUSER width in bits           (default 26)
//   SOFF_W  : session_offset field width    (default 24)
//   FIFO_D  : FIFO depth (must be power 2)  (default 16)
//   HWM     : High-watermark entry count    (default 12)
//             Must satisfy: 0 < HWM < FIFO_D
// ============================================================

module write_data_buffer #(
    parameter DATA_W  = 512,
    parameter KEEP_W  = DATA_W / 8,
    parameter TUSER_W = 26,      // stream_id(1) + session_offset(24) + tlast_flag(1)
    parameter SOFF_W  = 24,      // session_offset field width inside TUSER
    parameter FIFO_D  = 16,
    parameter HWM     = 12
)(
    // clock and reset
    input  wire                  clk,
    input  wire                  rst_n,

    // upstream — from Engineer A CDC output (AXI4-Stream, MWr only)
    input  wire [DATA_W-1:0]     s_tdata,
    input  wire [KEEP_W-1:0]     s_tkeep,
    input  wire                  s_tvalid,
    input  wire                  s_tlast,
    // NEW TUSER layout: {tlast_flag, session_offset[SOFF_W-1:0], stream_id}
    // tlast_flag  = s_tuser[TUSER_W-1]
    // session_offset = s_tuser[TUSER_W-2 : 1]   (SOFF_W bits)
    // stream_id   = s_tuser[0]
    input  wire [TUSER_W-1:0]    s_tuser,
    output reg                   s_tready,

    // downstream — to TLP packetizer (Module 5)
    output reg  [DATA_W-1:0]     m_tdata,
    output reg  [KEEP_W-1:0]     m_tkeep,
    output reg                   m_tvalid,
    output reg                   m_tlast,
    output reg  [TUSER_W-1:0]    m_tuser,
    input  wire                  m_tready,

    // error and status
    output reg                   err_tkeep_gap,        // non-contiguous or zero-on-non-last TKEEP
    output reg                   err_burst_interleave,  // TUSER changed mid-burst (AXI4 violation)
    output wire [$clog2(FIFO_D):0] fifo_occupancy,     // Bug 1 fix: parameterised width
    output wire                  fifo_full,
    output wire                  fifo_hwm
);

// ============================================================
// Elaboration-time parameter guard (Bug 3)
// ============================================================
// synthesis translate_off
initial begin
    if (HWM >= FIFO_D) begin
        $fatal(1, "[write_data_buffer] HWM (%0d) must be < FIFO_D (%0d). \
                   With registered s_tready, HWM==FIFO_D allows one \
                   write past full.", HWM, FIFO_D);
    end
    if ((FIFO_D & (FIFO_D-1)) != 0) begin
        $fatal(1, "[write_data_buffer] FIFO_D (%0d) must be a power of 2.",
               FIFO_D);
    end
    if (TUSER_W != (1 + SOFF_W + 1)) begin
        $fatal(1, "[write_data_buffer] TUSER_W (%0d) must equal \
                   1+SOFF_W+1 = %0d.", TUSER_W, 1+SOFF_W+1);
    end
end
// synthesis translate_on

// ============================================================
// Local parameters
// ============================================================
localparam FIFO_AW = $clog2(FIFO_D);
// Entry packing: {tuser, tlast, tkeep, tdata}
localparam ENTRY_W = DATA_W + KEEP_W + 1 + TUSER_W;

// TUSER field positions (for documentation; not used as slices
// inside this module — we treat TUSER as opaque and latch whole)
localparam STREAM_ID_BIT  = 0;
localparam SOFF_LSB       = 1;
localparam SOFF_MSB       = SOFF_W;          // = TUSER_W - 2
localparam TLAST_FLAG_BIT = TUSER_W - 1;

// ============================================================
// FIFO storage
// ============================================================
reg [ENTRY_W-1:0]    mem [0:FIFO_D-1];
reg [FIFO_AW-1:0]    wr_ptr;
reg [FIFO_AW-1:0]    rd_ptr;
reg [FIFO_AW:0]      occupancy_r;

// ============================================================
// Occupancy and status flags (Bug 1 fix — width parameterised)
// ============================================================
assign fifo_occupancy = occupancy_r;                 // full width, no truncation
assign fifo_full      = (occupancy_r == FIFO_D[FIFO_AW:0]);
assign fifo_hwm       = (occupancy_r >= HWM[FIFO_AW:0]);

// ============================================================
// Latched burst descriptor
//
// The new TUSER carries {tlast_flag, session_offset, stream_id}.
//
// session_offset IS NOT stable across beats — it increments by
// 64 (one 512-bit beat = 64 bytes) for each successive beat from
// the same stream.  Every beat therefore carries a different
// session_offset, which is stored verbatim in the FIFO entry so
// the downstream TLP packetizer can reconstruct the exact PCIe
// host address for each DW segment without a separate counter.
//
// stream_id IS stable for the duration of one logical AXI4
// burst — it identifies which use-case channel (0=SmartNIC,
// 1=ML) generated the beat.  Changing stream_id mid-burst
// without an intervening TLAST is an AXI4 interleave violation.
//
// Accordingly:
//   latched_tuser[STREAM_ID_BIT] is captured from the first beat
//     and compared on every subsequent beat of the same burst.
//   The FULL s_tuser (including per-beat session_offset) is
//     stored in each FIFO entry — latched_tuser is NOT replayed.
//
// NOTE on TUSER_W=26 (Point 3 — interface spec):
//   Bit 25 (TUSER_W-1 = tlast_flag) is a redundant mirror of
//   the TLAST wire.  It is carried in TUSER so that FIFO entries
//   are self-contained (a downstream block can determine burst
//   boundaries without a separate tlast entry).  Swaray's block
//   (link_layer_tx) should connect all 26 bits; bit 25 may be
//   ignored if m_tlast is already used for the same purpose.
// ============================================================
reg [TUSER_W-1:0]  latched_tuser;
reg                burst_active;

// ============================================================
// TKEEP contiguous checker — Bug 2 fix
//
// Replaces the non-synthesisable reg-division.
//
// A valid TKEEP has exactly one contiguous run of 1s
// (possibly at any position).  The subtraction-based test:
//
//   lowest = tkeep & (-tkeep)          isolate the lowest set bit
//   gap    = tkeep & (tkeep + lowest)  if no gap, adding lowest
//                                       carries through all the
//                                       1s and leaves a 0 AND;
//                                       any gap leaves residue
//
// This is pure combinational arithmetic with no division.
// Synthesis-safe on Vivado and OpenRoad.
//
// Special case: all-zero TKEEP — valid ONLY on TLAST beats.
// We do NOT flag it as a gap here; the FSM checks it separately
// (Bug 6 fix).
// ============================================================
function automatic tkeep_has_gap;
    input [KEEP_W-1:0] tkeep;
    input              is_last;   // Bug 6: need TLAST context
    reg   [KEEP_W-1:0] lowest;
    begin
        if (tkeep == {KEEP_W{1'b0}}) begin
            // all-zero TKEEP: gap = 0 only on last beat
            // Bug 6 fix: non-last with zero TKEEP IS an error
            tkeep_has_gap = ~is_last;   // 1 = error if not last
        end else begin
            lowest        = tkeep & (~tkeep + 1'b1); // tkeep & (-tkeep)
            // Bug 2 fix: subtraction-based check, no division
            tkeep_has_gap = |(tkeep & (tkeep + lowest));
        end
    end
endfunction

// ============================================================
// FSM states
// ============================================================
localparam ST_IDLE  = 1'b0;
localparam ST_BURST = 1'b1;

reg fsm_state;

// ============================================================
// Write side combinational signals
//
// Bug 5 fix: gap_detected gated by s_tready.
// Without this gate, a beat presented while s_tready=0 would
// generate a spurious err_tkeep_gap pulse every cycle the
// upstream holds s_tvalid high during back-pressure.
// The error should only fire on cycles where a beat would
// actually be accepted (both valid and ready).
// ============================================================
wire gap_detected = s_tvalid && s_tready &&
                    tkeep_has_gap(s_tkeep, s_tlast);   // Bug 5 + Bug 6
wire wr_en        = s_tvalid && s_tready && !gap_detected;

// ============================================================
// Write side FSM
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fsm_state            <= ST_IDLE;
        wr_ptr               <= {FIFO_AW{1'b0}};
        latched_tuser        <= {TUSER_W{1'b0}};
        burst_active         <= 1'b0;
        err_tkeep_gap        <= 1'b0;
        err_burst_interleave <= 1'b0;
    end else begin
        err_tkeep_gap        <= 1'b0;   // default: pulse cleared each cycle
        err_burst_interleave <= 1'b0;

        case (fsm_state)

            // ------------------------------------------------
            // ST_IDLE — waiting for first beat of a new burst
            // ------------------------------------------------
            ST_IDLE: begin
                if (s_tvalid) begin
                    if (gap_detected) begin
                        // Malformed first beat — flag and drop.
                        // Do not latch TUSER; stay in IDLE.
                        // The burst is abandoned at the source side.
                        err_tkeep_gap <= 1'b1;
                    end else if (s_tready) begin
                        // Latch TUSER from the first beat.
                        // Only latched_tuser[STREAM_ID_BIT] is used
                        // in subsequent cycles (to detect cross-stream
                        // interleaving).  session_offset is per-beat
                        // and is taken from s_tuser on every beat
                        // individually — latched_tuser is never
                        // replayed into the FIFO.
                        latched_tuser <= s_tuser;
                        burst_active  <= 1'b1;

                        // Write first beat into FIFO using s_tuser
                        // directly (latched_tuser not yet valid on
                        // this same cycle).
                        mem[wr_ptr] <= {s_tuser, s_tlast, s_tkeep, s_tdata};
                        wr_ptr      <= wr_ptr + 1'b1;

                        if (!s_tlast)
                            fsm_state <= ST_BURST;
                        else
                            burst_active <= 1'b0;  // single-beat burst
                    end
                end
            end

            // ------------------------------------------------
            // ST_BURST — inside a multi-beat burst
            // ------------------------------------------------
            ST_BURST: begin
                if (s_tvalid) begin
                    // STREAM_ID (bit 0) must be constant for all beats
                    // of a single AXI4 burst.  session_offset (bits
                    // [SOFF_W:1]) legitimately changes every beat —
                    // it carries the per-beat byte offset and is stored
                    // verbatim via s_tuser, NOT replayed from
                    // latched_tuser.  Do NOT compare session_offset.
                    if (s_tuser[STREAM_ID_BIT] !== latched_tuser[STREAM_ID_BIT]) begin
                        err_burst_interleave <= 1'b1;
                        // Drop the offending beat; stay in ST_BURST
                        // waiting for TLAST from the original stream.
                        // The FIFO entry for this beat is silently
                        // discarded — the burst is corrupted but we
                        // do not enter an undefined state.
                    end else if (gap_detected) begin
                        // Bug 5+6: gap_detected already gated by s_tready
                        // and catches zero-TKEEP on non-last beats.
                        err_tkeep_gap <= 1'b1;
                        // Drop the beat; burst continues.
                    end else if (s_tready) begin
                        // Store the full incoming TUSER (including the
                        // per-beat session_offset) in the FIFO entry.
                        // latched_tuser is NOT used here — it only
                        // provided the stream_id check above.
                        mem[wr_ptr] <= {s_tuser, s_tlast, s_tkeep, s_tdata};
                        wr_ptr      <= wr_ptr + 1'b1;

                        if (s_tlast) begin
                            fsm_state    <= ST_IDLE;
                            burst_active <= 1'b0;
                        end
                    end
                end
            end

        endcase
    end
end

// ============================================================
// s_tready — registered, de-asserts at (HWM-1) not HWM
//
// Bug 3 fix: the registered output means upstream sees
// de-assertion one cycle after occupancy_r changes.
// Without the -1 correction, occupancy can reach HWM+1 when
// the upstream sends a beat in the cycle s_tready de-asserts.
// Gating at (HWM-1) keeps the maximum reachable occupancy at
// exactly HWM, well below FIFO_D.
//
// Effective back-pressure watermark seen by Engineer A = HWM-1.
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        s_tready <= 1'b0;
    else
        s_tready <= (occupancy_r < (HWM - 1));  // Bug 3 fix: -1 margin
end

// ============================================================
// Occupancy counter
// ============================================================
wire rd_en = m_tvalid && m_tready;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        occupancy_r <= {(FIFO_AW+1){1'b0}};
    end else begin
        case ({wr_en, rd_en})
            2'b10:   occupancy_r <= occupancy_r + 1'b1;
            2'b01:   occupancy_r <= occupancy_r - 1'b1;
            default: occupancy_r <= occupancy_r;
        endcase
    end
end

// ============================================================
// Read side — output register to TLP packetizer
// ============================================================
wire fifo_empty = (occupancy_r == {(FIFO_AW+1){1'b0}});

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_ptr   <= {FIFO_AW{1'b0}};
        m_tvalid <= 1'b0;
        m_tdata  <= {DATA_W{1'b0}};
        m_tkeep  <= {KEEP_W{1'b0}};
        m_tlast  <= 1'b0;
        m_tuser  <= {TUSER_W{1'b0}};
    end else begin
        if (!fifo_empty && (!m_tvalid || m_tready)) begin
            {m_tuser, m_tlast, m_tkeep, m_tdata} <= mem[rd_ptr];
            m_tvalid <= 1'b1;
            rd_ptr   <= rd_ptr + 1'b1;
        end else if (m_tvalid && m_tready && fifo_empty) begin
            m_tvalid <= 1'b0;
        end
    end
end

// ============================================================
// Simulation-only assertions
// ============================================================
`ifdef SIMULATION
always @(posedge clk) begin
    // AXI4 violation: stream_id changed mid-burst
    if (err_burst_interleave)
        $error("[write_data_buffer] FATAL: stream_id changed mid-burst. \
                TUSER[0] differs from latched value. \
                Upstream master or CDC is non-compliant.");

    // Bug 4 fix: check wr_en (actual write), not s_tvalid && s_tready,
    // to avoid false firing during the HWM-1 lag cycle.
    if (fifo_full && wr_en)
        $error("[write_data_buffer] FATAL: FIFO overflow. \
                Beat written when FIFO was full. \
                Back-pressure watermark logic has a bug.");

    // stream_id changed mid-burst without TLAST — cross-stream
    // interleave attempt. session_offset changes every beat and
    // is intentional; only stream_id is checked here.
    if (burst_active && s_tvalid &&
        (s_tuser[STREAM_ID_BIT] !== latched_tuser[STREAM_ID_BIT]))
        $warning("[write_data_buffer] AXI4 violation: stream_id changed \
                  mid-burst without TLAST. Cross-stream interleave \
                  attempt — beat will be dropped.");

    // Bug 6 check: zero TKEEP arriving on a non-last beat while
    // s_tready is high should have been caught by gap_detected,
    // but assert here as a second line of defence.
    if (s_tvalid && s_tready && !s_tlast &&
        (s_tkeep == {KEEP_W{1'b0}}))
        $error("[write_data_buffer] Zero TKEEP on non-TLAST beat. \
                Beat should have been dropped by gap_detected.");
end
`endif

endmodule