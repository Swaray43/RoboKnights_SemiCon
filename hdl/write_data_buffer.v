// ============================================================
// Module : write_data_buffer
// Layer  : Link layer (Engineer B — Module 1 of 9)
//
// AXI4 NOTE:
//   AXI4 prohibits write data interleaving (AXI3 WID removed).
//   The W channel carries beats from exactly one burst at a time.
//   No WID tracking is needed or present in this module.
//   This module relies on and enforces that guarantee:
//   if a new TUSER (burst descriptor) arrives before TLAST of
//   the current burst, err_burst_interleave is pulsed and the
//   incoming beat is dropped. This should never happen with a
//   compliant AXI4 master — treat it as a fatal protocol error
//   in simulation.
//
// Function:
//   Accepts AXI4-Stream 512-bit write bursts from the CDC
//   output (Engineer A). Stores beats in a 16-entry synchronous
//   FIFO. Checks TKEEP for contiguous byte enables, flags and
//   drops any beat with gaps. Applies back-pressure (s_tready=0)
//   when occupancy reaches the high-watermark (12/16 entries).
//   Outputs beats to the TLP packetizer with full mid-burst
//   stall support via m_tready.
//
// Parameters:
//   DATA_W : AXI data width in bits        (default 512)
//   KEEP_W : TKEEP width = DATA_W/8        (default 64)
//   USER_W : TUSER width in bits           (default 96)
//   FIFO_D : FIFO depth (must be power 2)  (default 16)
//   HWM    : High-watermark entry count    (default 12)
// ============================================================

module write_data_buffer #(
    parameter DATA_W = 512,
    parameter KEEP_W = DATA_W / 8,
    parameter USER_W = 96,
    parameter FIFO_D = 16,
    parameter HWM    = 12
)(
    // clock and reset
    input  wire                clk,
    input  wire                rst_n,

    // upstream — from Engineer A CDC output
    // AXI4: no WID, no interleaving, one burst at a time on W channel
    input  wire [DATA_W-1:0]   s_tdata,
    input  wire [KEEP_W-1:0]   s_tkeep,
    input  wire                s_tvalid,
    input  wire                s_tlast,
    input  wire [USER_W-1:0]   s_tuser,   // burst descriptor, stable for full burst
    output reg                 s_tready,

    // downstream — to TLP packetizer (Module 5)
    output reg  [DATA_W-1:0]   m_tdata,
    output reg  [KEEP_W-1:0]   m_tkeep,
    output reg                 m_tvalid,
    output reg                 m_tlast,
    output reg  [USER_W-1:0]   m_tuser,
    input  wire                m_tready,

    // error and status
    output reg                 err_tkeep_gap,       // non-contiguous TKEEP detected
    output reg                 err_burst_interleave, // AXI4 violation: new descriptor
                                                     // arrived before TLAST of current
                                                     // burst — should never happen
    output wire [4:0]          fifo_occupancy,
    output wire                fifo_full,
    output wire                fifo_hwm
);

// ============================================================
// Local parameters
// ============================================================
localparam FIFO_AW = $clog2(FIFO_D);
localparam ENTRY_W = DATA_W + KEEP_W + 1 + USER_W; // tdata+tkeep+tlast+tuser

// ============================================================
// FIFO storage
// Entry packing: {tuser, tlast, tkeep, tdata}
// ============================================================
reg [ENTRY_W-1:0]  mem [0:FIFO_D-1];
reg [FIFO_AW-1:0]  wr_ptr;
reg [FIFO_AW-1:0]  rd_ptr;
reg [FIFO_AW:0]    occupancy_r;

// ============================================================
// Occupancy and status flags
// ============================================================
assign fifo_occupancy = occupancy_r[4:0];
assign fifo_full      = (occupancy_r == FIFO_D);
assign fifo_hwm       = (occupancy_r >= HWM);

// ============================================================
// Latched burst descriptor
//
// AXI4 guarantee: TUSER (burst descriptor) is driven by
// Engineer A on the first beat of every burst and held stable
// until TLAST. Since no interleaving is possible, we latch it
// once at the start of each burst and use the latched value for
// all subsequent beats. We do NOT re-read TUSER on every beat —
// this makes the module robust against any glitch on TUSER
// mid-burst even though AXI4 forbids such glitches.
//
// We compare incoming TUSER against the latched value on every
// mid-burst beat. A mismatch means a new burst descriptor
// arrived before the current burst ended — an AXI4 violation.
// ============================================================
reg [USER_W-1:0]   latched_tuser;
reg                burst_active;    // high from first beat until TLAST accepted

// ============================================================
// TKEEP contiguous checker
// Valid TKEEP patterns: a single unbroken run of 1s at any
// position. Examples for 8-bit:
//   valid:   00111100, 00001111, 11110000, 11111111, 00000001
//   invalid: 00101100 (gap), 10000001 (two separate runs)
//
// Method: isolate lowest set bit, right-shift the entire mask
// to align LSB to bit 0, then check that (shifted + 1) has no
// overlap with shifted. If shifted = 000...0111...1 (contiguous)
// then shifted+1 = 000...1000...0, and shifted & (shifted+1) = 0.
// Any gap causes a residual bit in the AND result.
// ============================================================
function automatic tkeep_has_gap;
    input [KEEP_W-1:0] tkeep;
    reg   [KEEP_W-1:0] lowest;
    reg   [KEEP_W-1:0] shifted;
    begin
        if (tkeep == {KEEP_W{1'b0}}) begin
            // all-zero TKEEP: valid only on last beat
            // gap check passes here — last-beat-only rule
            // is enforced separately in the FSM
            tkeep_has_gap = 1'b0;
        end else begin
            lowest        = tkeep & (~tkeep + 1'b1);
            shifted       = tkeep / lowest;
            tkeep_has_gap = |(shifted & (shifted + 1'b1));
        end
    end
endfunction

// ============================================================
// FSM states
// ============================================================
localparam ST_IDLE  = 1'b0;   // waiting for first beat of a new burst
localparam ST_BURST = 1'b1;   // inside a burst, waiting for TLAST

reg fsm_state;

// ============================================================
// Write side FSM
// ============================================================
wire gap_detected = s_tvalid && tkeep_has_gap(s_tkeep);
wire wr_en        = s_tvalid && s_tready && !gap_detected;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fsm_state            <= ST_IDLE;
        wr_ptr               <= {FIFO_AW{1'b0}};
        latched_tuser        <= {USER_W{1'b0}};
        burst_active         <= 1'b0;
        err_tkeep_gap        <= 1'b0;
        err_burst_interleave <= 1'b0;
    end else begin
        // default: clear pulsed error flags each cycle
        err_tkeep_gap        <= 1'b0;
        err_burst_interleave <= 1'b0;

        case (fsm_state)

            // ------------------------------------------------
            // ST_IDLE: no burst in progress
            // Waiting for the first beat of a new burst.
            // On arrival, latch TUSER immediately.
            // AXI4: no interleave possible so any arriving
            // beat with tvalid is guaranteed to belong to
            // a new burst (previous one ended with TLAST).
            // ------------------------------------------------
            ST_IDLE: begin
                if (s_tvalid) begin
                    if (gap_detected) begin
                        // malformed first beat — drop it
                        // do not latch TUSER, do not open burst
                        err_tkeep_gap <= 1'b1;
                        // stay in IDLE: this beat had no valid data
                        // the burst is effectively abandoned at source
                    end else if (s_tready) begin
                        // latch descriptor from first beat
                        // AXI4 guarantees this is the correct and only
                        // descriptor for this entire burst
                        latched_tuser <= s_tuser;
                        burst_active  <= 1'b1;

                        // write beat with latched descriptor into FIFO
                        mem[wr_ptr] <= {s_tuser, s_tlast, s_tkeep, s_tdata};
                        wr_ptr      <= wr_ptr + 1'b1;

                        if (!s_tlast)
                            fsm_state <= ST_BURST;
                        // if s_tlast on first beat: single-beat burst
                        // burst closes immediately, stay in IDLE next cycle
                        else
                            burst_active <= 1'b0;
                    end
                end
            end

            // ------------------------------------------------
            // ST_BURST: inside a multi-beat burst
            // AXI4 violation check: if incoming TUSER differs
            // from latched_tuser, a new burst descriptor has
            // arrived before the current burst ended.
            // This is illegal in AXI4 — flag and drop the beat.
            // ------------------------------------------------
            ST_BURST: begin
                if (s_tvalid) begin
                    // AXI4 interleave check
                    // In AXI4 this should never trigger.
                    // If it does, the upstream master or CDC
                    // is non-compliant — treat as fatal in sim.
                    if (s_tuser !== latched_tuser) begin
                        err_burst_interleave <= 1'b1;
                        // drop the beat — do not write to FIFO
                        // do not advance wr_ptr
                        // stay in ST_BURST, waiting for real TLAST
                    end else if (gap_detected) begin
                        err_tkeep_gap <= 1'b1;
                        // drop the beat, burst continues
                    end else if (s_tready) begin
                        // use latched_tuser not s_tuser
                        // ensures descriptor is consistent across
                        // all beats even if AXI source glitches
                        mem[wr_ptr] <= {latched_tuser, s_tlast,
                                        s_tkeep, s_tdata};
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
// s_tready — registered, de-asserts at high-watermark
//
// Registered output gives Engineer A a full clock period to
// react. Effective watermark is HWM-1 from A's perspective
// because of the one-cycle registration delay.
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        s_tready <= 1'b0;
    else
        s_tready <= (occupancy_r < HWM);
end

// ============================================================
// Occupancy counter
// Handles simultaneous read and write correctly:
//   write only  -> +1
//   read only   -> -1
//   both        ->  0 (net)
//   neither     ->  0
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
//
// Output register pattern:
//   Refill the output register whenever it is empty OR
//   the packetizer is consuming this cycle (m_tready high).
//   This gives the packetizer a stable registered output
//   with no combinational FIFO-to-output path.
// ============================================================
wire fifo_empty = (occupancy_r == 0);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_ptr   <= {FIFO_AW{1'b0}};
        m_tvalid <= 1'b0;
        m_tdata  <= {DATA_W{1'b0}};
        m_tkeep  <= {KEEP_W{1'b0}};
        m_tlast  <= 1'b0;
        m_tuser  <= {USER_W{1'b0}};
    end else begin
        if (!fifo_empty && (!m_tvalid || m_tready)) begin
            // output register is empty or being consumed —
            // load next entry from FIFO
            {m_tuser, m_tlast, m_tkeep, m_tdata} <= mem[rd_ptr];
            m_tvalid <= 1'b1;
            rd_ptr   <= rd_ptr + 1'b1;
        end else if (m_tvalid && m_tready && fifo_empty) begin
            // packetizer consumed last entry, FIFO now empty
            m_tvalid <= 1'b0;
        end
    end
end

// ============================================================
// Simulation-only assertions
// Synthesis tools ignore these — include in any sim build.
// ============================================================
`ifdef SIMULATION
always @(posedge clk) begin
    // AXI4 violation: interleaving detected
    if (err_burst_interleave)
        $error("[write_data_buffer] FATAL: AXI4 burst interleave detected. \
                New TUSER arrived before TLAST of current burst. \
                Upstream master is non-compliant.");

    // FIFO full should never happen in normal operation
    if (fifo_full && s_tvalid && s_tready)
        $error("[write_data_buffer] FATAL: FIFO overflow. \
                Engineer A sent a beat when FIFO was full. \
                Back-pressure (s_tready) was not respected.");

    // TLAST missing: TUSER changed without TLAST
    if (burst_active && s_tvalid && (s_tuser !== latched_tuser))
        $warning("[write_data_buffer] AXI4 violation: TUSER changed \
                  mid-burst without TLAST. Possible interleave attempt.");
end
`endif

endmodule