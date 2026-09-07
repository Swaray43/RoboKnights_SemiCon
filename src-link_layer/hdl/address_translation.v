// ============================================================
// Module : address_translation
// Layer  : Link layer (Engineer B — Module 3 of 9)
//
// Function:
//   Receives burst descriptors from the AR request splitter.
//   Computes the 64-bit PCIe host address for each descriptor:
//
//     pcie_addr = base_addr[stream_id] + session_offset
//
//   base_addr is a per-stream register programmed by the host
//   driver via Engineer C's config space block at DMA setup
//   time.  It represents the IOMMU-translated bus address of
//   the pinned host DMA buffer.
//
//   session_offset is extracted from TUSER[SOFF_HI:1] and
//   gives the byte offset within the session buffer.
//
//   If base_addr[stream_id] has not been programmed (reset
//   value = 0) xlat_error is pulsed and the descriptor is
//   dropped.  The TLP packetizer must not receive a descriptor
//   with an unprogrammed base address.
//
//   Two streams are supported:
//     stream_id=0 — SmartNIC path
//     stream_id=1 — ML accelerator path
//
// Architectural notes (change-report 2025):
//   - reqtype removed: always MWr in stream path
//   - TC and ATTR removed: fixed TC=0, ATTR=0
//   - BAR decode removed: outbound DMA writes do not need BAR
//     matching — host address comes from config-space registers
//   - IOMMU pass-through retained: base_addr IS the translated
//     bus address written by the host driver
//
// Parameters:
//   ADDR_W   : PCIe address width (default 64)
//   LEN_W    : length field width in DW (default 11)
//   TUSER_W  : TUSER width (default 26)
//   SOFF_W   : session_offset field width (default 24)
//   NUM_STREAMS : number of streams (default 2)
//
// Upstream interface (from AR request splitter):
//   s_addr        — full host address (pre-computed by splitter
//                   as base + offset for split sub-requests;
//                   for non-split, base_addr is still needed)
//   s_len_dw      — length in DW
//   s_tuser       — 26-bit TUSER: {tlast_flag, session_offset,
//                                   stream_id}
//   s_split_flag  — 1 if part of a split pair
//   s_sub_index   — 0=first, 1=second sub-descriptor
//   s_valid       — descriptor valid
//   s_ready       — output: ready to accept
//
// Downstream interface (to TLP packetizer):
//   m_pcie_addr   — translated 64-bit PCIe host address
//   m_len_dw      — length in DW (passed through)
//   m_stream_id   — stream identifier (passed through)
//   m_split_flag  — passed through
//   m_sub_index   — passed through
//   m_tlast_flag  — passed through from TUSER
//   m_valid       — descriptor valid
//   m_ready       — input: packetizer ready
//
// Config space register bus (from Engineer C):
//   cfg_base_addr_wr    — write enable pulse
//   cfg_base_addr_id    — which stream to write
//   cfg_base_addr_data  — 64-bit base address value
//   cfg_base_addr_clr   — synchronous clear all entries
//
// Error / status:
//   err_xlat_miss  — pulsed when stream_id has no base_addr
//   err_addr_ovf   — pulsed when base+offset overflows ADDR_W
//   xlat_busy      — high while a descriptor is being processed
// ============================================================

module address_translation #(
    parameter ADDR_W       = 64,
    parameter LEN_W        = 11,
    parameter TUSER_W      = 26,
    parameter SOFF_W       = 24,
    parameter NUM_STREAMS  = 2
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // upstream — from AR request splitter
    input  wire [ADDR_W-1:0]        s_addr,       // full addr (for split sub-req)
    input  wire [LEN_W-1:0]         s_len_dw,
    input  wire [TUSER_W-1:0]       s_tuser,
    input  wire                     s_split_flag,
    input  wire [1:0]               s_sub_index,
    input  wire                     s_valid,
    output wire                     s_ready,

    // downstream — to TLP packetizer (Module 5)
    output reg  [ADDR_W-1:0]        m_pcie_addr,
    output reg  [LEN_W-1:0]         m_len_dw,
    output reg                      m_stream_id,
    output reg                      m_split_flag,
    output reg  [1:0]               m_sub_index,
    output reg                      m_tlast_flag,
    output reg                      m_valid,
    input  wire                     m_ready,

    // config space register bus (driven by Engineer C)
    input  wire                     cfg_base_addr_wr,
    input  wire [$clog2(NUM_STREAMS)-1:0] cfg_base_addr_id,
    input  wire [ADDR_W-1:0]        cfg_base_addr_data,
    input  wire                     cfg_base_addr_clr,

    // error and status
    output reg                      err_xlat_miss,
    output reg                      err_addr_ovf,
    output wire                     xlat_busy
);

// ============================================================
// TUSER field extraction
// TUSER[0]              = stream_id
// TUSER[SOFF_W:1]       = session_offset  (SOFF_W bits)
// TUSER[TUSER_W-1]      = tlast_flag
// ============================================================
localparam SOFF_LO = 1;
localparam SOFF_HI = SOFF_W;   // inclusive: bits [SOFF_W:1]

wire                  s_stream_id     = s_tuser[0];
wire [SOFF_W-1:0]     s_sess_offset   = s_tuser[SOFF_HI:SOFF_LO];
wire                  s_tlast_flag    = s_tuser[TUSER_W-1];

// ============================================================
// Base address register file
// Two entries (one per stream), written by config space block.
// Dual-port style: Engineer C writes, this module reads.
// Reset value = 0 = unprogrammed sentinel.
// ============================================================
reg [ADDR_W-1:0] base_addr_reg [0:NUM_STREAMS-1];

integer ii;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (ii = 0; ii < NUM_STREAMS; ii = ii + 1)
            base_addr_reg[ii] <= {ADDR_W{1'b0}};
    end else if (cfg_base_addr_clr) begin
        for (ii = 0; ii < NUM_STREAMS; ii = ii + 1)
            base_addr_reg[ii] <= {ADDR_W{1'b0}};
    end else if (cfg_base_addr_wr) begin
        base_addr_reg[cfg_base_addr_id] <= cfg_base_addr_data;
    end
end

// ============================================================
// Address computation (combinational)
//
// For non-split descriptors:
//   pcie_addr = base_addr[stream_id] + session_offset
//
// For split sub-descriptors:
//   sub0 and sub1 addresses are already pre-computed by the
//   AR request splitter (s_addr carries the correct value).
//   We still validate that base_addr is programmed, but we
//   use s_addr directly rather than recomputing.
//
// Overflow detection:
//   Use a (ADDR_W+1)-bit adder. If bit ADDR_W is set,
//   base + offset wrapped — flag err_addr_ovf.
// ============================================================
wire [ADDR_W:0]   add_result;   // one extra bit for overflow
wire [ADDR_W-1:0] computed_addr;
wire              addr_ovf;
wire              base_unprog;  // base_addr not yet programmed

assign add_result    = {1'b0, base_addr_reg[s_stream_id]}
                     + {{(ADDR_W-SOFF_W){1'b0}}, s_sess_offset};
assign computed_addr = add_result[ADDR_W-1:0];
assign addr_ovf      = add_result[ADDR_W];
assign base_unprog   = (base_addr_reg[s_stream_id] == {ADDR_W{1'b0}});

// For split sub-descriptors the address is pre-computed
// by the splitter — use s_addr directly
wire [ADDR_W-1:0] xlat_addr = s_split_flag ? s_addr : computed_addr;

// ============================================================
// FSM states
// ============================================================
localparam ST_IDLE  = 2'd0;  // waiting for descriptor
localparam ST_XLAT  = 2'd1;  // address computed, output loaded
localparam ST_ERROR = 2'd2;  // xlat miss — holding error, drop

reg [1:0] fsm_state;

assign xlat_busy = (fsm_state != ST_IDLE);

// s_ready must be combinational, not registered: a "no idle bubble"
// design needs to accept a new descriptor in the very same cycle
// the current output is handed off to the packetizer. A registered
// s_ready can only announce acceptance one cycle after the fact,
// which either stalls the upstream source forever (it never sees
// s_ready go high for its pending descriptor) or, if pulsed a
// cycle late, causes the same descriptor to be captured twice by
// a downstream consumer since m_valid stays asserted throughout.
//
// Ready whenever idle, or whenever busy and the packetizer is
// accepting the current output this cycle (freeing us up).
assign s_ready = (fsm_state == ST_IDLE) ||
                  ((fsm_state == ST_XLAT) && m_ready);

// ============================================================
// FSM
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fsm_state     <= ST_IDLE;
        // s_ready is combinational and reads high immediately once
        // fsm_state resets to ST_IDLE (Bug 7 fix preserved).
        m_valid       <= 1'b0;
        m_pcie_addr   <= {ADDR_W{1'b0}};
        m_len_dw      <= {LEN_W{1'b0}};
        m_stream_id   <= 1'b0;
        m_split_flag  <= 1'b0;
        m_sub_index   <= 2'd0;
        m_tlast_flag  <= 1'b0;
        err_xlat_miss <= 1'b0;
        err_addr_ovf  <= 1'b0;
    end else begin
        // default: clear pulsed error flags
        err_xlat_miss <= 1'b0;
        err_addr_ovf  <= 1'b0;

        case (fsm_state)

            // ------------------------------------------------
            // ST_IDLE
            // Ready to accept a descriptor. Address computation
            // is combinational so the result is available in
            // the same cycle s_valid is seen.
            // ------------------------------------------------
            ST_IDLE: begin
                // s_ready is combinational (always high in ST_IDLE) —
                // see assign above.
                if (s_valid && s_ready) begin
                    if (base_unprog && !s_split_flag) begin
                        // base address not programmed for this
                        // stream — drop descriptor, flag error
                        err_xlat_miss <= 1'b1;
                        // stay in ST_IDLE: no output loaded
                        // s_ready stays high — ready for next

                    end else if (addr_ovf && !s_split_flag) begin
                        // base + offset overflows address space
                        err_addr_ovf <= 1'b1;
                        fsm_state    <= ST_ERROR;

                    end else begin
                        // translation successful — load output
                        m_pcie_addr  <= xlat_addr;
                        m_len_dw     <= s_len_dw;
                        m_stream_id  <= s_stream_id;
                        m_split_flag <= s_split_flag;
                        m_sub_index  <= s_sub_index;
                        m_tlast_flag <= s_tlast_flag;
                        m_valid      <= 1'b1;
                        fsm_state    <= ST_XLAT;
                    end
                end
            end

            // ------------------------------------------------
            // ST_XLAT
            // Output is valid. Wait for m_ready from packetizer.
            // On acceptance, check if a new descriptor is already
            // waiting (Bug 11 fix: eliminate idle bubble).
            // ------------------------------------------------
            ST_XLAT: begin
                // s_ready is combinational: high here exactly when
                // m_ready is high (see assign above), i.e. only when
                // we're about to free up this cycle.
                if (m_valid && m_ready) begin
                    // packetizer accepted this descriptor
                    if (s_valid) begin
                        // s_ready (comb) was high this same cycle
                        // (since m_ready is high here), so this
                        // descriptor is genuinely being accepted
                        // right now — process it in the same cycle
                        // the old output is handed off (no bubble)
                        if (base_unprog && !s_split_flag) begin
                            err_xlat_miss <= 1'b1;
                            m_valid       <= 1'b0;
                            fsm_state     <= ST_IDLE;

                        end else if (addr_ovf && !s_split_flag) begin
                            err_addr_ovf <= 1'b1;
                            m_valid      <= 1'b0;
                            fsm_state    <= ST_ERROR;

                        end else begin
                            // load next descriptor immediately
                            m_pcie_addr  <= xlat_addr;
                            m_len_dw     <= s_len_dw;
                            m_stream_id  <= s_stream_id;
                            m_split_flag <= s_split_flag;
                            m_sub_index  <= s_sub_index;
                            m_tlast_flag <= s_tlast_flag;
                            m_valid      <= 1'b1;
                            // stay in ST_XLAT
                        end
                    end else begin
                        // no next descriptor — return to idle
                        m_valid   <= 1'b0;
                        fsm_state <= ST_IDLE;
                    end
                end
            end

            // ------------------------------------------------
            // ST_ERROR
            // Address overflow detected. Hold one cycle to
            // ensure err_addr_ovf is visible, then return.
            // ------------------------------------------------
            ST_ERROR: begin
                // s_ready (comb) goes high automatically once
                // fsm_state returns to ST_IDLE below.
                m_valid   <= 1'b0;
                fsm_state <= ST_IDLE;
            end

        endcase
    end
end

// ============================================================
// Elaboration-time checks
// ============================================================
// synthesis translate_off
initial begin
    if (SOFF_W >= ADDR_W)
        $fatal(1, "[address_translation] SOFF_W(%0d) >= ADDR_W(%0d): session offset wider than address", SOFF_W, ADDR_W);
    if (TUSER_W < SOFF_W + 2)
        $fatal(1, "[address_translation] TUSER_W(%0d) too narrow for SOFF_W(%0d)+stream_id+tlast", TUSER_W, SOFF_W);
    if (NUM_STREAMS < 1 || NUM_STREAMS > 16)
        $fatal(1, "[address_translation] NUM_STREAMS(%0d) must be 1-16", NUM_STREAMS);
end
// synthesis translate_on

// ============================================================
// Simulation assertions
// ============================================================
`ifdef SIMULATION
always @(posedge clk) begin
    // xlat miss must not occur during a split sub-request
    // (base_addr is validated on the original descriptor)
    if (err_xlat_miss && s_split_flag)
        $error("[address_translation] xlat_miss fired on split sub-descriptor — base_addr validation should only apply to non-split descriptors");

    // m_valid must not assert while in ST_ERROR
    if (fsm_state == ST_ERROR && m_valid)
        $error("[address_translation] m_valid asserted during ST_ERROR — output register not cleared");

    // s_ready must never be high while m_valid is held un-consumed
    // (m_ready low) — that would let a new descriptor overwrite an
    // output the packetizer hasn't taken yet. s_ready and m_valid
    // being simultaneously high while m_ready is ALSO high is fine:
    // that's a legitimate same-cycle zero-bubble handoff.
    if (s_ready && m_valid && fsm_state != ST_IDLE && !m_ready)
        $warning("[address_translation] s_ready high while m_valid is stalled (m_ready low) — check FSM");
end
`endif

endmodule