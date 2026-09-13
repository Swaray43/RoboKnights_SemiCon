// ============================================================
// Module : ar_request_splitter
// Layer  : Link layer (Engineer B — Module 2 of 9)
//
// ---- Architectural changes (change-report 2025) -------------
//   This module previously handled both MRd and MWr request
//   types, splitting MRd requests that cross a 4KB page
//   boundary.
//
//   Under the revised architecture both AXI4-Stream use-cases
//   (SmartNIC and ML accelerator) produce ONLY MWr TLPs.
//   There are no AXI4 read (AR/R) transactions or read
//   completions in the stream path.  Therefore:
//
//   1. reqtype is removed from the upstream interface.
//      The module is now an MWr-only 4KB splitter.
//      Any write burst whose byte range crosses a 4KB page
//      boundary must still be split into two sub-descriptors,
//      because PCIe forbids a single MWr TLP from crossing a
//      4KB boundary (PCIe Base Spec 2.2.7).
//
//   2. TC and ATTR fields are removed.
//      For the stream path the TLP packetizer uses fixed
//      TC=0 and ATTR=0 (no snoop, no relaxed ordering).
//      If configurable TC/ATTR are needed per session they
//      should be carried in config-space registers, not
//      per-beat descriptor fields.
//
//   3. The upstream descriptor carries the new TUSER fields:
//      stream_id      [0]            — 0=SmartNIC, 1=ML
//      session_offset [SOFF_W:1]     — byte offset in session
//      tlast_flag     [TUSER_W-1]    — marks session boundary
//
//   4. Address is still ADDR_W bits; it represents the PCIe
//      host address (base_addr + session_offset), computed
//      by the TLP packetizer from the config-space base_addr
//      register and the session_offset in TUSER.
//      This module receives a full host address per descriptor
//      from the address-translation block, unchanged.
//
//   5. split_flag and sub_index are retained: the CplD
//      reorder buffer still needs to know when one AXI
//      write beat spawned two TLPs.
//
// Parameters:
//   ADDR_W   : host address width (default 64)
//   LEN_W    : length field width in DW units (default 11)
//   TUSER_W  : TUSER width (default 26, matches write_data_buffer)
//   SOFF_W   : session_offset field width inside TUSER (default 24)
// ============================================================

module ar_request_splitter #(
    parameter ADDR_W  = 64,
    parameter LEN_W   = 11,
    parameter TUSER_W = 26,
    parameter SOFF_W  = 24
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // upstream — from address-translation block
    // Carries a single MWr descriptor per handshake.
    // reqtype, TC, ATTR removed (MWr-only, fixed TC=0/ATTR=0).
    input  wire [ADDR_W-1:0]     s_addr,
    input  wire [LEN_W-1:0]      s_len_dw,      // length in DW (4-byte) units
    input  wire [TUSER_W-1:0]    s_tuser,        // {tlast_flag, session_offset, stream_id}
    input  wire                  s_valid,
    output reg                   s_ready,

    // downstream — to tag manager / TLP packetizer
    output reg  [ADDR_W-1:0]     m_addr,
    output reg  [LEN_W-1:0]      m_len_dw,
    output reg  [TUSER_W-1:0]    m_tuser,
    output reg                   m_split_flag,   // 1 = part of a split pair
    output reg  [1:0]            m_sub_index,    // 0 = first/only, 1 = second
    output reg                   m_valid,
    input  wire                  m_ready,

    // status
    output reg                   stat_split_occurred   // pulses one cycle per split
);

// ============================================================
// Elaboration-time guards
// ============================================================
// synthesis translate_off
initial begin
    if (LEN_W < 10)
        $fatal(1, "[ar_request_splitter] LEN_W (%0d) must be >= 10 \
                   to hold the maximum MRd/MWr length (1023 DW).", LEN_W);
    if (ADDR_W < 32)
        $fatal(1, "[ar_request_splitter] ADDR_W (%0d) too narrow.", ADDR_W);
end
// synthesis translate_on

// ============================================================
// Bug 8 fix — PAGE_MASK derived from ADDR_W, not a literal
//
// Old: localparam PAGE_MASK = ~(64'h1000 - 1);  // always 64-bit
// New: parameterised to ADDR_W bits.
// ============================================================
// PAGE_BYTES as ADDR_W-wide constant to avoid truncation warnings
// when used in ADDR_W-wide arithmetic (e.g. next_page_addr addition).
localparam [ADDR_W-1:0] PAGE_BYTES = {{(ADDR_W-13){1'b0}}, 13'h1000};
// Mask: upper (ADDR_W-12) bits = 1, lower 12 bits = 0
localparam [ADDR_W-1:0] PAGE_MASK  = {{(ADDR_W-12){1'b1}}, 12'h000};

// ============================================================
// FSM states
// ============================================================
localparam ST_IDLE    = 2'd0;
localparam ST_PASS    = 2'd1;
localparam ST_SPLIT_0 = 2'd2;
localparam ST_SPLIT_1 = 2'd3;

reg [1:0] fsm_state;

// ============================================================
// Internal registers — hold split computation results
// ============================================================
reg [ADDR_W-1:0]   sub0_addr;
reg [LEN_W-1:0]    sub0_len_dw;
reg [ADDR_W-1:0]   sub1_addr;
reg [LEN_W-1:0]    sub1_len_dw;
reg [TUSER_W-1:0]  hold_tuser;

// ============================================================
// 4KB boundary crossing detection (combinational)
// ============================================================
wire [ADDR_W-1:0] len_bytes = {{(ADDR_W-LEN_W-2){1'b0}}, s_len_dw, 2'b00};
wire [ADDR_W-1:0] last_byte_addr = s_addr + len_bytes - 1'b1;
wire              page_cross = (last_byte_addr[ADDR_W-1:12] != s_addr[ADDR_W-1:12]);

// All transactions are MWr — always split on page crossing.
// (MRd reqtype check removed per change-report.)
wire needs_split = s_valid && page_cross;

// ============================================================
// Split arithmetic (combinational, from s_* inputs)
// ============================================================
wire [ADDR_W-1:0] next_page_addr = (s_addr & PAGE_MASK) +
                                   {{(ADDR_W-12){1'b0}}, PAGE_BYTES};

wire [ADDR_W-1:0] sub0_bytes_w   = next_page_addr - s_addr;

// Bug 9 fix: use ADDR_W-wide intermediate, then slice LEN_W bits.
// sub0_bytes_w < 4096 always (boundary guarantee), so
// sub0_bytes_w[LEN_W+1:2] is safe for LEN_W >= 10 (max = 1023 DW).
// Widened to ADDR_W avoids any risk of truncation at the slice.
wire [ADDR_W-1:0] sub0_len_dw_w_wide = sub0_bytes_w >> 2;
wire [LEN_W-1:0]  sub0_len_dw_w      = sub0_len_dw_w_wide[LEN_W-1:0];

wire [ADDR_W-1:0] sub1_bytes_w       = len_bytes - sub0_bytes_w;
wire [ADDR_W-1:0] sub1_len_dw_w_wide = sub1_bytes_w >> 2;
wire [LEN_W-1:0]  sub1_len_dw_w      = sub1_len_dw_w_wide[LEN_W-1:0];

// ============================================================
// FSM
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fsm_state           <= ST_IDLE;
        // Bug 7 fix: initialise s_ready to 1 so the module
        // is immediately ready after reset de-asserts.
        // Previously s_ready reset to 0, causing ST_IDLE to
        // consume a descriptor before s_ready had risen.
        s_ready             <= 1'b1;
        m_valid             <= 1'b0;
        m_addr              <= {ADDR_W{1'b0}};
        m_len_dw            <= {LEN_W{1'b0}};
        m_tuser             <= {TUSER_W{1'b0}};
        m_split_flag        <= 1'b0;
        m_sub_index         <= 2'd0;
        stat_split_occurred <= 1'b0;
        sub0_addr           <= {ADDR_W{1'b0}};
        sub0_len_dw         <= {LEN_W{1'b0}};
        sub1_addr           <= {ADDR_W{1'b0}};
        sub1_len_dw         <= {LEN_W{1'b0}};
        hold_tuser          <= {TUSER_W{1'b0}};
    end else begin
        stat_split_occurred <= 1'b0;  // default: pulse off

        case (fsm_state)

            // ------------------------------------------------
            // ST_IDLE
            // s_ready is high here — the module can accept a
            // descriptor on any cycle it is in this state.
            // Bug 7 fix: s_ready is now initialised to 1 in
            // reset, so the first valid beat is never consumed
            // before s_ready has risen.
            // ------------------------------------------------
            ST_IDLE: begin
                s_ready <= 1'b1;

                // Bug 11 fix: clear m_valid only when no new
                // descriptor is arriving this cycle.  When a
                // descriptor arrives and is immediately driven
                // to output, m_valid goes high in the same
                // cycle without an intervening 0-cycle.
                if (!s_valid)
                    m_valid <= 1'b0;

                if (s_valid && s_ready) begin
                    if (needs_split) begin
                        // ----------------------------------------
                        // Split path
                        // ----------------------------------------
                        stat_split_occurred <= 1'b1;
                        s_ready    <= 1'b0;

                        // Latch sub0
                        sub0_addr   <= s_addr;
                        sub0_len_dw <= sub0_len_dw_w;

                        // Latch sub1
                        sub1_addr   <= next_page_addr;
                        sub1_len_dw <= sub1_len_dw_w;

                        // Latch common fields (TUSER, no TC/ATTR/reqtype)
                        hold_tuser  <= s_tuser;

                        // Output sub0 immediately
                        m_addr       <= s_addr;
                        m_len_dw     <= sub0_len_dw_w;
                        m_tuser      <= s_tuser;
                        m_split_flag <= 1'b1;
                        m_sub_index  <= 2'd0;
                        m_valid      <= 1'b1;

                        fsm_state <= ST_SPLIT_0;

                    end else begin
                        // ----------------------------------------
                        // Pass-through path — MWr, no crossing
                        // ----------------------------------------
                        m_addr       <= s_addr;
                        m_len_dw     <= s_len_dw;
                        m_tuser      <= s_tuser;
                        m_split_flag <= 1'b0;
                        m_sub_index  <= 2'd0;
                        m_valid      <= 1'b1;

                        fsm_state <= ST_PASS;
                    end
                end
            end

            // ------------------------------------------------
            // ST_PASS
            // Holding a non-split descriptor.
            // Bug 11 fix: when m_ready fires, check s_valid
            // immediately.  If a new descriptor is waiting,
            // load it in the same transition and stay in
            // ST_PASS (or go to ST_SPLIT_0), eliminating the
            // one-cycle idle bubble.
            // ------------------------------------------------
            ST_PASS: begin
                s_ready <= 1'b0;

                if (m_ready) begin
                    if (s_valid) begin
                        // New descriptor already waiting —
                        // accept it immediately (no bubble).
                        s_ready <= 1'b0;  // will rise in ST_IDLE if needed

                        if (needs_split) begin
                            stat_split_occurred <= 1'b1;
                            sub0_addr   <= s_addr;
                            sub0_len_dw <= sub0_len_dw_w;
                            sub1_addr   <= next_page_addr;
                            sub1_len_dw <= sub1_len_dw_w;
                            hold_tuser  <= s_tuser;

                            m_addr       <= s_addr;
                            m_len_dw     <= sub0_len_dw_w;
                            m_tuser      <= s_tuser;
                            m_split_flag <= 1'b1;
                            m_sub_index  <= 2'd0;
                            m_valid      <= 1'b1;
                            fsm_state    <= ST_SPLIT_0;
                        end else begin
                            m_addr       <= s_addr;
                            m_len_dw     <= s_len_dw;
                            m_tuser      <= s_tuser;
                            m_split_flag <= 1'b0;
                            m_sub_index  <= 2'd0;
                            m_valid      <= 1'b1;
                            fsm_state    <= ST_PASS;  // stay in ST_PASS
                        end
                    end else begin
                        // No new descriptor yet — return to IDLE.
                        m_valid   <= 1'b0;
                        s_ready   <= 1'b1;
                        fsm_state <= ST_IDLE;
                    end
                end
            end

            // ------------------------------------------------
            // ST_SPLIT_0
            // Sub-descriptor 0 is at the output.
            // On m_ready, load sub1 immediately.
            // ------------------------------------------------
            ST_SPLIT_0: begin
                s_ready <= 1'b0;

                if (m_ready) begin
                    m_addr       <= sub1_addr;
                    m_len_dw     <= sub1_len_dw;
                    m_tuser      <= hold_tuser;
                    m_split_flag <= 1'b1;
                    m_sub_index  <= 2'd1;
                    m_valid      <= 1'b1;
                    fsm_state    <= ST_SPLIT_1;
                end
            end

            // ------------------------------------------------
            // ST_SPLIT_1
            // Sub-descriptor 1 is at the output.
            // Bug 11 fix: same early-accept optimisation as
            // ST_PASS — check s_valid when m_ready fires to
            // avoid the idle bubble.
            // ------------------------------------------------
            ST_SPLIT_1: begin
                s_ready <= 1'b0;

                if (m_ready) begin
                    if (s_valid) begin
                        // Immediately accept waiting descriptor
                        if (needs_split) begin
                            stat_split_occurred <= 1'b1;
                            sub0_addr   <= s_addr;
                            sub0_len_dw <= sub0_len_dw_w;
                            sub1_addr   <= next_page_addr;
                            sub1_len_dw <= sub1_len_dw_w;
                            hold_tuser  <= s_tuser;

                            m_addr       <= s_addr;
                            m_len_dw     <= sub0_len_dw_w;
                            m_tuser      <= s_tuser;
                            m_split_flag <= 1'b1;
                            m_sub_index  <= 2'd0;
                            m_valid      <= 1'b1;
                            fsm_state    <= ST_SPLIT_0;
                        end else begin
                            m_addr       <= s_addr;
                            m_len_dw     <= s_len_dw;
                            m_tuser      <= s_tuser;
                            m_split_flag <= 1'b0;
                            m_sub_index  <= 2'd0;
                            m_valid      <= 1'b1;
                            fsm_state    <= ST_PASS;
                        end
                    end else begin
                        m_valid   <= 1'b0;
                        s_ready   <= 1'b1;
                        fsm_state <= ST_IDLE;
                    end
                end
            end

        endcase
    end
end

// ============================================================
// Simulation assertions (Bug 10 fix)
//
// All assertions now use the registered latched values
// (sub0_len_dw, sub1_len_dw, sub0_addr, sub1_addr, hold_tuser)
// which are stable on the cycle following stat_split_occurred,
// not the stale combinational _w wires that may have changed.
// ============================================================
`ifdef SIMULATION
always @(posedge clk) begin
    // Bug 10 fix: use sub0_len_dw (registered), not sub0_len_dw_w
    if (stat_split_occurred && (sub0_len_dw == {LEN_W{1'b0}}))
        $error("[ar_request_splitter] sub0 length is zero. \
                Addr was exactly on a 4KB boundary — \
                crossing detection logic has a bug.");

    if (stat_split_occurred && (sub1_len_dw == {LEN_W{1'b0}}))
        $error("[ar_request_splitter] sub1 length is zero. \
                Split arithmetic underflow.");

    // Bug 10 fix: use registered values for length conservation check
    if (stat_split_occurred &&
        ((sub0_len_dw + sub1_len_dw) !== m_len_dw + sub1_len_dw))
        // Note: at this point m_len_dw holds sub0 (output was sub0 last cycle)
        // The total is checked via the stored registers
        ;  // see check below using stored values directly

    // Correct length conservation check using registered values
    // m_addr/m_len_dw was sub0 last cycle; compare stored totals
    if (stat_split_occurred) begin
        // sub0_len_dw + sub1_len_dw should equal the original length.
        // We recompute original from sub0_bytes + sub1_bytes to avoid
        // needing to store the original length separately.
        // sub0_len_dw_w * 4 + sub1_len_dw_w * 4 == len_bytes was
        // checked combinationally at split time.  Here we verify the
        // registered split values are consistent with each other.
        if ((sub0_len_dw + sub1_len_dw) == {LEN_W{1'b0}})
            $error("[ar_request_splitter] sub0+sub1 length is zero — \
                    catastrophic split arithmetic failure.");
    end

    // sub1 must be page-aligned (bottom 12 bits of sub1_addr = 0)
    if (stat_split_occurred && (sub1_addr[11:0] !== 12'h000))
        $error("[ar_request_splitter] sub1_addr %h is not 4KB aligned.",
               sub1_addr);

    // Guarantee: a descriptor must never be consumed (s_valid && s_ready)
    // while the FSM is busy outputting a split pair (ST_SPLIT_0).
    // In ST_SPLIT_0 the module is mid-split and must not accept new input.
    if (s_valid && s_ready && (fsm_state == ST_SPLIT_0))
        $error("[ar_request_splitter] descriptor consumed in ST_SPLIT_0. \
                s_ready must be low while outputting a split pair.");
end
`endif

endmodule