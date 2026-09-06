// ============================================================
// Module : ar_request_splitter
// Layer  : Link layer (Engineer B — Module 2 of 9)
//
// Function:
//   Receives burst descriptors from the write data buffer
//   descriptor bus. For MRd (reqtype=01) requests that cross
//   a 4KB page boundary, splits the descriptor into two
//   sequential sub-descriptors:
//     Sub 0: addr=original addr, len=bytes to 4KB boundary
//     Sub 1: addr=next 4KB page base, len=remaining bytes
//   MWr (reqtype=00) descriptors pass through unmodified
//   in a single cycle. Non-crossing MRd also passes through
//   in a single cycle.
//
//   split_flag is asserted on both sub-descriptors of a split
//   so the downstream tag manager and reorder buffer know to
//   expect two completions for one original AXI request.
//
//   sub_index=0 on the first sub-descriptor (or on any
//   non-split descriptor). sub_index=1 on the second.
//
// AXI4 note:
//   No write data interleaving is possible (AXI4 guarantee).
//   Descriptors arrive one at a time in order. This module
//   processes one descriptor at a time with no pipelining
//   across descriptor boundaries.
//
// Parameters:
//   ADDR_W   : address width (default 64)
//   LEN_W    : length field width in DW units (default 10)
//   TC_W     : traffic class width (default 3)
//   ATTR_W   : attribute field width (default 3)
//   RTYPE_W  : reqtype width (default 2)
// ============================================================

module ar_request_splitter #(
    parameter ADDR_W  = 64,
    parameter LEN_W   = 10,
    parameter TC_W    = 3,
    parameter ATTR_W  = 3,
    parameter RTYPE_W = 2
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // upstream — from write data buffer descriptor bus
    input  wire [ADDR_W-1:0]    s_addr,
    input  wire [LEN_W-1:0]     s_len_dw,   // length in DW (4-byte) units
    input  wire [TC_W-1:0]      s_tc,
    input  wire [ATTR_W-1:0]    s_attr,
    input  wire [RTYPE_W-1:0]   s_reqtype,
    input  wire                 s_valid,
    output reg                  s_ready,

    // downstream — to tag manager / TLP packetizer
    output reg  [ADDR_W-1:0]    m_addr,
    output reg  [LEN_W-1:0]     m_len_dw,
    output reg  [TC_W-1:0]      m_tc,
    output reg  [ATTR_W-1:0]    m_attr,
    output reg  [RTYPE_W-1:0]   m_reqtype,
    output reg                  m_split_flag,  // 1 = part of a split pair
    output reg  [1:0]           m_sub_index,   // 0 = first, 1 = second
    output reg                  m_valid,
    input  wire                 m_ready,

    // status
    output reg                  stat_split_occurred  // pulses one cycle per split
);

// ============================================================
// Local parameters
// ============================================================
localparam RTYPE_MWR = 2'b00;
localparam RTYPE_MRD = 2'b01;

// 4KB page size in bytes and in DW
localparam PAGE_BYTES = 64'h0000_0000_0000_1000;  // 4096
localparam PAGE_MASK  = ~(PAGE_BYTES - 1);         // upper bits of page base

// ============================================================
// FSM states
// ============================================================
localparam ST_IDLE    = 2'd0;  // waiting for a descriptor
localparam ST_PASS    = 2'd1;  // outputting a non-split descriptor
localparam ST_SPLIT_0 = 2'd2;  // outputting first sub-descriptor of a split
localparam ST_SPLIT_1 = 2'd3;  // outputting second sub-descriptor of a split

reg [1:0] fsm_state;

// ============================================================
// Internal registers — hold split computation results
// ============================================================
reg [ADDR_W-1:0]  sub0_addr;
reg [LEN_W-1:0]   sub0_len_dw;
reg [ADDR_W-1:0]  sub1_addr;
reg [LEN_W-1:0]   sub1_len_dw;
reg [TC_W-1:0]    hold_tc;
reg [ATTR_W-1:0]  hold_attr;
reg [RTYPE_W-1:0] hold_reqtype;

// ============================================================
// 4KB boundary crossing detection (combinational)
//
// A crossing occurs when the last byte of the request is on
// a different 4KB page than the first byte.
//
// last_byte_addr = addr + (len_dw * 4) - 1
// crossing if last_byte_addr[63:12] != addr[63:12]
//
// len_bytes = len_dw << 2  (multiply by 4)
// ============================================================
wire [ADDR_W-1:0] len_bytes      = {{(ADDR_W-LEN_W-2){1'b0}}, s_len_dw, 2'b00};
wire [ADDR_W-1:0] last_byte_addr = s_addr + len_bytes - 1'b1;
wire              page_cross      = (last_byte_addr[ADDR_W-1:12] != s_addr[ADDR_W-1:12]);

// Only MRd requests are split — MWr goes straight through
wire needs_split = s_valid && (s_reqtype == RTYPE_MRD) && page_cross;

// ============================================================
// Split arithmetic (combinational, computed from s_* inputs)
//
// sub0: starts at s_addr, ends at the 4KB boundary
//   sub0_addr    = s_addr
//   next_page    = (s_addr & PAGE_MASK) + PAGE_BYTES
//   sub0_bytes   = next_page - s_addr
//   sub0_len_dw  = sub0_bytes >> 2
//
// sub1: starts at next_page, covers the remainder
//   sub1_addr    = next_page
//   sub1_bytes   = len_bytes - sub0_bytes
//   sub1_len_dw  = sub1_bytes >> 2
// ============================================================
wire [ADDR_W-1:0] next_page_addr  = (s_addr & PAGE_MASK) + PAGE_BYTES;
wire [ADDR_W-1:0] sub0_bytes_w    = next_page_addr - s_addr;
wire [LEN_W-1:0]  sub0_len_dw_w   = sub0_bytes_w[LEN_W+1:2];  // >> 2
wire [ADDR_W-1:0] sub1_bytes_w    = len_bytes - sub0_bytes_w;
wire [LEN_W-1:0]  sub1_len_dw_w   = sub1_bytes_w[LEN_W+1:2];  // >> 2

// ============================================================
// FSM
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fsm_state        <= ST_IDLE;
        s_ready          <= 1'b0;
        m_valid          <= 1'b0;
        m_addr           <= {ADDR_W{1'b0}};
        m_len_dw         <= {LEN_W{1'b0}};
        m_tc             <= {TC_W{1'b0}};
        m_attr           <= {ATTR_W{1'b0}};
        m_reqtype        <= {RTYPE_W{1'b0}};
        m_split_flag     <= 1'b0;
        m_sub_index      <= 2'd0;
        stat_split_occurred <= 1'b0;
        sub0_addr        <= {ADDR_W{1'b0}};
        sub0_len_dw      <= {LEN_W{1'b0}};
        sub1_addr        <= {ADDR_W{1'b0}};
        sub1_len_dw      <= {LEN_W{1'b0}};
        hold_tc          <= {TC_W{1'b0}};
        hold_attr        <= {ATTR_W{1'b0}};
        hold_reqtype     <= {RTYPE_W{1'b0}};
    end else begin
        stat_split_occurred <= 1'b0;  // default pulse off

        case (fsm_state)

            // ----------------------------------------------------
            // ST_IDLE
            // Waiting for a valid descriptor from upstream.
            // Compute split arithmetic combinationally and decide
            // which state to go to in the same cycle.
            // s_ready is high — we can accept a descriptor.
            // ----------------------------------------------------
            ST_IDLE: begin
                s_ready <= 1'b1;
                m_valid <= 1'b0;

                if (s_valid) begin
                    if (needs_split) begin
                        // ----------------------------------------
                        // Split path — latch both sub-descriptors
                        // and move to ST_SPLIT_0 to output them
                        // sequentially. De-assert s_ready so
                        // upstream stalls while we output two
                        // descriptors from one input.
                        // ----------------------------------------
                        stat_split_occurred <= 1'b1;
                        s_ready    <= 1'b0;

                        // latch sub0
                        sub0_addr   <= s_addr;
                        sub0_len_dw <= sub0_len_dw_w;

                        // latch sub1
                        sub1_addr   <= next_page_addr;
                        sub1_len_dw <= sub1_len_dw_w;

                        // latch common fields
                        hold_tc      <= s_tc;
                        hold_attr    <= s_attr;
                        hold_reqtype <= s_reqtype;

                        // output sub0 immediately this cycle
                        m_addr       <= s_addr;
                        m_len_dw     <= sub0_len_dw_w;
                        m_tc         <= s_tc;
                        m_attr       <= s_attr;
                        m_reqtype    <= s_reqtype;
                        m_split_flag <= 1'b1;
                        m_sub_index  <= 2'd0;
                        m_valid      <= 1'b1;

                        fsm_state <= ST_SPLIT_0;

                    end else begin
                        // ----------------------------------------
                        // Pass-through path — no split needed.
                        // Output the descriptor directly this cycle.
                        // MWr always comes here.
                        // Non-crossing MRd also comes here.
                        // ----------------------------------------
                        m_addr       <= s_addr;
                        m_len_dw     <= s_len_dw;
                        m_tc         <= s_tc;
                        m_attr       <= s_attr;
                        m_reqtype    <= s_reqtype;
                        m_split_flag <= 1'b0;
                        m_sub_index  <= 2'd0;
                        m_valid      <= 1'b1;

                        fsm_state <= ST_PASS;
                    end
                end
            end

            // ----------------------------------------------------
            // ST_PASS
            // Holding a non-split descriptor at the output.
            // Wait for m_ready from downstream before returning
            // to IDLE and accepting the next descriptor.
            // ----------------------------------------------------
            ST_PASS: begin
                s_ready <= 1'b0;  // hold off upstream until downstream accepts

                if (m_ready) begin
                    m_valid   <= 1'b0;
                    s_ready   <= 1'b1;
                    fsm_state <= ST_IDLE;
                end
            end

            // ----------------------------------------------------
            // ST_SPLIT_0
            // Sub-descriptor 0 is at the output (already loaded
            // in ST_IDLE). Wait for m_ready, then immediately
            // load sub-descriptor 1 and move to ST_SPLIT_1.
            // ----------------------------------------------------
            ST_SPLIT_0: begin
                s_ready <= 1'b0;

                if (m_ready) begin
                    // sub0 accepted — output sub1 immediately
                    m_addr       <= sub1_addr;
                    m_len_dw     <= sub1_len_dw;
                    m_tc         <= hold_tc;
                    m_attr       <= hold_attr;
                    m_reqtype    <= hold_reqtype;
                    m_split_flag <= 1'b1;
                    m_sub_index  <= 2'd1;
                    m_valid      <= 1'b1;

                    fsm_state <= ST_SPLIT_1;
                end
            end

            // ----------------------------------------------------
            // ST_SPLIT_1
            // Sub-descriptor 1 is at the output.
            // Wait for m_ready, then return to IDLE.
            // Re-assert s_ready so upstream can send next
            // descriptor.
            // ----------------------------------------------------
            ST_SPLIT_1: begin
                s_ready <= 1'b0;

                if (m_ready) begin
                    m_valid   <= 1'b0;
                    s_ready   <= 1'b1;
                    fsm_state <= ST_IDLE;
                end
            end

        endcase
    end
end

// ============================================================
// Simulation assertions
// ============================================================
`ifdef SIMULATION
always @(posedge clk) begin
    // sub0 length must never be zero — would mean addr is
    // exactly on a 4KB boundary, which cannot cross
    if (stat_split_occurred && (sub0_len_dw_w == 0))
        $error("[ar_request_splitter] sub0 length is zero. \
                Address is exactly on a 4KB boundary — \
                crossing detection logic has a bug.");

    // sub1 length must never be zero
    if (stat_split_occurred && (sub1_len_dw_w == 0))
        $error("[ar_request_splitter] sub1 length is zero. \
                Split arithmetic underflow.");

    // sub0 + sub1 must equal original length
    if (stat_split_occurred &&
        (sub0_len_dw_w + sub1_len_dw_w !== s_len_dw))
        $error("[ar_request_splitter] sub0_len + sub1_len != original len. \
                Split arithmetic is incorrect.");

    // MWr must never be split
    if (stat_split_occurred && (s_reqtype == RTYPE_MWR))
        $error("[ar_request_splitter] MWr request was split. \
                Only MRd requests should be split.");
end
`endif

endmodule