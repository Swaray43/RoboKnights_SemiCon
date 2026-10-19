// ============================================================
// Module : ar_request_splitter
// Layer  : Link layer (Engineer B — Module 2 of 9)
//
// MWr-only 4KB boundary splitter. Any write whose byte range crosses a
// 4KB page boundary is split into two sub-descriptors, because PCIe
// forbids a single MWr TLP from crossing a 4KB boundary.
//
//   upstream TUSER: {tlast_flag, session_offset, stream_id}
//   sub0 = original address, bytes up to the boundary
//   sub1 = next page address, the remainder
//   split_flag / sub_index mark the pair.
//
// ---- Handshake fix (bridge review follow-up, "S1") -------------------
//   s_ready used to be a register that did not match when the FSM
//   actually took a descriptor:
//     * ST_PASS / ST_SPLIT_1 consumed s_valid whenever m_ready was high,
//       even though s_ready had been driven 0 that cycle. The upstream
//       (address_translation) saw no handshake, kept the same descriptor,
//       and the splitter took it again -> duplicated TLPs.
//     * ST_IDLE left s_ready=1 on the way into ST_PASS. In ST_PASS with
//       m_ready=0 the upstream saw a handshake but the splitter ignored
//       the descriptor -> dropped TLPs.
//   Simulation of the stream path showed both (TLPs missing from the 5th
//   beat on). s_ready is now combinational and is exactly the condition
//   under which the FSM loads a new descriptor:
//     s_ready = IDLE  ||  ((PASS || SPLIT_1) && m_ready)
//   Everything else (split arithmetic, outputs, states) is unchanged.
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
    input  wire [ADDR_W-1:0]     s_addr,
    input  wire [LEN_W-1:0]      s_len_dw,      // length in DW (4-byte) units
    input  wire [TUSER_W-1:0]    s_tuser,       // {tlast_flag, session_offset, stream_id}
    input  wire                  s_valid,
    output wire                  s_ready,       // combinational (S1 fix)

    // downstream — to TLP packetizer
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

// synthesis translate_off
initial begin
    if (LEN_W < 10)
        $fatal(1, "[ar_request_splitter] LEN_W (%0d) must be >= 10.", LEN_W);
    if (ADDR_W < 32)
        $fatal(1, "[ar_request_splitter] ADDR_W (%0d) too narrow.", ADDR_W);
end
// synthesis translate_on

localparam [ADDR_W-1:0] PAGE_BYTES = {{(ADDR_W-13){1'b0}}, 13'h1000};
localparam [ADDR_W-1:0] PAGE_MASK  = {{(ADDR_W-12){1'b1}}, 12'h000};

localparam ST_IDLE    = 2'd0;
localparam ST_PASS    = 2'd1;
localparam ST_SPLIT_0 = 2'd2;
localparam ST_SPLIT_1 = 2'd3;

reg [1:0] fsm_state;

reg [ADDR_W-1:0]   sub1_addr;
reg [LEN_W-1:0]    sub1_len_dw;
reg [TUSER_W-1:0]  hold_tuser;
reg [LEN_W-1:0]    sub0_len_dw;   // kept for the assertions below

// ------------------------------------------------------------
// 4KB boundary crossing detection (combinational)
// ------------------------------------------------------------
wire [ADDR_W-1:0] len_bytes      = {{(ADDR_W-LEN_W-2){1'b0}}, s_len_dw, 2'b00};
wire [ADDR_W-1:0] last_byte_addr = s_addr + len_bytes - 1'b1;
wire              page_cross     = (last_byte_addr[ADDR_W-1:12] != s_addr[ADDR_W-1:12]);
wire              needs_split    = s_valid && page_cross;

wire [ADDR_W-1:0] next_page_addr     = (s_addr & PAGE_MASK) + PAGE_BYTES;
wire [ADDR_W-1:0] sub0_bytes_w       = next_page_addr - s_addr;
wire [ADDR_W-1:0] sub0_len_dw_w_wide = sub0_bytes_w >> 2;
wire [LEN_W-1:0]  sub0_len_dw_w      = sub0_len_dw_w_wide[LEN_W-1:0];
wire [ADDR_W-1:0] sub1_bytes_w       = len_bytes - sub0_bytes_w;
wire [ADDR_W-1:0] sub1_len_dw_w_wide = sub1_bytes_w >> 2;
wire [LEN_W-1:0]  sub1_len_dw_w      = sub1_len_dw_w_wide[LEN_W-1:0];

// ------------------------------------------------------------
// Handshake (S1 fix): s_ready is exactly "the FSM loads s_* this cycle"
// ------------------------------------------------------------
assign s_ready = (fsm_state == ST_IDLE) ||
                 (((fsm_state == ST_PASS) || (fsm_state == ST_SPLIT_1)) && m_ready);

wire take = s_valid && s_ready;

// ------------------------------------------------------------
// FSM
// ------------------------------------------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        fsm_state           <= ST_IDLE;
        m_valid             <= 1'b0;
        m_addr              <= {ADDR_W{1'b0}};
        m_len_dw            <= {LEN_W{1'b0}};
        m_tuser             <= {TUSER_W{1'b0}};
        m_split_flag        <= 1'b0;
        m_sub_index         <= 2'd0;
        stat_split_occurred <= 1'b0;
        sub0_len_dw         <= {LEN_W{1'b0}};
        sub1_addr           <= {ADDR_W{1'b0}};
        sub1_len_dw         <= {LEN_W{1'b0}};
        hold_tuser          <= {TUSER_W{1'b0}};
    end else begin
        stat_split_occurred <= 1'b0;

        if (take) begin
            // ---- load a new descriptor (from IDLE, or back-to-back) ----
            m_addr  <= s_addr;
            m_tuser <= s_tuser;
            m_valid <= 1'b1;
            m_sub_index <= 2'd0;
            if (needs_split) begin
                stat_split_occurred <= 1'b1;
                sub0_len_dw  <= sub0_len_dw_w;
                sub1_addr    <= next_page_addr;
                sub1_len_dw  <= sub1_len_dw_w;
                hold_tuser   <= s_tuser;
                m_len_dw     <= sub0_len_dw_w;
                m_split_flag <= 1'b1;
                fsm_state    <= ST_SPLIT_0;
            end else begin
                m_len_dw     <= s_len_dw;
                m_split_flag <= 1'b0;
                fsm_state    <= ST_PASS;
            end
        end else begin
            case (fsm_state)
                ST_IDLE: m_valid <= 1'b0;

                ST_PASS, ST_SPLIT_1: begin
                    if (m_ready) begin
                        // output consumed, nothing new waiting
                        m_valid   <= 1'b0;
                        fsm_state <= ST_IDLE;
                    end
                end

                ST_SPLIT_0: begin
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
            endcase
        end
    end
end

`ifdef SIMULATION
always @(posedge clk) begin
    if (stat_split_occurred && (sub0_len_dw == {LEN_W{1'b0}}))
        $error("[ar_request_splitter] sub0 length is zero.");
    if (stat_split_occurred && (sub1_len_dw == {LEN_W{1'b0}}))
        $error("[ar_request_splitter] sub1 length is zero.");
    if (stat_split_occurred && (sub1_addr[11:0] !== 12'h000))
        $error("[ar_request_splitter] sub1_addr %h is not 4KB aligned.", sub1_addr);
    if (s_valid && s_ready && (fsm_state == ST_SPLIT_0))
        $error("[ar_request_splitter] descriptor consumed in ST_SPLIT_0.");
end
`endif

endmodule
