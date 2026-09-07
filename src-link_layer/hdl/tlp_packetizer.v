module tlp_packetizer #(
    parameter MAX_PAYLOAD_SIZE_DW = 32,   // Default: 128 Bytes
    parameter REQ_ID              = 16'h0100
)(
    input  wire          clk,
    input  wire          rst_n,

    // Command Interface
    input  wire          req_valid,
    output wire          req_ready,
    input  wire          req_is_mwr, // 1: MWr64, 0: MRd64
    input  wire [63:0]   req_addr,   // Must be DW-aligned (addr[1:0]==2'b00)
    input  wire [9:0]    req_len,    // Total length in DW (1–1023)
    input  wire [9:0]    req_tag,    // 10-bit Extended Tag
    input  wire [3:0]    req_fbe,    // First Byte Enable
    input  wire [3:0]    req_lbe,    // Last Byte Enable

    // Payload Interface (Upstream AXI-Stream)
    input  wire [31:0]   s_tdata,
    input  wire          s_tvalid,
    output wire          s_tready,

    // Outbound TLP Stream
    output reg  [31:0]   m_tdata,
    output reg           m_tvalid,
    input  wire          m_tready,
    output reg           m_tlast
);

// ============================================================
// State machine encodings
// ============================================================
localparam IDLE    = 3'd0;
localparam HDR0    = 3'd1;
localparam HDR1    = 3'd2;
localparam HDR2    = 3'd3;
localparam HDR3    = 3'd4;
localparam PAYLOAD = 3'd5;

reg [2:0]  state;

// ============================================================
// Latched command fields
// ============================================================
reg [9:0]  rem_len;
reg [9:0]  orig_len;
reg [63:0] curr_addr;
reg [9:0]  dw_cnt;
reg        is_mwr;
reg [9:0]  tag_reg;
reg [3:0]  fbe_reg;
reg [3:0]  lbe_reg;

// ============================================================
// MPS / BE combinational signals
// ============================================================
wire [9:0] cur_tlp_len = (rem_len > MAX_PAYLOAD_SIZE_DW[9:0])
                         ? MAX_PAYLOAD_SIZE_DW[9:0]
                         : rem_len;

wire       is_first_tlp = (rem_len == orig_len);
wire       is_last_tlp  = (rem_len == cur_tlp_len);
wire [3:0] cur_fbe = is_first_tlp ? fbe_reg : 4'hF;
wire [3:0] cur_lbe = (cur_tlp_len == 10'd1) ? 4'h0 :
                     is_last_tlp            ? lbe_reg : 4'hF;

wire [7:0] fmt_type = is_mwr ? 8'h60 : 8'h20;

// ============================================================
// Interface handshake
// ============================================================
assign req_ready = (state == IDLE);
assign s_tready = (state == PAYLOAD) && (!m_tvalid || m_tready);

// ============================================================
// Next-cycle combinational values for header DWs
// ============================================================
reg [31:0] nxt_tdata;
reg        nxt_tvalid;
reg        nxt_tlast;

always @(*) begin
    nxt_tdata  = 32'h0;
    nxt_tvalid = 1'b0;
    nxt_tlast  = 1'b0;

    case (state)
        HDR0: begin
            nxt_tdata  = {fmt_type, tag_reg[9], 3'b000, tag_reg[8], 
                          1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 2'b00, 2'b00, cur_tlp_len};
            nxt_tvalid = 1'b1;
        end
        HDR1: begin
            nxt_tdata  = {REQ_ID, tag_reg[7:0], cur_lbe, cur_fbe};
            nxt_tvalid = 1'b1;
        end
        HDR2: begin
            nxt_tdata  = curr_addr[63:32];
            nxt_tvalid = 1'b1;
        end
        HDR3: begin
            nxt_tdata  = {curr_addr[31:2], 2'b00};
            nxt_tvalid = 1'b1;
            if (!is_mwr) nxt_tlast = 1'b1;
        end
        default: begin
            nxt_tdata  = 32'h0;
            nxt_tvalid = 1'b0;
            nxt_tlast  = 1'b0;
        end
    endcase
end

// ============================================================
// Sequential block — FSM & Outputs
// ============================================================
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state     <= IDLE;
        rem_len   <= 10'd0;
        orig_len  <= 10'd0;
        curr_addr <= 64'd0;
        dw_cnt    <= 10'd0;
        is_mwr    <= 1'b0;
        tag_reg   <= 10'd0;
        fbe_reg   <= 4'h0;
        lbe_reg   <= 4'h0;
        m_tdata   <= 32'h0;
        m_tvalid  <= 1'b0;
        m_tlast   <= 1'b0;
    end else begin
        case (state)

            IDLE: begin
                // Bug Fix: Only clear outputs when consumed by downstream.
                // Prevents dropping the final DW of a previous packet during backpressure.
                if (m_tvalid && m_tready) begin
                    m_tvalid <= 1'b0;
                    m_tlast  <= 1'b0;
                end
                
                if (req_valid) begin
                    rem_len   <= req_len;
                    orig_len  <= req_len;
                    curr_addr <= req_addr;
                    is_mwr    <= req_is_mwr;
                    tag_reg   <= req_tag;
                    fbe_reg   <= req_fbe;
                    lbe_reg   <= req_lbe;
                    state     <= HDR0;
                end
            end

            HDR0: begin
                if (!m_tvalid || m_tready) begin
                    m_tdata  <= nxt_tdata;
                    m_tvalid <= nxt_tvalid;
                    m_tlast  <= nxt_tlast;
                end
                if (m_tready || !m_tvalid) state <= HDR1;
            end

            HDR1: begin
                if (!m_tvalid || m_tready) begin
                    m_tdata  <= nxt_tdata;
                    m_tvalid <= nxt_tvalid;
                    m_tlast  <= nxt_tlast;
                end
                if (m_tready || !m_tvalid) state <= HDR2;
            end

            HDR2: begin
                if (!m_tvalid || m_tready) begin
                    m_tdata  <= nxt_tdata;
                    m_tvalid <= nxt_tvalid;
                    m_tlast  <= nxt_tlast;
                end
                if (m_tready || !m_tvalid) state <= HDR3;
            end

            HDR3: begin
                if (!m_tvalid || m_tready) begin
                    m_tdata  <= nxt_tdata;
                    m_tvalid <= nxt_tvalid;
                    m_tlast  <= nxt_tlast;
                end
                if (m_tready || !m_tvalid) begin
                    if (is_mwr) begin
                        dw_cnt   <= 10'd0;
                        state    <= PAYLOAD;
                        // Bug Fix: Removed explicit overrides forcing m_tvalid/m_tlast to 0
                    end else begin
                        if (rem_len > cur_tlp_len) begin
                            rem_len   <= rem_len - cur_tlp_len;
                            curr_addr <= curr_addr + {cur_tlp_len, 2'b00};
                            state     <= HDR0;
                        end else begin
                            state <= IDLE;
                        end
                    end
                end
            end

            PAYLOAD: begin
                if (s_tvalid && (!m_tvalid || m_tready)) begin
                    if (dw_cnt == cur_tlp_len - 10'd1) begin
                        m_tdata  <= s_tdata;
                        m_tvalid <= 1'b1;
                        m_tlast  <= 1'b1;

                        if (rem_len > cur_tlp_len) begin
                            rem_len   <= rem_len - cur_tlp_len;
                            curr_addr <= curr_addr + {cur_tlp_len, 2'b00};
                            dw_cnt    <= 10'd0;
                            state     <= HDR0;
                        end else begin
                            state <= IDLE;
                        end
                    end else begin
                        m_tdata  <= s_tdata;
                        m_tvalid <= 1'b1;
                        m_tlast  <= 1'b0;
                        dw_cnt   <= dw_cnt + 10'd1;
                    end
                end else if (m_tvalid && m_tready) begin
                    m_tvalid <= 1'b0;
                    m_tlast  <= 1'b0;
                end
            end

            default: begin
                state    <= IDLE;
                m_tvalid <= 1'b0;
                m_tlast  <= 1'b0;
            end

        endcase
    end
end
endmodule