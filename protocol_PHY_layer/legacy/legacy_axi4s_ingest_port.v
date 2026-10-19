// LEGACY (I2): this was a second definition of axi4s_ingest_port (16-bit
// offsets, SLICE_SIZE). Aditya's interconnect_layer owns the real
// axi4s_ingest_port (24-bit offsets the link layer expects), and the two
// clashed when all three layers were compiled together. Renamed and moved out
// of rtl/ so it is no longer on the build file list. Kept for reference only.
// ------------------------------------------------------------------
// legacy_axi4s_ingest_port: per-channel AXI4-Stream ingest with TUSER
// assembly and shared-downstream backpressure.
// One instance per channel (2 total, per work_distribution.md).
// ------------------------------------------------------------------
module legacy_axi4s_ingest_port #(
    parameter DATA_WIDTH    = 512,  // 64B beat
    parameter STREAM_ID     = 1,    // compile-time per-channel constant
    parameter OFFSET_WIDTH  = 16,
    parameter SLICE_SIZE    = 16'hFFFF  // compile-time bound (open Q#2)
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // From session_arm_ctrl (this channel's instance)
    input  wire                     session_valid,
    output reg                      session_end,      // TLAST-accepted or overflow abort

    // From overflow module (see #3) -- combinational, same cycle
    input  wire                     overflow_abort,

    // Shared downstream backpressure source (from fifo_hwm, OR'd across channels upstream if needed)
    input  wire                     downstream_full,

    // AXI4-Stream slave side (host -> us)
    input  wire [DATA_WIDTH-1:0]    s_tdata,
    input  wire                     s_tvalid,
    input  wire                     s_tlast,
    output wire                     s_tready,

    // AXI4-Stream master side (us -> interconnect arbiter / Module 5)
    output reg  [DATA_WIDTH-1:0]    m_tdata,
    output reg                      m_tvalid,
    output reg                      m_tlast,
    output reg  [0:0]               m_tuser_stream_id,
    output reg  [OFFSET_WIDTH-1:0]  m_tuser_session_offset,

    output reg  [OFFSET_WIDTH-1:0]  session_offset_dbg  // visibility only
);

    reg [OFFSET_WIDTH-1:0] session_offset;

    // Beat only accepted if: session armed, downstream has room, not overflowing
    wire beat_fire = s_tvalid && s_tready;
    assign s_tready = session_valid && !downstream_full && !overflow_abort;

    always @(posedge clk) begin
        if (!rst_n) begin
            session_offset          <= {OFFSET_WIDTH{1'b0}};
            m_tvalid                <= 1'b0;
            m_tlast                 <= 1'b0;
            m_tdata                 <= {DATA_WIDTH{1'b0}};
            m_tuser_stream_id       <= STREAM_ID[0];
            m_tuser_session_offset  <= {OFFSET_WIDTH{1'b0}};
            session_end             <= 1'b0;
            session_offset_dbg      <= {OFFSET_WIDTH{1'b0}};
        end else begin
            // Default: no end pulse unless set below
            session_end <= 1'b0;

            if (!session_valid) begin
                // Between sessions: offset resets, ready for next LATCH
                session_offset <= {OFFSET_WIDTH{1'b0}};
		session_offset_dbg  <= {OFFSET_WIDTH{1'b0}};
                m_tvalid       <= 1'b0;
            end else if (beat_fire) begin
                m_tdata                <= s_tdata;
                m_tvalid               <= 1'b1;
                m_tlast                <= s_tlast;
                m_tuser_stream_id      <= STREAM_ID[0];
                m_tuser_session_offset <= session_offset;

                session_offset <= session_offset + 1'b1;
                session_offset_dbg <= session_offset + 1'b1;

                if (s_tlast)
                    session_end <= 1'b1;   // clean end -> arm FSM allows next arm
            end else begin
                m_tvalid <= 1'b0;
            end

            if (overflow_abort)
                session_end <= 1'b1;       // abort also clears the way for re-arm
        end
    end

endmodule
