// ============================================================
// Module : write_data_buffer
// Layer  : Link layer (Engineer B — Module 1 of 9)
//
// Accepts AXI4-Stream 512-bit write bursts (MWr only) from the
// interconnect CDC output. Stores beats in a synchronous FIFO of depth
// FIFO_D. Checks TKEEP for contiguous byte enables and for zero-TKEEP on
// non-last beats, flags and drops any malformed beat. Applies
// back-pressure (s_tready=0) when occupancy reaches the high-watermark.
//
// TUSER (TUSER_W = 26):
//   [0]           stream_id      — 0=SmartNIC, 1=ML
//   [SOFF_W:1]    session_offset — byte offset in session buffer (per beat)
//   [TUSER_W-1]   tlast_flag     — redundant mirror of TLAST
//
// Earlier fixes retained: Bug 1-6 (occupancy width, synthesisable TKEEP
// gap check, HWM-1 back-pressure, overflow assertion on wr_en, gap error
// gated by s_tready, zero-TKEEP on non-last dropped) and Gap fix W1
// (per-channel burst tracking; err_burst_interleave tied to 0).
//
// ---- Read-side fix (bridge review follow-up, "W2") -------------------
//   occupancy_r counts every beat not yet handed downstream, INCLUDING
//   the one already sitting in the m_* output register. The read side
//   used "occupancy_r != 0" to decide whether mem[rd_ptr] held a beat.
//   With exactly one beat stored -- already in the output register --
//   the read side still saw occupancy 1, and on the handshake that
//   emptied the FIFO it loaded mem[rd_ptr]: an unwritten / stale entry,
//   presented as a new valid beat. Simulation showed this on every
//   drain (more beats out than in, old data re-sent, extra TLAST beats).
//   The read side now loads only when the RAM itself holds an unread
//   entry (wr_ptr != rd_ptr). occupancy_r, s_tready and the status
//   flags are unchanged. Pointers cannot alias because occupancy never
//   exceeds HWM < FIFO_D.
// ============================================================
`timescale 1ns/1ps
module write_data_buffer #(
    parameter DATA_W  = 512,
    parameter KEEP_W  = DATA_W / 8,
    parameter TUSER_W = 26,
    parameter SOFF_W  = 24,
    parameter FIFO_D  = 16,
    parameter HWM     = 12
)(
    input  wire                  clk,
    input  wire                  rst_n,
    input  wire [DATA_W-1:0]     s_tdata,
    input  wire [KEEP_W-1:0]     s_tkeep,
    input  wire                  s_tvalid,
    input  wire                  s_tlast,
    input  wire [TUSER_W-1:0]    s_tuser,
    output reg                   s_tready,
    output reg  [DATA_W-1:0]     m_tdata,
    output reg  [KEEP_W-1:0]     m_tkeep,
    output reg                   m_tvalid,
    output reg                   m_tlast,
    output reg  [TUSER_W-1:0]    m_tuser,
    input  wire                  m_tready,
    output reg                   err_tkeep_gap,
    output reg                   err_burst_interleave,
    output wire [$clog2(FIFO_D):0] fifo_occupancy,
    output wire                  fifo_full,
    output wire                  fifo_hwm
);
// synthesis translate_off
initial begin
    if (HWM >= FIFO_D)
        $fatal(1, "[write_data_buffer] HWM (%0d) must be < FIFO_D (%0d).", HWM, FIFO_D);
    if ((FIFO_D & (FIFO_D-1)) != 0)
        $fatal(1, "[write_data_buffer] FIFO_D (%0d) must be a power of 2.", FIFO_D);
    if (TUSER_W != (1 + SOFF_W + 1))
        $fatal(1, "[write_data_buffer] TUSER_W (%0d) must equal 1+SOFF_W+1 = %0d.", TUSER_W, 1+SOFF_W+1);
end
// synthesis translate_on

localparam FIFO_AW = $clog2(FIFO_D);
localparam ENTRY_W = DATA_W + KEEP_W + 1 + TUSER_W;
// Ticket G7 (Aditya + Swaray): STREAM_ID_BIT used to be re-derived here by
// hand; now sourced from shared/axis_tuser_defs.vh, the one place the
// TUSER layout is defined (see that file's header for the full writeup).
`include "axis_tuser_defs.vh"
reg [ENTRY_W-1:0]    mem [0:FIFO_D-1];
reg [FIFO_AW-1:0]    wr_ptr;
reg [FIFO_AW-1:0]    rd_ptr;
reg [FIFO_AW:0]      occupancy_r;
assign fifo_occupancy = occupancy_r;
assign fifo_full      = (occupancy_r == FIFO_D[FIFO_AW:0]);
assign fifo_hwm       = (occupancy_r >= HWM[FIFO_AW:0]);
reg burst_active_ch [0:1];
function automatic tkeep_has_gap;
    input [KEEP_W-1:0] tkeep;
    input              is_last;
    reg   [KEEP_W-1:0] lowest;
    begin
        if (tkeep == {KEEP_W{1'b0}}) begin
            tkeep_has_gap = ~is_last;
        end else begin
            lowest        = tkeep & (~tkeep + 1'b1);
            tkeep_has_gap = |(tkeep & (tkeep + lowest));
        end
    end
endfunction
wire gap_detected = s_tvalid && s_tready && tkeep_has_gap(s_tkeep, s_tlast);
wire wr_en        = s_tvalid && s_tready && !gap_detected;
wire ch_sel_w     = tuser_stream_id(s_tuser);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_ptr               <= {FIFO_AW{1'b0}};
        burst_active_ch[0]   <= 1'b0;
        burst_active_ch[1]   <= 1'b0;
        err_tkeep_gap        <= 1'b0;
        err_burst_interleave <= 1'b0;
    end else begin
        err_tkeep_gap        <= 1'b0;
        err_burst_interleave <= 1'b0;
        if (s_tvalid) begin
            if (gap_detected) begin
                err_tkeep_gap <= 1'b1;
            end else if (s_tready) begin
                // mem[] write moved below (OPT-5, BRAM inference)
                wr_ptr      <= wr_ptr + 1'b1;
                burst_active_ch[ch_sel_w] <= !s_tlast;
            end
        end
    end
end
// OPT-5: mem[] write lives in its own clocked-only block (no reset in the
// sensitivity list) so Vivado infers block RAM for this FIFO_D-entry
// array instead of flip-flops. wr_ptr/burst_active_ch stay above, async
// reset and all -- only the array write itself needed to move. wr_en is
// the exact same guard the write used above (s_tvalid && s_tready && !gap_detected).
always @(posedge clk) begin
    if (wr_en)
        mem[wr_ptr] <= {s_tuser, s_tlast, s_tkeep, s_tdata};
end
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) s_tready <= 1'b0;
    else        s_tready <= (occupancy_r < (HWM - 1));
end
wire rd_en = m_tvalid && m_tready;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) occupancy_r <= {(FIFO_AW+1){1'b0}};
    else begin
        case ({wr_en, rd_en})
            2'b10:   occupancy_r <= occupancy_r + 1'b1;
            2'b01:   occupancy_r <= occupancy_r - 1'b1;
            default: occupancy_r <= occupancy_r;
        endcase
    end
end
// W2 fix: "RAM holds an entry not yet loaded into the output register"
wire ram_has_data = (wr_ptr != rd_ptr);
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_ptr   <= {FIFO_AW{1'b0}};
        m_tvalid <= 1'b0;
        m_tdata  <= {DATA_W{1'b0}};
        m_tkeep  <= {KEEP_W{1'b0}};
        m_tlast  <= 1'b0;
        m_tuser  <= {TUSER_W{1'b0}};
    end else begin
        if (ram_has_data && (!m_tvalid || m_tready)) begin
            {m_tuser, m_tlast, m_tkeep, m_tdata} <= mem[rd_ptr];
            m_tvalid <= 1'b1;
            rd_ptr   <= rd_ptr + 1'b1;
        end else if (m_tvalid && m_tready) begin
            m_tvalid <= 1'b0;
        end
    end
end
`ifdef SIMULATION
always @(posedge clk) begin
    if (fifo_full && wr_en)
        $error("[write_data_buffer] FIFO overflow: beat written when FIFO was full.");
    if (s_tvalid && s_tready && !s_tlast && (s_tkeep == {KEEP_W{1'b0}}))
        $error("[write_data_buffer] Zero TKEEP on non-TLAST beat.");
end
`endif

endmodule