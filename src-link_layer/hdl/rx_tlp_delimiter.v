`timescale 1ns/1ps

// ============================================================
// Module : rx_tlp_delimiter
// Layer  : Link layer (Engineer B) -- receive side
//
// Inverse of flit_payload_builder. Takes accepted, in-order Flit TLP
// areas (59 DW each), skips NOP DWs (all-zero), finds TLP boundaries
// from each TLP header and re-emits every TLP as 512-bit AXIS beats:
//   tdata DW0 in [31:0], tkeep per byte, tlast on the TLP's last beat.
//
// Step 4 item 8 (+T2): up to 16 DWs per clock (was 1). Each clock moves one
// "piece": min(DWs left in the TLP, DWs left in the Flit, 16). A TLP that
// straddles two Flits therefore leaves as a short beat (end of Flit N)
// followed by the rest (Flit N+1); tkeep says how many DWs each beat holds,
// tlast still marks the TLP's last beat. A new TLP always starts a new beat.
// The pointer arithmetic is registered: the piece size and next pointer are
// computed from registered state only (T2: the old 1-DW walk drove a
// 1,888-bit load enable from a 6-deep carry chain).
//
// TLP size from DW0 (non-Flit header format, as the TX side emits):
//   Fmt = DW0[31:29]: Fmt[0]=1 -> 4DW header else 3DW; Fmt[1]=1 -> data
//   Length = DW0[9:0] (0 means 1024 DW)
//
// A Flit holds 59 DWs, so it takes at most a few clocks; Flits arrive at
// most every 8 clocks, so this keeps up with the link.
// ============================================================

module rx_tlp_delimiter #(
    parameter AXIS_DATA_W = 512
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     s_valid,
    input  wire [1887:0]            s_payload,
    output wire                     s_ready,

    output reg                      m_axis_tvalid,
    output reg  [AXIS_DATA_W-1:0]   m_axis_tdata,
    output reg  [AXIS_DATA_W/8-1:0] m_axis_tkeep,
    output reg                      m_axis_tlast,
    input  wire                     m_axis_tready
);

    localparam FL_DW   = 59;
    localparam BEAT_DW = AXIS_DATA_W / 32;      // 16

    reg [1887:0]  flit_r;      // shifted so the next unread DW is DW 0
    reg           flit_v;
    reg [6:0]     left;        // DWs of the Flit not yet consumed
    reg           in_tlp;
    reg [10:0]    rem;         // DWs still to come in the current TLP

    wire       out_free  = !m_axis_tvalid || m_axis_tready;
    wire [31:0] dw0      = flit_r[31:0];
    wire       rest_zero = (flit_r == 1888'd0);

    function [10:0] tlp_total_dw;
        input [31:0] h;
        reg   [10:0] len;
        begin
            len = (h[9:0] == 10'd0) ? 11'd1024 : {1'b0, h[9:0]};
            tlp_total_dw = (h[29] ? 11'd4 : 11'd3) + (h[30] ? len : 11'd0);
        end
    endfunction

    // what this clock does
    wire        start    = !in_tlp && (dw0 != 32'd0);           // a TLP header is at DW 0
    wire        nop      = !in_tlp && (dw0 == 32'd0);
    wire [10:0] rem_eff  = start ? tlp_total_dw(dw0) : rem;
    wire [10:0] lim1     = (rem_eff < {4'd0, left}) ? rem_eff : {4'd0, left};
    wire [4:0]  n        = (lim1 < BEAT_DW) ? lim1[4:0] : BEAT_DW[4:0];
    wire        step     = flit_v && out_free;
    wire        emit     = step && (in_tlp || start);
    // a NOP outside a TLP: Flit-mode padding is only ever at the tail, so
    // the rest of the Flit is done (a lone NOP mid-Flit is skipped by one DW)
    wire        skip_all = step && nop && rest_zero;
    wire        skip_one = step && nop && !rest_zero;
    wire [6:0]  used     = emit ? {2'd0, n} : (skip_one ? 7'd1 : 7'd0);
    wire        flit_done = flit_v && ((emit && (n == left)) || skip_all || (skip_one && left == 7'd1));

    assign s_ready = !flit_v || flit_done;

    reg [AXIS_DATA_W/8-1:0] keep_c;
    reg [AXIS_DATA_W-1:0]   data_c;
    integer q;
    always @(*) begin
        keep_c = {AXIS_DATA_W/8{1'b0}};
        data_c = flit_r[AXIS_DATA_W-1:0];
        for (q = 0; q < BEAT_DW; q = q + 1) begin
            if (q < n) keep_c[q*4 +: 4] = 4'hF;
            else       data_c[q*32 +: 32] = 32'd0;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flit_r        <= 1888'd0;
            flit_v        <= 1'b0;
            left          <= 7'd0;
            in_tlp        <= 1'b0;
            rem           <= 11'd0;
            m_axis_tvalid <= 1'b0;
            m_axis_tdata  <= {AXIS_DATA_W{1'b0}};
            m_axis_tkeep  <= {AXIS_DATA_W/8{1'b0}};
            m_axis_tlast  <= 1'b0;
        end else begin
            if (m_axis_tvalid && m_axis_tready)
                m_axis_tvalid <= 1'b0;

            if (emit) begin
                m_axis_tvalid <= 1'b1;
                m_axis_tdata  <= data_c;
                m_axis_tkeep  <= keep_c;
                m_axis_tlast  <= ({6'd0, n} == rem_eff);
                rem           <= rem_eff - {6'd0, n};
                in_tlp        <= ({6'd0, n} != rem_eff);
            end

            if (flit_done) begin
                flit_v <= 1'b0;
                left   <= 7'd0;
            end else if (step) begin
                flit_r <= flit_r >> (used * 32);
                left   <= left - used;
            end

            if (s_valid && s_ready) begin
                flit_r <= s_payload;
                flit_v <= 1'b1;
                left   <= FL_DW;
            end
        end
    end

endmodule
