`timescale 1ns/1ps

// ============================================================
// Module : rx_tlp_delimiter
// Layer  : Link layer (Engineer B) -- NEW, receive side
//
// Inverse of flit_payload_builder. Takes accepted, in-order Flit TLP
// areas (59 DW each), skips NOP DWs (all-zero), finds TLP boundaries
// from each TLP header and re-emits every TLP as 512-bit AXIS beats:
//   tdata DW0 in [31:0], tkeep per byte, tlast on the TLP's last beat.
// A TLP that straddles Flits is stitched back together.
//
// TLP size from DW0 (non-Flit header format, as tlp_packetizer emits):
//   Fmt = DW0[31:29]: Fmt[0]=1 -> 4DW header else 3DW; Fmt[1]=1 -> data
//   Length = DW0[9:0] (0 means 1024 DW)
//
// Throughput: 1 DW per clock for TLP DWs; trailing NOP padding is skipped
// in one clock. data_link_layer_top keeps a small Flit FIFO in front of
// this and NAKs (-> replay) if it would overflow.
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
    localparam BEAT_DW = AXIS_DATA_W / 32;
    localparam BIDX_W  = $clog2(BEAT_DW);

    reg [1887:0]           flit_r;
    reg                    flit_v;
    reg [5:0]              ptr;          // DW index into flit_r

    reg                    in_tlp;
    reg [10:0]             rem;          // DWs still to come in this TLP

    reg [AXIS_DATA_W-1:0]  beat_acc;
    reg [AXIS_DATA_W/8-1:0] keep_acc;
    reg [BIDX_W-1:0]       bidx;

    wire       out_free = !m_axis_tvalid || m_axis_tready;
    wire       step     = flit_v && out_free;
    wire [31:0] dw      = flit_r[ptr*32 +: 32];
    wire       last_dw_of_flit = (ptr == FL_DW - 1);
    // Outside a TLP with nothing but NOP padding left: finish the Flit now
    // instead of walking the padding one DW per clock.
    wire       rest_zero = ((flit_r >> (ptr * 32)) == 1888'd0);
    wire       flit_done = last_dw_of_flit || (!in_tlp && rest_zero);

    assign s_ready = !flit_v || (step && flit_done);

    // total DWs of a TLP from its first header DW
    function [10:0] tlp_total_dw;
        input [31:0] h;
        reg   [10:0] len;
        begin
            len = (h[9:0] == 10'd0) ? 11'd1024 : {1'b0, h[9:0]};
            tlp_total_dw = (h[29] ? 11'd4 : 11'd3) + (h[30] ? len : 11'd0);
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            flit_r        <= 1888'd0;
            flit_v        <= 1'b0;
            ptr           <= 6'd0;
            in_tlp        <= 1'b0;
            rem           <= 11'd0;
            beat_acc      <= {AXIS_DATA_W{1'b0}};
            keep_acc      <= {AXIS_DATA_W/8{1'b0}};
            bidx          <= {BIDX_W{1'b0}};
            m_axis_tvalid <= 1'b0;
            m_axis_tdata  <= {AXIS_DATA_W{1'b0}};
            m_axis_tkeep  <= {AXIS_DATA_W/8{1'b0}};
            m_axis_tlast  <= 1'b0;
        end else begin
            if (m_axis_tvalid && m_axis_tready)
                m_axis_tvalid <= 1'b0;

            if (step) begin : walk
                reg [10:0]             rem_n;
                reg                    place;
                reg [AXIS_DATA_W-1:0]  b_n;
                reg [AXIS_DATA_W/8-1:0] k_n;

                place = 1'b0;
                rem_n = rem;
                if (in_tlp) begin
                    place = 1'b1;
                end else if (dw != 32'd0) begin
                    place  = 1'b1;
                    rem_n  = tlp_total_dw(dw);
                end
                // else: NOP DW, skip

                if (place) begin
                    b_n = beat_acc;
                    k_n = keep_acc;
                    b_n[bidx*32 +: 32] = dw;
                    k_n[bidx*4  +: 4]  = 4'hF;
                    rem_n = rem_n - 11'd1;
                    if (rem_n == 11'd0 || bidx == BEAT_DW - 1) begin
                        m_axis_tvalid <= 1'b1;
                        m_axis_tdata  <= b_n;
                        m_axis_tkeep  <= k_n;
                        m_axis_tlast  <= (rem_n == 11'd0);
                        beat_acc      <= {AXIS_DATA_W{1'b0}};
                        keep_acc      <= {AXIS_DATA_W/8{1'b0}};
                        bidx          <= {BIDX_W{1'b0}};
                    end else begin
                        beat_acc <= b_n;
                        keep_acc <= k_n;
                        bidx     <= bidx + 1'b1;
                    end
                    in_tlp <= (rem_n != 11'd0);
                    rem    <= rem_n;
                end

                if (flit_done) begin
                    ptr    <= 6'd0;
                    flit_v <= 1'b0;
                end else begin
                    ptr <= ptr + 6'd1;
                end
            end

            if (s_valid && s_ready) begin
                flit_r <= s_payload;
                flit_v <= 1'b1;
                ptr    <= 6'd0;
            end
        end
    end

endmodule
