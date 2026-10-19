`timescale 1ns/1ps

// ============================================================
// Module : flit_payload_builder
// Layer  : Link layer (Engineer B) -- NEW
//
// Packs the TLP stream into 236-byte (59 DW) Flit TLP areas.
// Fixes review items L6, L7 and part of L9.
//
//   Input : 512-bit beats with a byte TKEEP. Valid DWs are the low
//           popcount(TKEEP)/4 DWs of the beat. s_tlast marks the last
//           beat of a TLP (tlp32_to_axis_packer flushes on TLP end, so a
//           beat never holds the tail of one TLP and the head of another).
//   Output: one 1888-bit TLP area per Flit, DW 0 in bits [31:0].
//
//   * DWs are packed contiguously: no slots, no lost bits (old L6/L7).
//   * Several TLPs share a Flit and a TLP may straddle two Flits, as in
//     PCIe 6.0 Flit mode. An 80-byte TLP no longer pads out to 256 B.
//   * A partly filled Flit is sent when the input has been idle for
//     IDLE_FLUSH cycles AND the stream is at a TLP boundary. The unused
//     DWs are zero, i.e. Flit-mode NOP TLPs, which the receiver skips.
//     It is never flushed mid-TLP, so padding can't land inside a TLP.
// ============================================================

module flit_payload_builder #(
    parameter AXIS_DATA_W = 512,
    parameter IDLE_FLUSH  = 8
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     s_tvalid,
    input  wire [AXIS_DATA_W-1:0]   s_tdata,
    input  wire [AXIS_DATA_W/8-1:0] s_tkeep,
    input  wire                     s_tlast,
    output wire                     s_tready,

    output reg                      m_valid,
    output reg  [1887:0]            m_payload,
    input  wire                     m_ready
);

    localparam FL_DW   = 59;
    localparam FL_W    = FL_DW * 32;               // 1888
    localparam BEAT_DW = AXIS_DATA_W / 32;         // 16
    localparam ACC_DW  = FL_DW + BEAT_DW;          // 75
    localparam ACC_W   = ACC_DW * 32;
    localparam FILL_W  = $clog2(ACC_DW + 1);
    localparam IDLE_W  = $clog2(IDLE_FLUSH + 1);

    reg [ACC_W-1:0]  acc;        // bits above fill*32 are always zero
    reg [FILL_W-1:0] fill;       // DWs held, always < FL_DW between beats
    reg              at_boundary;
    reg [IDLE_W-1:0] idle_cnt;

    // Output register is free this cycle?
    wire out_free = !m_valid || m_ready;

    assign s_tready = out_free;

    // DW count of the beat (TKEEP is contiguous from bit 0)
    integer i;
    reg [FILL_W-1:0]      beat_dw;
    reg [AXIS_DATA_W-1:0] beat_masked;
    always @(*) begin
        beat_dw     = {FILL_W{1'b0}};
        beat_masked = {AXIS_DATA_W{1'b0}};
        for (i = 0; i < BEAT_DW; i = i + 1) begin
            if (s_tkeep[i*4]) begin
                beat_dw = beat_dw + 1'b1;
                beat_masked[i*32 +: 32] = s_tdata[i*32 +: 32];
            end
        end
    end

    wire                accept   = s_tvalid && s_tready;
    wire [ACC_W-1:0]    acc_add  = acc | ({{(ACC_W-AXIS_DATA_W){1'b0}}, beat_masked} << (fill * 32));
    wire [FILL_W-1:0]   fill_add = fill + beat_dw;
    wire                flit_full = (fill_add >= FL_DW);

    wire flush = !s_tvalid && out_free && (fill != 0) && at_boundary &&
                 (idle_cnt >= IDLE_FLUSH - 1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc         <= {ACC_W{1'b0}};
            fill        <= {FILL_W{1'b0}};
            at_boundary <= 1'b1;
            idle_cnt    <= {IDLE_W{1'b0}};
            m_valid     <= 1'b0;
            m_payload   <= {FL_W{1'b0}};
        end else begin
            if (m_valid && m_ready)
                m_valid <= 1'b0;

            if (accept) begin
                idle_cnt    <= {IDLE_W{1'b0}};
                at_boundary <= s_tlast;
                if (flit_full) begin
                    m_payload <= acc_add[FL_W-1:0];
                    m_valid   <= 1'b1;
                    acc       <= acc_add >> FL_W;
                    fill      <= fill_add - FL_DW;
                end else begin
                    acc       <= acc_add;
                    fill      <= fill_add;
                end
            end else if (flush) begin
                m_payload <= acc[FL_W-1:0];      // rest is zero = NOP TLPs
                m_valid   <= 1'b1;
                acc       <= {ACC_W{1'b0}};
                fill      <= {FILL_W{1'b0}};
                idle_cnt  <= {IDLE_W{1'b0}};
            end else if (!s_tvalid && fill != 0 && at_boundary) begin
                if (idle_cnt != {IDLE_W{1'b1}})
                    idle_cnt <= idle_cnt + 1'b1;
            end else begin
                idle_cnt <= {IDLE_W{1'b0}};
            end
        end
    end

endmodule
