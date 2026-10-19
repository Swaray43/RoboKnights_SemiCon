`timescale 1ns/1ps

/*
 * Module: tlp32_to_axis_packer
 *
 * Purpose:
 *   Real width adapter between tx_engine's 32-bit-per-cycle TLP DW stream
 *   (m_tdata/m_tvalid/m_tlast/m_tready) and the AXIS_DATA_W-wide beat
 *   interface (e.g. 512 bits) expected by data_link_layer_top.
 *
 *   Accumulates NUM_DW = AXIS_WIDTH/32 words into one beat before
 *   presenting it, or flushes early (zero-padding the remaining DW slots)
 *   when tlast arrives mid-beat.
 *
 * Change (bridge review L7):
 *   m_tkeep added. The Flit builder downstream packs DWs back-to-back, so
 *   it must know which DWs of a short (tlast) beat are real; previously the
 *   zero padding was indistinguishable from data and was carried into the
 *   Flit as payload. m_tkeep has 4 bits set per valid DW, contiguous from
 *   bit 0.
 *
 * Backpressure:
 *   s_tready deasserts whenever a completed beat is waiting on the
 *   downstream (m_tvalid && !m_tready), so no incoming DW is ever dropped.
 */
module tlp32_to_axis_packer #(
    parameter integer DW_WIDTH   = 32,
    parameter integer AXIS_WIDTH = 512
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // Narrow TLP DW-stream input (from tlp_packetizer / tx_engine)
    input  wire                    s_tvalid,
    input  wire [DW_WIDTH-1:0]     s_tdata,
    input  wire                    s_tlast,
    output wire                    s_tready,

    // Packed wide AXIS-Stream output (to data_link_layer_top)
    output reg                     m_tvalid,
    output reg  [AXIS_WIDTH-1:0]   m_tdata,
    output reg  [AXIS_WIDTH/8-1:0] m_tkeep,
    output reg                     m_tlast,
    input  wire                    m_tready
);

    localparam integer NUM_DW = AXIS_WIDTH / DW_WIDTH;
    localparam integer CNT_W  = (NUM_DW <= 1) ? 1 : $clog2(NUM_DW);
    localparam integer DW_B   = DW_WIDTH / 8;

    // synthesis translate_off
    initial begin
        if (AXIS_WIDTH % DW_WIDTH != 0) begin
            $display("ERROR: tlp32_to_axis_packer AXIS_WIDTH (%0d) must be a multiple of DW_WIDTH (%0d)",
                      AXIS_WIDTH, DW_WIDTH);
            $finish;
        end
    end
    // synthesis translate_on

    reg [AXIS_WIDTH-1:0]   acc_data;
    reg [AXIS_WIDTH/8-1:0] acc_keep;
    reg [CNT_W-1:0]        dw_idx;

    assign s_tready = !(m_tvalid && !m_tready);

    wire accept        = s_tvalid && s_tready;
    wire beat_full     = (dw_idx == NUM_DW-1);
    wire beat_complete = accept && (beat_full || s_tlast);

    reg [AXIS_WIDTH-1:0]   next_acc_data;
    reg [AXIS_WIDTH/8-1:0] next_acc_keep;

    always @(*) begin
        next_acc_data = acc_data;
        next_acc_keep = acc_keep;
        if (accept) begin
            next_acc_data[dw_idx*DW_WIDTH +: DW_WIDTH] = s_tdata;
            next_acc_keep[dw_idx*DW_B     +: DW_B]     = {DW_B{1'b1}};
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_data <= {AXIS_WIDTH{1'b0}};
            acc_keep <= {(AXIS_WIDTH/8){1'b0}};
            dw_idx   <= {CNT_W{1'b0}};
            m_tvalid <= 1'b0;
            m_tdata  <= {AXIS_WIDTH{1'b0}};
            m_tkeep  <= {(AXIS_WIDTH/8){1'b0}};
            m_tlast  <= 1'b0;
        end else begin
            if (m_tvalid && m_tready) begin
                m_tvalid <= 1'b0;
            end

            if (beat_complete) begin
                m_tvalid <= 1'b1;
                m_tdata  <= next_acc_data;
                m_tkeep  <= next_acc_keep;
                m_tlast  <= s_tlast;

                acc_data <= {AXIS_WIDTH{1'b0}};
                acc_keep <= {(AXIS_WIDTH/8){1'b0}};
                dw_idx   <= {CNT_W{1'b0}};
            end else if (accept) begin
                acc_data <= next_acc_data;
                acc_keep <= next_acc_keep;
                dw_idx   <= dw_idx + 1'b1;
            end
        end
    end

endmodule
