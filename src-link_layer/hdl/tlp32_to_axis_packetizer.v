`timescale 1ns/1ps

/*
 * Module: tlp32_to_axis_packer
 *
 * Purpose:
 *   Real width adapter between tx_engine's 32-bit-per-cycle TLP DW stream
 *   (m_tdata/m_tvalid/m_tlast/m_tready) and the AXIS_DATA_W-wide beat
 *   interface (e.g. 512 bits) expected by data_link_layer_top / replay_buffer.
 *
 *   Replaces a zero-pad stub that dropped a single 32-bit word into the low
 *   bits of a wide beat and asserted tvalid immediately, wasting
 *   (AXIS_WIDTH/32 - 1) DW slots per beat and mis-packing every multi-DW
 *   TLP. This module instead accumulates NUM_DW = AXIS_WIDTH/32 words into
 *   one beat before presenting it, or flushes early (zero-padding the
 *   remaining DW slots) when tlast arrives mid-beat -- matching the rest of
 *   this codebase, which carries no tkeep/byte-enable signals anywhere and
 *   relies on TLP header length fields (not per-beat DW enables) to mark
 *   the true payload extent downstream.
 *
 * Backpressure:
 *   s_tready deasserts whenever a completed beat is waiting on the
 *   downstream (m_tvalid && !m_tready), so no incoming DW is ever dropped.
 */
module tlp32_to_axis_packer #(
    parameter integer DW_WIDTH   = 32,
    parameter integer AXIS_WIDTH = 512
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // Narrow TLP DW-stream input (from tx_engine)
    input  wire                  s_tvalid,
    input  wire [DW_WIDTH-1:0]   s_tdata,
    input  wire                  s_tlast,
    output wire                  s_tready,

    // Packed wide AXIS-Stream output (to data_link_layer_top / replay_buffer)
    output reg                   m_tvalid,
    output reg  [AXIS_WIDTH-1:0] m_tdata,
    output reg                   m_tlast,
    input  wire                  m_tready
);

    localparam integer NUM_DW = AXIS_WIDTH / DW_WIDTH;
    localparam integer CNT_W  = (NUM_DW <= 1) ? 1 : $clog2(NUM_DW);

    // Elaboration-time sanity check: AXIS_WIDTH must be an integer multiple
    // of DW_WIDTH, or the packing math below is meaningless.
    // synthesis translate_off
    initial begin
        if (AXIS_WIDTH % DW_WIDTH != 0) begin
            $display("ERROR: tlp32_to_axis_packer AXIS_WIDTH (%0d) must be a multiple of DW_WIDTH (%0d)",
                      AXIS_WIDTH, DW_WIDTH);
            $finish;
        end
    end
    // synthesis translate_on

    reg [AXIS_WIDTH-1:0] acc_data;
    reg [CNT_W-1:0]      dw_idx;

    // Only accept a new DW while no completed beat is stalled on downstream.
    assign s_tready = !(m_tvalid && !m_tready);

    wire accept        = s_tvalid && s_tready;
    wire beat_full     = (dw_idx == NUM_DW-1);
    wire beat_complete = accept && (beat_full || s_tlast);

    // Combinational view of the accumulator including the DW being accepted
    // this cycle, so a completed beat can be latched to the output stage in
    // the same cycle it becomes complete (no extra pipeline bubble).
    reg [AXIS_WIDTH-1:0] next_acc_data;

    always @(*) begin
        next_acc_data = acc_data;
        if (accept) begin
            next_acc_data[dw_idx*DW_WIDTH +: DW_WIDTH] = s_tdata;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            acc_data <= {AXIS_WIDTH{1'b0}};
            dw_idx   <= {CNT_W{1'b0}};
            m_tvalid <= 1'b0;
            m_tdata  <= {AXIS_WIDTH{1'b0}};
            m_tlast  <= 1'b0;
        end else begin
            // Downstream consumed the beat currently held in the output stage.
            if (m_tvalid && m_tready) begin
                m_tvalid <= 1'b0;
            end

            if (beat_complete) begin
                // Present the just-completed beat (full, or short because
                // tlast arrived early -- remaining DW slots stay zero) to
                // the output stage, and reset the accumulator.
                m_tvalid <= 1'b1;
                m_tdata  <= next_acc_data;
                m_tlast  <= s_tlast;

                acc_data <= {AXIS_WIDTH{1'b0}};
                dw_idx   <= {CNT_W{1'b0}};
            end else if (accept) begin
                acc_data <= next_acc_data;
                dw_idx   <= dw_idx + 1'b1;
            end
        end
    end

endmodule