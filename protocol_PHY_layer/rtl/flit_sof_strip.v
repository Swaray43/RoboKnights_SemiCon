`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// FLIT-mode SOF token strip (PCIe 6.0 uses FLIT mode, not 128b/130b - see
// design notes). SOF = 0xFB on all 4 lanes simultaneously, one full cycle,
// immediately preceding each FLIT burst of PAYLOAD_CYCLES lane-cycles.
//
// PAYLOAD_CYCLES (new parameter):
//   8  (default) - SOF once per 32-byte word. Original behaviour; the unit
//                  test tb_flit_sof_strip relies on it.
//   64           - SOF once per 256-byte Flit (8 words x 8 lane-cycles). This
//                  is the PCIe meaning of "Flit" and what Swaray's link layer
//                  produces, so pcie_phy_rx uses 64. (Seam C naming clash.)
//
// RESYNC_MIDFLIT (new parameter):
//   1 (default) - original behaviour: an all-lanes 0xFB cycle in the middle of
//                 a Flit restarts the count. Payload that happens to contain
//                 0xFB on all 4 lanes in one cycle will misfire.
//   0           - ignore SOF-looking payload while counting a Flit. Safe when
//                 TX and RX run in lockstep (they do here), and removes the
//                 0xFB-in-payload limitation. pcie_phy_rx uses 0.
//
// sof_out (new port): high together with the FIRST payload cycle after each
// SOF, so downstream logic (pcie_phy_rx) can rebuild sof/eof per Flit.
//
// hunting (K6, new port): high while waiting for SOF, i.e. the current input
// cycle is between Flits (logical idle or SOF). pcie_phy_rx uses it to check
// that between-Flit bytes decode as IDL/SOF (symbol alignment still good).
// -----------------------------------------------------------------------------
module flit_sof_strip #(
parameter PAYLOAD_CYCLES = 8,
parameter RESYNC_MIDFLIT = 1
)(
input  wire       clk,
input  wire       rst_n,
input  wire [7:0] lane_in_0,
input  wire [7:0] lane_in_1,
input  wire [7:0] lane_in_2,
input  wire [7:0] lane_in_3,
input  wire       lane_in_valid,
output reg  [7:0] lane_out_0,
output reg  [7:0] lane_out_1,
output reg  [7:0] lane_out_2,
output reg  [7:0] lane_out_3,
output reg  [3:0] lane_valid_out,
output reg        sof_out,
output wire       hunting
);

localparam SOF_TOKEN = 8'hFB;
localparam WAIT_SOF  = 1'b0;
localparam PASS_FLIT = 1'b1;
localparam CW = (PAYLOAD_CYCLES <= 2)   ? 1 :
                (PAYLOAD_CYCLES <= 4)   ? 2 :
                (PAYLOAD_CYCLES <= 8)   ? 3 :
                (PAYLOAD_CYCLES <= 16)  ? 4 :
                (PAYLOAD_CYCLES <= 32)  ? 5 :
                (PAYLOAD_CYCLES <= 64)  ? 6 :
                (PAYLOAD_CYCLES <= 128) ? 7 : 8;

reg          state;
reg [CW-1:0] payload_cnt;
reg          first_cycle;   // next payload cycle is the first of a Flit
assign hunting = (state == WAIT_SOF);

wire is_sof_cycle = lane_in_valid &&
                     (lane_in_0 == SOF_TOKEN) && (lane_in_1 == SOF_TOKEN) &&
                     (lane_in_2 == SOF_TOKEN) && (lane_in_3 == SOF_TOKEN);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    state          <= WAIT_SOF;
    payload_cnt    <= {CW{1'b0}};
    first_cycle    <= 1'b0;
    lane_valid_out <= 4'b0000;
    sof_out        <= 1'b0;
    lane_out_0     <= 8'd0;
    lane_out_1     <= 8'd0;
    lane_out_2     <= 8'd0;
    lane_out_3     <= 8'd0;
  end else begin
    lane_valid_out <= 4'b0000;
    sof_out        <= 1'b0;

    case (state)
      WAIT_SOF: begin
        if (is_sof_cycle) begin
          state       <= PASS_FLIT;
          payload_cnt <= {CW{1'b0}};
          first_cycle <= 1'b1;
        end
        // non-SOF bytes here (idle/training traffic) are silently dropped
      end

      PASS_FLIT: begin
        if (lane_in_valid) begin
          if (RESYNC_MIDFLIT && is_sof_cycle) begin
            // Defensive resync: a real SOF appearing mid-FLIT means sync was lost.
            state       <= PASS_FLIT;
            payload_cnt <= {CW{1'b0}};
            first_cycle <= 1'b1;
          end else begin
            lane_out_0     <= lane_in_0;
            lane_out_1     <= lane_in_1;
            lane_out_2     <= lane_in_2;
            lane_out_3     <= lane_in_3;
            lane_valid_out <= 4'b1111;
            sof_out        <= first_cycle;
            first_cycle    <= 1'b0;

            if (payload_cnt == PAYLOAD_CYCLES - 1) begin
              state       <= WAIT_SOF;
              payload_cnt <= {CW{1'b0}};
            end else begin
              payload_cnt <= payload_cnt + 1'b1;
            end
          end
        end
      end
    endcase
  end
end

`ifdef FORMAL
  // Carried over from pci_stuff.v's copy of this module (it was the only copy
  // with properties). Moved below the declarations so it elaborates.
  initial assume(!rst_n);

  always @(posedge clk) begin
    if (!rst_n) begin
      assert(state == WAIT_SOF);
      assert(payload_cnt == 0);
      assert(lane_valid_out == 4'b0000);
    end else begin
      assert(state == WAIT_SOF || state == PASS_FLIT);
      if (state == WAIT_SOF) assert(payload_cnt == 0);
      if (state == PASS_FLIT) assert(payload_cnt <= PAYLOAD_CYCLES - 1);
    end
  end
`endif
endmodule
