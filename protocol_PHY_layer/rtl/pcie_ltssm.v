// =============================================================================
// pcie_ltssm - simplified PCIe Link Training and Status State Machine
//
// Replaces "link up = first two matching TS1s". States follow the PCIe base
// spec's top-level LTSSM, reduced to what this PHY models:
//
//   DETECT   TX quiet for DETECT_CLKS (stands in for receiver detection,
//            which is an analog SerDes function), then -> POLLING.
//   POLLING  (Polling.Active) TX sends TS1. When RX_TS_MIN ordered sets
//            (TS1 or TS2) have been received on all 4 lanes together
//            (i.e. after deskew) -> CONFIG.
//   CONFIG   (Polling.Configuration + Configuration, merged) TX sends TS2.
//            -> L0 when RX_TS_MIN TS2s have been received on all lanes, at
//            least TX_TS2_MIN TS2s have been SENT since the first TS2 was
//            received (so the partner sees enough of ours too - the spec's
//            "16 TS2 after receiving one" rule), and lane numbering has been
//            resolved (lanes_identified, from lane_reversal_latch).
//   L0       link up: TX sends data, link_up = 1.
//
//   POLLING or CONFIG not finishing within TIMEOUT_CLKS -> back to DETECT
//   (stands in for the spec's 24 ms / 48 ms timeouts).
//   Hot reset / PERST#: rst_n (datapath_rst_n) returns everything to DETECT.
//
// Not modelled: Recovery, L0s/L1/L2, speed change / equalization, link and
// lane number negotiation (lane numbers are fixed per physical lane), PAD.
// =============================================================================
module pcie_ltssm #(
    parameter DETECT_CLKS  = 32,
    parameter RX_TS_MIN    = 8,      // spec: 8 consecutive TS received
    parameter TX_TS2_MIN   = 16,     // spec: 16 TS2 sent after receiving one
    parameter TIMEOUT_CLKS = 16384
)(
    input  wire       clk,
    input  wire       rst_n,

    // ---- from the 4 deskewed TS detectors ----
    input  wire [3:0] os_valid,          // a complete TS1/TS2 just ended on lane i
    input  wire [3:0] os_ts2,            // ...and it was a TS2
    input  wire       lanes_identified,  // lane_reversal_latch has decided the lane order

    // ---- to / from TX ----
    input  wire       tx_os_done,        // TX finished sending one ordered set
    output reg  [1:0] tx_os_mode,        // 0 = no ordered sets, 1 = TS1, 2 = TS2

    output reg  [1:0] state,             // 0 DETECT, 1 POLLING, 2 CONFIG, 3 L0
    output wire       link_up
);

localparam DETECT  = 2'd0;
localparam POLLING = 2'd1;
localparam CONFIG  = 2'd2;
localparam L0      = 2'd3;

assign link_up = (state == L0);

wire all_os  = &os_valid;            // same ordered set completed on all 4 lanes
wire all_ts2 = all_os && (&os_ts2);

reg [5:0]  rx_cnt;                   // ordered sets received in this state
reg [5:0]  tx_ts2_cnt;               // TS2 sent since the first TS2 was received
reg        got_ts2;
reg [14:0] timer;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    state      <= DETECT;
    tx_os_mode <= 2'd0;
    rx_cnt     <= 6'd0;
    tx_ts2_cnt <= 6'd0;
    got_ts2    <= 1'b0;
    timer      <= 15'd0;
  end else begin
    timer <= timer + 1'b1;
    case (state)
      DETECT: begin
        tx_os_mode <= 2'd0;
        if (timer >= DETECT_CLKS - 1) begin
          state <= POLLING; tx_os_mode <= 2'd1; timer <= 0; rx_cnt <= 0;
        end
      end

      POLLING: begin
        if (all_os && rx_cnt != 6'h3F) rx_cnt <= rx_cnt + 1'b1;
        if (rx_cnt >= RX_TS_MIN) begin
          state <= CONFIG; tx_os_mode <= 2'd2; timer <= 0;
          rx_cnt <= 0; tx_ts2_cnt <= 0; got_ts2 <= 1'b0;
        end else if (timer >= TIMEOUT_CLKS - 1) begin
          state <= DETECT; tx_os_mode <= 2'd0; timer <= 0;
        end
      end

      CONFIG: begin
        if (all_ts2) begin
          got_ts2 <= 1'b1;
          if (rx_cnt != 6'h3F) rx_cnt <= rx_cnt + 1'b1;
        end
        if (tx_os_done && (got_ts2 || all_ts2) && tx_ts2_cnt != 6'h3F)
          tx_ts2_cnt <= tx_ts2_cnt + 1'b1;
        if (rx_cnt >= RX_TS_MIN && tx_ts2_cnt >= TX_TS2_MIN && lanes_identified) begin
          state <= L0; tx_os_mode <= 2'd0;
        end else if (timer >= TIMEOUT_CLKS - 1) begin
          state <= DETECT; tx_os_mode <= 2'd0; timer <= 0;
        end
      end

      L0: begin
        tx_os_mode <= 2'd0;
        timer      <= timer;            // hold
      end
    endcase
  end
end

endmodule
