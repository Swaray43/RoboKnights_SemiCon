`timescale 1ns/1ps
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
//   L0       link up: TX sends data (or logical idle), link_up = 1.
//
//   POLLING or CONFIG not finishing within TIMEOUT_CLKS -> back to DETECT
//   (stands in for the spec's 24 ms / 48 ms timeouts).
//   Hot reset / PERST#: rst_n (datapath_rst_n) returns everything to DETECT.
//
// K6 - leaving L0 on link loss ("Recovery-lite"):
//   In L0 the far end's TX never goes quiet (pcie_phy_tx sends logical idle
//   between Flits), so the receiver can tell a dead link from an idle one.
//   Once the receive side is "armed" (one clean between-Flit cycle - all
//   lanes IDL or all lanes SOF - has been seen in this L0), L0 is left for
//   DETECT when any of these happens:
//     1  signal loss   rx_valid not 4'b1111 (any lane), or rx_err (deskew
//                      FIFO overflow: lane-to-lane skew changed), for
//                      LOSS_CLKS consecutive clocks
//     2  lost framing  GAP_ERR_MAX between-Flit lane cycles that are neither
//                      all-IDL nor all-SOF, with no clean one in between.
//                      Catches a short glitch that slipped the symbol
//                      alignment: after a slip no IDL/SOF byte ever decodes
//                      again. A noise hit that only loses one SOF costs one
//                      Flit (<= 65 cycles) and the count restarts at the next
//                      clean cycle, so channel noise does not drop the link.
//     3  partner retraining  L0_TS1_EXIT TS1 ordered sets received on all
//                      lanes (the partner left L0 and is back in Polling; the
//                      spec enters Recovery on received TS1s). TS2s are not
//                      counted: the partner's last few Configuration TS2s
//                      legitimately arrive after we enter L0.
//   Before arming (e.g. training through the ts_byte_* injection port with
//   no lane traffic at all) exits 1 and 2 are disabled.
//
//   Leaving L0 drops link_up. pcie_reset_controller sees link_trained fall
//   outside a hot reset and pulses datapath_rst_n, the same as for a hot
//   reset (an Upstream Port going DL_Down is reset): deskew, lane order,
//   link layer and function state all start clean and the link retrains from
//   DETECT. axi_rst_n is held until re-training plus a fresh UpdateFC.
//   l0_exit / exit_reason report why (exit_reason is cleared by that reset;
//   catch the l0_exit pulse).
//
// Not modelled: a real Recovery state (TS1/TS2 exchange without DL_Down),
// L0s/L1/L2, speed change / equalization, link and lane number negotiation
// (lane numbers are fixed per physical lane), PAD.
// =============================================================================
module pcie_ltssm #(
    parameter DETECT_CLKS  = 32,
    parameter RX_TS_MIN    = 8,      // spec: 8 consecutive TS received
    parameter TX_TS2_MIN   = 16,     // spec: 16 TS2 sent after receiving one
    parameter TIMEOUT_CLKS = 16384,
    parameter LOSS_CLKS    = 64,     // K6: clocks of lost signal in L0 before link down (1..4095)
    parameter GAP_ERR_MAX  = 512,    // K6: unframed between-Flit lane cycles before link down (1..4095)
    parameter L0_TS1_EXIT  = 2       // K6: TS1s received in L0 before link down (1..3)
)(
    input  wire       clk,
    input  wire       rst_n,

    // ---- from the 4 deskewed TS detectors ----
    input  wire [3:0] os_valid,          // a complete TS1/TS2 just ended on lane i
    input  wire [3:0] os_ts2,            // ...and it was a TS2
    input  wire       lanes_identified,  // lane_reversal_latch has decided the lane order

    // ---- K6: receive-side health, used in L0 only ----
    input  wire [3:0] rx_valid,          // rx_level_valid straight from the SerDes
    input  wire       rx_err,            // lane_err: RX deskew FIFO overflowed (sticky)
    input  wire       rx_gap_ok,         // between Flits: all lanes IDL, or all lanes SOF
    input  wire       rx_gap_bad,        // between Flits: anything else

    // ---- to / from TX ----
    input  wire       tx_os_done,        // TX finished sending one ordered set
    output reg  [1:0] tx_os_mode,        // 0 = no ordered sets, 1 = TS1, 2 = TS2

    output reg  [1:0] state,             // 0 DETECT, 1 POLLING, 2 CONFIG, 3 L0
    output wire       link_up,

    // ---- K6: debug ----
    output reg        l0_exit,           // 1-clk pulse: L0 was left (not by reset)
    output reg  [1:0] exit_reason,       // 1 signal loss, 2 lost framing, 3 TS1 received
    output reg        rx_armed           // L0 receive-side checks are active
);

localparam DETECT  = 2'd0;
localparam POLLING = 2'd1;
localparam CONFIG  = 2'd2;
localparam L0      = 2'd3;

assign link_up = (state == L0);

wire all_os  = &os_valid;            // same ordered set completed on all 4 lanes
wire all_ts2 = all_os && (&os_ts2);
wire all_ts1 = all_os && !(|os_ts2);

reg [5:0]  rx_cnt;                   // ordered sets received in this state
reg [5:0]  tx_ts2_cnt;               // TS2 sent since the first TS2 was received
reg        got_ts2;
reg [14:0] timer;

// ---- K6 L0 health counters ----
reg [11:0] loss_cnt;                 // consecutive clocks of lost signal
reg [11:0] gap_err_cnt;              // unframed between-Flit cycles since the last clean one
reg [1:0]  ts1_cnt;                  // TS1s received in this L0

wire sig_lost  = (rx_armed && (rx_valid != 4'b1111)) || rx_err;
wire loss_exit = sig_lost && (loss_cnt >= LOSS_CLKS - 1);
wire gap_exit  = rx_armed && rx_gap_bad && !rx_gap_ok && (gap_err_cnt >= GAP_ERR_MAX - 1);
wire ts1_exit  = all_ts1 && ({1'b0, ts1_cnt} + 3'd1 >= L0_TS1_EXIT);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    state       <= DETECT;
    tx_os_mode  <= 2'd0;
    rx_cnt      <= 6'd0;
    tx_ts2_cnt  <= 6'd0;
    got_ts2     <= 1'b0;
    timer       <= 15'd0;
    loss_cnt    <= 12'd0;
    gap_err_cnt <= 12'd0;
    ts1_cnt     <= 2'd0;
    rx_armed    <= 1'b0;
    l0_exit     <= 1'b0;
    exit_reason <= 2'd0;
  end else begin
    timer   <= timer + 1'b1;
    l0_exit <= 1'b0;
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
          loss_cnt <= 0; gap_err_cnt <= 0; ts1_cnt <= 0; rx_armed <= 1'b0;
        end else if (timer >= TIMEOUT_CLKS - 1) begin
          state <= DETECT; tx_os_mode <= 2'd0; timer <= 0;
        end
      end

      L0: begin
        tx_os_mode <= 2'd0;
        timer      <= timer;            // hold

        if (rx_gap_ok) rx_armed <= 1'b1;

        if (sig_lost) begin
          if (loss_cnt != 12'hFFF) loss_cnt <= loss_cnt + 1'b1;
        end else
          loss_cnt <= 12'd0;

        if (rx_gap_ok)
          gap_err_cnt <= 12'd0;
        else if (rx_armed && rx_gap_bad && gap_err_cnt != 12'hFFF)
          gap_err_cnt <= gap_err_cnt + 1'b1;

        if (all_ts1 && ts1_cnt != 2'd3) ts1_cnt <= ts1_cnt + 1'b1;

        if (loss_exit || gap_exit || ts1_exit) begin
          state       <= DETECT;
          tx_os_mode  <= 2'd0;
          timer       <= 0;
          rx_cnt      <= 0;
          rx_armed    <= 1'b0;
          l0_exit     <= 1'b1;
          exit_reason <= loss_exit ? 2'd1 : gap_exit ? 2'd2 : 2'd3;
        end
      end
    endcase
  end
end

`ifndef SYNTHESIS
// Make every K6 link drop visible in simulation logs (no bench is silently hit).
always @(posedge clk)
  if (rst_n && l0_exit)
    $display("pcie_ltssm @%0t: link down, L0 left on %0s", $time,
             exit_reason == 2'd1 ? "signal loss" : exit_reason == 2'd2 ? "lost framing" : "TS1 from partner");
`endif

endmodule
