// -----------------------------------------------------------------------------
// Reset controller
// Real inputs: perst_n (async, from connector), link_trained (from
// lane_reversal_latch - the only real, already-built signal this consumes).
// Stubbed inputs, standing in for blocks not yet built: hot_reset_request
// (would come from TS1/TS2 Training Control byte decode - not yet decoded),
// flr_request (would come from config space CfgWr decode - not built),
// updatefc_received (would come from DLLP flow-control logic - not built).
//
// K6 - link down without hot reset: when link_trained falls while LINK_UP and
// no hot reset is pending (pcie_ltssm left L0 on signal loss / lost framing /
// partner retraining), datapath_rst_n is pulsed low for LINKDOWN_PULSE clocks,
// exactly like a hot reset. An Upstream Port going DL_Down is reset, and here
// it is also what makes retraining possible: rx_lane_deskew's lock, the lane
// reversal latch and the link layer's sequence/replay/credit state only clear
// on reset. axi_rst_n is held as before until re-training + a fresh UpdateFC.
// No deadlock: the pulse is started by a falling edge of link_trained, times
// out on its own, and never waits for link_trained (same rule as hot reset).
// LINKDOWN_RESET=0 restores the old behaviour (axi_rst_n only).
// -----------------------------------------------------------------------------
module pcie_reset_controller #(
parameter LINKDOWN_RESET = 1,
parameter LINKDOWN_PULSE = 3     // clocks of datapath_rst_n low (>= 2)
)(
input  wire clk,
input  wire perst_n,
input  wire hot_reset_request,
input  wire flr_request,
input  wire link_trained,
input  wire updatefc_received,

output reg  datapath_rst_n,
output reg  axi_rst_n,
output reg  flr_active
);

localparam LINK_DOWN = 2'd0;
localparam TRAINING  = 2'd1;
localparam LINK_UP   = 2'd2;

reg [1:0] link_state;
reg link_trained_d;
reg hot_reset_pending;
reg [2:0] linkdown_cnt;     // K6: clocks of link-down datapath reset left

// ---- Hot-reset pending latch (external protocol event, decoupled from
// this reset domain's own output - this is what avoids the deadlock below) ----
always @(posedge clk or negedge perst_n) begin
  if (!perst_n) begin
    hot_reset_pending <= 1'b0;
  end else if (hot_reset_request) begin
    hot_reset_pending <= 1'b1;
  end else if (link_state == TRAINING) begin
    hot_reset_pending <= 1'b0; // cleared once re-training begins
  end
end

// ---- PERST#: fully asynchronous, immediate reset of all datapath logic ----
// IMPORTANT: datapath_rst_n must NEVER be gated on link_state/link_trained.
// Doing so creates a deadlock: link_trained can only ever be produced by
// logic that lives inside this same reset domain (TS1/TS2 detector, lane
// reversal latch, etc). If datapath_rst_n waited for LINK_UP before
// deasserting, none of that logic could ever run to produce link_trained
// in the first place. hot_reset_pending is a legitimate SEPARATE trigger
// here precisely because it's driven by an external protocol event, not
// by anything downstream of this reset itself.
always @(posedge clk or negedge perst_n) begin
  if (!perst_n) begin
    datapath_rst_n <= 1'b0;
  end else if (hot_reset_pending) begin
    datapath_rst_n <= 1'b0;   // synchronous reset pulse on confirmed hot-reset
  end else if (linkdown_cnt != 3'd0) begin
    datapath_rst_n <= 1'b0;   // K6: same pulse on an unexpected link down
  end else begin
    datapath_rst_n <= 1'b1;
  end
end

// ---- Link state tracking + link-down detection ----
always @(posedge clk or negedge perst_n) begin
  if (!perst_n) begin
    link_state     <= LINK_DOWN;
    link_trained_d <= 1'b0;
    linkdown_cnt   <= 3'd0;
  end else begin
    link_trained_d <= link_trained;
    if (linkdown_cnt != 3'd0) linkdown_cnt <= linkdown_cnt - 3'd1;

    case (link_state)
      LINK_DOWN: begin
        if (link_trained) link_state <= LINK_UP;
        else               link_state <= TRAINING;
      end
      TRAINING: begin
        if (link_trained) link_state <= LINK_UP;
      end
      LINK_UP: begin
        if (hot_reset_pending) begin
          link_state <= TRAINING;
        end else if (link_trained_d && !link_trained) begin
          // Link-down detection: link_trained fell unexpectedly (not via
          // hot-reset) - spec requires holding AXI reset until re-trained.
          link_state <= LINK_DOWN;
          if (LINKDOWN_RESET != 0) linkdown_cnt <= LINKDOWN_PULSE;   // K6
        end
      end
    endcase
  end
end

// ---- FLR: resets function state, but explicitly does NOT drop the link ----
// datapath_rst_n / link_state are untouched here - FLR is scoped narrower
// than hot-reset by spec ("keeping the link up"). Since config space/MSI-X
// (the actual "function state") aren't built yet, flr_active is exposed as
// a pulse those blocks would consume once implemented.
always @(posedge clk or negedge perst_n) begin
  if (!perst_n) begin
    flr_active <= 1'b0;
  end else begin
    flr_active <= flr_request;
  end
end

// ---- AXI reset sequencing ----
// Deasserts only after link_trained AND first UpdateFC DLLP received;
// re-asserts immediately on any link_state leaving LINK_UP (covers both
// hot-reset and unexpected link-down with one condition, since both are
// really the same invariant: "is the link currently trustworthy").
reg updatefc_seen;
always @(posedge clk or negedge perst_n) begin
  if (!perst_n) begin
    axi_rst_n     <= 1'b0;
    updatefc_seen <= 1'b0;
  end else begin
    if (link_state != LINK_UP) begin
      axi_rst_n     <= 1'b0;
      updatefc_seen <= 1'b0;  // must see a fresh UpdateFC after every re-train
    end else begin
      if (updatefc_received)
        updatefc_seen <= 1'b1;
      axi_rst_n <= updatefc_seen || updatefc_received;
    end
  end
end

endmodule
