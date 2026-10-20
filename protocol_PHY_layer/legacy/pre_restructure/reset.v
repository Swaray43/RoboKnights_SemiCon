module pcie_reset_controller (
input  wire clk,
input  wire perst_n,            // async, active-low PERST# from the connector

input  wire hot_reset_request,  // pulse - stand-in for decoded TS1/TS2 hot-reset bit
input  wire flr_request,        // pulse - stand-in for decoded CfgWr to FLR bit
input  wire link_trained,       // real signal, from lane_reversal_latch
input  wire updatefc_received,  // pulse - stand-in for first UpdateFC DLLP (not yet built)

output reg  datapath_rst_n,     // drives reset for striper/destriper/gray/pam4/etc
output reg  axi_rst_n,          // held low until link trained AND first UpdateFC seen
output reg  flr_active          // pulses/holds while function-level reset is in progress
);

localparam LINK_DOWN   = 2'd0;
localparam TRAINING    = 2'd1;
localparam LINK_UP     = 2'd2;

reg [1:0] link_state;
reg link_trained_d;   // for edge detection on link_trained

// ---- PERST#: fully asynchronous, immediate reset of all datapath logic ----
// This is the only reset source that bypasses the clock entirely - matches
// real PCIe PERST# behavior (must reset even without a stable clock present).
always @(posedge clk or negedge perst_n) begin
  if (!perst_n) begin
    datapath_rst_n <= 1'b0;
  end else begin
    datapath_rst_n <= 1'b1;
  end
end

// ---- Hot-reset: re-enter training, without needing a full PERST# cycle ----
// Real PCIe distinguishes hot-reset from PERST# in scope (hot-reset doesn't
// require re-powering the link), but for our datapath modules there's no
// meaningful difference in what needs resetting - so hot-reset re-asserts
// datapath_rst_n through the same path PERST# uses, just synchronously.
reg hot_reset_pending;
always @(posedge clk or negedge perst_n) begin
  if (!perst_n) begin
    hot_reset_pending <= 1'b0;
  end else if (hot_reset_request) begin
    hot_reset_pending <= 1'b1;
  end else if (link_state == TRAINING) begin
    hot_reset_pending <= 1'b0; // cleared once re-training begins
  end
end

// ---- Link state tracking + link-down detection ----
always @(posedge clk or negedge perst_n) begin
  if (!perst_n) begin
    link_state     <= LINK_DOWN;
    link_trained_d <= 1'b0;
  end else begin
    link_trained_d <= link_trained;

    case (link_state)
      LINK_DOWN: begin
        if (link_trained) link_state <= LINK_UP;
        else               link_state <= TRAINING;
      end
      TRAINING: begin
        if (link_trained) link_state <= LINK_UP;
        // hot_reset_pending clears itself once here, per above
      end
      LINK_UP: begin
        if (hot_reset_pending) begin
          link_state <= TRAINING;   // hot-reset: drop back to training
        end else if (link_trained_d && !link_trained) begin
          // Link-down detection: link_trained fell unexpectedly (not via
          // hot-reset) - spec requires holding AXI reset until re-trained.
          link_state <= LINK_DOWN;
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
    flr_active <= flr_request; // one-cycle pulse forward; extend to a hold
                                 // period if/when config space needs longer
  end
end

// ---- AXI reset sequencing ----
// Deasserts only after link_trained AND first UpdateFC DLLP received -
// re-asserts immediately on any link_state leaving LINK_UP (covers both
// hot-reset and unexpected link-down in one condition).
reg updatefc_seen;
always @(posedge clk or negedge perst_n) begin
  if (!perst_n) begin
    axi_rst_n     <= 1'b0;
    updatefc_seen <= 1'b0;
  end else begin
    if (link_state != LINK_UP) begin
      axi_rst_n     <= 1'b0;
      updatefc_seen <= 1'b0;       // must see a fresh UpdateFC after every re-train
    end else begin
      if (updatefc_received)
        updatefc_seen <= 1'b1;
      axi_rst_n <= updatefc_seen || updatefc_received;
    end
  end
end

endmodule
