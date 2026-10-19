// -----------------------------------------------------------------------------
// pcie_reset_controller_checker (plain Verilog-2001)
//
// Equivalent to the 6 SVA properties in pcie_reset_controller_sva.sv, but
// written as ordinary procedural checks so they actually execute in ModelSim
// Intel FPGA Edition. That edition's compiler accepted the SVA syntax
// without error, but printed:
//   "SystemVerilog testbench feature (assertion) detected... only supported
//    in Questasim" - meaning those 6 properties were parsed but NEVER
//    EVALUATED. This module is the real, running equivalent.
//
// Instantiate alongside pcie_reset_controller, wired to the same signals
// plus its internal link_state/hot_reset_pending via hierarchical reference
// from the testbench (see tb_pcie_reset_controller below).
// -----------------------------------------------------------------------------
module pcie_reset_controller_checker (
input wire clk,
input wire perst_n,
input wire datapath_rst_n,
input wire axi_rst_n,
input wire flr_active,
input wire [1:0] link_state,
input wire hot_reset_pending
);

localparam LINK_DOWN = 2'd0;
localparam TRAINING  = 2'd1;
localparam LINK_UP   = 2'd2;

reg dr_prev1, dr_prev2;      // history of datapath_rst_n, for the P2 min-hold check
reg [1:0] link_state_prev;   // for the P4 "clears in TRAINING" check

initial begin
  dr_prev1 = 1'b0;
  dr_prev2 = 1'b0;
  link_state_prev = LINK_DOWN;
end

always @(posedge clk) begin
  // ---- P1: PERST# asserted -> both resets must be low ----
  if (!perst_n) begin
    if (datapath_rst_n !== 1'b0 || axi_rst_n !== 1'b0)
      $display("SVA-EQUIV P1 VIOLATION @%0t: a reset output was high while perst_n was low (datapath_rst_n=%b axi_rst_n=%b)",
                $time, datapath_rst_n, axi_rst_n);
  end

  // ---- P2: datapath_rst_n rising edge must have been preceded by >=2 low cycles
  // (Swaray's FIFO requires this minimum hold - see To_KSHITIJ__From_Swaray.txt) ----
  if (datapath_rst_n && !dr_prev1) begin
    if (dr_prev2 !== 1'b0)
      $display("SVA-EQUIV P2 VIOLATION @%0t: datapath_rst_n deasserted without >=2 cycles low", $time);
  end

  // ---- P3: axi_rst_n high requires link_state == LINK_UP ----
  if (axi_rst_n && (link_state != LINK_UP))
    $display("SVA-EQUIV P3 VIOLATION @%0t: axi_rst_n high while link_state=%0d (!= LINK_UP)", $time, link_state);

  // ---- P4: hot_reset_pending must clear the cycle after TRAINING is entered ----
  if ((link_state_prev == TRAINING) && hot_reset_pending)
    $display("SVA-EQUIV P4 VIOLATION @%0t: hot_reset_pending did not clear after entering TRAINING", $time);

  // ---- P5: FLR must never coincide with datapath_rst_n being low ----
  if (flr_active && !datapath_rst_n)
    $display("SVA-EQUIV P5 VIOLATION @%0t: flr_active asserted while datapath_rst_n low - FLR must not drop the link", $time);

  // ---- P6: link_state must always be a legal value ----
  if (link_state !== LINK_DOWN && link_state !== TRAINING && link_state !== LINK_UP)
    $display("SVA-EQUIV P6 VIOLATION @%0t: link_state took an illegal/undefined value (%b)", $time, link_state);

  // update history
  dr_prev2 <= dr_prev1;
  dr_prev1 <= datapath_rst_n;
  link_state_prev <= link_state;
end
endmodule
