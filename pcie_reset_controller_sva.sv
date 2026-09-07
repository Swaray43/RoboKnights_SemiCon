// =============================================================================
// SVA properties for pcie_reset_controller
//
// COMPILE NOTE: this file uses SystemVerilog assertion syntax (assert property,
// $past, $rose) which plain Verilog-2001 does not support. The main project
// file (pcie_phy_datapath.v) is compiled WITHOUT the -sv flag (confirmed by
// the earlier fork/join_any failure), so this file must be compiled
// SEPARATELY, with -sv:
//
//   vlog -sv pcie_reset_controller_sva.sv
//
// The checker below is bound into pcie_reset_controller via `bind`, so it
// does not require modifying pcie_reset_controller's own source at all -
// it attaches to every instance of that module automatically once both
// files are compiled into the same simulation.
//
// These properties are also the intended basis for the SymbiYosys formal
// proof of the reset sequencing FSM called out in work_distribution.md -
// the same property set applies whether run under simulation (bound here)
// or under a formal tool (typically via a separate .sby config referencing
// this checker module).
// =============================================================================

module pcie_reset_controller_checker (
input wire clk,
input wire perst_n,
input wire hot_reset_request,
input wire flr_request,
input wire link_trained,
input wire updatefc_received,
input wire datapath_rst_n,
input wire axi_rst_n,
input wire flr_active,
input wire [1:0] link_state,
input wire hot_reset_pending
);

localparam LINK_DOWN = 2'd0;
localparam TRAINING  = 2'd1;
localparam LINK_UP   = 2'd2;

// ---- P1: While PERST# is asserted (low), both datapath_rst_n and axi_rst_n
// must be held low. This is the core safety property of the whole module -
// nothing should ever look "ready" while the connector says otherwise. ----
property p_perst_holds_all_resets;
  @(posedge clk) !perst_n |-> (!datapath_rst_n && !axi_rst_n);
endproperty
assert property (p_perst_holds_all_resets)
  else $error("P1 VIOLATION: a reset output was high while perst_n was low");

// ---- P2: Swaray's module requires rst_n held low for >= 2 clock cycles to
// guarantee a clean reset (see To_KSHITIJ__From_Swaray.txt). This checks
// that whenever datapath_rst_n rises (deasserts), it was low for at least
// the two preceding cycles, not just released early. ----
property p_min_reset_hold;
  @(posedge clk) $rose(datapath_rst_n) |-> ($past(!datapath_rst_n,1) && $past(!datapath_rst_n,2));
endproperty
assert property (p_min_reset_hold)
  else $error("P2 VIOLATION: datapath_rst_n deasserted without being held low for >= 2 cycles - Swaray's FIFO requires this minimum");

// ---- P3: axi_rst_n must never be high unless link_state == LINK_UP. This
// is the core "don't tell the AXI side it's safe when the link isn't"
// invariant - the spec's real requirement restated as a checkable property. ----
property p_axi_rst_only_when_linkup;
  @(posedge clk) axi_rst_n |-> (link_state == LINK_UP);
endproperty
assert property (p_axi_rst_only_when_linkup)
  else $error("P3 VIOLATION: axi_rst_n was high while link_state != LINK_UP");

// ---- P4: hot_reset_pending must clear once TRAINING is (re-)entered, per
// the RTL's own intended behavior - if this doesn't hold, a stale pending
// flag could incorrectly force a second reset cycle after retraining. ----
property p_hot_reset_clears_in_training;
  @(posedge clk) (link_state == TRAINING) |=> !hot_reset_pending;
endproperty
assert property (p_hot_reset_clears_in_training)
  else $error("P4 VIOLATION: hot_reset_pending did not clear after entering TRAINING");

// ---- P5: FLR must never coincide with datapath_rst_n dropping. Per spec,
// FLR resets function state "while keeping the link up" - if flr_active
// and !datapath_rst_n are ever both true, FLR is incorrectly taking down
// the datapath, which contradicts its own stated scope. ----
property p_flr_does_not_drop_datapath;
  @(posedge clk) flr_active |-> datapath_rst_n;
endproperty
assert property (p_flr_does_not_drop_datapath)
  else $error("P5 VIOLATION: flr_active was asserted while datapath_rst_n was low - FLR must not drop the link");

// ---- P6: link_state must always be a legal, defined value. Mainly useful
// under formal (where state could otherwise be reached via an X-propagation
// path this simulation-only checklist wouldn't catch) - included here so
// the same property set is directly reusable for the SymbiYosys pass. ----
property p_link_state_legal;
  @(posedge clk) (link_state == LINK_DOWN) || (link_state == TRAINING) || (link_state == LINK_UP);
endproperty
assert property (p_link_state_legal)
  else $error("P6 VIOLATION: link_state took on an illegal/undefined value");

endmodule

// Bind the checker to every instance of pcie_reset_controller automatically -
// no need to touch pcie_reset_controller's own source or any testbench that
// instantiates it.
bind pcie_reset_controller pcie_reset_controller_checker u_checker (
  .clk(clk), .perst_n(perst_n),
  .hot_reset_request(hot_reset_request), .flr_request(flr_request),
  .link_trained(link_trained), .updatefc_received(updatefc_received),
  .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n), .flr_active(flr_active),
  .link_state(link_state), .hot_reset_pending(hot_reset_pending)
);
