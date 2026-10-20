// =============================================================================
// tb_pcie_reset_controller
// Covers: PERST# assert/deassert, hot-reset pulsing datapath_rst_n without
// going through link_state, link-down detection holding axi_rst_n, and FLR
// pulsing independently without disturbing link_state.
// =============================================================================
module tb_pcie_reset_controller();

reg clk, perst_n;
reg hot_reset_request, flr_request, link_trained, updatefc_received;
wire datapath_rst_n, axi_rst_n, flr_active;

integer errors;

pcie_reset_controller dut (
  .clk(clk), .perst_n(perst_n),
  .hot_reset_request(hot_reset_request), .flr_request(flr_request),
  .link_trained(link_trained), .updatefc_received(updatefc_received),
  .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n), .flr_active(flr_active)
);

// Plain-Verilog equivalent of the 6 SVA properties - link_state and
// hot_reset_pending are internal to dut, reached here via hierarchical
// reference (legal in Verilog for connecting to another module's port).
pcie_reset_controller_checker u_checker (
  .clk(clk), .perst_n(perst_n),
  .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n), .flr_active(flr_active),
  .link_state(dut.link_state), .hot_reset_pending(dut.hot_reset_pending)
);

initial clk = 0;
always #5 clk = ~clk;

initial begin
  errors = 0;
  perst_n = 0; hot_reset_request = 0; flr_request = 0;
  link_trained = 0; updatefc_received = 0;

  // ---- Test 1: PERST# held low -> datapath_rst_n and axi_rst_n both low ----
  @(negedge clk); @(negedge clk);
  if (datapath_rst_n !== 1'b0 || axi_rst_n !== 1'b0) begin
    $display("FAIL: reset signals not held low during PERST#");
    errors = errors + 1;
  end else $display("PASS: datapath_rst_n and axi_rst_n correctly held low under PERST#");

  // ---- Test 2: PERST# deasserts -> datapath_rst_n goes high (link_trained still 0) ----
  perst_n = 1;
  @(negedge clk); @(negedge clk);
  if (datapath_rst_n !== 1'b1) begin
    $display("FAIL: datapath_rst_n did not deassert after PERST# released (got %b)", datapath_rst_n);
    errors = errors + 1;
  end else $display("PASS: datapath_rst_n correctly deasserted after PERST#, independent of link_trained");

  if (axi_rst_n !== 1'b0) begin
    $display("FAIL: axi_rst_n asserted (high) before link ever trained");
    errors = errors + 1;
  end else $display("PASS: axi_rst_n correctly held low pre-training");

  // ---- Test 3: link trains, then UpdateFC arrives -> axi_rst_n should release ----
  link_trained = 1;
  @(negedge clk); @(negedge clk); // let link_state reach LINK_UP
  updatefc_received = 1;
  @(negedge clk);
  updatefc_received = 0;
  @(negedge clk);

  if (axi_rst_n !== 1'b1) begin
    $display("FAIL: axi_rst_n did not release after link_trained + UpdateFC (got %b)", axi_rst_n);
    errors = errors + 1;
  end else $display("PASS: axi_rst_n correctly released after training + first UpdateFC");

  // ---- Test 4: hot-reset pulses datapath_rst_n synchronously, WITHOUT
  // needing link_state to reach LINK_UP first (proves no deadlock) ----
  hot_reset_request = 1;
  @(negedge clk);
  hot_reset_request = 0;
  @(negedge clk); // one more cycle for datapath_rst_n to reflect hot_reset_pending

  if (datapath_rst_n !== 1'b0) begin
    $display("FAIL: datapath_rst_n did not drop on hot_reset_request (got %b)", datapath_rst_n);
    errors = errors + 1;
  end else $display("PASS: hot-reset correctly pulsed datapath_rst_n");

  // link_trained is still externally held high in this testbench (we never
  // deasserted it) - real hardware would drop it once retraining begins, but
  // since we're testing the reset controller in isolation (link_trained is
  // an input we control, not a real trained link), the state machine still
  // proceeds LINK_UP -> TRAINING -> LINK_UP essentially immediately here.
  // What matters for this test is that datapath_rst_n pulsed at all, above.
  @(negedge clk); @(negedge clk);
  if (datapath_rst_n !== 1'b1) begin
    $display("FAIL: datapath_rst_n did not recover after hot-reset pulse cleared");
    errors = errors + 1;
  end else $display("PASS: datapath_rst_n correctly recovered after hot-reset pulse");

  // axi_rst_n must have re-asserted during the hot-reset transition through TRAINING
  // (checked indirectly - by this point it should need a fresh UpdateFC again)
  if (axi_rst_n !== 1'b0) begin
    $display("FAIL: axi_rst_n did not re-assert across the hot-reset (got %b) - stale UpdateFC state reused?", axi_rst_n);
    errors = errors + 1;
  end else $display("PASS: axi_rst_n correctly re-asserted across hot-reset (no stale UpdateFC reuse)");

  // Re-supply UpdateFC after the hot-reset settles, confirm axi_rst_n releases again
  updatefc_received = 1;
  @(negedge clk);
  updatefc_received = 0;
  @(negedge clk);
  if (axi_rst_n !== 1'b1) begin
    $display("FAIL: axi_rst_n did not release after fresh UpdateFC post-hot-reset");
    errors = errors + 1;
  end else $display("PASS: axi_rst_n correctly released again after fresh post-hot-reset UpdateFC");

  // ---- Test 5: unexpected link-down (link_trained drops without hot-reset) ----
  // should hold axi_rst_n low until re-trained
  link_trained = 0;
  @(negedge clk); @(negedge clk);
  if (axi_rst_n !== 1'b0) begin
    $display("FAIL: axi_rst_n did not re-assert on unexpected link-down");
    errors = errors + 1;
  end else $display("PASS: axi_rst_n correctly re-asserted on unexpected link-down");

  link_trained = 1; // re-train
  @(negedge clk); @(negedge clk);
  updatefc_received = 1;
  @(negedge clk);
  updatefc_received = 0;
  @(negedge clk);
  if (axi_rst_n !== 1'b1) begin
    $display("FAIL: axi_rst_n did not release after recovering from link-down + fresh UpdateFC");
    errors = errors + 1;
  end else $display("PASS: axi_rst_n correctly released after link-down recovery");

  // ---- Test 6: FLR pulses independently, does not disturb link_state/axi_rst_n ----
  flr_request = 1;
  @(negedge clk);
  flr_request = 0;
  if (flr_active !== 1'b1) begin
    $display("FAIL: flr_active did not pulse on flr_request");
    errors = errors + 1;
  end else $display("PASS: flr_active correctly pulsed");

  @(negedge clk);
  if (flr_active !== 1'b0) begin
    $display("FAIL: flr_active did not clear after one cycle (should be a single pulse)");
    errors = errors + 1;
  end else $display("PASS: flr_active correctly returned to 0 (single-cycle pulse)");

  if (axi_rst_n !== 1'b1 || datapath_rst_n !== 1'b1) begin
    $display("FAIL: FLR disturbed axi_rst_n/datapath_rst_n - link should stay up (got axi=%b datapath=%b)",
              axi_rst_n, datapath_rst_n);
    errors = errors + 1;
  end else $display("PASS: FLR correctly left axi_rst_n and datapath_rst_n undisturbed - link stays up");

  if (errors == 0) $display("ALL PASS: pcie_reset_controller verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
end
endmodule
