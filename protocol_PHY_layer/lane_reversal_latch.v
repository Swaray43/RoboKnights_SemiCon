module lane_reversal_latch (
input  wire clk,
input  wire rst_n,
input  wire reversal_detected_valid,
input  wire lane_reversed_in,
output reg  lane_reversed_locked,
output reg  link_trained
);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    lane_reversed_locked <= 1'b0;
    link_trained         <= 1'b0;
  end else if (reversal_detected_valid && !link_trained) begin
    // Latch once on first confirmed detection; real PCIe re-evaluates only
    // on retraining, not continuously during normal data transfer.
    lane_reversed_locked <= lane_reversed_in;
    link_trained         <= 1'b1;
  end
end
endmodule
