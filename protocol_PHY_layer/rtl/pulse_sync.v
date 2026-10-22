`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// pulse_sync - toggle-based single-pulse clock-domain crossing (fixes I3)
//
// A 1-cycle pulse on src_pulse (src_clk) becomes a 1-cycle pulse on dst_pulse
// (dst_clk), 2-3 dst clocks later. Works for any clock ratio as long as
// source pulses are spaced at least 3 dst_clk cycles apart (session arm/end
// events are thousands of cycles apart).
//
// Used for: session_start  pcie_clk -> aclk  (interconnect needs a pulse on aclk)
//           session_end    aclk -> pcie_clk  (back into session_arm_ctrl)
// -----------------------------------------------------------------------------
module pulse_sync (
    input  wire src_clk,
    input  wire src_rst_n,
    input  wire src_pulse,
    input  wire dst_clk,
    input  wire dst_rst_n,
    output wire dst_pulse
);

reg src_toggle;
always @(posedge src_clk or negedge src_rst_n) begin
    if (!src_rst_n)     src_toggle <= 1'b0;
    else if (src_pulse) src_toggle <= ~src_toggle;
end

(* ASYNC_REG = "TRUE" *) reg sync1, sync2;
reg sync3;
always @(posedge dst_clk or negedge dst_rst_n) begin
    if (!dst_rst_n) begin
        sync1 <= 1'b0; sync2 <= 1'b0; sync3 <= 1'b0;
    end else begin
        sync1 <= src_toggle;
        sync2 <= sync1;
        sync3 <= sync2;
    end
end

assign dst_pulse = sync2 ^ sync3;
endmodule
