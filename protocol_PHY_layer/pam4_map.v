// -----------------------------------------------------------------------------
// PAM4 mapper / de-mapper
// Locked mapping: g=00,01,11,10 -> level=-3,-1,+1,+3 (monotonic, Gray-adjacent)
// Case order intentionally follows the GRAY sequence, not binary count order.
// -----------------------------------------------------------------------------
module pam4_map(g, level);
input  [1:0] g;
output reg signed [2:0] level;

always @(*) begin
  case (g)
    2'b00: level = -3;
    2'b01: level = -1;
    2'b11: level = 1;
    2'b10: level = 3;
    default: level = 3'sbx; // unreachable: 2-bit input is fully exhaustive above
  endcase
end
endmodule
