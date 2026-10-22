`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// pam4_demap - PAM4 level -> Gray symbol slicer (inverse of pam4_map).
// -----------------------------------------------------------------------------
module pam4_demap(level_in, g_out);
input  signed [2:0] level_in;
output reg [1:0] g_out;

// Threshold slicer (not a reverse-lookup) so this degrades gracefully once
// a noisy channel model is added later - nearest-level decision, not "no match".
always @(*) begin
  if (level_in < -2)      g_out = 2'b00;
  else if (level_in < 0)  g_out = 2'b01;
  else if (level_in < 2)  g_out = 2'b11;
  else                    g_out = 2'b10;
end
endmodule
