`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// gray_enc - 2-bit binary -> Gray encoder (PAM4 symbol coding)
// Mapping convention (locked spec): a=00,01,10,11 -> g=00,01,11,10
// Exact inverse: gray_dec.v. Port names (a, g) are canonical for both blocks.
// -----------------------------------------------------------------------------
module gray_enc(a, g);
input  [1:0] a;
output [1:0] g;
assign g[1] = a[1];
assign g[0] = a[1] ^ a[0];
endmodule
