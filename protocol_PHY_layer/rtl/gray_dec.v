// -----------------------------------------------------------------------------
// gray_dec - 2-bit Gray -> binary decoder, exact inverse of gray_enc.
// Port names (g, a) are canonical - the old gray_enc_dec.v used (g, b).
// -----------------------------------------------------------------------------
module gray_dec(g, a);
input  [1:0] g;
output [1:0] a;
assign a[1] = g[1];
assign a[0] = g[0] ^ g[1];
endmodule
