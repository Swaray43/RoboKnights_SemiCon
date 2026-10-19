module formal_gray_inverse (
input [1:0] b_in
);
wire [1:0] g_mid;
wire [1:0] b_out;

gray_enc enc (.a(b_in), .g(g_mid));
gray_dec dec (.g(g_mid), .a(b_out));

`ifdef FORMAL
always @(*) begin
assert(b_in == b_out);
end
`endif
endmodule
