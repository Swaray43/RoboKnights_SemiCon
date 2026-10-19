module tb_pam4_roundtrip();
reg[1:0] A;
wire [1:0] G, G_rx;
wire signed [2:0] LEVEL;
wire [1:0] DEC;

gray_enc   u0 (A, G);
pam4_map   u1 (G, LEVEL);
pam4_demap u2 (LEVEL, G_rx);
gray_dec   u3 (G_rx, DEC);

initial begin
#10 A = 2'b00; #1 check;
#10 A = 2'b01; #1 check;
#10 A = 2'b10; #1 check;
#10 A = 2'b11; #1 check;
$stop;
end

task check;
if (DEC !== A)
$display("FAIL: A=%b G=%b LEVEL=%0d G_rx=%b DEC=%b", A, G, LEVEL, G_rx, DEC);
else
$display("PASS: A=%b -> LEVEL=%0d -> DEC=%b", A, LEVEL, DEC);
endtask
endmodule
