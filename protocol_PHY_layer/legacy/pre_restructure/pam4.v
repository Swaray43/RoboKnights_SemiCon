module pam4_map(g, level);
input[1:0] g;
output reg signed [2:0] level;

always @(*) begin
case (g)
2'b00: level = -3;
2'b01: level = -1;
2'b11: level = 1;
2'b10: level = 3;
default: level = 3'sbx; // shouldn't happen, 2 bits only has 4 cases
endcase
end
endmodule

module pam4_demap(level_in, g_out);
input signed [2:0] level_in;
output reg [1:0] g_out;

always @(*) begin
if (level_in < -2)g_out = 2'b00;
else if (level_in < 0)g_out = 2'b01;
else if (level_in < 2)g_out = 2'b11;
else g_out = 2'b10;
end
endmodule


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


