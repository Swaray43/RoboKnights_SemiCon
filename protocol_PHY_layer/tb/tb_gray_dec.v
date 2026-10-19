module tb_gray_dec();
reg[1:0] G;
wire[1:0] B;

gray_dec uut1(G,B);

initial begin
#10 G = 2'b00; #1 check(2'b00);
#10 G = 2'b01; #1 check(2'b01);
#10 G = 2'b11; #1 check(2'b10);
#10 G = 2'b10; #1 check(2'b11);
$stop;
end

task check(input [1:0] expected);
if (B !== expected)
$display("FAIL: G=%b expected B=%b got B=%b", G, expected, B);
else
$display("PASS: G=%b B=%b", G, B);
endtask

endmodule
