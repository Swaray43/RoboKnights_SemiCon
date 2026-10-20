module tb_gray_enc();
integer errors = 0;
reg[1:0] B;
wire[1:0] G;

gray_enc uut0(B,G);

initial begin
#10 B = 2'b00; #1 check(2'b00);
#10 B = 2'b01; #1 check(2'b01);
#10 B = 2'b10; #1 check(2'b11);
#10 B = 2'b11; #1 check(2'b10);
$display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
end

task check(input [1:0] expected);
if (G !== expected)
begin errors = errors + 1; $display("FAIL: B=%b expected G=%b got G=%b", B, expected, G); end
else
$display("PASS: B=%b G=%b", B, G);
endtask

endmodule
