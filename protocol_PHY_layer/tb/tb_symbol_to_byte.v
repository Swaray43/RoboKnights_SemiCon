module tb_symbol_to_byte();
reg clk, rst_n;
reg symbol_valid;
reg [1:0] symbol_in;
wire byte_valid;
wire [7:0] byte_out;

integer errors;
reg [7:0] test_byte;
reg [1:0] sym0, sym1, sym2, sym3;
integer trial;

symbol_to_byte dut(.clk(clk), .rst_n(rst_n), .symbol_valid(symbol_valid),
                    .symbol_in(symbol_in), .byte_valid(byte_valid), .byte_out(byte_out));

initial clk = 0;
always #5 clk = ~clk;

task send_byte(input [7:0] b);
  begin
    sym0 = b[7:6]; sym1 = b[5:4]; sym2 = b[3:2]; sym3 = b[1:0]; // MSB-first, matches TX convention
    symbol_valid = 1; symbol_in = sym0; @(negedge clk);
    symbol_in = sym1; @(negedge clk);
    symbol_in = sym2; @(negedge clk);
    symbol_in = sym3; @(negedge clk);
    symbol_valid = 0;
  end
endtask

initial begin
  errors = 0;
  rst_n = 0; symbol_valid = 0; symbol_in = 0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;
  @(negedge clk);

  for (trial = 0; trial < 8; trial = trial + 1) begin
    test_byte = trial * 8'd17; // arbitrary varied pattern, not all-zero/all-same
    send_byte(test_byte);
    // byte_valid pulses on the same edge byte_out becomes valid (4th symbol cycle)
    if (byte_valid !== 1'b1 || byte_out !== test_byte) begin
      $display("FAIL trial=%0d: sent=%h byte_out=%h byte_valid=%b", trial, test_byte, byte_out, byte_valid);
      errors = errors + 1;
    end else begin
      $display("PASS trial=%0d: byte_out=%h", trial, byte_out);
    end
    @(negedge clk); // let byte_valid deassert before next send
  end

  if (errors == 0) $display("ALL PASS: symbol_to_byte verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule
