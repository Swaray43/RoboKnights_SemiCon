module tb_byte_to_symbol();
reg clk, rst_n;
reg byte_valid;
reg [7:0] byte_in;
wire symbol_valid;
wire [1:0] symbol_out;
wire busy;

integer errors, trial;
reg [7:0] test_byte;
reg [1:0] expected [0:3];
integer k;

byte_to_symbol dut(.clk(clk), .rst_n(rst_n), .byte_valid(byte_valid), .byte_in(byte_in),
                    .symbol_valid(symbol_valid), .symbol_out(symbol_out), .busy(busy));

initial clk = 0;
always #5 clk = ~clk;

initial begin
  errors = 0;
  rst_n = 0; byte_valid = 0; byte_in = 0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;
  @(negedge clk);

  for (trial = 0; trial < 8; trial = trial + 1) begin
    test_byte = trial * 8'd17; // same varied pattern used for symbol_to_byte
    expected[0] = test_byte[7:6];
    expected[1] = test_byte[5:4];
    expected[2] = test_byte[3:2];
    expected[3] = test_byte[1:0];

    byte_valid = 1; byte_in = test_byte;
    @(negedge clk); // first symbol appears same cycle byte accepted
    byte_valid = 0;

    for (k = 0; k < 4; k = k + 1) begin
      if (symbol_valid !== 1'b1 || symbol_out !== expected[k]) begin
        $display("FAIL trial=%0d symbol=%0d: expected %b got %b (valid=%b)",
                  trial, k, expected[k], symbol_out, symbol_valid);
        errors = errors + 1;
      end else begin
        $display("PASS trial=%0d symbol=%0d: %b", trial, k, symbol_out);
      end
      @(negedge clk);
    end

    if (busy !== 1'b0) begin
      $display("FAIL trial=%0d: busy still high after 4 symbols emitted", trial);
      errors = errors + 1;
    end
  end

  if (errors == 0) $display("ALL PASS: byte_to_symbol verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
end
endmodule
