module tb_flit_lane_striper();

reg clk;
reg rst_n;
reg valid_in;
reg [255:0] flit_in;
wire ready_out;
wire [3:0] lane_valid_out;
wire [31:0] lane_data_out;
integer cycle;
integer errors;

flit_lane_striper dut (
.clk(clk),
.rst_n(rst_n),
.valid_in(valid_in),
.flit_in(flit_in),
.ready_out(ready_out),
.lane_valid_out(lane_valid_out),
.lane_data_out(lane_data_out)
);

// Clock generation
initial clk = 0;
always #5 clk = ~clk;

// Build flit so byte N (bits [8N+7:8N]) has value N.
// e.g. byte 0 = 8'd0, byte 1 = 8'd1, ... byte 31 = 8'd31
integer i;
initial begin
	flit_in = 256'd0;
	for (i = 0; i < 32; i = i + 1) begin
		flit_in[i*8 +: 8] = i[7:0];
	end
end

task check_cycle(input integer c);
	reg [7:0] exp_lane0, exp_lane1, exp_lane2, exp_lane3;
	begin
		// Byte layout per cycle: chunk = flit_reg[c*32 +: 32]
		// byte0 -> lane0, byte1 -> lane1, byte2 -> lane2, byte3 -> lane3
		exp_lane0 = (c*4 + 0);
		exp_lane1 = (c*4 + 1);
		exp_lane2 = (c*4 + 2);
		exp_lane3 = (c*4 + 3);

		if (lane_valid_out !== 4'b1111) begin
			$display("FAIL cycle=%0d: lane_valid_out=%b expected 1111", c, lane_valid_out);
			errors = errors + 1;
		end
		if (lane_data_out[7:0] !== exp_lane0) begin
			$display("FAIL cycle=%0d lane0: got %0d expected %0d", c, lane_data_out[7:0], exp_lane0);
			errors = errors + 1;
		end
		if (lane_data_out[15:8] !== exp_lane1) begin
			$display("FAIL cycle=%0d lane1: got %0d expected %0d", c, lane_data_out[15:8], exp_lane1);
			errors = errors + 1;
		end
		if (lane_data_out[23:16] !== exp_lane2) begin
			$display("FAIL cycle=%0d lane2: got %0d expected %0d", c, lane_data_out[23:16], exp_lane2);
			errors = errors + 1;
		end
		if (lane_data_out[31:24] !== exp_lane3) begin
			$display("FAIL cycle=%0d lane3: got %0d expected %0d", c, lane_data_out[31:24], exp_lane3);
			errors = errors + 1;
		end
		if (lane_data_out[7:0]===exp_lane0 && lane_data_out[15:8]===exp_lane1 &&lane_data_out[23:16]===exp_lane2 && lane_data_out[31:24]===exp_lane3 && lane_valid_out===4'b1111) begin
			$display("PASS cycle=%0d: lane0=%0d lane1=%0d lane2=%0d lane3=%0d", c, lane_data_out[7:0], lane_data_out[15:8], lane_data_out[23:16], lane_data_out[31:24]);
		end
	end
endtask

initial begin
	errors = 0;
	rst_n = 0;
	valid_in = 0;
	cycle = 0;

	// Hold reset for a couple cycles
	@(negedge clk);
	@(negedge clk);
	rst_n = 1;

	// Push the FLIT in on a cycle where ready_out is high
	@(negedge clk);
	if (!ready_out) begin
		$display("FAIL: ready_out low when idle, cannot start test");
		$display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
	end
	valid_in = 1;
	@(negedge clk); // flit_reg now loaded, active=1, cycle_cnt=0 -> this edge shows cycle 0
	valid_in = 0; // don't push a new flit mid-stream

	// Check all 8 cycles (cycle_cnt = 0..7)
	for (cycle = 0; cycle < 8; cycle = cycle + 1) begin
		check_cycle(cycle);
		@(negedge clk);
	end

	if (errors == 0)
		$display("ALL PASS: lane striping round-robin verified for all 8 cycles");
	else
		$display("TOTAL FAILURES: %0d", errors);
	$display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
end

endmodule
