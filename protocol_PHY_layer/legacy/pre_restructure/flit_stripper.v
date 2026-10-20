module flit_lane_striper (
	input wire clk,
	input wire rst_n,

	// Upstream Interface (256-bit FLIT)
	input wire valid_in,
	input wire [255:0] flit_in,
	output wire ready_out,
 
	// Downstream Interface (Flattened 4 Lanes: Lane0=[7:0], Lane1=[15:8], etc.)
	output reg [3:0] lane_valid_out,
	output reg [31:0] lane_data_out
);
reg [255:0] flit_reg;
reg [2:0] cycle_cnt;
reg active;
	// Ready to accept a new FLIT when idle or finishing the last cycle of the current FLIT
assign ready_out = !active || (active && (cycle_cnt == 3'd7));
	// Stateful FLIT capture and cycle tracking
always @(posedge clk or negedge rst_n) begin
		if (!rst_n) begin
			active <= 1'b0;
			cycle_cnt <= 3'd0;
			flit_reg <= 256'd0;
		end else begin
			if (valid_in && ready_out) begin
				flit_reg <= flit_in;
				active<= 1'b1;
				cycle_cnt <= 3'd0;
			end else if (active) begin
			if (cycle_cnt == 3'd7) begin
				active <= 1'b0;
			end else begin
			cycle_cnt <= cycle_cnt + 3'd1;
			end
		end
	end
end

// Combinational routing distributing the bytes round-robin[cite: 1]
always @(*) begin
	lane_valid_out = 4'b0000;
	lane_data_out = 32'd0;

	if (active) begin
		lane_valid_out = 4'b1111;
		// Multiplex 4 bytes (32 bits) per cycle using Verilog-2001 indexed part-select
		// Cycle 0: Extracts bits [31:0](Bytes 0, 1, 2, 3)
		// Cycle 1: Extracts bits [63:32] (Bytes 4, 5, 6, 7)
		lane_data_out = flit_reg[cycle_cnt * 32 +: 32];
	end
end

endmodule

module flit_lane_destriper (
input wire clk,
input wire rst_n,

// Upstream Interface (from lane realignment / demap+decode side)
input wire [3:0] lane_valid_in,
input wire [31:0] lane_data_in,

// Downstream Interface (reassembled 256-bit FLIT)
output reg flit_valid_out,
output reg [255:0] flit_out
);

reg [255:0] flit_reg;
reg [2:0] cycle_cnt;
reg active;

wire all_lanes_valid = &lane_valid_in; // 4'b1111 check

// Stateful reassembly and cycle tracking
always @(posedge clk or negedge rst_n) begin
if (!rst_n) begin
active         <= 1'b0;
cycle_cnt      <= 3'd0;
flit_reg       <= 256'd0;
flit_valid_out <= 1'b0;
flit_out       <= 256'd0;
end else begin
flit_valid_out <= 1'b0; // default: pulse only on completion

if (all_lanes_valid) begin
// Write this cycle's 32 bits into the matching slice, inverse of the striper's extraction
flit_reg[cycle_cnt * 32 +: 32] <= lane_data_in;
active <= 1'b1;

if (cycle_cnt == 3'd7) begin
// Last cycle of this FLIT: latch the completed FLIT out, reset for next
flit_out       <= {lane_data_in, flit_reg[223:0]};
flit_valid_out <= 1'b1;
cycle_cnt      <= 3'd0;
active         <= 1'b0;
end else begin
cycle_cnt <= cycle_cnt + 3'd1;
end
end
end
end

endmodule

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
		$stop;
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
	$stop;
end

endmodule

module tb_flit_lane_destriper();

reg clk, rst_n;
reg [3:0] lane_valid_in;
reg [31:0] lane_data_in;
wire flit_valid_out;
wire [255:0] flit_out;

integer cycle, errors;
reg [255:0] expected_flit;
integer i;

flit_lane_destriper dut (
  .clk(clk), .rst_n(rst_n),
  .lane_valid_in(lane_valid_in),
  .lane_data_in(lane_data_in),
  .flit_valid_out(flit_valid_out),
  .flit_out(flit_out)
);

initial clk = 0;
always #5 clk = ~clk;

initial begin
  expected_flit = 256'd0;
  for (i = 0; i < 32; i = i + 1)
    expected_flit[i*8 +: 8] = i[7:0];
end

initial begin
  errors = 0;
  rst_n = 0;
  lane_valid_in = 4'b0000;
  lane_data_in = 32'd0;

  @(negedge clk);
  @(negedge clk);
  rst_n = 1;

  // Drive 8 cycles of lane data, byte N = index N, same pattern as the striper TB
  for (cycle = 0; cycle < 8; cycle = cycle + 1) begin
    lane_valid_in = 4'b1111;
//    lane_data_in  = {(cycle*4+3), (cycle*4+2), (cycle*4+1), (cycle*4+0)};
    lane_data_in = {8'((cycle*4+3)), 8'((cycle*4+2)), 8'((cycle*4+1)), 8'((cycle*4+0))};//{Lane 3,Lane 2,Lane 1,Lane 0}
    @(negedge clk);
  end
  lane_valid_in = 4'b0000;

  if (flit_valid_out !== 1'b1) begin
    $display("FAIL: flit_valid_out not asserted after 8 cycles");
    errors = errors + 1;
  end

  if (flit_out !== expected_flit) begin
    $display("FAIL: flit_out mismatch");
    $display("  got:      %h", flit_out);
    $display("  expected: %h", expected_flit);
    errors = errors + 1;
  end else begin
    $display("PASS: flit_out reconstructed correctly = %h", flit_out);
  end

  if (errors == 0)
    $display("ALL PASS: de-striper round-trip verified");
  else
    $display("TOTAL FAILURES: %0d", errors);

  $stop;
end

endmodule

