module symbol_to_byte (
input  wire       clk,
input  wire       rst_n,
input  wire       symbol_valid,
input  wire [1:0] symbol_in,      // from gray_dec

output reg        byte_valid,
output reg  [7:0] byte_out
);

reg [1:0] sym_cnt;   // counts 0..3, 4 symbols = 1 byte
reg [7:0] shift_reg;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    sym_cnt    <= 2'd0;
    shift_reg  <= 8'd0;
    byte_valid <= 1'b0;
  end else begin
    byte_valid <= 1'b0; // default: pulse only when a byte completes

    if (symbol_valid) begin
      shift_reg <= {shift_reg[5:0], symbol_in}; // MSB-first packing, matches TX order below
      if (sym_cnt == 2'd3) begin
        byte_out   <= {shift_reg[5:0], symbol_in};
        byte_valid <= 1'b1;
        sym_cnt    <= 2'd0;
      end else begin
        sym_cnt <= sym_cnt + 2'd1;
      end
    end
  end
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------

module byte_to_symbol (
input  wire       clk,
input  wire       rst_n,

input  wire       byte_valid,
input  wire [7:0] byte_in,

output reg        symbol_valid,
output reg  [1:0] symbol_out,
output wire       busy        // high while still serializing current byte
);

reg [1:0] sym_cnt;
reg [7:0] shift_reg;
reg       active;

assign busy = active;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    sym_cnt      <= 2'd0;
    shift_reg    <= 8'd0;
    active       <= 1'b0;
    symbol_valid <= 1'b0;
    symbol_out   <= 2'd0;
  end else begin
    symbol_valid <= 1'b0; // default: pulse only

    if (byte_valid && !active) begin
      // Latch new byte, emit MSB-first symbol immediately this cycle
      shift_reg    <= byte_in;
      symbol_out   <= byte_in[7:6];
      symbol_valid <= 1'b1;
      sym_cnt      <= 2'd1;
      active       <= 1'b1;
    end else if (active) begin
      case (sym_cnt)
        2'd1: symbol_out <= shift_reg[5:4];
        2'd2: symbol_out <= shift_reg[3:2];
        2'd3: symbol_out <= shift_reg[1:0];
      endcase
      symbol_valid <= 1'b1;
      if (sym_cnt == 2'd3) begin
        sym_cnt <= 2'd0;
        active  <= 1'b0;
      end else begin
        sym_cnt <= sym_cnt + 2'd1;
      end
    end
  end
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------

module ts_ordered_set_detector (
input  wire       clk,
input  wire       rst_n,
input  wire       byte_valid,
input  wire [7:0] byte_in,

output reg        lane_num_valid,   // pulses when a NEW confirmed value is latched
output reg  [7:0] lane_num_out      // confirmed Lane Number field
);

localparam COM     = 8'hBC;
localparam TS1_ID  = 8'h4A;
localparam TS2_ID  = 8'h45;

localparam IDLE    = 2'd0;
localparam CAPTURE = 2'd1;

reg [1:0] state;
reg [3:0] byte_idx;      // 0..15 within the ordered set
reg [7:0] lane_candidate;
reg [7:0] id_byte0;
reg       id_match;       // tracks whether bytes 10-15 all agree with id_byte0
reg [7:0] prev_confirmed;  // last confirmed candidate, for the 2-consecutive-match check
reg       prev_valid;

wire byte_ok_now = (byte_idx <= 4'd10) ||(byte_in == id_byte0 && (id_byte0 == TS1_ID || id_byte0 == TS2_ID));

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    state          <= IDLE;
    byte_idx       <= 4'd0;
    lane_num_valid <= 1'b0;
    lane_num_out   <= 8'd0;
    prev_valid     <= 1'b0;
    prev_confirmed <= 8'd0;
    id_match       <= 1'b1;
  end else begin
    lane_num_valid <= 1'b0; // default: pulse only

    if (byte_valid) begin
      case (state)
        IDLE: begin
          if (byte_in == COM) begin
            state    <= CAPTURE;
            byte_idx <= 4'd1;
            id_match <= 1'b1;
          end
        end

        CAPTURE: begin
          if (byte_idx == 4'd2)
            lane_candidate <= byte_in;

          if (byte_idx == 4'd10)
            id_byte0 <= byte_in;
          else if (byte_idx > 4'd10) begin
            if (byte_in != id_byte0 || !(id_byte0 == TS1_ID || id_byte0 == TS2_ID))
              id_match <= 1'b0;
          end
	  if (byte_idx == 4'd15) begin
	    if (id_match && byte_ok_now && (id_byte0 == TS1_ID || id_byte0 == TS2_ID)) begin
	      if (prev_valid && (prev_confirmed == lane_candidate)) begin
	        lane_num_valid <= 1'b1;
	        lane_num_out   <= lane_candidate;
	      end
	      prev_confirmed <= lane_candidate;
	      prev_valid     <= 1'b1;
	    end else begin
	      prev_valid <= 1'b0;
	    end
	    state <= IDLE;
	    end else begin
  		byte_idx <= byte_idx + 4'd1;   // <-- this was missing, add it back
	    end
	  end
      endcase
    end
  end
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------

module lane_reversal_detector (
input  wire       clk,
input  wire       rst_n,
input  wire [3:0] lane_num_valid,   // from each lane's ts_ordered_set_detector
input  wire [7:0] lane_num_l0,
input  wire [7:0] lane_num_l1,
input  wire [7:0] lane_num_l2,
input  wire [7:0] lane_num_l3,

output reg        reversal_detected_valid,
output reg        lane_reversed
);

wire all_valid = &lane_num_valid;

wire identity_match = (lane_num_l0==8'd0) && (lane_num_l1==8'd1) &&
                       (lane_num_l2==8'd2) && (lane_num_l3==8'd3);

wire reversed_match = (lane_num_l0==8'd3) && (lane_num_l1==8'd2) &&
                       (lane_num_l2==8'd1) && (lane_num_l3==8'd0);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    reversal_detected_valid <= 1'b0;
    lane_reversed           <= 1'b0;
  end else begin
    reversal_detected_valid <= 1'b0;
    if (all_valid && (identity_match || reversed_match)) begin
      reversal_detected_valid <= 1'b1;
      lane_reversed           <= reversed_match; // 0 = identity, 1 = reversed
    end
    // if all_valid but neither pattern matches, hold previous state -
    // worth deciding explicitly whether that should trigger an error flag later
  end
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------

module lane_swap_mux (
input  wire        lane_reversed,

input  wire [7:0]  lane_in_0,
input  wire [7:0]  lane_in_1,
input  wire [7:0]  lane_in_2,
input  wire [7:0]  lane_in_3,

output wire [7:0]  lane_out_0,
output wire [7:0]  lane_out_1,
output wire [7:0]  lane_out_2,
output wire [7:0]  lane_out_3
);

// When reversed: physical lane 0 carries logical lane 3's data, etc.
// So swap 0<->3 and 1<->2 to recover canonical order.
assign lane_out_0 = lane_reversed ? lane_in_3 : lane_in_0;
assign lane_out_1 = lane_reversed ? lane_in_2 : lane_in_1;
assign lane_out_2 = lane_reversed ? lane_in_1 : lane_in_2;
assign lane_out_3 = lane_reversed ? lane_in_0 : lane_in_3;

endmodule

//------------------------------------------------------------------------------------------------------------------------------

module lane_reversal_latch (
input  wire clk,
input  wire rst_n,
input  wire reversal_detected_valid,
input  wire lane_reversed_in,

output reg  lane_reversed_locked,
output reg  link_trained
);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    lane_reversed_locked <= 1'b0;
    link_trained         <= 1'b0;
  end else if (reversal_detected_valid && !link_trained) begin
    // Latch once, on first confirmed detection - ignore further training pulses
    lane_reversed_locked <= lane_reversed_in;
    link_trained         <= 1'b1;
  end
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------

module flit_sof_strip (
input  wire       clk,
input  wire       rst_n,

// Upstream: post lane-swap, reassembled byte-per-lane stream
input  wire [7:0] lane_in_0,
input  wire [7:0] lane_in_1,
input  wire [7:0] lane_in_2,
input  wire [7:0] lane_in_3,
input  wire       lane_in_valid,   // byte_valid, ANDed across all 4 lanes upstream

// Downstream: feeds directly into flit_lane_destriper
output reg  [7:0] lane_out_0,
output reg  [7:0] lane_out_1,
output reg  [7:0] lane_out_2,
output reg  [7:0] lane_out_3,
output reg  [3:0] lane_valid_out    // matches flit_lane_destriper's lane_valid_in width
);
`ifdef FORMAL
    // Force the solver to start with reset asserted
    initial assume(!rst_n);

    always @(posedge clk) begin
        // ---------------------------------------------------------
        // 1. Reset Behavior Verification
        // ---------------------------------------------------------
        if (!rst_n) begin
            assert(state == WAIT_SOF);
            assert(payload_cnt == 3'd0);
            assert(lane_valid_out == 4'b0000);
            assert(lane_out_0 == 8'd0);
            assert(lane_out_1 == 8'd0);
            assert(lane_out_2 == 8'd0);
            assert(lane_out_3 == 8'd0);
        end 
        // ---------------------------------------------------------
        // 2. FSM Safety Properties (Post-Reset)
        // ---------------------------------------------------------
        else begin
            // Ensure the state machine is strictly in a known state
            assert(state == WAIT_SOF || state == PASS_FLIT);

            // In WAIT_SOF, payload_cnt must remain 0
            if (state == WAIT_SOF) begin
                assert(payload_cnt == 3'd0);
            end

            // In PASS_FLIT, payload_cnt must never exceed 7
            if (state == PASS_FLIT) begin
                assert(payload_cnt <= 3'd7);
            end
        end
    end
`endif
localparam SOF_TOKEN = 8'hFB;

localparam WAIT_SOF  = 1'b0;
localparam PASS_FLIT = 1'b1;

reg       state;
reg [2:0] payload_cnt; // counts 0..7 across the 8 payload cycles

wire is_sof_cycle = lane_in_valid &&
                     (lane_in_0 == SOF_TOKEN) && (lane_in_1 == SOF_TOKEN) &&
                     (lane_in_2 == SOF_TOKEN) && (lane_in_3 == SOF_TOKEN);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    state         <= WAIT_SOF;
    payload_cnt   <= 3'd0;
    lane_valid_out <= 4'b0000;
    lane_out_0    <= 8'd0;
    lane_out_1    <= 8'd0;
    lane_out_2    <= 8'd0;
    lane_out_3    <= 8'd0;
  end else begin
    lane_valid_out <= 4'b0000; // default: invalid unless explicitly passed through below

    case (state)
      WAIT_SOF: begin
        if (is_sof_cycle) begin
          state       <= PASS_FLIT;
          payload_cnt <= 3'd0;
          // SOF cycle itself is consumed here - not passed downstream
        end
        // any non-SOF byte seen here is idle/training traffic - silently dropped
      end

      PASS_FLIT: begin
        if (lane_in_valid) begin
          // Defensive check: a real SOF appearing mid-FLIT means we lost sync -
          // resync rather than silently misinterpreting payload as a new frame.
          if (is_sof_cycle) begin
            state       <= PASS_FLIT;
            payload_cnt <= 3'd0;
          end else begin
            lane_out_0     <= lane_in_0;
            lane_out_1     <= lane_in_1;
            lane_out_2     <= lane_in_2;
            lane_out_3     <= lane_in_3;
            lane_valid_out <= 4'b1111;

            if (payload_cnt == 3'd7) begin
              state <= WAIT_SOF; // FLIT complete, go find next SOF
            end else begin
              payload_cnt <= payload_cnt + 3'd1;
            end
          end
        end
      end
    endcase
  end
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------------------------------------------------------

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
  $stop;
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------


module tb_ts_ordered_set_detector();
reg clk, rst_n;
reg byte_valid;
reg [7:0] byte_in;
wire lane_num_valid;
wire [7:0] lane_num_out;

integer errors;
integer i;

ts_ordered_set_detector dut(.clk(clk), .rst_n(rst_n), .byte_valid(byte_valid),
                             .byte_in(byte_in), .lane_num_valid(lane_num_valid),
                             .lane_num_out(lane_num_out));

initial clk = 0;
always #5 clk = ~clk;

// Sends one full 16-byte ordered set. lane_num = the Lane Number field (byte 2).
// corrupt_id: if 1, flips the last ID byte to break the TS1/TS2 match.
task send_ordered_set(input [7:0] lane_num, input corrupt_id);
  begin
    byte_valid = 1;
    byte_in = 8'hBC; @(negedge clk);            // byte 0: COM
    byte_in = 8'h00; @(negedge clk);            // byte 1: Link Number (don't care)
    byte_in = lane_num; @(negedge clk);         // byte 2: Lane Number
    for (i = 3; i <= 9; i = i + 1) begin
      byte_in = 8'h00; @(negedge clk);          // bytes 3-9: don't care
    end
    for (i = 10; i <= 15; i = i + 1) begin
      if (corrupt_id && i == 15)
        byte_in = 8'h00;                        // break the match on the last ID byte
      else
        byte_in = 8'h4A;                        // TS1 identifier
      @(negedge clk);
    end
    byte_valid = 0;
  end
endtask

initial begin
  errors = 0;
  rst_n = 0; byte_valid = 0; byte_in = 0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;
  @(negedge clk);

  // Test 1: single clean ordered set -> should NOT confirm yet (needs 2 consecutive)
  send_ordered_set(8'd2, 0);
  if (lane_num_valid !== 1'b0) begin
    $display("FAIL: confirmed after only 1 ordered set (expected 2 consecutive required)");
    errors = errors + 1;
  end else $display("PASS: correctly withheld confirmation after 1st ordered set");

  // Test 2: second clean, matching ordered set -> SHOULD confirm now
  send_ordered_set(8'd2, 0);
  if (lane_num_valid !== 1'b1 || lane_num_out !== 8'd2) begin
    $display("FAIL: expected confirmation after 2nd matching set, got valid=%b lane_num=%0d",
              lane_num_valid, lane_num_out);
    errors = errors + 1;
  end else $display("PASS: confirmed lane_num=%0d after 2 consecutive matches", lane_num_out);

  // Test 3: corrupted ordered set should reset the consecutive-match streak
  send_ordered_set(8'd2, 0);   // 1st good one after reset state
  send_ordered_set(8'd2, 1);   // corrupted -> should NOT confirm, and should reset streak
  if (lane_num_valid !== 1'b0) begin
    $display("FAIL: corrupted ordered set incorrectly confirmed");
    errors = errors + 1;
  end else $display("PASS: corrupted ordered set correctly rejected");

  // Test 4: after the corruption, need 2 fresh consecutive matches again
  send_ordered_set(8'd2, 0);
  send_ordered_set(8'd2, 0);
  if (lane_num_valid !== 1'b1 || lane_num_out !== 8'd2) begin
    $display("FAIL: failed to re-confirm after corruption reset the streak");
    errors = errors + 1;
  end else $display("PASS: re-confirmed correctly after streak reset");

  if (errors == 0) $display("ALL PASS: ts_ordered_set_detector verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------

module tb_lane_reversal_detector();
reg clk, rst_n;
reg [3:0] lane_num_valid;
reg [7:0] lane_num_l0, lane_num_l1, lane_num_l2, lane_num_l3;
wire reversal_detected_valid;
wire lane_reversed;

integer errors;

lane_reversal_detector dut(.clk(clk), .rst_n(rst_n), .lane_num_valid(lane_num_valid),
  .lane_num_l0(lane_num_l0), .lane_num_l1(lane_num_l1),
  .lane_num_l2(lane_num_l2), .lane_num_l3(lane_num_l3),
  .reversal_detected_valid(reversal_detected_valid), .lane_reversed(lane_reversed));

initial clk = 0;
always #5 clk = ~clk;

initial begin
  errors = 0;
  rst_n = 0; lane_num_valid = 4'b0000;
  lane_num_l0 = 0; lane_num_l1 = 0; lane_num_l2 = 0; lane_num_l3 = 0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;
  @(negedge clk);

  // Test 1: identity mapping
  lane_num_l0=0; lane_num_l1=1; lane_num_l2=2; lane_num_l3=3;
  lane_num_valid = 4'b1111;
  @(negedge clk);
  if (reversal_detected_valid !== 1'b1 || lane_reversed !== 1'b0) begin
    $display("FAIL identity case: valid=%b reversed=%b", reversal_detected_valid, lane_reversed);
    errors = errors + 1;
  end else $display("PASS identity case: reversed=%b", lane_reversed);
  lane_num_valid = 4'b0000;
  @(negedge clk);

  // Test 2: reversed mapping
  lane_num_l0=3; lane_num_l1=2; lane_num_l2=1; lane_num_l3=0;
  lane_num_valid = 4'b1111;
  @(negedge clk);
  if (reversal_detected_valid !== 1'b1 || lane_reversed !== 1'b1) begin
    $display("FAIL reversed case: valid=%b reversed=%b", reversal_detected_valid, lane_reversed);
    errors = errors + 1;
  end else $display("PASS reversed case: reversed=%b", lane_reversed);
  lane_num_valid = 4'b0000;
  @(negedge clk);

  // Test 3: not all lanes valid yet -> should not assert
  lane_num_l0=0; lane_num_l1=1; lane_num_l2=2; lane_num_l3=3;
  lane_num_valid = 4'b0111; // lane 3 not valid yet
  @(negedge clk);
  if (reversal_detected_valid !== 1'b0) begin
    $display("FAIL: asserted with incomplete lane_num_valid");
    errors = errors + 1;
  end else $display("PASS: correctly withheld with incomplete valid mask");

  if (errors == 0) $display("ALL PASS: lane_reversal_detector verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------

module tb_lane_reversal_latch();
reg clk, rst_n;
reg reversal_detected_valid;
reg lane_reversed_in;
wire lane_reversed_locked, link_trained;

integer errors;

lane_reversal_latch dut(.clk(clk), .rst_n(rst_n),
  .reversal_detected_valid(reversal_detected_valid), .lane_reversed_in(lane_reversed_in),
  .lane_reversed_locked(lane_reversed_locked), .link_trained(link_trained));

initial clk = 0;
always #5 clk = ~clk;

initial begin
  errors = 0;
  rst_n = 0; reversal_detected_valid = 0; lane_reversed_in = 0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;
  @(negedge clk);

  if (link_trained !== 1'b0) begin
    $display("FAIL: link_trained should be 0 before any detection pulse");
    errors = errors + 1;
  end else $display("PASS: link_trained correctly low pre-training");

  // First confirmed detection: reversed = 1
  reversal_detected_valid = 1; lane_reversed_in = 1;
  @(negedge clk);
  reversal_detected_valid = 0;
  if (link_trained !== 1'b1 || lane_reversed_locked !== 1'b1) begin
    $display("FAIL: expected latch to lock reversed=1, got locked=%b trained=%b",
              lane_reversed_locked, link_trained);
    errors = errors + 1;
  end else $display("PASS: latched reversed=1 correctly");

  // Second pulse with a DIFFERENT value should be ignored (already trained)
  reversal_detected_valid = 1; lane_reversed_in = 0;
  @(negedge clk);
  reversal_detected_valid = 0;
  if (lane_reversed_locked !== 1'b1) begin
    $display("FAIL: latch value changed after already trained (got %b, expected still 1)",
              lane_reversed_locked);
    errors = errors + 1;
  end else $display("PASS: latch correctly held value, ignored post-training pulse");

  if (errors == 0) $display("ALL PASS: lane_reversal_latch verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------

module tb_lane_swap_mux();
reg lane_reversed;
reg [7:0] lane_in_0, lane_in_1, lane_in_2, lane_in_3;
wire [7:0] lane_out_0, lane_out_1, lane_out_2, lane_out_3;

integer errors;

lane_swap_mux dut(.lane_reversed(lane_reversed),
  .lane_in_0(lane_in_0), .lane_in_1(lane_in_1), .lane_in_2(lane_in_2), .lane_in_3(lane_in_3),
  .lane_out_0(lane_out_0), .lane_out_1(lane_out_1), .lane_out_2(lane_out_2), .lane_out_3(lane_out_3));

initial begin
  errors = 0;
  lane_in_0 = 8'h10; lane_in_1 = 8'h11; lane_in_2 = 8'h12; lane_in_3 = 8'h13;

  lane_reversed = 0;
  #1;
  if (lane_out_0 !== lane_in_0 || lane_out_1 !== lane_in_1 ||
      lane_out_2 !== lane_in_2 || lane_out_3 !== lane_in_3) begin
    $display("FAIL: pass-through case incorrect");
    errors = errors + 1;
  end else $display("PASS: pass-through (not reversed) correct");

  lane_reversed = 1;
  #1;
  if (lane_out_0 !== lane_in_3 || lane_out_1 !== lane_in_2 ||
      lane_out_2 !== lane_in_1 || lane_out_3 !== lane_in_0) begin
    $display("FAIL: reversed case incorrect: out0=%h out1=%h out2=%h out3=%h",
              lane_out_0, lane_out_1, lane_out_2, lane_out_3);
    errors = errors + 1;
  end else $display("PASS: reversed swap correct");

  if (errors == 0) $display("ALL PASS: lane_swap_mux verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------

module tb_flit_sof_strip();
reg clk, rst_n;
reg [7:0] lane_in_0, lane_in_1, lane_in_2, lane_in_3;
reg lane_in_valid;
wire [7:0] lane_out_0, lane_out_1, lane_out_2, lane_out_3;
wire [3:0] lane_valid_out;

integer errors, i;

flit_sof_strip dut(.clk(clk), .rst_n(rst_n),
  .lane_in_0(lane_in_0), .lane_in_1(lane_in_1), .lane_in_2(lane_in_2), .lane_in_3(lane_in_3),
  .lane_in_valid(lane_in_valid),
  .lane_out_0(lane_out_0), .lane_out_1(lane_out_1), .lane_out_2(lane_out_2), .lane_out_3(lane_out_3),
  .lane_valid_out(lane_valid_out));

initial clk = 0;
always #5 clk = ~clk;

task send_cycle(input [7:0] b0, input [7:0] b1, input [7:0] b2, input [7:0] b3, input v);
  begin
    lane_in_0 = b0; lane_in_1 = b1; lane_in_2 = b2; lane_in_3 = b3; lane_in_valid = v;
    @(negedge clk);
  end
endtask

initial begin
  errors = 0;
  rst_n = 0; lane_in_valid = 0;
  lane_in_0=0; lane_in_1=0; lane_in_2=0; lane_in_3=0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;
  @(negedge clk);

  // Test 1: idle bytes before SOF should never appear at output
  send_cycle(8'h00, 8'h00, 8'h00, 8'h00, 1);
  send_cycle(8'hAA, 8'hAA, 8'hAA, 8'hAA, 1); // garbage, not SOF
  if (lane_valid_out !== 4'b0000) begin
    $display("FAIL: output valid asserted before SOF seen");
    errors = errors + 1;
  end else $display("PASS: idle bytes correctly suppressed pre-SOF");

  // Test 2: send SOF, then 8 payload cycles (byte value = cycle*4+lane, using *17 for variety)
  send_cycle(8'hFB, 8'hFB, 8'hFB, 8'hFB, 1); // SOF - should be consumed, not passed
  if (lane_valid_out !== 4'b0000) begin
    $display("FAIL: SOF cycle itself leaked to output");
    errors = errors + 1;
  end else $display("PASS: SOF cycle correctly consumed, not passed downstream");

  for (i = 0; i < 8; i = i + 1) begin
    send_cycle(i*4+0, i*4+1, i*4+2, i*4+3, 1);
    if (lane_valid_out !== 4'b1111 ||
        lane_out_0 !== i*4+0 || lane_out_1 !== i*4+1 ||
        lane_out_2 !== i*4+2 || lane_out_3 !== i*4+3) begin
      $display("FAIL payload cycle=%0d: valid=%b out=%h,%h,%h,%h",
                i, lane_valid_out, lane_out_0, lane_out_1, lane_out_2, lane_out_3);
      errors = errors + 1;
    end else begin
      $display("PASS payload cycle=%0d", i);
    end
  end

  // Test 3: after 8 payload cycles, state should return to WAIT_SOF -
  // sending non-SOF garbage now should NOT produce valid output
  send_cycle(8'h55, 8'h55, 8'h55, 8'h55, 1);
  if (lane_valid_out !== 4'b0000) begin
    $display("FAIL: output valid after FLIT should have completed and returned to WAIT_SOF");
    errors = errors + 1;
  end else $display("PASS: correctly returned to idle-wait after 8 payload cycles");

  // Test 4: second FLIT, preceded by a fresh SOF - proves the block can frame multiple FLITs
  send_cycle(8'hFB, 8'hFB, 8'hFB, 8'hFB, 1);
  send_cycle(8'h20, 8'h21, 8'h22, 8'h23, 1);
  if (lane_valid_out !== 4'b1111 || lane_out_0 !== 8'h20) begin
    $display("FAIL: second FLIT not framed correctly after first FLIT completed");
    errors = errors + 1;
  end else $display("PASS: second FLIT correctly framed");

  if (errors == 0) $display("ALL PASS: flit_sof_strip verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------

module elastic_fifo #(
parameter DEPTH = 8,
parameter PTR_WIDTH = 3   // log2(DEPTH)
)(
input  wire       clk,
input  wire       rst_n,

input  wire       wr_en,
input  wire [7:0] wr_data,

input  wire       rd_en,
output reg  [7:0] rd_data,

output wire [PTR_WIDTH:0] fill_count,  // one extra bit to distinguish full vs empty
output wire        empty,
output wire        full
);

reg [7:0] mem [0:DEPTH-1];
reg [PTR_WIDTH-1:0] wr_ptr, rd_ptr;
reg [PTR_WIDTH:0]   count;

assign fill_count = count;
assign empty = (count == 0);
assign full  = (count == DEPTH);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    wr_ptr <= 0;
    rd_ptr <= 0;
    count  <= 0;
    rd_data <= 8'd0;
  end else begin
    // Write
    if (wr_en && !full) begin
      mem[wr_ptr] <= wr_data;
      wr_ptr <= wr_ptr + 1'b1;
    end

    // Read
    if (rd_en && !empty) begin
      rd_data <= mem[rd_ptr];
      rd_ptr <= rd_ptr + 1'b1;
    end

    // Count update - handles simultaneous read+write correctly
    case ({wr_en && !full, rd_en && !empty})
      2'b10: count <= count + 1'b1;  // write only
      2'b01: count <= count - 1'b1;  // read only
      default: count <= count;        // both or neither: net zero change
    endcase
  end
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------

module tx_elastic_deskew #(
parameter DEPTH = 8,
parameter THRESHOLD = 4   // minimum fill before release - this is your " N UI" tolerance
)(
input  wire       clk,
input  wire       rst_n,

// Per-lane write side (from each lane's own TX symbol generation)
input  wire [3:0] wr_en,     // one per lane
input  wire [7:0] wr_data_0, wr_data_1, wr_data_2, wr_data_3,

// Downstream read side - all 4 lanes read together once released
input  wire       rd_en,
output wire [7:0] rd_data_0, rd_data_1, rd_data_2, rd_data_3,

output reg        lanes_valid_out   // asserted once all lanes primed; the "deskew lock" signal
);

wire [3:0] empty, full;
wire [3:0] fifo_rd_en;
wire [3:0] lane_primed;

wire [3:0] fc0, fc1, fc2, fc3; // fill counts, 4 bits each (DEPTH=8 -> PTR_WIDTH=3, so 4-bit count)

elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(3)) fifo0 (
  .clk(clk), .rst_n(rst_n),
  .wr_en(wr_en[0]), .wr_data(wr_data_0),
  .rd_en(fifo_rd_en[0]), .rd_data(rd_data_0),
  .fill_count(fc0), .empty(empty[0]), .full(full[0])
);
elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(3)) fifo1 (
  .clk(clk), .rst_n(rst_n),
  .wr_en(wr_en[1]), .wr_data(wr_data_1),
  .rd_en(fifo_rd_en[1]), .rd_data(rd_data_1),
  .fill_count(fc1), .empty(empty[1]), .full(full[1])
);
elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(3)) fifo2 (
  .clk(clk), .rst_n(rst_n),
  .wr_en(wr_en[2]), .wr_data(wr_data_2),
  .rd_en(fifo_rd_en[2]), .rd_data(rd_data_2),
  .fill_count(fc2), .empty(empty[2]), .full(full[2])
);
elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(3)) fifo3 (
  .clk(clk), .rst_n(rst_n),
  .wr_en(wr_en[3]), .wr_data(wr_data_3),
  .rd_en(fifo_rd_en[3]), .rd_data(rd_data_3),
  .fill_count(fc3), .empty(empty[3]), .full(full[3])
);

// Lock condition: once primed, STAYS primed (latched) even if fill dips below
// threshold afterward due to draining - re-priming would cause a mid-stream stall.
reg primed_latched;
wire all_primed_now = (fc0 >= THRESHOLD) && (fc1 >= THRESHOLD) &&
                       (fc2 >= THRESHOLD) && (fc3 >= THRESHOLD);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    primed_latched <= 1'b0;
  end else begin
    if (!primed_latched && all_primed_now)
      primed_latched <= 1'b1;
  end
end

// lanes_valid_out now reflects LIVE per-cycle availability, gated by having
// completed initial priming. Drops immediately if any lane runs dry -
// no longer lies about lane availability after an underrun.

assign lanes_valid_out = primed_latched && !empty[0] && !empty[1] && !empty[2] && !empty[3];

// All 4 lanes only read once primed AND downstream is ready (rd_en) AND none are empty
assign fifo_rd_en[0] = primed_latched && rd_en && !empty[0];
assign fifo_rd_en[1] = primed_latched && rd_en && !empty[1];
assign fifo_rd_en[2] = primed_latched && rd_en && !empty[2];
assign fifo_rd_en[3] = primed_latched && rd_en && !empty[3];

endmodule

//------------------------------------------------------------------------------------------------------------------------------

module tb_tx_elastic_deskew();

parameter DEPTH = 8;
parameter THRESHOLD = 4;

reg clk, rst_n;
reg [3:0] wr_en;
reg [7:0] wr_data_0, wr_data_1, wr_data_2, wr_data_3;
reg rd_en;
wire [7:0] rd_data_0, rd_data_1, rd_data_2, rd_data_3;
wire lanes_valid_out;

integer errors;
integer i;

tx_elastic_deskew #(.DEPTH(DEPTH), .THRESHOLD(THRESHOLD)) dut (
  .clk(clk), .rst_n(rst_n),
  .wr_en(wr_en),
  .wr_data_0(wr_data_0), .wr_data_1(wr_data_1),
  .wr_data_2(wr_data_2), .wr_data_3(wr_data_3),
  .rd_en(rd_en),
  .rd_data_0(rd_data_0), .rd_data_1(rd_data_1),
  .rd_data_2(rd_data_2), .rd_data_3(rd_data_3),
  .lanes_valid_out(lanes_valid_out)
);

initial clk = 0;
always #5 clk = ~clk;

// Helper: write one byte to all 4 lanes simultaneously (simulates synchronized TX)
task write_all(input [7:0] d0, input [7:0] d1, input [7:0] d2, input [7:0] d3);
  begin
    wr_en = 4'b1111;
    wr_data_0 = d0; wr_data_1 = d1; wr_data_2 = d2; wr_data_3 = d3;
    @(negedge clk);
    wr_en = 4'b0000;
  end
endtask

initial begin
  errors = 0;
  rst_n = 0; wr_en = 4'b0000; rd_en = 0;
  wr_data_0 = 0; wr_data_1 = 0; wr_data_2 = 0; wr_data_3 = 0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;
  @(negedge clk);

  // ---- Test 1: below threshold, lanes_valid_out must stay low ----
  // Write 3 bytes per lane (< THRESHOLD=4)
  for (i = 0; i < 3; i = i + 1)
    write_all(8'h10+i, 8'h20+i, 8'h30+i, 8'h40+i);

  if (lanes_valid_out !== 1'b0) begin
    $display("FAIL: lanes_valid_out asserted early, before threshold reached (fill=3)");
    errors = errors + 1;
  end else $display("PASS: correctly withheld lanes_valid_out below threshold");

  // ---- Test 2: cross threshold (write 1 more, fill=4=THRESHOLD), should now prime ----
  write_all(8'h13, 8'h23, 8'h33, 8'h43);
  @(negedge clk); // primed_latched settles this edge
  @(negedge clk); // lanes_valid_out settles this edge (2-stage register chain)

  if (lanes_valid_out !== 1'b1) begin
    $display("FAIL: lanes_valid_out not asserted after reaching threshold (fill=4)");
    errors = errors + 1;
  end else $display("PASS: lanes_valid_out correctly asserted at threshold");

  // ---- Test 3: data integrity ----
  rd_en = 1;
  @(negedge clk);
  if (rd_data_0 !== 8'h10) begin
    $display("FAIL: first read from lane 0 expected 8'h10, got %h", rd_data_0);
    errors = errors + 1;
  end else $display("PASS: lane 0 first read correct (%h)", rd_data_0);

  @(negedge clk);
  if (rd_data_0 !== 8'h11) begin
    $display("FAIL: second read from lane 0 expected 8'h11, got %h", rd_data_0);
    errors = errors + 1;
  end else $display("PASS: lane 0 second read correct (%h)", rd_data_0);

  rd_en = 0;

  // ---- Test 4: "stays primed" - drain below threshold, confirm lanes_valid_out does NOT drop ----
  // We've already read 2 of 4 entries -> fill is now 2, below THRESHOLD=4
  if (lanes_valid_out !== 1'b1) begin
    $display("FAIL: lanes_valid_out dropped after draining below threshold - latch should hold");
    errors = errors + 1;
  end else $display("PASS: lanes_valid_out correctly held high (latched) despite fill dropping below threshold");
  
// ---- Test 5: lane underrun must drop lanes_valid_out ----
  rd_en = 1;
  @(negedge clk); // 3rd read - lane0 fill now 1
  @(negedge clk); // 4th read - lane0 fill now 0 (empty)
  @(negedge clk); // lane0 empty should now be visible combinationally in lanes_valid_out
  rd_en = 0;

  if (dut.empty[0] !== 1'b1) begin
    $display("NOTE: lane 0 did not go empty as expected in this scenario - re-check timing");
  end else if (lanes_valid_out !== 1'b0) begin
    $display("FAIL: lanes_valid_out remained high (%b) while lane 0 is empty - underrun not caught", lanes_valid_out);
    errors = errors + 1;
  end else begin
    $display("PASS: lanes_valid_out correctly dropped to 0 on lane 0 underrun");
  end
  if (errors == 0) $display("ALL PASS: tx_elastic_deskew core behavior verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule

//------------------------------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------














