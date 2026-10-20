// =============================================================================
// PCIe 6.0 PHY - Protocol-to-PHY layer (Kshitij's scope)
// Consolidated RTL: Gray coding -> PAM4 -> lane striping -> SOF framing ->
// lane de-striping, plus the RX-side lane reversal (TS1/TS2) detection path
// and the TX-side elastic deskew buffer.
//
// This file collects everything built and individually self-tested so far,
// with every bug fix we traced during verification already applied:
//   - byte_ok_now same-cycle fix in ts_ordered_set_detector
//   - restored byte_idx increment in ts_ordered_set_detector
//   - lanes_valid_out now LIVE (not just latched) in tx_elastic_deskew
//
// Bottom of file: tb_end_to_end_datapath - full symbol-level round-trip test
// (striper -> per-lane Gray/PAM4 -> ideal channel -> per-lane demap/decode ->
//  lane swap -> SOF strip -> destriper -> compare to original FLIT).
//
// NOT included here (separate, not yet integrated into this chain):
//   - ts_ordered_set_detector / lane_reversal_detector / lane_reversal_latch
//     are training-time blocks; this datapath test assumes lane_reversed=0
//     and does not exercise TS1/TS2 traffic. See work_distribution.md scope
//     notes for what remains deferred.
// =============================================================================


// -----------------------------------------------------------------------------
// Gray encoder / decoder
// Mapping convention (locked spec): a=00,01,10,11 -> g=00,01,11,10
// i.e. standard binary-to-Gray on encode, exact inverse on decode.
// -----------------------------------------------------------------------------
module gray_enc(a, g);
input  [1:0] a;
output [1:0] g;
assign g[1] = a[1];
assign g[0] = a[1] ^ a[0];   // fixed: reads a[1] directly, not g[1] (style cleanup)
endmodule

module gray_dec(g, a);
input  [1:0] g;
output [1:0] a;
assign a[1] = g[1];
assign a[0] = g[0] ^ g[1];   // exact inverse of gray_enc
endmodule


// -----------------------------------------------------------------------------
// PAM4 mapper / de-mapper
// Locked mapping: g=00,01,11,10 -> level=-3,-1,+1,+3 (monotonic, Gray-adjacent)
// Case order intentionally follows the GRAY sequence, not binary count order.
// -----------------------------------------------------------------------------
module pam4_map(g, level);
input  [1:0] g;
output reg signed [2:0] level;

always @(*) begin
  case (g)
    2'b00: level = -3;
    2'b01: level = -1;
    2'b11: level = 1;
    2'b10: level = 3;
    default: level = 3'sbx; // unreachable: 2-bit input is fully exhaustive above
  endcase
end
endmodule

module pam4_demap(level_in, g_out);
input  signed [2:0] level_in;
output reg [1:0] g_out;

// Threshold slicer (not a reverse-lookup) so this degrades gracefully once
// a noisy channel model is added later - nearest-level decision, not "no match".
always @(*) begin
  if (level_in < -2)      g_out = 2'b00;
  else if (level_in < 0)  g_out = 2'b01;
  else if (level_in < 2)  g_out = 2'b11;
  else                    g_out = 2'b10;
end
endmodule


// -----------------------------------------------------------------------------
// TX serializer: one byte -> four sequential 2-bit Gray-domain symbols, MSB-first
// -----------------------------------------------------------------------------
module byte_to_symbol (
input  wire       clk,
input  wire       rst_n,
input  wire       byte_valid,
input  wire [7:0] byte_in,
output reg        symbol_valid,
output reg  [1:0] symbol_out,
output wire       busy
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
      shift_reg    <= byte_in;
      symbol_out   <= byte_in[7:6];   // first symbol emitted same cycle - full byte already available
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


// -----------------------------------------------------------------------------
// RX deserializer: four sequential 2-bit symbols -> one reassembled byte, MSB-first
// Must match byte_to_symbol's bit ordering exactly (interface contract).
// -----------------------------------------------------------------------------
module symbol_to_byte (
input  wire       clk,
input  wire       rst_n,
input  wire       symbol_valid,
input  wire [1:0] symbol_in,
output reg        byte_valid,
output reg  [7:0] byte_out
);

reg [1:0] sym_cnt;
reg [7:0] shift_reg;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    sym_cnt    <= 2'd0;
    shift_reg  <= 8'd0;
    byte_valid <= 1'b0;
  end else begin
    byte_valid <= 1'b0; // default: pulse only when a byte completes

    if (symbol_valid) begin
      shift_reg <= {shift_reg[5:0], symbol_in};
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


// -----------------------------------------------------------------------------
// Per-lane symbol-domain pipe: wraps byte_to_symbol -> gray_enc -> pam4_map ->
// (ideal channel, zero-latency) -> pam4_demap -> gray_dec -> symbol_to_byte.
// Keeps the top-level testbench from repeating this chain 4x inline, which is
// exactly the kind of repetition that hid the destriper truncation bug earlier.
// -----------------------------------------------------------------------------
module lane_symbol_pipe (
input  wire       clk,
input  wire       rst_n,
input  wire       byte_in_valid,
input  wire [7:0] byte_in,
output wire       byte_out_valid,
output wire [7:0] byte_out,
output wire       busy            // forwards byte_to_symbol's busy - lets upstream
                                   // (e.g. the elastic deskew buffer) know not to
                                   // push a new byte while this lane is still serializing
);

wire       tx_sym_valid;
wire [1:0] tx_sym;
wire       tx_busy;
wire [1:0] gray_g;
wire signed [2:0] level;
wire [1:0] rx_g;
wire [1:0] rx_dec;

assign busy = tx_busy;

byte_to_symbol u_ser (
  .clk(clk), .rst_n(rst_n),
  .byte_valid(byte_in_valid), .byte_in(byte_in),
  .symbol_valid(tx_sym_valid), .symbol_out(tx_sym), .busy(tx_busy)
);

gray_enc u_genc (.a(tx_sym), .g(gray_g));
pam4_map u_map  (.g(gray_g), .level(level));

// Ideal channel: no noise/attenuation modeled yet (deferred to FEC counter work)
pam4_demap u_demap (.level_in(level), .g_out(rx_g));
gray_dec   u_gdec  (.g(rx_g), .a(rx_dec));

symbol_to_byte u_deser (
  .clk(clk), .rst_n(rst_n),
  .symbol_valid(tx_sym_valid), .symbol_in(rx_dec),
  .byte_valid(byte_out_valid), .byte_out(byte_out)
);
endmodule


// -----------------------------------------------------------------------------
// TX lane striper: 256-bit FLIT -> 4 lanes, byte-interleaved round-robin.
// Byte-index-is-lane-index falls out of the indexed part-select directly:
// cycle c covers bytes {4c,4c+1,4c+2,4c+3} -> lane0 gets 0,4,8...; lane1 1,5,9...
// -----------------------------------------------------------------------------
module flit_lane_striper (
input wire clk,
input wire rst_n,
input wire valid_in,
input wire [255:0] flit_in,
output wire ready_out,
output reg [3:0] lane_valid_out,
output reg [31:0] lane_data_out
);

reg [255:0] flit_reg;
reg [2:0] cycle_cnt;
reg active;

assign ready_out = !active || (active && (cycle_cnt == 3'd7));

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    active    <= 1'b0;
    cycle_cnt <= 3'd0;
    flit_reg  <= 256'd0;
  end else begin
    if (valid_in && ready_out) begin
      flit_reg  <= flit_in;
      active    <= 1'b1;
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

always @(*) begin
  lane_valid_out = 4'b0000;
  lane_data_out  = 32'd0;
  if (active) begin
    lane_valid_out = 4'b1111;
    lane_data_out  = flit_reg[cycle_cnt * 32 +: 32];
  end
end
endmodule


// -----------------------------------------------------------------------------
// RX lane de-striper: inverse of flit_lane_striper. Reassembles 4 lanes'
// worth of bytes per cycle back into the 256-bit FLIT over 8 cycles.
// Completion word is built directly from {lane_data_in, flit_reg[223:0]}
// rather than reading flit_reg post-update, to avoid a same-cycle NBA race
// on the final chunk.
// -----------------------------------------------------------------------------
module flit_lane_destriper (
input wire clk,
input wire rst_n,
input wire [3:0] lane_valid_in,
input wire [31:0] lane_data_in,
output reg flit_valid_out,
output reg [255:0] flit_out
);

reg [255:0] flit_reg;
reg [2:0] cycle_cnt;
reg active;

wire all_lanes_valid = &lane_valid_in;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    active         <= 1'b0;
    cycle_cnt      <= 3'd0;
    flit_reg       <= 256'd0;
    flit_valid_out <= 1'b0;
    flit_out       <= 256'd0;
  end else begin
    flit_valid_out <= 1'b0;

    if (all_lanes_valid) begin
      flit_reg[cycle_cnt * 32 +: 32] <= lane_data_in;
      active <= 1'b1;

      if (cycle_cnt == 3'd7) begin
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


// -----------------------------------------------------------------------------
// Lane reversal support (TS1/TS2, scoped to lane-reversal detection only -
// full rate/equalization negotiation explicitly out of scope). Symbol-level
// input, reassembled to bytes upstream via symbol_to_byte before reaching here.
// -----------------------------------------------------------------------------
module ts_ordered_set_detector (
input  wire       clk,
input  wire       rst_n,
input  wire       byte_valid,
input  wire [7:0] byte_in,
output reg        lane_num_valid,
output reg  [7:0] lane_num_out
);

localparam COM     = 8'hBC;
localparam TS1_ID  = 8'h4A;
localparam TS2_ID  = 8'h45;
localparam IDLE    = 2'd0;
localparam CAPTURE = 2'd1;

reg [1:0] state;
reg [3:0] byte_idx;
reg [7:0] lane_candidate;
reg [7:0] id_byte0;
reg       id_match;
reg [7:0] prev_confirmed;
reg       prev_valid;

// Same-cycle check for THIS byte, folded into the byte-15 decision directly -
// fixes the one-cycle-lag bug where a corrupted byte 15 wasn't caught in time.
wire byte_ok_now = (byte_idx <= 4'd10) ||
                   (byte_in == id_byte0 && (id_byte0 == TS1_ID || id_byte0 == TS2_ID));

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
    lane_num_valid <= 1'b0;

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
            byte_idx <= byte_idx + 4'd1;   // fix: this increment was previously dropped
          end
        end
      endcase
    end
  end
end
endmodule


module lane_reversal_detector (
input  wire       clk,
input  wire       rst_n,
input  wire [3:0] lane_num_valid,
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
      lane_reversed           <= reversed_match;
    end
    // NOTE (known limitation): all_valid true but neither pattern matches ->
    // holds previous state silently. Partial reversal / wiring fault not flagged.
  end
end
endmodule


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
    // Latch once on first confirmed detection; real PCIe re-evaluates only
    // on retraining, not continuously during normal data transfer.
    lane_reversed_locked <= lane_reversed_in;
    link_trained         <= 1'b1;
  end
end
endmodule


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

assign lane_out_0 = lane_reversed ? lane_in_3 : lane_in_0;
assign lane_out_1 = lane_reversed ? lane_in_2 : lane_in_1;
assign lane_out_2 = lane_reversed ? lane_in_1 : lane_in_2;
assign lane_out_3 = lane_reversed ? lane_in_0 : lane_in_3;
endmodule


// -----------------------------------------------------------------------------
// FLIT-mode SOF token strip (PCIe 6.0 uses FLIT mode, not 128b/130b - see
// design notes. SOF = 0xFB on all 4 lanes simultaneously, one full cycle,
// immediately preceding each 8-cycle FLIT burst).
// KNOWN LIMITATION: no byte-stuffing/escaping - if 0xFB appears as genuine
// mid-FLIT payload, this will misfire and resync early. Stated, not solved,
// given project scope.
// -----------------------------------------------------------------------------
module flit_sof_strip (
input  wire       clk,
input  wire       rst_n,
input  wire [7:0] lane_in_0,
input  wire [7:0] lane_in_1,
input  wire [7:0] lane_in_2,
input  wire [7:0] lane_in_3,
input  wire       lane_in_valid,
output reg  [7:0] lane_out_0,
output reg  [7:0] lane_out_1,
output reg  [7:0] lane_out_2,
output reg  [7:0] lane_out_3,
output reg  [3:0] lane_valid_out
);

localparam SOF_TOKEN = 8'hFB;
localparam WAIT_SOF  = 1'b0;
localparam PASS_FLIT = 1'b1;

reg       state;
reg [2:0] payload_cnt;

wire is_sof_cycle = lane_in_valid &&
                     (lane_in_0 == SOF_TOKEN) && (lane_in_1 == SOF_TOKEN) &&
                     (lane_in_2 == SOF_TOKEN) && (lane_in_3 == SOF_TOKEN);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    state          <= WAIT_SOF;
    payload_cnt    <= 3'd0;
    lane_valid_out <= 4'b0000;
    lane_out_0     <= 8'd0;
    lane_out_1     <= 8'd0;
    lane_out_2     <= 8'd0;
    lane_out_3     <= 8'd0;
  end else begin
    lane_valid_out <= 4'b0000;

    case (state)
      WAIT_SOF: begin
        if (is_sof_cycle) begin
          state       <= PASS_FLIT;
          payload_cnt <= 3'd0;
        end
        // non-SOF bytes here (idle/training traffic) are silently dropped
      end

      PASS_FLIT: begin
        if (lane_in_valid) begin
          if (is_sof_cycle) begin
            // Defensive resync: a real SOF appearing mid-FLIT means sync was lost.
            state       <= PASS_FLIT;
            payload_cnt <= 3'd0;
          end else begin
            lane_out_0     <= lane_in_0;
            lane_out_1     <= lane_in_1;
            lane_out_2     <= lane_in_2;
            lane_out_3     <= lane_in_3;
            lane_valid_out <= 4'b1111;

            if (payload_cnt == 3'd7) begin
              state <= WAIT_SOF;
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


// -----------------------------------------------------------------------------
// TX elastic deskew buffer (fixed-depth model). Per-lane FIFO + shared release
// gate. NOTE ON NAMING: this is "four-lane buffering with a common release
// threshold," not a true UI-level skew measurement - it never timestamps or
// compares arrival time between lanes, only fill count. True deskew would
// need a sequence-marker mechanism; deferred given project scope.
// -----------------------------------------------------------------------------
module elastic_fifo #(
parameter DEPTH = 8,
parameter PTR_WIDTH = 3
)(
input  wire       clk,
input  wire       rst_n,
input  wire       wr_en,
input  wire [7:0] wr_data,
input  wire       rd_en,
output reg  [7:0] rd_data,
output wire [PTR_WIDTH:0] fill_count,
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
    wr_ptr  <= 0;
    rd_ptr  <= 0;
    count   <= 0;
    rd_data <= 8'd0;
  end else begin
    if (wr_en && !full) begin
      mem[wr_ptr] <= wr_data;
      wr_ptr <= wr_ptr + 1'b1;
    end
    if (rd_en && !empty) begin
      rd_data <= mem[rd_ptr];
      rd_ptr <= rd_ptr + 1'b1;
    end
    case ({wr_en && !full, rd_en && !empty})
      2'b10: count <= count + 1'b1;
      2'b01: count <= count - 1'b1;
      default: count <= count;
    endcase
  end
end
endmodule


module tx_elastic_deskew #(
parameter DEPTH = 8,
parameter THRESHOLD = 4
)(
input  wire       clk,
input  wire       rst_n,
input  wire [3:0] wr_en,
input  wire [7:0] wr_data_0, wr_data_1, wr_data_2, wr_data_3,
input  wire       rd_en,
output wire [7:0] rd_data_0, rd_data_1, rd_data_2, rd_data_3,
output wire       lanes_valid_out   // LIVE per-cycle availability, gated by initial priming
);

wire [3:0] empty, full;
wire [3:0] fifo_rd_en;
wire [3:0] fc0, fc1, fc2, fc3;

elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(3)) fifo0 (
  .clk(clk), .rst_n(rst_n), .wr_en(wr_en[0]), .wr_data(wr_data_0),
  .rd_en(fifo_rd_en[0]), .rd_data(rd_data_0), .fill_count(fc0), .empty(empty[0]), .full(full[0]));
elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(3)) fifo1 (
  .clk(clk), .rst_n(rst_n), .wr_en(wr_en[1]), .wr_data(wr_data_1),
  .rd_en(fifo_rd_en[1]), .rd_data(rd_data_1), .fill_count(fc1), .empty(empty[1]), .full(full[1]));
elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(3)) fifo2 (
  .clk(clk), .rst_n(rst_n), .wr_en(wr_en[2]), .wr_data(wr_data_2),
  .rd_en(fifo_rd_en[2]), .rd_data(rd_data_2), .fill_count(fc2), .empty(empty[2]), .full(full[2]));
elastic_fifo #(.DEPTH(DEPTH), .PTR_WIDTH(3)) fifo3 (
  .clk(clk), .rst_n(rst_n), .wr_en(wr_en[3]), .wr_data(wr_data_3),
  .rd_en(fifo_rd_en[3]), .rd_data(rd_data_3), .fill_count(fc3), .empty(empty[3]), .full(full[3]));

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

// Fixed: previously this was latched off primed_latched alone, which stayed
// high even after a lane went empty (a real underrun bug caught in testing).
// Now it drops immediately if ANY lane runs dry, post-priming.
assign lanes_valid_out = primed_latched && !empty[0] && !empty[1] && !empty[2] && !empty[3];

assign fifo_rd_en[0] = primed_latched && rd_en && !empty[0];
assign fifo_rd_en[1] = primed_latched && rd_en && !empty[1];
assign fifo_rd_en[2] = primed_latched && rd_en && !empty[2];
assign fifo_rd_en[3] = primed_latched && rd_en && !empty[3];
endmodule


// =============================================================================
// TOP-LEVEL INTEGRATION TESTBENCH
// striper -> 4x lane_symbol_pipe -> lane_swap_mux (reversal=0) -> flit_sof_strip
// -> flit_lane_destriper -> compare reconstructed FLIT to original.
//
// Elastic buffer and TS1/TS2 training are deliberately NOT included in this
// test - those are startup/training-time concerns and are verified separately
// per-block. This test isolates the core data-path claim only.
//
// KNOWN RISK, stated up front: the SOF-then-payload cycle alignment below is
// a best-effort estimate, not a hand-traced guarantee, given the striper's
// own load latency plus each lane pipe's multi-cycle serialize/deserialize
// latency. A timeout guard is included so a misalignment fails loudly
// instead of hanging the simulation. If it fails, check the waveform for
// when striper_lane_valid first pulses vs. when pipe_valid0 first pulses.
// =============================================================================
module tb_end_to_end_datapath();

reg clk, rst_n;
reg valid_in;
reg [255:0] flit_in;
wire striper_ready;
wire [3:0] striper_lane_valid;
wire [31:0] striper_lane_data;

wire [7:0] pipe_out0, pipe_out1, pipe_out2, pipe_out3;
wire pipe_valid0, pipe_valid1, pipe_valid2, pipe_valid3;
wire pipe_busy0, pipe_busy1, pipe_busy2, pipe_busy3;

// tx_elastic_deskew sits between the striper (which emits one byte/lane every
// cycle, back-to-back) and the lane pipes (which take 4 cycles per byte to
// serialize). Without this, most striper bytes get silently dropped because
// byte_to_symbol ignores new input while busy - this was the actual cause of
// the "timed out" failure, not a cycle-count off-by-one.
wire [7:0] deskew_out0, deskew_out1, deskew_out2, deskew_out3;
wire deskew_valid;
wire lanes_ready = !pipe_busy0 && !pipe_busy1 && !pipe_busy2 && !pipe_busy3;

tx_elastic_deskew #(.DEPTH(8), .THRESHOLD(4)) deskew (
  .clk(clk), .rst_n(rst_n),
  .wr_en(striper_lane_valid),
  .wr_data_0(striper_lane_data[7:0]),   .wr_data_1(striper_lane_data[15:8]),
  .wr_data_2(striper_lane_data[23:16]), .wr_data_3(striper_lane_data[31:24]),
  .rd_en(lanes_ready),
  .rd_data_0(deskew_out0), .rd_data_1(deskew_out1),
  .rd_data_2(deskew_out2), .rd_data_3(deskew_out3),
  .lanes_valid_out(deskew_valid)
);

// elastic_fifo's rd_data is a registered output - valid one cycle after the
// read that produced it - so byte_in_valid to the lane pipes must lag rd_en
// by exactly one cycle to line up with the data it's meant to describe.
reg deskew_valid_d;
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) deskew_valid_d <= 1'b0;
  else        deskew_valid_d <= deskew_valid && lanes_ready;
end

reg  sof_phase;
wire [7:0] strip_in0, strip_in1, strip_in2, strip_in3;
wire strip_in_valid;

wire [7:0] swap_out0, swap_out1, swap_out2, swap_out3;
wire [7:0] strip_out0, strip_out1, strip_out2, strip_out3;
wire [3:0] strip_valid_out;

wire destriper_flit_valid;
wire [255:0] destriper_flit_out;

integer errors, cyc;

flit_lane_striper striper (
  .clk(clk), .rst_n(rst_n),
  .valid_in(valid_in), .flit_in(flit_in), .ready_out(striper_ready),
  .lane_valid_out(striper_lane_valid), .lane_data_out(striper_lane_data)
);

lane_symbol_pipe lane0 (.clk(clk), .rst_n(rst_n),
  .byte_in_valid(deskew_valid_d), .byte_in(deskew_out0),
  .byte_out_valid(pipe_valid0), .byte_out(pipe_out0), .busy(pipe_busy0));
lane_symbol_pipe lane1 (.clk(clk), .rst_n(rst_n),
  .byte_in_valid(deskew_valid_d), .byte_in(deskew_out1),
  .byte_out_valid(pipe_valid1), .byte_out(pipe_out1), .busy(pipe_busy1));
lane_symbol_pipe lane2 (.clk(clk), .rst_n(rst_n),
  .byte_in_valid(deskew_valid_d), .byte_in(deskew_out2),
  .byte_out_valid(pipe_valid2), .byte_out(pipe_out2), .busy(pipe_busy2));
lane_symbol_pipe lane3 (.clk(clk), .rst_n(rst_n),
  .byte_in_valid(deskew_valid_d), .byte_in(deskew_out3),
  .byte_out_valid(pipe_valid3), .byte_out(pipe_out3), .busy(pipe_busy3));

// SOF injection mux: swap between the injected SOF byte and live pipe output
assign strip_in0 = sof_phase ? 8'hFB : pipe_out0;
assign strip_in1 = sof_phase ? 8'hFB : pipe_out1;
assign strip_in2 = sof_phase ? 8'hFB : pipe_out2;
assign strip_in3 = sof_phase ? 8'hFB : pipe_out3;
assign strip_in_valid = sof_phase ? 1'b1 : pipe_valid0; // assumes all 4 lanes run in lockstep

lane_swap_mux swap (
  .lane_reversed(1'b0),
  .lane_in_0(strip_in0), .lane_in_1(strip_in1),
  .lane_in_2(strip_in2), .lane_in_3(strip_in3),
  .lane_out_0(swap_out0), .lane_out_1(swap_out1),
  .lane_out_2(swap_out2), .lane_out_3(swap_out3)
);

flit_sof_strip sof_strip (
  .clk(clk), .rst_n(rst_n),
  .lane_in_0(swap_out0), .lane_in_1(swap_out1),
  .lane_in_2(swap_out2), .lane_in_3(swap_out3),
  .lane_in_valid(strip_in_valid),
  .lane_out_0(strip_out0), .lane_out_1(strip_out1),
  .lane_out_2(strip_out2), .lane_out_3(strip_out3),
  .lane_valid_out(strip_valid_out)
);

flit_lane_destriper destriper (
  .clk(clk), .rst_n(rst_n),
  .lane_valid_in(strip_valid_out),
  .lane_data_in({strip_out3, strip_out2, strip_out1, strip_out0}),
  .flit_valid_out(destriper_flit_valid),
  .flit_out(destriper_flit_out)
);

initial clk = 0;
always #5 clk = ~clk;

initial begin
  errors = 0;
  rst_n = 0; valid_in = 0; flit_in = 256'd0; sof_phase = 0;
  @(negedge clk); @(negedge clk);
  rst_n = 1;

  // Known test pattern: byte N = value N, makes any mismatch immediately readable
  for (cyc = 0; cyc < 32; cyc = cyc + 1)
    flit_in[cyc*8 +: 8] = cyc[7:0];

  // Inject one SOF cycle first
  sof_phase = 1;
  @(negedge clk);
  sof_phase = 0;

  // Push the real FLIT into the striper
  @(negedge clk);
  if (!striper_ready) begin
    $display("FAIL: striper not ready to accept FLIT");
    errors = errors + 1;
  end
  valid_in = 1;
  @(negedge clk);
  valid_in = 0;

  // Wait for completion with a timeout guard, so a misaligned SOF/payload
  // sequencing fails loudly instead of hanging the simulation forever.
  begin : wait_for_completion
    integer timeout_cnt;
    timeout_cnt = 0;
    while (!destriper_flit_valid && timeout_cnt < 400) begin
      @(negedge clk);
      timeout_cnt = timeout_cnt + 1;
    end
    if (!destriper_flit_valid) begin
      $display("FAIL: timed out waiting for destriper_flit_valid - chain stalled somewhere");
      errors = errors + 1;
      $stop;
    end
  end

  if (destriper_flit_out !== flit_in) begin
    $display("FAIL: reconstructed FLIT mismatch");
    $display("  got:      %h", destriper_flit_out);
    $display("  expected: %h", flit_in);
    errors = errors + 1;
  end else begin
    $display("PASS: end-to-end FLIT round-trip bit-exact");
  end

  if (errors == 0) $display("ALL PASS: full data-path integration verified");
  else $display("TOTAL FAILURES: %0d", errors);
  $stop;
end
endmodule
