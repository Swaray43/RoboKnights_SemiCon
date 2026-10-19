// =============================================================================
// pcie_phy_tx - synthesizable TX datapath of the Protocol-to-PHY layer
// (fixes P1, P2 TX half, P3 / Seam C)
//
//   link layer --256b word, sof, eof, valid/READY--> Flit FIFO
//     -> flit_lane_striper (byte-interleave 4 lanes, 8 clk / word)
//     -> SOF insertion (0xFB on all 4 lanes, once before each sof word)
//     -> tx_elastic_deskew (4 lane FIFOs)
//     -> lane scheduler (one byte per lane every 4 clk, all lanes in lockstep)
//     -> 4x lane_tx (byte -> symbols -> Gray -> PAM4)
//     -> tx_level[11:0] / tx_level_valid[3:0]  ==> SerDes
//
// ---- Link-side handshake (Seam C) -------------------------------------------
// flit_ready is a normal valid/ready handshake per 256-bit word: a word moves
// when flit_valid && flit_ready on a rising edge. The link layer MUST hold the
// word (and stop its output word counter) while flit_ready is low. The PHY
// drains 1 byte per clock (4 lanes x 2 bits), so a 256-byte Flit takes 260
// clocks (256 + 4 for the SOF byte) while the link can write it in 8 clocks;
// without this stall Flits are lost. FIFO_DEPTH words of slack are provided.
//
// ---- Framing -----------------------------------------------------------------
// The SOF token is inserted once per word that arrives with flit_sof=1. With
// Swaray's link layer that is word 0 of each 8-word (256 B) Flit, matching
// pcie_phy_rx's WORDS_PER_FLIT=8 default (the PCIe meaning of "Flit").
// flit_eof is accepted for interface symmetry; framing is checked against
// WORDS_PER_FLIT and a mismatch sets framing_err (sticky).
//
// ---- Lane byte order -----------------------------------------------------------
// Unchanged from flit_lane_striper: word byte k goes to lane (k mod 4), lowest
// byte first. tx_level lane i is tx_level[3*i +: 3] (signed -3/-1/+1/+3).
// =============================================================================
module pcie_phy_tx #(
    parameter FIFO_DEPTH     = 16,   // Flit FIFO depth in 256-bit words (power of 2)
    parameter WORDS_PER_FLIT = 8,
    parameter DESKEW_DEPTH   = 16    // per-lane byte FIFO depth (power of 2, >= 16)
)(
    input  wire         clk,
    input  wire         rst_n,

    // ---- from link layer (link_layer_tx phy_tx side) ----
    input  wire [255:0] flit_data,
    input  wire         flit_valid,
    input  wire         flit_sof,
    input  wire         flit_eof,
    output wire         flit_ready,

    // ---- 4-lane PAM4 symbol bus to the SerDes ----
    output wire [11:0]  tx_level,        // lane i = tx_level[3*i +: 3], signed
    output wire [3:0]   tx_level_valid,

    // ---- status ----
    output wire         tx_idle,         // nothing buffered anywhere in the TX path
    output reg          framing_err      // sticky: sof/eof not on an 8-word boundary
);

localparam FAW = (FIFO_DEPTH <= 2) ? 1 : (FIFO_DEPTH <= 4) ? 2 : (FIFO_DEPTH <= 8) ? 3 :
                 (FIFO_DEPTH <= 16) ? 4 : (FIFO_DEPTH <= 32) ? 5 : 6;
localparam [7:0] SOF_TOKEN = 8'hFB;

// -----------------------------------------------------------------------------
// 1. Flit FIFO (first-word-fall-through, 256b data + sof flag)
// -----------------------------------------------------------------------------
reg [256:0]   ff_mem [0:FIFO_DEPTH-1];
reg [FAW-1:0] ff_wr_ptr, ff_rd_ptr;
reg [FAW:0]   ff_count;

wire ff_full  = (ff_count == FIFO_DEPTH);
wire ff_empty = (ff_count == 0);
assign flit_ready = !ff_full;

wire          ff_push = flit_valid && flit_ready;
wire          ff_pop;
wire [256:0]  ff_head = ff_mem[ff_rd_ptr];

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    ff_wr_ptr <= {FAW{1'b0}};
    ff_rd_ptr <= {FAW{1'b0}};
    ff_count  <= {(FAW+1){1'b0}};
  end else begin
    if (ff_push) begin
      ff_mem[ff_wr_ptr] <= {flit_sof, flit_data};
      ff_wr_ptr <= ff_wr_ptr + 1'b1;
    end
    if (ff_pop) ff_rd_ptr <= ff_rd_ptr + 1'b1;
    case ({ff_push, ff_pop})
      2'b10:   ff_count <= ff_count + 1'b1;
      2'b01:   ff_count <= ff_count - 1'b1;
      default: ff_count <= ff_count;
    endcase
  end
end

// Framing check on the link side: sof must start word 0, eof must end word N-1
reg [3:0] in_word_idx;
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    in_word_idx <= 4'd0;
    framing_err <= 1'b0;
  end else if (ff_push) begin
    if (flit_sof != (in_word_idx == 4'd0)) framing_err <= 1'b1;
    if (flit_eof != (in_word_idx == WORDS_PER_FLIT - 1)) framing_err <= 1'b1;
    in_word_idx <= flit_sof ? ((WORDS_PER_FLIT == 1) ? 4'd0 : 4'd1)
                 : (in_word_idx == WORDS_PER_FLIT - 1) ? 4'd0 : in_word_idx + 4'd1;
  end
end

// -----------------------------------------------------------------------------
// 2. Striper + SOF insertion
//    A word is started only when the striper is fully idle (so the SOF write
//    can never collide with the previous word's last chunk) and every lane
//    FIFO has room for SOF + 8 bytes.
// -----------------------------------------------------------------------------
wire        strp_ready;
wire [3:0]  strp_lane_valid;
wire [31:0] strp_lane_data;
wire        dsk_room_ok;
reg  [3:0]  strp_left;          // cycles until the striper is idle again

wire start_word = !ff_empty && (strp_left == 4'd0) && dsk_room_ok;
assign ff_pop   = start_word;
wire sof_inject = start_word && ff_head[256];

always @(posedge clk or negedge rst_n) begin
  if (!rst_n)            strp_left <= 4'd0;
  else if (start_word)   strp_left <= 4'd8;
  else if (strp_left!=0) strp_left <= strp_left - 4'd1;
end

flit_lane_striper u_striper (
  .clk(clk), .rst_n(rst_n),
  .valid_in(start_word), .flit_in(ff_head[255:0]), .ready_out(strp_ready),
  .lane_valid_out(strp_lane_valid), .lane_data_out(strp_lane_data)
);

wire [3:0]  dsk_wr_en   = sof_inject ? 4'b1111 : strp_lane_valid;
wire [31:0] dsk_wr_data = sof_inject ? {4{SOF_TOKEN}} : strp_lane_data;

// -----------------------------------------------------------------------------
// 3. Elastic buffer + lane scheduler
//    Read all four lane FIFOs together every 4th clock. rd_data is registered,
//    so the byte reaches lane_tx one clock after the read, exactly when the
//    previous byte's 4th symbol has gone out and byte_to_symbol is free again.
// -----------------------------------------------------------------------------
wire [7:0] dsk_out0, dsk_out1, dsk_out2, dsk_out3;
wire       dsk_valid;
reg  [1:0] phase;
reg        byte_valid;
wire       dsk_rd = (phase == 2'd0) && dsk_valid;

tx_elastic_deskew #(.DEPTH(DESKEW_DEPTH), .THRESHOLD(4), .ROOM_BYTES(9)) u_deskew (
  .clk(clk), .rst_n(rst_n),
  .wr_en(dsk_wr_en),
  .wr_data_0(dsk_wr_data[7:0]),   .wr_data_1(dsk_wr_data[15:8]),
  .wr_data_2(dsk_wr_data[23:16]), .wr_data_3(dsk_wr_data[31:24]),
  .rd_en(dsk_rd),
  .rd_data_0(dsk_out0), .rd_data_1(dsk_out1), .rd_data_2(dsk_out2), .rd_data_3(dsk_out3),
  .lanes_valid_out(dsk_valid),
  .room_ok(dsk_room_ok)
);

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    phase      <= 2'd0;
    byte_valid <= 1'b0;
  end else begin
    byte_valid <= dsk_rd;
    if (dsk_rd)              phase <= 2'd1;
    else if (phase != 2'd0)  phase <= phase + 2'd1;   // 1 -> 2 -> 3 -> 0
  end
end

// -----------------------------------------------------------------------------
// 4. Four TX lanes
// -----------------------------------------------------------------------------
wire [3:0] lane_busy;
wire signed [2:0] lvl0, lvl1, lvl2, lvl3;

lane_tx u_lane0 (.clk(clk), .rst_n(rst_n), .byte_in_valid(byte_valid), .byte_in(dsk_out0),
                 .busy(lane_busy[0]), .tx_level_valid(tx_level_valid[0]), .tx_level(lvl0));
lane_tx u_lane1 (.clk(clk), .rst_n(rst_n), .byte_in_valid(byte_valid), .byte_in(dsk_out1),
                 .busy(lane_busy[1]), .tx_level_valid(tx_level_valid[1]), .tx_level(lvl1));
lane_tx u_lane2 (.clk(clk), .rst_n(rst_n), .byte_in_valid(byte_valid), .byte_in(dsk_out2),
                 .busy(lane_busy[2]), .tx_level_valid(tx_level_valid[2]), .tx_level(lvl2));
lane_tx u_lane3 (.clk(clk), .rst_n(rst_n), .byte_in_valid(byte_valid), .byte_in(dsk_out3),
                 .busy(lane_busy[3]), .tx_level_valid(tx_level_valid[3]), .tx_level(lvl3));

assign tx_level = {lvl3, lvl2, lvl1, lvl0};

assign tx_idle = ff_empty && (strp_left == 4'd0) && !dsk_valid && (phase == 2'd0) &&
                 !byte_valid && (lane_busy == 4'b0000) && (tx_level_valid == 4'b0000);

`ifndef SYNTHESIS
// The scheduler relies on byte_to_symbol being free exactly when a byte arrives.
always @(posedge clk) begin
  if (rst_n && byte_valid && (lane_busy != 4'b0000))
    $display("ERROR pcie_phy_tx @%0t: byte presented to a busy lane - byte dropped", $time);
end
`endif

endmodule
