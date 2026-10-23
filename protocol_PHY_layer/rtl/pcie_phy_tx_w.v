`timescale 1ns/1ps
// =============================================================================
// pcie_phy_tx_w - wide TX datapath (Step 4 item 3): 8 bytes = 32 PAM4 symbols
// per lane per clock, i.e. one 256-bit link word per clock on 4 lanes
// (was 1 byte per 4 clocks per lane). Selected by pcie_phy_top PHY_WIDE=1.
//
//   link layer --256b word, sof, eof, valid/ready--> Flit FIFO
//     -> per clock one "lane cycle" (8 bytes on each of the 4 lanes):
//          TS1/TS2 half ordered set (16 bytes per lane = 2 cycles), or
//          SOF  (0xFB in all 32 bytes) once before each Flit, or
//          one Flit word, byte k on lane k%4 at lane byte k/4, or
//          logical idle (0x1B in all 32 bytes, K6)
//     -> 32 symbols per lane (byte MSB symbol first) -> Gray -> PAM4 level
//     -> tx_level[96*lane + 3*sym +: 3], symbol 0 is the earliest
//
// Same framing as the narrow PHY at lane-cycle granularity: a Flit is one SOF
// cycle + WORDS_PER_FLIT word cycles, so the far end's strip logic counts the
// same way. A Flit is only started once all its words are in the FIFO, so
// there is never an underrun inside a Flit (no filler needed).
// Rate: 256 B per 9 clocks = 28.4 B/clk on the wire (narrow: ~1 B/clk).
// =============================================================================
module pcie_phy_tx_w #(
    parameter FIFO_DEPTH     = 16,
    parameter WORDS_PER_FLIT = 8,
    parameter L0_IDLE_FILL   = 1
)(
    input  wire          clk,
    input  wire          rst_n,

    input  wire [1:0]    os_mode,         // 0 none, 1 TS1, 2 TS2
    input  wire          data_en,         // L0
    output wire          ts_sending,
    output reg           os_done,

    input  wire [255:0]  flit_data,
    input  wire          flit_valid,
    input  wire          flit_sof,
    input  wire          flit_eof,
    output wire          flit_ready,

    output reg  [383:0]  tx_level,
    output reg  [3:0]    tx_level_valid,

    output wire          tx_idle,
    output reg           framing_err
);

localparam FAW = $clog2(FIFO_DEPTH);
localparam [7:0] SOF_TOKEN = 8'hFB;
localparam [7:0] IDL_BYTE  = 8'h1B;
localparam [7:0] COM       = 8'hBC;

// ---- Flit FIFO ----
reg [255:0]  ff_mem [0:FIFO_DEPTH-1];
reg [FAW-1:0] ff_wp, ff_rp;
reg [FAW:0]   ff_cnt;
assign flit_ready = (ff_cnt != FIFO_DEPTH);
wire ff_push = flit_valid && flit_ready;
wire ff_pop;
always @(posedge clk) if (ff_push) ff_mem[ff_wp] <= flit_data;
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin ff_wp <= 0; ff_rp <= 0; ff_cnt <= 0; end
  else begin
    if (ff_push) ff_wp <= ff_wp + 1'b1;
    if (ff_pop)  ff_rp <= ff_rp + 1'b1;
    case ({ff_push, ff_pop})
      2'b10: ff_cnt <= ff_cnt + 1'b1;
      2'b01: ff_cnt <= ff_cnt - 1'b1;
      default: ;
    endcase
  end
end

reg [3:0] in_word_idx;
always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin in_word_idx <= 4'd0; framing_err <= 1'b0; end
  else if (ff_push) begin
    if (flit_sof != (in_word_idx == 4'd0)) framing_err <= 1'b1;
    if (flit_eof != (in_word_idx == WORDS_PER_FLIT - 1)) framing_err <= 1'b1;
    in_word_idx <= flit_sof ? ((WORDS_PER_FLIT == 1) ? 4'd0 : 4'd1)
                 : (in_word_idx == WORDS_PER_FLIT - 1) ? 4'd0 : in_word_idx + 4'd1;
  end
end

// ---- lane-cycle scheduler ----
reg        ts_half;            // 1: second half of an ordered set is due
reg        ts_is2;
reg        in_flit;
reg [3:0]  wcnt;
wire       ts_want  = (os_mode != 2'd0);
wire       do_ts    = ts_half || ts_want;
wire       cur_is2  = ts_half ? ts_is2 : (os_mode == 2'd2);
wire       do_sof   = !do_ts && data_en && !in_flit && (ff_cnt >= WORDS_PER_FLIT);
wire       do_word  = !do_ts && data_en && in_flit;
wire       do_idl   = !do_ts && data_en && !in_flit && !do_sof && (L0_IDLE_FILL != 0);
assign     ff_pop   = do_word;
assign     ts_sending = ts_want || ts_half;

// bytes of this lane cycle: lane l, lane byte j
function [7:0] ts_byte;
  input [3:0] k; input [1:0] lane; input is2;
  begin
    if (k == 4'd0)       ts_byte = COM;
    else if (k == 4'd2)  ts_byte = {6'd0, lane};
    else if (k >= 4'd10) ts_byte = is2 ? 8'h45 : 8'h4A;
    else                 ts_byte = 8'h00;
  end
endfunction

reg [255:0] lc;                // lane cycle: lane l byte j at [64*l + 8*j +: 8]
integer l, j;
always @(*) begin
  lc = 256'd0;
  for (l = 0; l < 4; l = l + 1)
    for (j = 0; j < 8; j = j + 1) begin
      if (do_ts)        lc[64*l + 8*j +: 8] = ts_byte({ts_half, 3'd0} + j, l, cur_is2);
      else if (do_sof)  lc[64*l + 8*j +: 8] = SOF_TOKEN;
      else if (do_word) lc[64*l + 8*j +: 8] = ff_mem[ff_rp][8*(4*j + l) +: 8];
      else              lc[64*l + 8*j +: 8] = IDL_BYTE;
    end
end
wire lc_valid = do_ts || do_sof || do_word || do_idl;

// byte -> 4 symbols (MSB first) -> Gray -> PAM4 level
function [2:0] lvl_of; input [1:0] a; reg [1:0] g; begin
  g = {a[1], a[1] ^ a[0]};
  case (g) 2'b00: lvl_of = 3'b101; 2'b01: lvl_of = 3'b111; 2'b11: lvl_of = 3'b001; default: lvl_of = 3'b011; endcase
end endfunction

reg [383:0] lv;
integer s;
always @(*) begin
  for (l = 0; l < 4; l = l + 1)
    for (s = 0; s < 32; s = s + 1)
      lv[96*l + 3*s +: 3] = lvl_of(lc[64*l + 8*(s/4) + 2*(3 - (s % 4)) +: 2]);
end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    ts_half <= 1'b0; ts_is2 <= 1'b0; in_flit <= 1'b0; wcnt <= 4'd0; os_done <= 1'b0;
    tx_level <= {128{3'b101}}; tx_level_valid <= 4'b0000;
  end else begin
    os_done <= 1'b0;
    if (do_ts) begin
      ts_is2  <= cur_is2;
      ts_half <= !ts_half;
      if (ts_half) os_done <= 1'b1;
    end
    if (do_sof) begin in_flit <= 1'b1; wcnt <= 4'd0; end
    if (do_word) begin
      if (wcnt == WORDS_PER_FLIT - 1) in_flit <= 1'b0;
      wcnt <= wcnt + 4'd1;
    end
    tx_level_valid <= {4{lc_valid}};
    if (lc_valid) tx_level <= lv;
  end
end

assign tx_idle = !ts_sending && (ff_cnt == 0) && !in_flit;

endmodule
