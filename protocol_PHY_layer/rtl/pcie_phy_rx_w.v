`timescale 1ns/1ps
// =============================================================================
// pcie_phy_rx_w - wide RX datapath (Step 4 item 3), inverse of pcie_phy_tx_w.
//
//   rx_level[96*lane + 3*sym +: 3] (32 PAM4 symbols per lane per clock)
//     -> slicer + Gray decode -> 32 symbols per lane
//     -> per-lane symbol aligner: while align_en (training) the 8-byte lane
//        word is re-aligned so that a COM (0xBC) found at any symbol offset
//        lands on byte 0 (a TS ordered set is exactly 2 lane words); the
//        offset is frozen when training ends
//     -> lane deskew at word level: lanes are held until all four present a
//        COM word together, then leave in lockstep (lane_err if a lane FIFO
//        overflows after lock)
//     -> lane swap (lane_reversed)
//     -> SOF strip: all 32 bytes 0xFB starts a Flit of WORDS_PER_FLIT word
//        cycles; between Flits every cycle should be all-IDL or all-SOF
//        (gap_ok / gap_bad for the K6 checks in pcie_ltssm)
//     -> destripe: word byte k = lane k%4, lane byte k/4
//     -> flit_out_* with sof/eof, one 256-bit word per clock
// Deskewed pre-swap lane words go to the wide TS detectors (lane_words).
// =============================================================================
module pcie_phy_rx_w #(
    parameter WORDS_PER_FLIT = 8,
    parameter DSK_DEPTH      = 8
)(
    input  wire          clk,
    input  wire          rst_n,

    input  wire [383:0]  rx_level,
    input  wire [3:0]    rx_level_valid,

    input  wire          lane_reversed,
    input  wire          align_en,

    output reg  [255:0]  flit_out_data,
    output reg           flit_out_valid,
    output reg           flit_out_sof,
    output reg           flit_out_eof,

    output reg           lane_err,
    output wire          deskew_locked,
    output wire          gap_ok,
    output wire          gap_bad,

    output wire [255:0]  lane_words,       // lane i = [64*i +: 64], byte j = [64*i + 8*j +: 8]
    output wire          lane_words_valid
);

localparam [7:0] COM      = 8'hBC;
localparam [7:0] SOF_TOK  = 8'hFB;
localparam [7:0] IDL_BYTE = 8'h1B;
localparam DAW = $clog2(DSK_DEPTH);

// ---------------- slicer + Gray decode ----------------
function [1:0] sym_of; input [2:0] lvl; reg signed [2:0] v; reg [1:0] g; begin
  v = lvl;
  if (v < -2)     g = 2'b00;
  else if (v < 0) g = 2'b01;
  else if (v < 2) g = 2'b11;
  else            g = 2'b10;
  sym_of = {g[1], g[0] ^ g[1]};
end endfunction

reg [63:0] cur_sym [0:3];         // 32 symbols x 2 bits, symbol s at [2s +: 2]
integer l, s, p, j;
always @(*) begin
  for (l = 0; l < 4; l = l + 1)
    for (s = 0; s < 32; s = s + 1)
      cur_sym[l][2*s +: 2] = sym_of(rx_level[96*l + 3*s +: 3]);
end

// ---------------- per-lane symbol aligner ----------------
reg [63:0] prev_sym [0:3];
reg [4:0]  off [0:3];             // symbol offset into {cur, prev}
reg [3:0]  al_v;                  // aligned word valid (one clock after the input word)
reg [63:0] al_w [0:3];            // aligned 8-byte lane word

function [7:0] byte_at;           // byte from 4 symbols starting at seq index t of {cur, prev}
  input [127:0] seq; input integer t; integer u; reg [7:0] b; begin
    b = 8'd0;
    for (u = 0; u < 4; u = u + 1) b = {b[5:0], seq[2*(t+u) +: 2]};
    byte_at = b;
  end endfunction

reg [127:0] seq_c [0:3];
reg [4:0]   off_n [0:3];
reg [63:0]  word_c [0:3];
always @(*) begin
  for (l = 0; l < 4; l = l + 1) begin
    seq_c[l] = {cur_sym[l], prev_sym[l]};       // index 0 = oldest symbol
    off_n[l] = off[l];
    if (align_en)
      for (p = 31; p >= 0; p = p - 1)            // lowest matching offset wins
        if (byte_at(seq_c[l], p) == COM) off_n[l] = p;
    for (j = 0; j < 8; j = j + 1)
      word_c[l][8*j +: 8] = byte_at(seq_c[l], off_n[l] + 4*j);
  end
end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    for (l = 0; l < 4; l = l + 1) begin prev_sym[l] <= 64'd0; off[l] <= 5'd0; al_w[l] <= 64'd0; end
    al_v <= 4'b0000;
  end else begin
    for (l = 0; l < 4; l = l + 1) begin
      al_v[l] <= rx_level_valid[l];
      if (rx_level_valid[l]) begin
        prev_sym[l] <= cur_sym[l];
        off[l]      <= off_n[l];
        al_w[l]     <= word_c[l];
      end
    end
  end
end

// ---------------- word deskew ----------------
reg [63:0]  dq [0:3][0:DSK_DEPTH-1];
reg [DAW:0] dq_wp [0:3], dq_rp [0:3];
reg         locked;
wire [3:0]  dq_empty, dq_full, head_com;
genvar gl;
generate for (gl = 0; gl < 4; gl = gl + 1) begin : g_dq
  assign dq_empty[gl] = (dq_wp[gl] == dq_rp[gl]);
  assign dq_full[gl]  = ((dq_wp[gl] - dq_rp[gl]) == DSK_DEPTH);
  assign head_com[gl] = !dq_empty[gl] && (dq[gl][dq_rp[gl][DAW-1:0]][7:0] == COM);
end endgenerate

wire all_ready = (dq_empty == 4'b0000);
wire pop_all   = locked ? all_ready : (all_ready && (head_com == 4'b1111));
// hunting: drop the head of every lane that is not at a COM word
wire [3:0] pop_hunt = (!locked && !(all_ready && head_com == 4'b1111)) ? (~head_com & ~dq_empty) : 4'b0000;
wire relock = locked && align_en && all_ready && (head_com != 4'b0000) && (head_com != 4'b1111);

assign deskew_locked = locked;

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    for (l = 0; l < 4; l = l + 1) begin dq_wp[l] <= 0; dq_rp[l] <= 0; end
    locked <= 1'b0; lane_err <= 1'b0;
  end else begin
    for (l = 0; l < 4; l = l + 1) begin
      if (al_v[l]) begin
        if (dq_full[l]) begin
          if (locked) lane_err <= 1'b1;
          dq_rp[l] <= dq_rp[l] + 1'b1;          // keep the newest words
        end
        dq_wp[l] <= dq_wp[l] + 1'b1;
      end
      if ((pop_all || pop_hunt[l]) && !(al_v[l] && dq_full[l]))
        dq_rp[l] <= dq_rp[l] + 1'b1;
    end
    if (!locked && pop_all) locked <= 1'b1;
    if (relock) locked <= 1'b0;
  end
end
always @(posedge clk) begin
  for (l = 0; l < 4; l = l + 1)
    if (al_v[l]) dq[l][dq_wp[l][DAW-1:0]] <= al_w[l];
end

reg [255:0] d_words;
always @(*) for (l = 0; l < 4; l = l + 1) d_words[64*l +: 64] = dq[l][dq_rp[l][DAW-1:0]];
wire d_valid = pop_all;

assign lane_words       = d_words;
assign lane_words_valid = d_valid;

// ---------------- lane swap ----------------
wire [255:0] sw = lane_reversed ? {d_words[63:0], d_words[127:64], d_words[191:128], d_words[255:192]} : d_words;

// ---------------- SOF strip + gap check + destripe ----------------
reg       in_flit;
reg [3:0] wcnt;
wire all_sof = (sw == {32{SOF_TOK}});
wire all_idl = (sw == {32{IDL_BYTE}});
assign gap_ok  = d_valid && !in_flit &&  (all_sof || all_idl);
assign gap_bad = d_valid && !in_flit && !(all_sof || all_idl);

reg [255:0] dstr;
integer k;
always @(*) for (k = 0; k < 32; k = k + 1) dstr[8*k +: 8] = sw[64*(k % 4) + 8*(k / 4) +: 8];

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    in_flit <= 1'b0; wcnt <= 4'd0;
    flit_out_valid <= 1'b0; flit_out_sof <= 1'b0; flit_out_eof <= 1'b0; flit_out_data <= 256'd0;
  end else begin
    flit_out_valid <= 1'b0; flit_out_sof <= 1'b0; flit_out_eof <= 1'b0;
    if (d_valid) begin
      if (!in_flit) begin
        if (all_sof) begin in_flit <= 1'b1; wcnt <= 4'd0; end
      end else begin
        flit_out_valid <= 1'b1;
        flit_out_data  <= dstr;
        flit_out_sof   <= (wcnt == 4'd0);
        flit_out_eof   <= (wcnt == WORDS_PER_FLIT - 1);
        wcnt           <= wcnt + 4'd1;
        if (wcnt == WORDS_PER_FLIT - 1) in_flit <= 1'b0;
      end
    end
  end
end

endmodule
