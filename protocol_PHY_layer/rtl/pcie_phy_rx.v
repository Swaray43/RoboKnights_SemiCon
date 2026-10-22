// =============================================================================
// pcie_phy_rx - synthesizable RX datapath of the Protocol-to-PHY layer
// (fixes P1 RX half and P2 RX half)
//
//   SerDes ==> rx_level[11:0] / rx_level_valid[3:0]
//     -> 4x lane_rx (PAM4 slicer -> Gray decode -> symbols -> byte, COM lock)
//     -> rx_lane_deskew (per-lane FIFOs aligned on the TS1 COM symbol, so
//        lane-to-lane skew in the channel is removed; lanes leave in lockstep)
//     -> lane_swap_mux (lane_reversed from training)
//     -> flit_sof_strip (waits for 0xFB on all lanes, then passes one Flit
//        = WORDS_PER_FLIT x 8 lane-cycles, ignoring 0xFB inside the payload)
//     -> flit_lane_destriper (back to 256-bit words)
//     -> sof/eof rebuilt per Flit for link_layer_rx
//
// There is no backpressure on RX (as in PCIe, the receiver must sink at line
// rate); link_layer_rx must accept a word on every flit_out_valid.
// =============================================================================
module pcie_phy_rx #(
    parameter WORDS_PER_FLIT = 8
)(
    input  wire         clk,
    input  wire         rst_n,

    // ---- 4-lane PAM4 symbol bus from the SerDes ----
    input  wire [11:0]  rx_level,        // lane i = rx_level[3*i +: 3], signed
    input  wire [3:0]   rx_level_valid,

    input  wire         lane_reversed,   // from lane_reversal_latch
    input  wire         align_en,        // byte alignment on COM while training (link not up)

    // ---- to link layer (link_layer_rx phy_rx side) ----
    output reg  [255:0] flit_out_data,
    output reg          flit_out_valid,
    output reg          flit_out_sof,
    output reg          flit_out_eof,

    output reg          lane_err,        // sticky: deskew FIFO overflowed after lock (skew grew)
    output wire         deskew_locked,   // lanes aligned on COM

    // ---- K6: between-Flit check for pcie_ltssm (L0 link-loss detection) ----
    // While no Flit is being received, every lane cycle should be logical idle
    // (pcie_phy_tx IDL_BYTE on all lanes) or a SOF (0xFB on all lanes).
    output wire         gap_ok,          // between Flits: all lanes IDL, or all lanes SOF
    output wire         gap_bad,         // between Flits: anything else (noise, slip, junk)

    // ---- deskewed per-lane bytes BEFORE the lane swap, for the TS1/TS2
    // detectors. Training must see physical lane numbers to detect reversal,
    // and lane_reversal_detector needs all 4 lanes' ordered sets in the same
    // cycle, which only holds after deskew.
    output wire [31:0]  lane_bytes,      // lane i = [8*i +: 8]
    output wire [3:0]   lane_bytes_valid
);

wire [7:0] b0, b1, b2, b3;
wire [3:0] bv;

lane_rx u_lane0 (.clk(clk), .rst_n(rst_n), .align_en(align_en), .rx_level_valid(rx_level_valid[0]), .rx_level(rx_level[2:0]),
                 .byte_out_valid(bv[0]), .byte_out(b0));
lane_rx u_lane1 (.clk(clk), .rst_n(rst_n), .align_en(align_en), .rx_level_valid(rx_level_valid[1]), .rx_level(rx_level[5:3]),
                 .byte_out_valid(bv[1]), .byte_out(b1));
lane_rx u_lane2 (.clk(clk), .rst_n(rst_n), .align_en(align_en), .rx_level_valid(rx_level_valid[2]), .rx_level(rx_level[8:6]),
                 .byte_out_valid(bv[2]), .byte_out(b2));
lane_rx u_lane3 (.clk(clk), .rst_n(rst_n), .align_en(align_en), .rx_level_valid(rx_level_valid[3]), .rx_level(rx_level[11:9]),
                 .byte_out_valid(bv[3]), .byte_out(b3));

wire [31:0] d_bytes;
wire        d_valid;
wire        d_overflow;
rx_lane_deskew #(.DEPTH(16), .MAX_SKEW(8)) u_deskew (
  .clk(clk), .rst_n(rst_n), .align_en(align_en),
  .in_bytes({b3, b2, b1, b0}), .in_valid(bv),
  .out_bytes(d_bytes), .out_valid(d_valid),
  .locked(deskew_locked), .overflow(d_overflow)
);

assign lane_bytes       = d_bytes;
assign lane_bytes_valid = {4{d_valid}};

wire [7:0] s0, s1, s2, s3;
lane_swap_mux u_swap (
  .lane_reversed(lane_reversed),
  .lane_in_0(d_bytes[7:0]), .lane_in_1(d_bytes[15:8]), .lane_in_2(d_bytes[23:16]), .lane_in_3(d_bytes[31:24]),
  .lane_out_0(s0), .lane_out_1(s1), .lane_out_2(s2), .lane_out_3(s3)
);

// K6: classify each between-Flit lane cycle (after deskew and lane swap)
localparam [7:0] IDL_BYTE = 8'h1B;     // must match pcie_phy_tx
wire strip_hunting;
wire all_idl = (s0 == IDL_BYTE) && (s1 == IDL_BYTE) && (s2 == IDL_BYTE) && (s3 == IDL_BYTE);
wire all_sof = (s0 == 8'hFB) && (s1 == 8'hFB) && (s2 == 8'hFB) && (s3 == 8'hFB);
assign gap_ok  = d_valid && strip_hunting &&  (all_idl || all_sof);
assign gap_bad = d_valid && strip_hunting && !(all_idl || all_sof);

wire [7:0] p0, p1, p2, p3;
wire [3:0] pv;
wire       p_sof;
flit_sof_strip #(.PAYLOAD_CYCLES(8 * WORDS_PER_FLIT), .RESYNC_MIDFLIT(0)) u_sofstrip (
  .clk(clk), .rst_n(rst_n),
  .lane_in_0(s0), .lane_in_1(s1), .lane_in_2(s2), .lane_in_3(s3),
  .lane_in_valid(d_valid),
  .lane_out_0(p0), .lane_out_1(p1), .lane_out_2(p2), .lane_out_3(p3),
  .lane_valid_out(pv), .sof_out(p_sof), .hunting(strip_hunting)
);

wire         w_valid;
wire [255:0] w_data;
flit_lane_destriper u_destriper (
  .clk(clk), .rst_n(rst_n),
  .lane_valid_in(pv), .lane_data_in({p3, p2, p1, p0}),
  .flit_valid_out(w_valid), .flit_out(w_data)
);

// ---- sof/eof rebuild: the strip marks the first payload cycle of each Flit;
// the next word the destriper completes is word 0 of that Flit.
reg       pending_sof;
reg [3:0] word_idx;      // index of the NEXT word to leave

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    pending_sof    <= 1'b0;
    word_idx       <= 4'd0;
    flit_out_valid <= 1'b0;
    flit_out_sof   <= 1'b0;
    flit_out_eof   <= 1'b0;
    flit_out_data  <= 256'd0;
    lane_err       <= 1'b0;
  end else begin
    flit_out_valid <= 1'b0;
    flit_out_sof   <= 1'b0;
    flit_out_eof   <= 1'b0;

    if (d_overflow) lane_err <= 1'b1;

    if (w_valid) begin
      flit_out_valid <= 1'b1;
      flit_out_data  <= w_data;
      flit_out_sof   <= pending_sof;
      if (pending_sof) begin
        flit_out_eof <= (WORDS_PER_FLIT == 1);
        word_idx     <= (WORDS_PER_FLIT == 1) ? 4'd0 : 4'd1;
      end else begin
        flit_out_eof <= (word_idx == WORDS_PER_FLIT - 1);
        word_idx     <= (word_idx == WORDS_PER_FLIT - 1) ? 4'd0 : word_idx + 4'd1;
      end
      pending_sof <= 1'b0;
    end
    if (p_sof) pending_sof <= 1'b1;   // takes priority over the clear above
  end
end

endmodule
