`timescale 1ns/1ps

// ============================================================
// Module : link_layer_rx
// Layer  : Link layer (Engineer B) -- Flit deframer + FEC + CRC check
//
// Rewritten against the AXI4-PCIe6 bridge review (L6, L8, L3).
//
//   * Collects the 8 x 256-bit words of one Flit (resyncs on sof).
//   * Runs the 3-way interleaved SEC-DED decoder over bytes 0..249 and
//     corrects single-bit errors per interleave (L8; fec_err used to be
//     tied to 0). Uncorrectable errors raise fec_err.
//   * Checks CRC-64 over the corrected TLP area + DLP.
//   * Hands the decoded Flit to data_link_layer_top: TLP area (59 DW,
//     nothing dropped or zero-filled, L6), sequence number, kind, and the
//     piggy-backed ACK/NAK the far end sent us (L3).
//   * Sequencing, duplicate filtering and NAK generation live in
//     data_link_layer_top, because they need the TX side.
//
// Latency: 3 cycles after the 8th word (Step 4 T1 decode pipeline), output registered.
// No ready on this side: the PHY cannot be stalled. The consumer must
// take rx_flit_valid pulses when they come; data_link_layer_top NAKs a
// Flit it has no room for and the sender replays it.
// ============================================================

module link_layer_rx #(
    parameter SYS_BUS_W = 256,
    parameter SEQ_W     = 12
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // System-synchronous 256-bit bus from PHY
    input  wire                  s_sys_bus_valid,
    input  wire [SYS_BUS_W-1:0]  s_sys_bus_data,
    input  wire                  s_sys_bus_sof,
    input  wire                  s_sys_bus_eof,

    // Decoded Flit (one-cycle pulse per Flit)
    output reg                   rx_flit_valid,
    output reg                   rx_flit_ok,        // FEC ok/corrected AND CRC good
    output reg  [1:0]            rx_kind,
    output reg  [SEQ_W-1:0]      rx_seq,
    output reg                   rx_ack_valid,
    output reg                   rx_nak_valid,
    output reg  [SEQ_W-1:0]      rx_ackn_seq,
    output reg  [1887:0]         rx_payload,
    output reg                   rx_fc_dllp,        // G2: DLP[28] -- control Flit carries FC DLLPs

    // Status pulses
    output reg                   crc_err,           // CRC mismatch after FEC
    output reg                   fec_err,           // uncorrectable FEC error
    output reg                   fec_corrected      // FEC fixed at least one bit
);

    `include "flit_defs.vh"

    reg [SYS_BUS_W*7-1:0] word_buf;     // words 0..6
    reg [2:0]             word_idx;
    reg                   in_flit;

    reg [FLIT_W-1:0]      raw_r;
    reg                   raw_v;

    // ---------------- word collection ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            word_buf <= {SYS_BUS_W*7{1'b0}};
            word_idx <= 3'd0;
            in_flit  <= 1'b0;
            raw_r    <= {FLIT_W{1'b0}};
            raw_v    <= 1'b0;
        end else begin
            raw_v <= 1'b0;
            if (s_sys_bus_valid) begin
                if (s_sys_bus_sof) begin
                    // start (or resync) a Flit
                    word_buf[0 +: SYS_BUS_W] <= s_sys_bus_data;
                    word_idx <= 3'd1;
                    in_flit  <= 1'b1;
                end else if (in_flit) begin
                    if (word_idx == 3'd7) begin
                        raw_r    <= {s_sys_bus_data, word_buf};
                        raw_v    <= 1'b1;
                        in_flit  <= 1'b0;
                        word_idx <= 3'd0;
                    end else begin
                        word_buf[word_idx*SYS_BUS_W +: SYS_BUS_W] <= s_sys_bus_data;
                        word_idx <= word_idx + 3'd1;
                    end
                end
                // words outside a Flit (no sof seen) are ignored
            end
        end
    end

    // ---------------- FEC decode + CRC check ----------------
    // Step 4 T1: the old single-cycle decode (syndrome -> correction -> CRC
    // over the corrected data) was the worst path of the design (17 logic
    // levels, -4.4 ns at 250 MHz). It is now three register stages:
    //   D1  per-interleave syndrome and overall parity vs the received FEC bits
    //   D2  single-bit correction (or flag uncorrectable)
    //   D3  CRC-64 over the corrected body, compare, decode the DLP
    // Same results as flit_fec_decode / flit_crc64, two clocks later.
    // A Flit arrives at most every 8 clocks; the pipeline takes one per clock.
    reg                  d1_v;
    reg [FL_PROT_W-1:0]  d1_d;
    reg [9:0]            d1_s  [0:2];
    reg                  d1_o  [0:2];
    reg                  d2_v;
    reg [FL_PROT_W-1:0]  d2_fixed;
    reg                  d2_unc, d2_cor;

    integer jj, bb, kk;             // combinational correction loop only
    integer js;                     // X.3: own index for the clocked D1 loop (one integer in two always blocks = two drivers)
    reg [10:0] sp_c, p_c;
    reg [9:0]  c_rx_c;
    reg        ovr_rx_c;

    wire [32:0] raw_sp;
    flit_fec_syn u_syn (.d(raw_r[FL_PROT_W-1:0]), .sp(raw_sp));   // Wave C: explicit XOR trees

    // D2 combinational correction from the D1 registers
    reg [FL_PROT_W-1:0] fix_c;
    reg                 unc_c, cor_c, found_c;
    always @(*) begin
        fix_c = d1_d; unc_c = 1'b0; cor_c = 1'b0;
        p_c = 11'd2; found_c = 1'b0;     // X.3: assigned on every path, so no latch / logic loop
        for (jj = 0; jj < 3; jj = jj + 1) begin
            p_c = 11'd2; found_c = 1'b0;
            if (d1_s[jj] == 10'd0 && !d1_o[jj]) begin
                // clean
            end else if (d1_o[jj]) begin
                if (d1_s[jj] == 10'd0 || fl_is_pow2({1'b0, d1_s[jj]})) begin
                    cor_c = 1'b1;                       // error in a check bit / overall bit
                end else begin
                    p_c = 11'd2; found_c = 1'b0;
                    for (bb = jj; bb < FL_FEC_BYTES; bb = bb + 3) begin
                        for (kk = 0; kk < 8; kk = kk + 1) begin
                            p_c = p_c + 11'd1;
                            if (fl_is_pow2(p_c)) p_c = p_c + 11'd1;
                            if (p_c[9:0] == d1_s[jj] && p_c[10] == 1'b0) begin
                                fix_c[bb*8+kk] = ~fix_c[bb*8+kk];
                                found_c = 1'b1;
                            end
                        end
                    end
                    if (found_c) cor_c = 1'b1; else unc_c = 1'b1;
                end
            end else begin
                unc_c = 1'b1;                           // s != 0, even parity: double error
            end
        end
    end

    // Wave C: CRC-64 in two stages (D3 per-chunk parts, D4 combine + compare)
    reg                  d3_v;
    reg [FL_PROT_W-1:0]  d3_fixed;
    reg                  d3_unc, d3_cor;
    reg [511:0]          d3_parts;
    wire [511:0]         d2_parts;
    flit_crc64_parts u_crc_parts (.d(d2_fixed[FL_BODY_W-1:0]), .parts(d2_parts));   // explicit XOR trees
    wire [63:0]          crc_calc = flit_crc64_combine(d3_parts);
    wire                 crc_ok   = (crc_calc == d3_fixed[FL_PROT_W-1:FL_CRC_LO]);
    wire [FL_DLP_W-1:0]  dlp      = d3_fixed[FL_BODY_W-1:FL_DLP_LO];
`ifndef SYNTHESIS
    always @(posedge clk) if (rst_n && d3_v && (crc_calc !== flit_crc64(d3_fixed[FL_BODY_W-1:0])))
        $display("ERROR link_layer_rx @%0t: pipelined CRC-64 differs from flit_crc64", $time);
`endif

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            d1_v <= 1'b0; d1_d <= {FL_PROT_W{1'b0}};
            d1_s[0] <= 10'd0; d1_s[1] <= 10'd0; d1_s[2] <= 10'd0;
            d1_o[0] <= 1'b0;  d1_o[1] <= 1'b0;  d1_o[2] <= 1'b0;
            d2_v <= 1'b0; d2_fixed <= {FL_PROT_W{1'b0}}; d2_unc <= 1'b0; d2_cor <= 1'b0;
            d3_v <= 1'b0; d3_fixed <= {FL_PROT_W{1'b0}}; d3_unc <= 1'b0; d3_cor <= 1'b0; d3_parts <= 512'd0;
        end else begin
            d1_v <= raw_v;
            if (raw_v) begin
                d1_d <= raw_r[FL_PROT_W-1:0];
                for (js = 0; js < 3; js = js + 1) begin
                    sp_c     = raw_sp[11*js +: 11];
                    c_rx_c   = raw_r[FL_FEC_LO + js*FEC_CW_BITS +: 10];
                    ovr_rx_c = raw_r[FL_FEC_LO + js*FEC_CW_BITS + 10];
                    d1_s[js] <= sp_c[9:0] ^ c_rx_c;
                    d1_o[js] <= sp_c[10] ^ (^c_rx_c) ^ ovr_rx_c;
                end
            end
            d2_v <= d1_v;
            if (d1_v) begin
                d2_fixed <= fix_c;
                d2_unc   <= unc_c;
                d2_cor   <= cor_c;
            end
            d3_v <= d2_v;
            if (d2_v) begin
                d3_fixed <= d2_fixed;
                d3_unc   <= d2_unc;
                d3_cor   <= d2_cor;
                d3_parts <= d2_parts;
            end
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_flit_valid <= 1'b0;
            rx_flit_ok    <= 1'b0;
            rx_kind       <= 2'd0;
            rx_seq        <= {SEQ_W{1'b0}};
            rx_ack_valid  <= 1'b0;
            rx_nak_valid  <= 1'b0;
            rx_ackn_seq   <= {SEQ_W{1'b0}};
            rx_payload    <= {FL_TLP_W{1'b0}};
            rx_fc_dllp    <= 1'b0;
            crc_err       <= 1'b0;
            fec_err       <= 1'b0;
            fec_corrected <= 1'b0;
        end else begin
            rx_flit_valid <= d3_v;
            crc_err       <= d3_v && !d3_unc && !crc_ok;
            fec_err       <= d3_v && d3_unc;
            fec_corrected <= d3_v && d3_cor && !d3_unc;
            if (d3_v) begin
                rx_flit_ok   <= !d3_unc && crc_ok;
                rx_kind      <= dlp[13:12];
                rx_seq       <= dlp[11:0];
                rx_ack_valid <= dlp[14];
                rx_nak_valid <= dlp[15];
                rx_ackn_seq  <= dlp[27:16];
                rx_payload   <= d3_fixed[FL_TLP_W-1:0];
                rx_fc_dllp   <= dlp[FL_DLP_FC_BIT];
            end
        end
    end

endmodule
