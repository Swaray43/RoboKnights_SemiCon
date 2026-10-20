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
// Latency: 1 cycle after the 8th word to decode/check, output registered.
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
    wire [FL_PROT_W+1:0] dec      = flit_fec_decode(raw_r[FL_PROT_W-1:0], raw_r[FLIT_W-1:FL_FEC_LO]);
    wire                 dec_unc  = dec[FL_PROT_W+1];
    wire                 dec_cor  = dec[FL_PROT_W];
    wire [FL_PROT_W-1:0] fixed    = dec[FL_PROT_W-1:0];
    wire [63:0]          crc_calc = flit_crc64(fixed[FL_BODY_W-1:0]);
    wire                 crc_ok   = (crc_calc == fixed[FL_PROT_W-1:FL_CRC_LO]);
    wire [FL_DLP_W-1:0]  dlp      = fixed[FL_BODY_W-1:FL_DLP_LO];

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
            rx_flit_valid <= raw_v;
            crc_err       <= raw_v && !dec_unc && !crc_ok;
            fec_err       <= raw_v && dec_unc;
            fec_corrected <= raw_v && dec_cor && !dec_unc;
            if (raw_v) begin
                rx_flit_ok   <= !dec_unc && crc_ok;
                rx_kind      <= dlp[13:12];
                rx_seq       <= dlp[11:0];
                rx_ack_valid <= dlp[14];
                rx_nak_valid <= dlp[15];
                rx_ackn_seq  <= dlp[27:16];
                rx_payload   <= fixed[FL_TLP_W-1:0];
                rx_fc_dllp   <= dlp[FL_DLP_FC_BIT];
            end
        end
    end

endmodule
