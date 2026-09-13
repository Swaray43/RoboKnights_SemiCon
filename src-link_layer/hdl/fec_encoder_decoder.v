`timescale 1ns/1ps

/*
 * Module: fec_encoder_decoder
 *
 * This version computes parity generically from DATA_WIDTH/FEC_WIDTH so it
 * scales to any width, using the standard extended-Hamming construction:
 *   - POS_BITS = FEC_WIDTH-1 position-parity bits: parity[k] covers every
 *     data bit whose index has bit k set.
 *   - 1 overall-parity bit (index POS_BITS) to distinguish a correctable
 *     single-bit error from an uncorrectable double-bit error.
 * This requires 2^(FEC_WIDTH-1) >= DATA_WIDTH (enough position codes to
 * address every data bit); this is checked at elaboration time below.
 * For example FLIT_WIDTH=2048 needs FEC_WIDTH >= 12.
 */
module fec_encoder_decoder #(
    parameter DATA_WIDTH = 128,
    parameter FEC_WIDTH  = 8    // must satisfy 2^(FEC_WIDTH-1) >= DATA_WIDTH
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // TX Path (Encoder)
    input  wire                  tx_val,
    input  wire [DATA_WIDTH-1:0] tx_data,
    output reg  [DATA_WIDTH-1:0] tx_fec_data,
    output reg  [FEC_WIDTH-1:0]  tx_fec_code,

    // RX Path (Decoder & Corrector)
    input  wire                  rx_val,
    input  wire [DATA_WIDTH-1:0] rx_fec_data,
    input  wire [FEC_WIDTH-1:0]  rx_fec_code,
    output reg  [DATA_WIDTH-1:0] rx_corr_data,
    output reg                   rx_uncorrectable_err,
    output reg                   rx_corrected_err
);

    localparam integer POS_BITS = FEC_WIDTH - 1;

    // synthesis translate_off
    initial begin
        if ((1 << POS_BITS) < DATA_WIDTH) begin
            $display("ERROR: fec_encoder_decoder: FEC_WIDTH=%0d cannot cover DATA_WIDTH=%0d (need 2^(FEC_WIDTH-1) >= DATA_WIDTH)",
                      FEC_WIDTH, DATA_WIDTH);
            $finish;
        end
    end
    // synthesis translate_on

    // Generic SEC-DED parity generator: parity[k] = XOR of every data bit
    // whose bit index has bit k set; parity[POS_BITS] is the overall
    // (whole-word) parity bit.
    function [FEC_WIDTH-1:0] calc_parity;
        input [DATA_WIDTH-1:0] d;
        integer i, k;
        begin
            calc_parity = {FEC_WIDTH{1'b0}};
            for (k = 0; k < POS_BITS; k = k + 1) begin
                for (i = 0; i < DATA_WIDTH; i = i + 1) begin
                    if (i[k])
                        calc_parity[k] = calc_parity[k] ^ d[i];
                end
            end
            calc_parity[POS_BITS] = ^d;
        end
    endfunction

    wire [FEC_WIDTH-1:0] generated_tx_fec = calc_parity(tx_data);
    wire [FEC_WIDTH-1:0] expected_rx_fec  = calc_parity(rx_fec_data);
    wire [FEC_WIDTH-1:0] syndrome         = rx_fec_code ^ expected_rx_fec;
    wire [POS_BITS-1:0]  err_pos          = syndrome[POS_BITS-1:0];

    // TX Encoding Register Pipeline
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_fec_data <= {DATA_WIDTH{1'b0}};
            tx_fec_code <= {FEC_WIDTH{1'b0}};
        end else if (tx_val) begin
            tx_fec_data <= tx_data;
            tx_fec_code <= generated_tx_fec;
        end
    end

    // Real-Time Bit Repair Helper Function (generic width)
    function [DATA_WIDTH-1:0] repair_data;
        input [DATA_WIDTH-1:0] data;
        input [POS_BITS-1:0]   pos;
        begin
            repair_data = data;
            if (pos < DATA_WIDTH)
                repair_data[pos] = ~data[pos];
        end
    endfunction

    // RX Decoding, Error Detection & Correction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_corr_data         <= {DATA_WIDTH{1'b0}};
            rx_uncorrectable_err <= 1'b0;
            rx_corrected_err     <= 1'b0;
        end else if (rx_val) begin
            if (syndrome == {FEC_WIDTH{1'b0}}) begin
                // No Error
                rx_corr_data         <= rx_fec_data;
                rx_uncorrectable_err <= 1'b0;
                rx_corrected_err     <= 1'b0;
            end else if (syndrome[POS_BITS] == 1'b1) begin
                // Single-Bit Correctable Error
                rx_corr_data         <= repair_data(rx_fec_data, err_pos);
                rx_uncorrectable_err <= 1'b0;
                rx_corrected_err     <= 1'b1;
            end else begin
                // Double-Bit or Multi-Bit Uncorrectable Error
                rx_corr_data         <= rx_fec_data;
                rx_uncorrectable_err <= 1'b1;
                rx_corrected_err     <= 1'b0;
            end
        end else begin
            rx_uncorrectable_err <= 1'b0;
            rx_corrected_err     <= 1'b0;
        end
    end

endmodule