`timescale 1ns/1ps

module fec_encoder_decoder #(
    parameter DATA_WIDTH = 128,
    parameter FEC_WIDTH  = 8
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

    // SEC-DED Parity Generator Function
    function [7:0] calc_parity;
        input [127:0] d;
        begin
            calc_parity[0] = ^(d & 128'h0000000000000000FFFFFFFFFFFFFFFF);
            calc_parity[1] = ^(d & 128'h00000000FFFFFFFF00000000FFFFFFFF);
            calc_parity[2] = ^(d & 128'h0000FFFF0000FFFF0000FFFF0000FFFF);
            calc_parity[3] = ^(d & 128'h00FF00FF00FF00FF00FF00FF00FF00FF);
            calc_parity[4] = ^(d & 128'h0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F0F);
            calc_parity[5] = ^(d & 128'h33333333333333333333333333333333);
            calc_parity[6] = ^(d & 128'h55555555555555555555555555555555);
            calc_parity[7] = ^d; // Overall Parity Bit
        end
    endfunction

    wire [7:0] generated_tx_fec = calc_parity(tx_data);
    wire [7:0] expected_rx_fec  = calc_parity(rx_fec_data);
    wire [7:0] syndrome         = rx_fec_code ^ expected_rx_fec;

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

    // Real-Time Bit Repair Helper Function
    function [127:0] repair_data;
        input [127:0] data;
        input [7:0]   syn;
        integer i;
        begin
            repair_data = data;
            for (i = 0; i < 128; i = i + 1) begin
                if (syn[6:0] == (7'd127 - i[6:0])) begin
                    repair_data[i] = ~data[i];
                end
            end
        end
    endfunction

    // RX Decoding, Error Detection & Correction
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_corr_data         <= {DATA_WIDTH{1'b0}};
            rx_uncorrectable_err <= 1'b0;
            rx_corrected_err     <= 1'b0;
        end else if (rx_val) begin
            if (syndrome == 8'h00) begin
                // No Error
                rx_corr_data         <= rx_fec_data;
                rx_uncorrectable_err <= 1'b0;
                rx_corrected_err     <= 1'b0;
            end else if (syndrome[7] == 1'b1) begin
                // Single-Bit Correctable Error
                rx_corr_data         <= repair_data(rx_fec_data, syndrome);
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