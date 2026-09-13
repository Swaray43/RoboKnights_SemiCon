`timescale 1ns/1ps

/*
 * Module: dllp_parser
 * Function: 
 *  1. Snoops the incoming RX FLIT stream.
 *  2. Identifies ACK and NAK Data Link Layer Packets.
 *  3. Extracts the Sequence Number and pulses the valid lines for the Replay Buffer.
 */
module dllp_parser #(
    parameter FLIT_WIDTH = 2048,
    parameter SEQ_WIDTH  = 12
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // Snoop Interface from RX Engine
    input  wire                   rx_valid,
    input  wire [FLIT_WIDTH-1:0]  rx_data,

    // Feedback to Replay Buffer
    output reg                    ack_valid,
    output reg  [SEQ_WIDTH-1:0]   ack_seq,
    output reg                    nak_valid,
    output reg  [SEQ_WIDTH-1:0]   nak_seq
);

    // Protocol specific signature offsets (Simplified for demonstration)
    localparam ACK_SIGNATURE = 8'hA5;
    localparam NAK_SIGNATURE = 8'hE1;

    wire [7:0]          dllp_type = rx_data[FLIT_WIDTH-1 : FLIT_WIDTH-8];
    wire [SEQ_WIDTH-1:0] seq_num  = rx_data[FLIT_WIDTH-9 : FLIT_WIDTH-8-SEQ_WIDTH];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            ack_valid <= 1'b0;
            ack_seq   <= 0;
            nak_valid <= 1'b0;
            nak_seq   <= 0;
        end else begin
            ack_valid <= 1'b0;
            nak_valid <= 1'b0;

            if (rx_valid) begin
                if (dllp_type == ACK_SIGNATURE) begin
                    ack_valid <= 1'b1;
                    ack_seq   <= seq_num;
                end else if (dllp_type == NAK_SIGNATURE) begin
                    nak_valid <= 1'b1;
                    nak_seq   <= seq_num;
                end
            end
        end
    end
endmodule