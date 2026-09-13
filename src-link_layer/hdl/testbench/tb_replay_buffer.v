`timescale 1ns/1ps

module tb_replay_buffer;

    parameter FLIT_WIDTH = 64; // Scaled down for easy viewing
    parameter SEQ_WIDTH  = 12;

    reg clk, rst_n;
    reg s_valid, m_ready;
    reg [FLIT_WIDTH-1:0] s_data;
    wire s_ready, m_valid;
    wire [FLIT_WIDTH-1:0] m_data;

    reg rx_ack_valid, rx_nak_valid;
    reg [SEQ_WIDTH-1:0] rx_ack_seq, rx_nak_seq;

    replay_buffer #(
        .FLIT_WIDTH(FLIT_WIDTH),
        .SEQ_WIDTH(SEQ_WIDTH),
        .BUF_DEPTH(8)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .s_valid(s_valid), .s_data(s_data), .s_ready(s_ready),
        .m_valid(m_valid), .m_data(m_data), .m_ready(m_ready),
        .rx_ack_valid(rx_ack_valid), .rx_ack_seq(rx_ack_seq),
        .rx_nak_valid(rx_nak_valid), .rx_nak_seq(rx_nak_seq)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0; rst_n = 0;
        s_valid = 0; s_data = 0; m_ready = 1;
        rx_ack_valid = 0; rx_nak_valid = 0;
        
        #20 rst_n = 1;

        // 1. Send 3 FLITs
        @(posedge clk) s_valid = 1; s_data = 64'hAAAA;
        @(posedge clk) s_valid = 1; s_data = 64'hBBBB;
        @(posedge clk) s_valid = 1; s_data = 64'hCCCC;
        @(posedge clk) s_valid = 0;

        #30;
        
        // 2. NAK Sequence 1 (BBBB dropped on wire, require replay)
        @(posedge clk) rx_nak_valid = 1; rx_nak_seq = 1;
        @(posedge clk) rx_nak_valid = 0;

        #40;
        
        // 3. ACK Sequence 2 (All cleared)
        @(posedge clk) rx_ack_valid = 1; rx_ack_seq = 2;
        @(posedge clk) rx_ack_valid = 0;

        #50 $finish;
    end
endmodule