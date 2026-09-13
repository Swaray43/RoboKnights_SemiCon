`timescale 1ns/1ps

module tb_tlp_packetizer_mps;
    reg clk, rst_n;
    reg req_valid, req_is_write, tx_engine_ready;
    reg [63:0] req_addr;
    reg [9:0] req_length;
    reg [15:0] req_rid;
    reg [7:0] req_tag;
    
    wire req_ready, tlp_valid;
    wire [127:0] tlp_header;
    wire [9:0] tlp_payload_len;

    tlp_packetizer #(
        .MAX_PAYLOAD_DW(10'd32) // Artificially lower MPS to 32 DW for quick testing
    ) dut (.*);

    always #5 clk = ~clk;

    initial begin
        clk = 0; rst_n = 0; req_valid = 0; tx_engine_ready = 1;
        #15 rst_n = 1;

        // Issue a single 80 DW read request. 
        // Expected output: 3 TLPs -> Lengths: 32, 32, 16.
        @(posedge clk);
        req_valid = 1; req_is_write = 0;
        req_addr = 64'h0000_1111_2222_0000;
        req_length = 10'd80; 
        req_rid = 16'h0100;
        req_tag = 8'h0A;
        
        @(posedge clk);
        req_valid = 0; // De-assert after acceptance

        // Wait for FSM to segment and output all 3 TLPs
        #150 $finish;
    end
endmodule