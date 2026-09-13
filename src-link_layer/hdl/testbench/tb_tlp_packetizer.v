`timescale 1ns/1ps

module tb_tlp_packetizer;
    reg clk;
    reg rst_n;
    reg req_valid, req_is_write;
    reg [63:0] req_addr;
    reg [9:0] req_length;
    reg [15:0] req_rid;
    reg [7:0] req_tag;
    reg [255:0] req_data;
    
    wire req_ready, tlp_valid;
    wire [127:0] tlp_header;
    wire [255:0] tlp_data;

    tlp_packetizer dut (.*);

    always #5 clk = ~clk;

    initial begin
        clk = 0; rst_n = 0; req_valid = 0;
        #15 rst_n = 1;

        // Test 1: Generate MRd64 (Memory Read 64-bit)
        @(posedge clk);
        req_valid = 1; req_is_write = 0;
        req_addr = 64'h0000_0001_DEAD_BEE0;
        req_length = 10'd16; // 16 DWs
        req_rid = 16'h0100;  // Bus 1, Dev 0, Func 0
        req_tag = 8'hAA;
        
        // Test 2: Generate MWr64 (Memory Write 64-bit)
        @(posedge clk);
        req_is_write = 1;
        req_addr = 64'h0000_0002_CAFE_BABE; // Note: lower 2 bits ignored in header
        req_data = 256'hFFFF_EEEE_DDDD_CCCC_BBBB_AAAA_9999_8888;
        
        @(posedge clk);
        req_valid = 0;
        
        #30 $finish;
    end
endmodule