`timescale 1ns/1ps

module tb_step9_flit_fec;

    reg clk;
    reg rst_n;

    // TX TLP Stream Input
    reg         tx_tlp_valid;
    reg  [31:0] tx_tlp_data;
    reg         tx_tlp_sof;
    reg         tx_tlp_eof;
    wire        tx_tlp_ready;

    // Physical Link (with error injection capability)
    wire         link_tx_valid;
    wire [127:0] link_tx_data;
    wire [7:0]   link_tx_fec;
    
    reg          link_rx_valid;
    reg  [127:0] link_rx_data;
    reg  [7:0]   link_rx_fec;

    // Error Injection Controls
    reg [127:0]  err_mask;
    
    // RX TLP Stream Output (from Deframer)
    wire         rx_tlp_valid;
    wire [31:0]  rx_tlp_data;
    wire         rx_tlp_sof;
    wire         rx_tlp_eof;
    wire         rx_flit_err;

    // Parsed Output (to RX Engine)
    wire         parsed_rx_valid;
    wire         parsed_rx_sof;
    wire         parsed_rx_eof;
    wire [31:0]  parsed_rx_data;
    wire [4:0]   parsed_type;
    wire [9:0]   parsed_length_dw;
    wire [9:0]   parsed_tag;
    wire [6:0]   parsed_lower_addr;
    wire [2:0]   parsed_cpl_status;
    wire [63:0]  parsed_addr;

    // Instantiate FLIT Framer (which contains FEC Encoder/Decoder)
    flit_framer #(
        .FLIT_WIDTH(128),
        .FEC_WIDTH(8)
    ) dut_framer (
        .clk(clk),
        .rst_n(rst_n),
        .tx_tlp_valid(tx_tlp_valid),
        .tx_tlp_data(tx_tlp_data),
        .tx_tlp_sof(tx_tlp_sof),
        .tx_tlp_eof(tx_tlp_eof),
        .tx_tlp_ready(tx_tlp_ready),
        .m_flit_tx_valid(link_tx_valid),
        .m_flit_tx_data(link_tx_data),
        .m_flit_tx_fec(link_tx_fec),
        .s_flit_rx_valid(link_rx_valid),
        .s_flit_rx_data(link_rx_data),
        .s_flit_rx_fec(link_rx_fec),
        .rx_tlp_valid(rx_tlp_valid),
        .rx_tlp_data(rx_tlp_data),
        .rx_tlp_sof(rx_tlp_sof),
        .rx_tlp_eof(rx_tlp_eof),
        .rx_flit_err(rx_flit_err)
    );

    // Instantiate Header Parser Bridge
    tlp_header_parser #(
        .TAG_WIDTH(10),
        .PCIE_ADDR_W(64)
    ) dut_parser (
        .clk(clk),
        .rst_n(rst_n),
        .rx_tlp_valid(rx_tlp_valid),
        .rx_tlp_data(rx_tlp_data),
        .rx_tlp_sof(rx_tlp_sof),
        .rx_tlp_eof(rx_tlp_eof),
        .parsed_rx_valid(parsed_rx_valid),
        .parsed_rx_sof(parsed_rx_sof),
        .parsed_rx_eof(parsed_rx_eof),
        .parsed_rx_data(parsed_rx_data),
        .parsed_type(parsed_type),
        .parsed_length_dw(parsed_length_dw),
        .parsed_tag(parsed_tag),
        .parsed_lower_addr(parsed_lower_addr),
        .parsed_cpl_status(parsed_cpl_status),
        .parsed_addr(parsed_addr)
    );

    // Clock Generation
    always #5 clk = ~clk;

    // Physical Link Pass-through with Error Injection Mux
    always @(*) begin
        link_rx_valid = link_tx_valid;
        link_rx_data  = link_tx_data ^ err_mask; // XOR mask to flip bits
        link_rx_fec   = link_tx_fec;
    end

    // Test Sequence
    initial begin
        clk   = 0;
        rst_n = 0;
        tx_tlp_valid = 0;
        err_mask = 128'd0;
        
        #20 rst_n = 1;
        $display("--- STEP 9 TEST: FLIT FRAMER & FEC VERIFICATION ---");

        // TEST 1: Clean MWr 3-DW Transmission
        #10;
        $display("[ %8.3f ns ] Injecting Clean 3-DW MWr TLP...", $time);
        tx_tlp_valid = 1; tx_tlp_sof = 1; tx_tlp_eof = 0; tx_tlp_data = 32'h40000001; // MWr, 1 DW
        #10 tx_tlp_sof = 0; tx_tlp_data = 32'h0000010F; // Requester ID / Tag
        #10 tx_tlp_eof = 1; tx_tlp_data = 32'h80001000; // Address
        #10 tx_tlp_valid = 0; tx_tlp_eof = 0;

        #50;
        
        // TEST 2: Single-Bit Error Correction (1-DW CplD TLP)
        $display("[ %8.3f ns ] Injecting 1-DW CplD TLP with SINGLE-BIT ERROR on link...", $time);
        err_mask = 128'h00000000000000000000000000000008; // Flip bit 3 on physical link
        tx_tlp_valid = 1; tx_tlp_sof = 1; tx_tlp_eof = 0; tx_tlp_data = 32'h4A000001; // CplD
        #10 tx_tlp_sof = 0; tx_tlp_data = 32'h00000000;
        #10 tx_tlp_eof = 1; tx_tlp_data = 32'hA1B2C3D4; // Payload
        #10 tx_tlp_valid = 0; tx_tlp_eof = 0;
        
        #50;

        // TEST 3: Double-Bit Uncorrectable Error
        $display("[ %8.3f ns ] Injecting 3-DW MRd TLP with DOUBLE-BIT ERROR on link...", $time);
        err_mask = 128'h00000000000000000000000000000018; // Flip bits 3 and 4
        tx_tlp_valid = 1; tx_tlp_sof = 1; tx_tlp_eof = 0; tx_tlp_data = 32'h00000001; // MRd
        #10 tx_tlp_sof = 0; tx_tlp_data = 32'h0000020F; // Tag = 0x02
        #10 tx_tlp_eof = 1; tx_tlp_data = 32'h40002000; // Address
        #10 tx_tlp_valid = 0; tx_tlp_eof = 0;
        err_mask = 128'd0;

        #50;
        $display("--- STEP 9 COMPLETE ---");
        $finish;
    end

    // Real-time protocol monitors
    always @(posedge clk) begin
        if (parsed_rx_valid && parsed_rx_sof) begin
            $display("[ %8.3f ns ] PARSER: SOF Detected | Type: 0x%02x | Len: %0d DW", 
                     $time, parsed_type, parsed_length_dw);
        end
        if (parsed_rx_valid && dut_parser.header_phase == 2'd2) begin
            $display("[ %8.3f ns ] PARSER: Extracted Tag: 0x%02x | Addr/Payload: 0x%08x", 
                     $time, parsed_tag, parsed_addr[31:0]);
        end
        if (rx_flit_err) begin
            $display("[ %8.3f ns ] FEC ALARM: Uncorrectable Multi-bit Error Detected! FLIT dropped.", $time);
        end
    end

endmodule