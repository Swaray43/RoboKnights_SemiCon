`timescale 1ns/1ps

module tb_tlp_packetizer;

    // Parameters
    parameter MAX_PAYLOAD_SIZE_DW = 4; // Set small to test MPS splitting easily
    parameter REQ_ID              = 16'h0200;

    // Clock and Reset
    reg clk;
    reg rst_n;

    // Command Interface
    reg         req_valid;
    wire        req_ready;
    reg         req_is_mwr;
    reg  [63:0] req_addr;
    reg  [9:0]  req_len;
    reg  [9:0]  req_tag;
    reg  [3:0]  req_fbe;
    reg  [3:0]  req_lbe;

    // Payload Interface (Upstream AXI-Stream)
    reg  [31:0] s_tdata;
    reg         s_tvalid;
    wire        s_tready;

    // Outbound TLP Stream (Downstream)
    wire [31:0] m_tdata;
    wire        m_tvalid;
    reg         m_tready;
    wire        m_tlast;

    // Loop integer for Verilog-2001 compatibility
    integer i;

    // DUT Instantiation
    tlp_packetizer #(
        .MAX_PAYLOAD_SIZE_DW(MAX_PAYLOAD_SIZE_DW),
        .REQ_ID(REQ_ID)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .req_valid(req_valid),
        .req_ready(req_ready),
        .req_is_mwr(req_is_mwr),
        .req_addr(req_addr),
        .req_len(req_len),
        .req_tag(req_tag),
        .req_fbe(req_fbe),
        .req_lbe(req_lbe),
        .s_tdata(s_tdata),
        .s_tvalid(s_tvalid),
        .s_tready(s_tready),
        .m_tdata(m_tdata),
        .m_tvalid(m_tvalid),
        .m_tready(m_tready),
        .m_tlast(m_tlast)
    );

    // Clock Generation (100 MHz -> 10ns Period)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Monitor Output
    always @(posedge clk) begin
        if (m_tvalid && m_tready) begin
            $display("[%0t ns] TLP OUT: Data=0x%08h, TLAST=%b", $time, m_tdata, m_tlast);
        end
    end

    // Synchronous Command Handshake Task
    task send_cmd;
        input        is_mwr;
        input [63:0] addr;
        input [9:0]  len;
        input [9:0]  tag;
        input [3:0]  fbe;
        input [3:0]  lbe;
        begin
            @(posedge clk);
            req_valid  <= 1'b1;
            req_is_mwr <= is_mwr;
            req_addr   <= addr;
            req_len    <= len;
            req_tag    <= tag;
            req_fbe    <= fbe;
            req_lbe    <= lbe;
            
            // Wait for clock edge where req_ready is asserted
            @(posedge clk);
            while (!req_ready) begin
                @(posedge clk);
            end
            req_valid <= 1'b0;
        end
    endtask

    // Synchronous Payload Handshake Task
    task send_payload;
        input [31:0] data;
        begin
            @(posedge clk);
            s_tvalid <= 1'b1;
            s_tdata  <= data;
            
            @(posedge clk);
            while (!s_tready) begin
                @(posedge clk);
            end
            s_tvalid <= 1'b0;
        end
    endtask

    // Task to wait for a TLP packet to finish transmitting (m_tlast handshake)
    task wait_tlp_end;
        begin
            @(posedge clk);
            while (!(m_tvalid && m_tready && m_tlast)) begin
                @(posedge clk);
            end
        end
    endtask

    // Main Test Sequence
    initial begin
        // Initialize Signals
        rst_n      = 0;
        req_valid  = 0;
        req_is_mwr = 0;
        req_addr   = 0;
        req_len    = 0;
        req_tag    = 0;
        req_fbe    = 0;
        req_lbe    = 0;
        s_tvalid   = 0;
        s_tdata    = 0;
        m_tready   = 1'b1;

        // Reset Sequence
        #20;
        rst_n = 1;
        #20;

        $display("\n--- TEST 1: MRd64 (No Payload, Length=2) ---");
        send_cmd(0, 64'h0000_0000_0000_1000, 10'd2, 10'h011, 4'hF, 4'hF);
        wait_tlp_end();
        #20;

        $display("\n--- TEST 2: MWr64 1-DW (Corner Case for LastBE Bug D) ---");
        fork
            begin
                send_cmd(1, 64'h0000_0000_0000_2000, 10'd1, 10'h022, 4'hF, 4'hF);
            end
            begin
                send_payload(32'hDEADBEEF);
            end
        join
        wait_tlp_end();
        #20;

        $display("\n--- TEST 3: MWr64 Large Transfer (MPS Split Test - Bug A/F) ---");
        fork
            begin
                send_cmd(1, 64'h0000_0000_0000_3000, 10'd9, 10'h033, 4'hF, 4'hF);
            end
            begin
                for (i = 0; i < 9; i = i + 1) begin
                    send_payload(32'hA000_0000 + i);
                end
            end
        join
        
        // Wait for all 3 split TLP packets (9 DWs split by MPS=4 -> 3 packets)
        wait_tlp_end(); // Packet 1 (4 DW payload)
        wait_tlp_end(); // Packet 2 (4 DW payload)
        wait_tlp_end(); // Packet 3 (1 DW payload)
        #50;

        $display("\n--- ALL TESTS COMPLETE ---");
        $finish;
    end

    // Safety Watchdog Timer (Prevents infinite hanging if DUT stalls)
    initial begin
        #10000;
        $display("\n[ERROR] Simulation watchdog timeout reached!");
        $finish;
    end

endmodule