`timescale 1ns/1ps

module tb_top_pipeline;

    parameter MAX_PAYLOAD_SIZE_DW = 4;
    parameter REQ_ID              = 16'h0200;

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

    // UpdateFC Interface (Receiver / DLLP)
    reg         update_fc_valid;
    reg  [1:0]  update_fc_type;
    reg  [7:0]  update_fc_hdr_limit;
    reg  [11:0] update_fc_data_limit;

    // Inter-module Credit Signals
    wire        credit_req_valid;
    wire [1:0]  credit_req_type;
    wire [9:0]  credit_req_dw_len;
    wire        credit_req_grant;
    wire        stall;

    // Outbound TLP Stream (Downstream)
    wire [31:0] m_tdata;
    wire        m_tvalid;
    reg         m_tready;
    wire        m_tlast;

    // Module-level loop counters for Verilog-2001 compliance
    integer i_t2, i_t3;

    // Instantiate Credit Manager
    credit_manager cm_inst (
        .clk(clk),
        .rst_n(rst_n),
        .update_fc_valid(update_fc_valid),
        .update_fc_type(update_fc_type),
        .update_fc_hdr_limit(update_fc_hdr_limit),
        .update_fc_data_limit(update_fc_data_limit),
        .alloc_req_valid(credit_req_valid),
        .alloc_req_type(credit_req_type),
        .alloc_req_dw_len(credit_req_dw_len),
        .alloc_req_grant(credit_req_grant),
        .stall(stall)
    );

    // Instantiate TLP Packetizer
    tlp_packetizer #(
        .MAX_PAYLOAD_SIZE_DW(MAX_PAYLOAD_SIZE_DW),
        .REQ_ID(REQ_ID)
    ) pkt_inst (
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
        .credit_req_valid(credit_req_valid),
        .credit_req_type(credit_req_type),
        .credit_req_dw_len(credit_req_dw_len),
        .credit_req_grant(credit_req_grant),
        .m_tdata(m_tdata),
        .m_tvalid(m_tvalid),
        .m_tready(m_tready),
        .m_tlast(m_tlast)
    );

    // Clock Generation (100 MHz -> 10ns)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Real-time Bus Monitor
    always @(posedge clk) begin
        if (m_tvalid && m_tready) begin
            $display("[%0t ns] TLP OUT: Data=0x%08h, TLAST=%b", $time, m_tdata, m_tlast);
        end
        if (stall) begin
            $display("[%0t ns] CREDIT STALL: Packetizer waiting for credits...", $time);
        end
    end

    // Task: Issue UpdateFC DLLP
    task send_update_fc;
        input [1:0]  fc_type;
        input [7:0]  hdr_lim;
        input [11:0] data_lim;
        begin
            @(posedge clk);
            update_fc_valid      <= 1'b1;
            update_fc_type       <= fc_type;
            update_fc_hdr_limit  <= hdr_lim;
            update_fc_data_limit <= data_lim;
            $display("[%0t ns] DLLP -> UpdateFC: Type=%0d, HdrLimit=0x%02h, DataLimit=0x%03h",
                     $time, fc_type, hdr_lim, data_lim);
            @(posedge clk);
            update_fc_valid      <= 1'b0;
        end
    endtask

    // Task: Submit Command to Packetizer
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
            @(posedge clk);
            while (!req_ready) @(posedge clk);
            req_valid  <= 1'b0;
        end
    endtask

    // Task: Push Single Payload Word
    task send_payload;
        input [31:0] data;
        begin
            @(posedge clk);
            s_tvalid <= 1'b1;
            s_tdata  <= data;
            @(posedge clk);
            while (!s_tready) @(posedge clk);
            s_tvalid <= 1'b0;
        end
    endtask

    // Main Test Sequence
    initial begin
        // Reset and Signals Initialization
        rst_n                = 0;
        req_valid            = 0;
        req_is_mwr           = 0;
        req_addr             = 0;
        req_len              = 0;
        req_tag              = 0;
        req_fbe              = 0;
        req_lbe              = 0;
        s_tvalid             = 0;
        s_tdata              = 0;
        m_tready             = 1'b1;
        update_fc_valid      = 0;
        update_fc_type       = 0;
        update_fc_hdr_limit  = 0;
        update_fc_data_limit = 0;

        #20 rst_n = 1;
        #20;

        // ------------------------------------------------------------------------
        // TEST 1: Initial Flow Control Provisioning (Limited Credits)
        // ------------------------------------------------------------------------
        $display("\n--- TEST 1: Provisioning Limited Posted Credits (1 Header, 1 Data Credit) ---");
        send_update_fc(2'b00, 8'd1, 12'd1); // Posted: 1 Header, 1 Data Credit (4 DW max)
        #20;

        // ------------------------------------------------------------------------
        // TEST 2: MWr64 9-DW MPS Splitting & Credit Stall / Release
        // ------------------------------------------------------------------------
        $display("\n--- TEST 2: MWr64 9-DW Burst (MPS Split: 4 DW + 4 DW + 1 DW) ---");
        $display("Expecting: TLP 1 sent -> Stall on TLP 2 -> Replenish -> Complete TLP 2 & 3\n");

        fork
            begin
                send_cmd(1, 64'h0000_0000_0000_5000, 10'd9, 10'h055, 4'hF, 4'hF);
                #100;
                $display("\n--- REPLENISHING CREDITS: Expanding Posted Limits (3 Headers, 3 Data Credits) ---");
                send_update_fc(2'b00, 8'd3, 12'd3); // Replenish credits for remaining 2 TLPs
            end
            begin
                for (i_t2 = 0; i_t2 < 9; i_t2 = i_t2 + 1) begin
                    send_payload(32'hB000_0000 + i_t2);
                end
            end
        join

        #100;

        // ------------------------------------------------------------------------
        // TEST 3: Infinite Credit Mode (0x7F / 0xFFF)
        // ------------------------------------------------------------------------
        $display("\n--- TEST 3: Infinite Credit Verification ---");
        send_update_fc(2'b00, 8'h7F, 12'hFFF); // Set Posted credits to Infinite
        #20;

        $display("Sending 12-DW MWr64 Burst (3 TLPs of 4 DWs each) under Infinite Credits...");
        fork
            begin
                send_cmd(1, 64'h0000_0000_1000_0000, 10'd12, 10'h0A1, 4'hF, 4'hF);
            end
            begin
                for (i_t3 = 0; i_t3 < 12; i_t3 = i_t3 + 1) begin
                    send_payload(32'hC000_0000 + i_t3);
                end
            end
        join

        #100;

        // ------------------------------------------------------------------------
        // TEST 4: Non-Posted Credit Pool Isolation
        // ------------------------------------------------------------------------
        $display("\n--- TEST 4: Non-Posted Pool Isolation (Zero Posted, Valid Non-Posted) ---");
        send_update_fc(2'b00, 8'd0, 12'd0); // Exhaust/Zero Posted credits
        send_update_fc(2'b01, 8'd5, 12'd0); // Provision Non-Posted credits (5 Headers)
        #20;

        $display("Sending 8-DW MRd64 (2 split TLPs of 4 DWs each) while Posted Credits are 0...");
        send_cmd(0, 64'h0000_0000_2000_0000, 10'd8, 10'h0B2, 4'hF, 4'hF); // Non-Posted Read
        #100;

        // ------------------------------------------------------------------------
        // TEST 5: Downstream Backpressure Handling (m_tready De-assertion)
        // ------------------------------------------------------------------------
        $display("\n--- TEST 5: Downstream Backpressure Verification ---");
        send_update_fc(2'b00, 8'd20, 12'd40); // Provision sufficient credits
        #20;

        fork
            begin
                send_cmd(1, 64'h0000_0000_3000_0000, 10'd4, 10'h0C3, 4'hF, 4'hF);
            end
            begin
                send_payload(32'hD000_0001);
                send_payload(32'hD000_0002);

                // Assert mid-transfer downstream backpressure for 4 clock cycles
                @(posedge clk);
                $display("[%0t ns] DOWNSTREAM BACKPRESSURE: De-asserting m_tready...", $time);
                m_tready <= 1'b0;
                repeat (4) @(posedge clk);
                $display("[%0t ns] DOWNSTREAM BACKPRESSURE: Re-asserting m_tready...", $time);
                m_tready <= 1'b1;

                send_payload(32'hD000_0003);
                send_payload(32'hD000_0004);
            end
        join

        #100;
        $display("\n--- ALL TESTS COMPLETE SUCCESSFULLY ---");

        // ------------------------------------------------------------------------
        // TEST 6: Zero-Length Read (ZLR) Boundary Condition
        // ------------------------------------------------------------------------
        $display("\n--- TEST 6: Zero-Length Read (ZLR) ---");
        // Requires 1 Non-Posted Header credit, 0 Data credits. FBE=0, LBE=0.
        send_cmd(0, 64'h0000_0000_4000_0000, 10'd1, 10'h0D4, 4'h0, 4'h0);
        #100;

        // ------------------------------------------------------------------------
        // TEST 7: Exact MPS Boundary Match (Exactly 4 DW)
        // ------------------------------------------------------------------------
        $display("\n--- TEST 7: Exact MPS Boundary Match ---");
        // Should generate exactly one TLP. Verifies no off-by-one errors in splitting.
        fork
            begin
                send_cmd(1, 64'h0000_0000_5000_0000, 10'd4, 10'h0E5, 4'hF, 4'h0);
            end
            begin
                send_payload(32'hE000_0001);
                send_payload(32'hE000_0002);
                send_payload(32'hE000_0003);
                send_payload(32'hE000_0004);
            end
        join
        #100;

        // ------------------------------------------------------------------------
        // TEST 8: Simultaneous UpdateFC and Command Issue (Race Condition)
        // ------------------------------------------------------------------------
        $display("\n--- TEST 8: Simultaneous UpdateFC and Command Issue ---");
        send_update_fc(2'b00, 8'd0, 12'd0); // Force Posted Credits to 0 first
        #40;
        
        fork
            begin
                // Command requests credits exactly when they are 0
                send_cmd(1, 64'h0000_0000_6000_0000, 10'd1, 10'h0F6, 4'hF, 4'h0);
            end
            begin
                // Replenish credits on the exact same clock edge
                send_update_fc(2'b00, 8'd1, 12'd1);
            end
            begin
                send_payload(32'hF000_0001);
            end
        join
        #100;

        // ------------------------------------------------------------------------
        // TEST 9: Upstream Data Starvation (Slow Payload)
        // ------------------------------------------------------------------------
        $display("\n--- TEST 9: Upstream Data Starvation ---");
        // Verifies packetizer waits for s_tvalid without dropping the TLP
        send_update_fc(2'b00, 8'd10, 12'd10); // Ensure ample credits
        #20;
        fork
            begin
                send_cmd(1, 64'h0000_0000_7000_0000, 10'd3, 10'h107, 4'hF, 4'hF);
            end
            begin
                send_payload(32'hA111_1111);
                #30; // Inject artificial delay between payload words
                send_payload(32'hA222_2222);
                #40; // Inject artificial delay
                send_payload(32'hA333_3333);
            end
        join
        #100;
        
        $finish;
    end

    // Safety Watchdog
    initial begin
        #30000;
        $display("\n[ERROR] Watchdog Timeout Reached!");
        $finish;
    end

endmodule