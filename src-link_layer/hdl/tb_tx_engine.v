`timescale 1ns/1ps

module tb_tx_engine;

    // Global Test Signals
    reg        clk;
    reg        rst_n;

    // Posted Interface
    reg        p_req_valid;
    wire       p_req_ready;
    reg [63:0] p_req_addr;
    reg [9:0]  p_req_len;
    reg [9:0]  p_req_tag;
    reg [3:0]  p_req_fbe;
    reg [3:0]  p_req_lbe;
    reg [31:0] p_s_tdata;
    reg        p_s_tvalid;
    wire       p_s_tready;

    // Non-Posted Interface
    reg        np_req_valid;
    wire       np_req_ready;
    reg [63:0] np_req_addr;
    reg [9:0]  np_req_len;
    reg [9:0]  np_req_tag;
    reg [3:0]  np_req_fbe;
    reg [3:0]  np_req_lbe;

    // UpdateFC Interface
    reg        update_fc_valid;
    reg [1:0]  update_fc_type;
    reg [7:0]  update_fc_hdr_limit;
    reg [11:0] update_fc_data_limit;

    // Outbound TLP Stream
    wire [31:0] m_tdata;
    wire        m_tvalid;
    reg         m_tready;
    wire        m_tlast;

    // Instantiate DUT
    tx_engine #(
        .MAX_PAYLOAD_SIZE_DW(4),
        .REQ_ID(16'h0200)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .p_req_valid(p_req_valid),
        .p_req_ready(p_req_ready),
        .p_req_addr(p_req_addr),
        .p_req_len(p_req_len),
        .p_req_tag(p_req_tag),
        .p_req_fbe(p_req_fbe),
        .p_req_lbe(p_req_lbe),
        .p_s_tdata(p_s_tdata),
        .p_s_tvalid(p_s_tvalid),
        .p_s_tready(p_s_tready),
        .np_req_valid(np_req_valid),
        .np_req_ready(np_req_ready),
        .np_req_addr(np_req_addr),
        .np_req_len(np_req_len),
        .np_req_tag(np_req_tag),
        .np_req_fbe(np_req_fbe),
        .np_req_lbe(np_req_lbe),
        .update_fc_valid(update_fc_valid),
        .update_fc_type(update_fc_type),
        .update_fc_hdr_limit(update_fc_hdr_limit),
        .update_fc_data_limit(update_fc_data_limit),
        .m_tdata(m_tdata),
        .m_tvalid(m_tvalid),
        .m_tready(m_tready),
        .m_tlast(m_tlast)
    );

    // Clock Generation (10ns Period / 100MHz)
    always #5 clk = ~clk;

    // Task for Credit Updates
    task send_update_fc(input [1:0] fc_type, input [7:0] hdr_lim, input [11:0] data_lim);
        begin
            @(posedge clk);
            update_fc_valid      <= 1'b1;
            update_fc_type       <= fc_type;
            update_fc_hdr_limit  <= hdr_lim;
            update_fc_data_limit <= data_lim;
            $display("[%10.3f ns] DLLP -> UpdateFC: Type=%0d, HdrLimit=0x%02h, DataLimit=0x%03h",
                     $time, fc_type, hdr_lim, data_lim);
            @(posedge clk);
            update_fc_valid      <= 1'b0;
        end
    endtask

    // Handshake Monitoring & Valid Clearing
    always @(posedge clk) begin
        if (p_req_ready)  p_req_valid  <= 1'b0;
        if (np_req_ready) np_req_valid <= 1'b0;
    end

    // Stream TLP Data Logger
    always @(posedge clk) begin
        if (m_tvalid && m_tready) begin
            $display("[%10.3f ns] TX TLP OUT: Data=0x%08h, TLAST=%0b", $time, m_tdata, m_tlast);
        end
    end

    // Payload Stream Generator for Posted Writes
    reg [1:0] p_word_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_s_tvalid <= 1'b0;
            p_s_tdata  <= 32'h0;
            p_word_cnt <= 2'd0;
        end else if (p_req_valid) begin
            p_s_tvalid <= 1'b1;
            if (p_s_tready && m_tready) begin
                p_word_cnt <= p_word_cnt + 1'b1;
                if (p_word_cnt == 2'd0)      p_s_tdata <= 32'h1111_2222;
                else if (p_word_cnt == 2'd1) p_s_tdata <= 32'h3333_4444;
                else                         p_s_tdata <= 32'h5555_6666;
            end
        end else begin
            p_s_tvalid <= 1'b0;
            p_word_cnt <= 2'd0;
            p_s_tdata  <= 32'h1111_2222;
        end
    end

    // Watchdog Timer (10us)
    initial begin
        #10000;
        $display("\n[ERROR] Watchdog Timeout Reached!");
        $finish;
    end

    // Main Test Stimulus
    initial begin
        clk = 0;
        rst_n = 0;
        m_tready = 1;
        p_req_valid = 0;
        np_req_valid = 0;
        update_fc_valid = 0;
        update_fc_type = 0;
        update_fc_hdr_limit = 0;
        update_fc_data_limit = 0;
        p_req_addr = 0;
        p_req_len = 0;
        p_req_tag = 0;
        p_req_fbe = 0;
        p_req_lbe = 0;
        np_req_addr = 0;
        np_req_len = 0;
        np_req_tag = 0;
        np_req_fbe = 0;
        np_req_lbe = 0;

        #20;
        rst_n = 1;
        #25; // Align to 45ns

        $display("\n--- TEST 1: Rule 1 - Posted Requests CAN Bypass Stalled Non-Posted Requests ---");
        // Grant Posted credits: HdrLimit = 5, DataLimit = 10 credits (40 DWs)
        send_update_fc(2'b00, 8'h05, 12'h00A);

        // Zero Non-Posted credits
        send_update_fc(2'b01, 8'h00, 12'h000);

        // 1. Issue Non-Posted Read first (32-bit addr, 1 DW, Extended Tag 0x2AA)
        @(posedge clk);
        np_req_valid <= 1'b1;
        np_req_addr  <= 64'h0000_0000_1000_0000;
        np_req_len   <= 10'd1;
        np_req_tag   <= 10'h2AA;
        np_req_fbe   <= 4'hF;
        np_req_lbe   <= 4'h0;

        // 2. Issue Posted Write second (32-bit addr, 2 DWs, Extended Tag 0x1BB)
        @(posedge clk);
        p_req_valid <= 1'b1;
        p_req_addr  <= 64'h0000_0000_2000_0000;
        p_req_len   <= 10'd2;
        p_req_tag   <= 10'h1BB;
        p_req_fbe   <= 4'hF;
        p_req_lbe   <= 4'hF;

        // Wait for Posted Write to pass NP Read and complete
        wait(p_req_ready);
        $display("[%10.3f ns] Posted Write Passed NP and Completed!", $time);

        #50;
        $display("\n--- Replenishing Non-Posted credits to allow NP Read to finish ---");
        send_update_fc(2'b01, 8'h05, 12'h000);

        wait(np_req_ready);
        $display("[%10.3f ns] NP Read Completed Transmission!", $time);

        #100;
        $display("\n--- TEST 2: Rule 2 - Non-Posted Requests MUST NOT Bypass Stalled Posted Requests ---");
        // Clear Posted Credits (HdrLimit matches consumed = 1)
        send_update_fc(2'b00, 8'h01, 12'h001);

        // Grant Non-Posted Credits
        send_update_fc(2'b01, 8'h05, 12'h000);

        // 1. Issue Posted Write first (64-bit addr -> 4DW header, 1 DW, Tag 0x0CC)
        @(posedge clk);
        p_req_valid <= 1'b1;
        p_req_addr  <= 64'h0000_0001_3000_0000;
        p_req_len   <= 10'd1;
        p_req_tag   <= 10'h0CC;
        p_req_fbe   <= 4'hF;
        p_req_lbe   <= 4'h0;

        // 2. Issue Non-Posted Read second (32-bit addr -> 3DW header, 1 DW, Tag 0x0DD)
        @(posedge clk);
        np_req_valid <= 1'b1;
        np_req_addr  <= 64'h0000_0000_4000_0000;
        np_req_len   <= 10'd1;
        np_req_tag   <= 10'h0DD;
        np_req_fbe   <= 4'hF;
        np_req_lbe   <= 4'h0;

        #100;
        $display("\n--- Replenishing Posted credits: Posted finishes first, then NP ---");
        send_update_fc(2'b00, 8'h05, 12'h00A);

        wait(p_req_ready);
        $display("[%10.3f ns] Posted Write Completed!", $time);

        wait(np_req_ready);
        $display("[%10.3f ns] NP Read Completed!", $time);

        #100;
        $display("\n--- TX ENGINE ORDERING VERIFICATION COMPLETE ---");
        $finish;
    end

endmodule