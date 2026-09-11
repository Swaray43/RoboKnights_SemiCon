`timescale 1ns/1ps

module tb_credit_manager;

    reg        clk;
    reg        rst_n;

    // DLLP Interface
    reg        update_fc_valid;
    reg  [7:0] update_fc_hdr_limit;
    reg [11:0] update_fc_data_limit;

    // Allocation Request Interface
    reg        alloc_req_valid;
    reg  [7:0] alloc_hdr_credits;
    reg [11:0] alloc_data_credits;

    // DUT Outputs
    wire        alloc_req_grant;
    wire        stall;
    wire  [7:0] avail_hdr_credits;
    wire [11:0] avail_data_credits;

    // DUT Instantiation
    credit_manager dut (
        .clk(clk),
        .rst_n(rst_n),
        .update_fc_valid(update_fc_valid),
        .update_fc_hdr_limit(update_fc_hdr_limit),
        .update_fc_data_limit(update_fc_data_limit),
        .alloc_req_valid(alloc_req_valid),
        .alloc_hdr_credits(alloc_hdr_credits),
        .alloc_data_credits(alloc_data_credits),
        .alloc_req_grant(alloc_req_grant),
        .stall(stall),
        .avail_hdr_credits(avail_hdr_credits),
        .avail_data_credits(avail_data_credits)
    );

    // Clock Generation (100 MHz -> 10ns)
    initial begin
        clk = 0;
        forever #5 clk = ~clk;
    end

    // Output Monitor
    always @(posedge clk) begin
        if (alloc_req_valid) begin
            $display("[%0t ns] REQ: H=%0d, D=%0d | AVAIL: H=%0d, D=%0d | GRANT=%b | STALL=%b",
                     $time, alloc_hdr_credits, alloc_data_credits,
                     avail_hdr_credits, avail_data_credits,
                     alloc_req_grant, stall);
        end
    end

    // Task: Issue UpdateFC DLLP
    task send_update_fc;
        input [7:0]  hdr_lim;
        input [11:0] data_lim;
        begin
            @(posedge clk);
            update_fc_valid      <= 1'b1;
            update_fc_hdr_limit  <= hdr_lim;
            update_fc_data_limit <= data_lim;
            $display("[%0t ns] DLLP -> UpdateFC: HdrLimit=%0d, DataLimit=%0d",
                     $time, hdr_lim, data_lim);
            @(posedge clk);
            update_fc_valid      <= 1'b0;
        end
    endtask

    // Test Sequence
    initial begin
        // Reset and Initial Conditions
        rst_n                = 0;
        update_fc_valid      = 0;
        update_fc_hdr_limit  = 0;
        update_fc_data_limit = 0;
        alloc_req_valid      = 0;
        alloc_hdr_credits    = 0;
        alloc_data_credits   = 0;

        #20 rst_n = 1;
        #20;

        $display("\n--- TEST 1: Initial Flow Control Initialization ---");
        send_update_fc(8'd4, 12'd16);
        #20;

        $display("\n--- TEST 2: Valid Transactions Within Credit Limits ---");
        // Transaction 1: 1 Header, 8 Data Credits
        @(posedge clk);
        alloc_req_valid    <= 1'b1;
        alloc_hdr_credits  <= 8'd1;
        alloc_data_credits <= 12'd8;
        @(posedge clk);
        alloc_req_valid    <= 1'b0;
        #20;

        // Transaction 2: 1 Header, 8 Data Credits (Exhausts Data Credits)
        @(posedge clk);
        alloc_req_valid    <= 1'b1;
        alloc_hdr_credits  <= 8'd1;
        alloc_data_credits <= 12'd8;
        @(posedge clk);
        alloc_req_valid    <= 1'b0;
        #20;

        $display("\n--- TEST 3: Credit Exhaustion & Stall Verification ---");
        // Transaction 3: Needs 4 Data Credits, 0 Available -> Stall Asserted
        @(posedge clk);
        alloc_req_valid    <= 1'b1;
        alloc_hdr_credits  <= 8'd1;
        alloc_data_credits <= 12'd4;
        
        repeat (3) @(posedge clk);

        $display("\n--- TEST 4: Synthetic UpdateFC Release Verification ---");
        // Synthetic UpdateFC expands Data Limit to 32 credits
        send_update_fc(8'd4, 12'd32);

        // Keep transaction requested and wait for stall release and grant
        @(posedge clk);
        alloc_req_valid <= 1'b1;
        wait(alloc_req_grant == 1'b1);
        @(posedge clk);
        alloc_req_valid <= 1'b0;
        #20;

        $display("\n--- ALL TESTS COMPLETE ---");
        $finish;
    end

    // Safety Watchdog Timer
    initial begin
        #5000;
        $display("\n[ERROR] Watchdog Timeout Reached!");
        $finish;
    end

endmodule