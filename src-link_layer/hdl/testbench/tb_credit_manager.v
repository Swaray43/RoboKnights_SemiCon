`timescale 1ns/1ps
// ============================================================
// tb_credit_manager -- S6 (rank 8)
//
// Revived against the current ports: update_fc_type (2-bit,
// selects Posted/Non-Posted/Completion) instead of separate
// hdr/data-limit buses per type, and alloc_req_type +
// alloc_req_dw_len instead of alloc_hdr_credits/alloc_data_credits.
// avail_hdr_credits/avail_data_credits no longer exist as ports;
// behaviour is checked purely through alloc_req_grant/stall.
//
// Checks grant/stall for each credit type (Posted, Non-Posted,
// Completion), credit exhaustion causing a stall, UpdateFC
// replenishment releasing the stall, and infinite-credit mode
// (0x7F header / 0xFFF data) always granting.
// ============================================================
module tb_credit_manager;

    reg        clk, rst_n;

    reg        update_fc_valid;
    reg  [1:0] update_fc_type;
    reg  [7:0] update_fc_hdr_limit;
    reg [11:0] update_fc_data_limit;

    reg        alloc_req_valid;
    reg  [1:0] alloc_req_type;
    reg  [9:0] alloc_req_dw_len;

    wire       alloc_req_grant;
    wire       stall;

    integer errors;

    credit_manager dut (
        .clk(clk), .rst_n(rst_n),
        .update_fc_valid(update_fc_valid),
        .update_fc_type(update_fc_type),
        .update_fc_hdr_limit(update_fc_hdr_limit),
        .update_fc_data_limit(update_fc_data_limit),
        .alloc_req_valid(alloc_req_valid),
        .alloc_req_type(alloc_req_type),
        .alloc_req_dw_len(alloc_req_dw_len),
        .alloc_req_grant(alloc_req_grant),
        .stall(stall)
    );

    always #5 clk = ~clk;

    task check_eq(input [31:0] got, input [31:0] exp, input [255:0] label);
        begin
            if (got !== exp) begin
                $display("FAIL %0s: got=%0d expected=%0d", label, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    task send_update_fc(input [1:0] fc_type, input [7:0] hdr_lim, input [11:0] data_lim);
        begin
            @(negedge clk);
            update_fc_valid      = 1;
            update_fc_type       = fc_type;
            update_fc_hdr_limit  = hdr_lim;
            update_fc_data_limit = data_lim;
            @(negedge clk);
            update_fc_valid = 0;
        end
    endtask

    // Issue one allocation request for exactly one cycle and sample
    // grant/stall at the posedge where alloc_req_valid is seen (read-
    // only there, so no race with the DUT's own clocked update).
    reg last_grant, last_stall;
    task do_alloc(input [1:0] req_type, input [9:0] dw_len);
        begin
            @(negedge clk);
            alloc_req_valid  = 1;
            alloc_req_type   = req_type;
            alloc_req_dw_len = dw_len;
            @(posedge clk);
            last_grant = alloc_req_grant;
            last_stall = stall;
            @(negedge clk);
            alloc_req_valid = 0;
        end
    endtask

    initial begin
        clk = 0; rst_n = 0;
        update_fc_valid = 0; update_fc_type = 0;
        update_fc_hdr_limit = 0; update_fc_data_limit = 0;
        alloc_req_valid = 0; alloc_req_type = 0; alloc_req_dw_len = 0;
        errors = 0;

        @(negedge clk); @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // ---- Test 1: no credits provisioned yet -> stall ----
        do_alloc(2'b00, 10'd4); // Posted, 4 DW
        check_eq(last_grant, 0, "T1: Posted request with 0 credits -> no grant");
        check_eq(last_stall, 1, "T1: Posted request with 0 credits -> stall");

        // ---- Test 2: provision Posted credits (4 hdr, 16 data) ----
        send_update_fc(2'b00, 8'd4, 12'd16);
        do_alloc(2'b00, 10'd8); // needs ceil(8/4)=2 data credits
        check_eq(last_grant, 1, "T2: Posted request within credits -> grant");
        check_eq(last_stall, 0, "T2: Posted request within credits -> no stall");

        // Second request consumes 2 more data credits (4 total consumed, 12 left)
        do_alloc(2'b00, 10'd8);
        check_eq(last_grant, 1, "T2b: second Posted request still within credits -> grant");

        // ---- Test 3: exhaust remaining Posted data credits ----
        // 12 data credits left (3 header credits left too); ask for 60 DW
        // (needs ceil(60/4)=15 data credits > 12 available) -> stall
        do_alloc(2'b00, 10'd60);
        check_eq(last_grant, 0, "T3: Posted request exceeding remaining data credits -> no grant");
        check_eq(last_stall, 1, "T3: Posted request exceeding remaining data credits -> stall");

        // ---- Test 4: Non-Posted pool is independent, still at 0 ----
        do_alloc(2'b01, 10'd0); // MRd, 0 DW (just a header credit)
        check_eq(last_grant, 0, "T4: Non-Posted request with 0 credits -> no grant");
        check_eq(last_stall, 1, "T4: Non-Posted request with 0 credits -> stall");

        send_update_fc(2'b01, 8'd2, 12'd0);
        do_alloc(2'b01, 10'd0);
        check_eq(last_grant, 1, "T4b: Non-Posted request after UpdateFC -> grant");

        // ---- Test 5: UpdateFC replenishment releases a stalled Posted request ----
        send_update_fc(2'b00, 8'd10, 12'd40); // raise Posted limits well above consumed
        do_alloc(2'b00, 10'd60); // same request that stalled in T3
        check_eq(last_grant, 1, "T5: Posted request granted after replenishment");
        check_eq(last_stall, 0, "T5: no stall after replenishment");

        // ---- Test 6: infinite credit mode (Completion pool) ----
        send_update_fc(2'b10, 8'h7F, 12'hFFF); // 0x7F header / 0xFFF data = infinite
        do_alloc(2'b10, 10'd1000); // huge request -- infinite credits must still grant
        check_eq(last_grant, 1, "T6: Completion request under infinite credits -> grant");
        check_eq(last_stall, 0, "T6: infinite credits never stall");

        // A second huge request right after should still grant (infinite
        // credits never get consumed down).
        do_alloc(2'b10, 10'd1000);
        check_eq(last_grant, 1, "T6b: infinite credits still grant on repeated huge requests");

        if (errors == 0) $display("RESULT: PASS");
        else              $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #20000;
        $display("[ERROR] Watchdog timeout");
        $display("RESULT: FAIL (watchdog)");
        $finish;
    end

endmodule