`timescale 1ns/1ps
module tb_session_arm_ctrl;

    reg clk = 0;
    reg rst_n;
    reg [63:0] base_addr_in;
    reg [2:0]  tc_in;
    reg        session_arm;
    reg        session_end;
    wire [63:0] base_addr_latched;
    wire [2:0]  tc_latched;
    wire        session_valid;
    wire [7:0]  arm_while_active_cnt;

    integer errors = 0;

    session_arm_ctrl #(.ADDR_WIDTH(64), .TC_WIDTH(3), .ARM_CNT_WIDTH(8)) dut (
        .clk(clk), .rst_n(rst_n),
        .base_addr_in(base_addr_in), .tc_in(tc_in),
        .session_arm(session_arm), .session_end(session_end),
        .base_addr_latched(base_addr_latched), .tc_latched(tc_latched),
        .session_valid(session_valid),
        .session_arm_while_active_cnt(arm_while_active_cnt)
    );

    always #5 clk = ~clk;

    // Sample/settle point: wait for the edge, THEN let NBA updates land
    // before any blocking read or stimulus decision. This is the fix --
    // without the #1, testbench reads race the DUT's non-blocking
    // updates at the same time step.
    task tick;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task check(input cond, input [255:0] msg);
        begin
            if (!cond) begin
                $display("FAIL: %s @ t=%0t", msg, $time);
                errors = errors + 1;
            end else begin
                $display("PASS: %s", msg);
            end
        end
    endtask

    initial begin
        rst_n = 0; base_addr_in = 0; tc_in = 0; session_arm = 0; session_end = 0;
        tick; tick;
        rst_n = 1;
        tick;
        check(session_valid == 0, "reset: session_valid low");

        // ---- Case 1: normal arm -> LATCH -> ACTIVE ----
        base_addr_in = 64'hA000_0000;
        tc_in        = 3'b010;
        session_arm  = 1;
        tick;                    // IDLE -> LATCH lands
        session_arm = 0;
        tick;                    // LATCH -> ACTIVE lands, base_addr/tc latch, session_valid rises
        check(session_valid == 1, "case1: session_valid asserted after arm");
        check(base_addr_latched == 64'hA000_0000, "case1: base_addr latched correctly");
        check(tc_latched == 3'b010, "case1: tc latched correctly");

        // ---- Case 2 (THE BUG FIX): re-arm while ACTIVE must be rejected ----
        base_addr_in = 64'hDEAD_BEEF;
        session_arm  = 1;
        tick;
        session_arm = 0;
        tick;
        check(session_valid == 1, "case2: session_valid stays high through rejected re-arm");
        check(base_addr_latched == 64'hA000_0000, "case2: base_addr UNCHANGED (re-arm ignored)");
        check(arm_while_active_cnt == 8'd1, "case2: arm_while_active_cnt incremented exactly once");

        // ---- Case 3: legitimate session_end, then verify re-arm now works ----
        session_end = 1;
        tick;
        session_end = 0;
        tick;
        check(session_valid == 0, "case3: session_valid drops after session_end");

        base_addr_in = 64'hDEAD_BEEF;
        session_arm  = 1;
        tick;
        session_arm = 0;
        tick;
        check(session_valid == 1, "case3: new session_valid after clean re-arm");
        check(base_addr_latched == 64'hDEAD_BEEF, "case3: new base_addr latched after clean re-arm");
        check(arm_while_active_cnt == 8'd1, "case3: counter unchanged by legitimate re-arm");

        // ---- Case 4: multiple rejected re-arms during one ACTIVE window ----
        session_arm = 1; tick; session_arm = 0; tick;
        session_arm = 1; tick; session_arm = 0; tick;
        check(arm_while_active_cnt == 8'd3, "case4: counter accumulates across multiple rejected re-arms");
        check(base_addr_latched == 64'hDEAD_BEEF, "case4: base_addr still unchanged by rejected re-arms");

        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== %0d TEST(S) FAILED ===", errors);

        $stop;
    end

endmodule
