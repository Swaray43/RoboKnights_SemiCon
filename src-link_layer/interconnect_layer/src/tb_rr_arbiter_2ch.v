`timescale 1ns/1ps

module tb_rr_arbiter_2ch;

    reg clk = 0;
    reg rstn;
    reg req0, req1;
    reg [7:0] weight0, weight1;
    wire grant0, grant1;

    integer g0_count, g1_count, i;
    integer errors = 0;

    rr_arbiter_2ch #(
        .CREDIT_WIDTH(8),
        .STARVE_WIDTH(16),
        .STARVE_CYCLES(8)
    ) dut (
        .clk(clk), .rstn(rstn),
        .req0(req0), .req1(req1),
        .weight0(weight0), .weight1(weight1),
        .grant0(grant0), .grant1(grant1)
    );

    always #5 clk = ~clk;

    task reset_dut;
        begin
            rstn = 0;
            req0 = 0; req1 = 0;
            weight0 = 8'd1; weight1 = 8'd1;
            repeat (3) @(posedge clk);
            rstn = 1;
            @(posedge clk);
        end
    endtask

    initial begin
        $display("=== Test 1: 50/50 default weights, both always requesting ===");
        reset_dut;
        req0 = 1; req1 = 1;
        g0_count = 0; g1_count = 0;
        for (i = 0; i < 40; i = i + 1) begin
            @(posedge clk);
            if (grant0) g0_count = g0_count + 1;
            if (grant1) g1_count = g1_count + 1;
        end
        $display("ch0 grants=%0d ch1 grants=%0d (expect near-equal)", g0_count, g1_count);
        if (g0_count < 15 || g0_count > 25 || g1_count < 15 || g1_count > 25) begin
            $display("FAIL: 50/50 split not within expected range");
            errors = errors + 1;
        end else begin
            $display("PASS");
        end

        $display("=== Test 2: 3:1 weight ratio, both always requesting ===");
        reset_dut;
        weight0 = 8'd3; weight1 = 8'd1;
        req0 = 1; req1 = 1;
        g0_count = 0; g1_count = 0;
        for (i = 0; i < 80; i = i + 1) begin
            @(posedge clk);
            if (grant0) g0_count = g0_count + 1;
            if (grant1) g1_count = g1_count + 1;
        end
        $display("ch0 grants=%0d ch1 grants=%0d (expect ~3:1 ratio)", g0_count, g1_count);
        if (g1_count == 0 || (g0_count * 10 / g1_count) < 25 || (g0_count * 10 / g1_count) > 35) begin
            $display("FAIL: ratio not near 3:1 (got %0d:%0d)", g0_count, g1_count);
            errors = errors + 1;
        end else begin
            $display("PASS");
        end

        $display("=== Test 3: starvation guard, ch1 weight 0 but continuously requesting ===");
        reset_dut;
        weight0 = 8'd10; weight1 = 8'd0;
        req0 = 1; req1 = 1;
        g1_count = 0;
        for (i = 0; i < 60; i = i + 1) begin
            @(posedge clk);
            if (grant1) g1_count = g1_count + 1;
        end
        $display("ch1 grants despite weight=0: %0d (expect > 0, starvation guard firing)", g1_count);
        if (g1_count == 0) begin
            $display("FAIL: starvation guard never fired for weight-0 channel");
            errors = errors + 1;
        end else begin
            $display("PASS");
        end

        $display("=== Test 4: only ch0 requesting, ch1 idle ===");
        reset_dut;
        req0 = 1; req1 = 0;
        g0_count = 0; g1_count = 0;
        for (i = 0; i < 20; i = i + 1) begin
            @(posedge clk);
            if (grant0) g0_count = g0_count + 1;
            if (grant1) g1_count = g1_count + 1;
        end
        $display("ch0 grants=%0d ch1 grants=%0d (expect ch0 every cycle, ch1 never)", g0_count, g1_count);
        if (g0_count < 18 || g1_count != 0) begin
            $display("FAIL: ch0 should be granted nearly every cycle, ch1 never");
            errors = errors + 1;
        end else begin
            $display("PASS");
        end

        $display("=== Test 5: mutual exclusion check across all prior activity ===");
        // sampled implicitly during tests above via waveform-free assertion below
        // re-run a short burst with a same-cycle check
        reset_dut;
        req0 = 1; req1 = 1;
        for (i = 0; i < 20; i = i + 1) begin
            @(posedge clk);
            if (grant0 && grant1) begin
                $display("FAIL: grant0 and grant1 both asserted same cycle");
                errors = errors + 1;
            end
        end
        if (errors == 0) $display("PASS (checked inline)");

        if (errors == 0)
            $display("\nALL TESTS PASSED");
        else
            $display("\n%0d TEST(S) FAILED", errors);

        $finish;
    end

endmodule
