`timescale 1ns/1ps
module tb_rst_sync;

    reg clk = 0;
    reg async_rst_n;
    wire sync_rst_n;

    integer errors = 0;

    rst_sync dut (
        .clk(clk),
        .async_rst_n(async_rst_n),
        .sync_rst_n(sync_rst_n)
    );

    always #5 clk = ~clk;

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
        async_rst_n = 1;
        @(posedge clk); #1;
        @(posedge clk); #1;

        // ---- Case 1: async assert takes effect immediately, NOT waiting for a clock edge ----
        // Deassert clk-independence: drop async_rst_n mid-cycle, between edges,
        // and check sync_rst_n reacts within delta-time, not on the next posedge.
        #2;                       // land somewhere mid-cycle, away from any edge
        async_rst_n = 0;
        #1;                       // small settle, well before next posedge
        check(sync_rst_n == 0, "case1: sync_rst_n drops immediately on async assert, no clock edge needed");

        // ---- Case 2: deassertion is synchronized -- takes exactly 2 clock edges ----
        // Re-assert async_rst_n (i.e. release reset) at a known point, then count edges.
        @(negedge clk);           // release reset away from a posedge, to make edge-counting unambiguous
        async_rst_n = 1;

        @(posedge clk); #1;
        check(sync_rst_n == 0, "case2: sync_rst_n STILL low after 1st posedge post-release (stage1 only)");

        @(posedge clk); #1;
        check(sync_rst_n == 1, "case2: sync_rst_n goes high after 2nd posedge post-release (stage2 lands)");

        // ---- Case 3: once high, stays high with no further async_rst_n changes ----
        @(posedge clk); #1;
        check(sync_rst_n == 1, "case3: sync_rst_n remains high on subsequent clean cycles");

        // ---- Case 4: re-assert mid-run, confirm immediate drop again (not a one-shot behavior) ----
        @(negedge clk);
        async_rst_n = 0;
        #1;
        check(sync_rst_n == 0, "case4: re-assertion later in sim still drops sync_rst_n immediately");

        // ---- Case 5: minimum hold -- confirm sync_rst_n takes >=2 registered
        //              clock edges to go high after release, regardless of the
        //              phase at which async_rst_n happens to be released ----
        @(negedge clk);
        async_rst_n = 1;   // release
        begin : edge_count_block
            integer edge_count;
            edge_count = 0;
            while (sync_rst_n == 0) begin
                @(posedge clk); #1;
                edge_count = edge_count + 1;   // count every edge waited on, unconditionally
            end
            check(edge_count >= 2, "case5: sync_rst_n takes >=2 clock edges to deassert after release");
        end

        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== %0d TEST(S) FAILED ===", errors);

        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
    end

endmodule
