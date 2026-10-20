`timescale 1ns/1ps
module tb_pcie_tag_manager;

    localparam TAG_WIDTH       = 5;
    localparam MAX_OUTSTANDING = 8;

    reg clk = 0;
    reg rst_n;

    reg tag_alloc_req;
    wire tag_alloc_ready;
    wire [TAG_WIDTH-1:0] tag_alloc_val;

    reg tag_free_req;
    reg [TAG_WIDTH-1:0] tag_free_val;

    integer errors = 0;

    pcie_tag_manager #(
        .TAG_WIDTH(TAG_WIDTH), .MAX_OUTSTANDING(MAX_OUTSTANDING)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .tag_alloc_req(tag_alloc_req), .tag_alloc_ready(tag_alloc_ready), .tag_alloc_val(tag_alloc_val),
        .tag_free_req(tag_free_req), .tag_free_val(tag_free_val)
    );

    always #5 clk = ~clk;

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

    // Perform one allocation: assert req, wait for the granting edge,
    // capture the granted tag, deassert. Checks belong AFTER this task
    // returns, since tag_alloc_val only becomes valid post-edge.
    task do_alloc(output [TAG_WIDTH-1:0] granted_tag);
        begin
            tag_alloc_req = 1;
            tick;                    // granting edge happens here
            granted_tag = tag_alloc_val;
            tag_alloc_req = 0;
            tick;                    // settle
        end
    endtask

    integer i;
    reg [TAG_WIDTH-1:0] got_tag;

    initial begin
        rst_n = 0; tag_alloc_req = 0; tag_free_req = 0; tag_free_val = 0;
        tick; tick;
        rst_n = 1;
        tick;

        // ---- Case 1: pool starts fully free ----
        check(tag_alloc_ready == 1, "case1: alloc_ready high with full free pool");

        // ---- Case 2: allocate in order, expect lowest-free-tag priority ----
        for (i = 0; i < MAX_OUTSTANDING; i = i + 1) begin
            check(tag_alloc_ready == 1, "case2: alloc_ready high before exhaustion");
            do_alloc(got_tag);
            check(got_tag == i[TAG_WIDTH-1:0], "case2: lowest-free-tag priority allocates in order");
        end

        // ---- Case 3: pool exhausted ----
        check(tag_alloc_ready == 0, "case3: alloc_ready low once all tags allocated");

        // ---- Case 4: freeing a middle tag makes exactly it available ----
        tag_free_req = 1;
        tag_free_val = 5'd3;
        tick;
        tag_free_req = 0;
        tick;
        check(tag_alloc_ready == 1, "case4: alloc_ready rises after a single tag is freed");
        do_alloc(got_tag);
        check(got_tag == 5'd3, "case4: freed tag (3) is the one re-allocated");
        check(tag_alloc_ready == 0, "case4: pool exhausted again after re-consuming the freed tag");

        // ---- Case 5: free a different tag, confirm it's the one granted next ----
        tag_free_req = 1;
        tag_free_val = 5'd7;
        tick;
        tag_free_req = 0;
        tick;
        check(tag_alloc_ready == 1, "case5: alloc_ready rises after freeing tag 7");
        do_alloc(got_tag);
        check(got_tag == 5'd7, "case5: newly freed tag 7 available for allocation");

        // ---- Case 6: same-cycle alloc+free of the SAME tag --
        //              freed tag not reusable until the cycle after the free ----
        check(tag_alloc_ready == 0, "case6 setup: pool exhausted before same-cycle alloc+free test");
        tag_free_req  = 1;
        tag_free_val  = 5'd0;
        tag_alloc_req = 1;
        check(tag_alloc_ready == 0, "case6: alloc_ready still 0 THIS cycle despite simultaneous free");
        tick;                         // free lands; this cycle's alloc attempt used PRE-free free_mask, so no grant
        tag_free_req  = 0;
        // tag_alloc_req is still 1 here -- now that free_mask reflects the freed
        // tag, THIS cycle's request should be granted
        check(tag_alloc_ready == 1, "case6: alloc_ready rises the cycle after the free");
        tick;                         // granting edge for the still-asserted request
        check(tag_alloc_val == 5'd0, "case6: tag 0 is granted on the first opportunity after being freed");
        tag_alloc_req = 0;
        tick;

        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== %0d TEST(S) FAILED ===", errors);

        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
    end

endmodule
