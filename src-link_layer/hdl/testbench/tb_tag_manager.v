`timescale 1ns/1ps
// ============================================================
// tb_tag_manager -- S9 (rank 12)
//
// Built and ran before, but only printed -- nothing failed
// automatically. Added real checks and a RESULT: PASS/FAIL line:
//   - first allocation returns tag 0 (deterministic free-pool order)
//   - pool exhaustion: exactly N tags allocated, all distinct,
//     before s_tready drops
//   - timeout -> UR: ur_valid pulses for one of the outstanding tags
//   - posted (MWr) requests never pull a tag (m_tag_valid stays low)
// ============================================================
module tb_tag_manager;

    // Scaled down parameters for simulation speed
    localparam N = 4;
    localparam TIMEOUT = 20;

    reg clk;
    reg rst_n;

    reg s_valid;
    reg s_is_non_posted;
    wire s_tready;
    wire [9:0] m_tag;
    wire m_tag_valid;

    reg cpl_valid;
    reg [9:0] cpl_tag;

    wire ur_valid;
    wire [9:0] ur_tag;

    integer errors;

    tag_manager #(
        .N(N),
        .TIMEOUT_CYCLES(TIMEOUT)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .s_valid(s_valid),
        .s_is_non_posted(s_is_non_posted),
        .s_tready(s_tready),
        .m_tag(m_tag),
        .m_tag_valid(m_tag_valid),
        .cpl_valid(cpl_valid),
        .cpl_tag(cpl_tag),
        .ur_valid(ur_valid),
        .ur_tag(ur_tag)
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

    integer alloc_cnt;
    integer ur_cnt;
    integer posted_tag_valid_cnt;
    reg [9:0] first_tag;
    reg       first_tag_seen;
    reg       seen [0:N-1];
    integer   distinct_cnt;
    integer   i;
    reg [9:0] last_ur_tag;

    always @(posedge clk) begin
        if (m_tag_valid) begin
            alloc_cnt = alloc_cnt + 1;
            if (!first_tag_seen) begin
                first_tag = m_tag;
                first_tag_seen = 1'b1;
            end
            if (m_tag < N && !seen[m_tag]) begin
                seen[m_tag] = 1'b1;
                distinct_cnt = distinct_cnt + 1;
            end
        end
        if (ur_valid) begin
            ur_cnt = ur_cnt + 1;
            last_ur_tag = ur_tag;
        end
    end

    initial begin
        clk = 0;
        rst_n = 0;
        s_valid = 0;
        s_is_non_posted = 0;
        cpl_valid = 0;
        cpl_tag = 0;
        errors = 0;
        alloc_cnt = 0; ur_cnt = 0; posted_tag_valid_cnt = 0;
        first_tag_seen = 0; distinct_cnt = 0; first_tag = 0; last_ur_tag = 0;
        for (i = 0; i < N; i = i + 1) seen[i] = 1'b0;

        #20;
        rst_n = 1;
        #20;

        // -------- Test 1: basic allocation & completion --------
        // Inputs change at negedge so they never race the DUT's own
        // posedge-triggered always block.
        @(negedge clk);
        s_valid = 1;
        s_is_non_posted = 1;
        @(negedge clk);
        s_valid = 0;

        #30;

        check_eq(first_tag_seen, 1, "T1: an allocation happened");
        check_eq(first_tag, 0, "T1: first allocated tag is 0 (deterministic free-pool order)");

        @(negedge clk);
        cpl_valid = 1;
        cpl_tag = first_tag; // complete it, returning it to the pool
        @(negedge clk);
        cpl_valid = 0;

        // -------- Test 2: pool exhaustion --------
        #20;
        alloc_cnt = 0;
        for (i = 0; i < N; i = i + 1) seen[i] = 1'b0;
        distinct_cnt = 0;

        @(negedge clk);
        s_valid = 1;
        s_is_non_posted = 1;
        @(posedge clk);
        while (s_tready == 1) @(posedge clk);
        @(negedge clk);
        s_valid = 0;

        check_eq(alloc_cnt, N, "T2: exactly N tags allocated before pool exhausts");
        check_eq(distinct_cnt, N, "T2: all N allocated tags are distinct");
        check_eq(s_tready, 0, "T2: s_tready low once the pool is empty");

        // -------- Test 3: timeout & UR generation --------
        // Leave the N allocated tags uncompleted; wait past TIMEOUT.
        ur_cnt = 0;
        #(TIMEOUT * 10 + 100);

        check_eq(ur_cnt >= 1, 1, "T3: at least one timeout UR fired for an outstanding tag");
        check_eq(last_ur_tag < N, 1, "T3: UR tag is one of the allocated tags");

        // -------- Test 4: posted request bypass (no tag pulled) --------
        posted_tag_valid_cnt = 0;
        @(negedge clk);
        s_valid = 1;
        s_is_non_posted = 0; // MWr -- posted, no tag needed
        @(posedge clk);
        if (m_tag_valid) posted_tag_valid_cnt = posted_tag_valid_cnt + 1;
        @(negedge clk);
        s_valid = 0;
        @(posedge clk);
        if (m_tag_valid) posted_tag_valid_cnt = posted_tag_valid_cnt + 1;

        check_eq(posted_tag_valid_cnt, 0, "T4: posted (MWr) request never asserts m_tag_valid");

        #50;
        if (errors == 0) $display("RESULT: PASS");
        else              $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    // Event Monitor (kept from the original bench, for human eyes)
    always @(posedge clk) begin
        if (m_tag_valid) $display("[%0t] ALLOCATED: Tag %0d", $time, m_tag);
        if (cpl_valid)   $display("[%0t] COMPLETED: Tag %0d", $time, cpl_tag);
        if (ur_valid)    $display("[%0t] TIMEOUT UR: Tag %0d forcefully freed", $time, ur_tag);
    end

    initial begin
        #20000;
        $display("[ERROR] Watchdog timeout");
        $display("RESULT: FAIL (watchdog)");
        $finish;
    end

endmodule