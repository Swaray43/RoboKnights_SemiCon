// ============================================================
// Testbench : ar_request_splitter_tb
//
// Verifies that for every descriptor driven in, the correct
// descriptor(s) come out:
//   - Non-crossing MRd : one output, identical fields
//   - MWr              : one output, identical fields, no split
//   - Crossing MRd     : two outputs, sub0+sub1 fields correct,
//                        both have split_flag=1
//   - Data in = Data out check on all fields
//
// Tests:
//   1. MWr — any address, any length — must pass through
//   2. MRd — no boundary crossing — must pass through
//   3. MRd — crosses 4KB boundary — must split
//   4. MRd — exactly at 4KB boundary, no crossing — must pass
//   5. MRd — crosses by exactly 1 DW — minimal split
//   6. MRd — crosses by large amount — large split
//   7. Back-pressure — m_ready held low mid-split — verify
//      no data lost or corrupted
//   8. Multiple descriptors back to back — verify ordering
// ============================================================

`timescale 1ns/1ps

module ar_request_splitter_tb;

// ============================================================
// Parameters matching DUT
// ============================================================
localparam ADDR_W  = 64;
localparam LEN_W   = 10;
localparam TC_W    = 3;
localparam ATTR_W  = 3;
localparam RTYPE_W = 2;

localparam RTYPE_MWR = 2'b00;
localparam RTYPE_MRD = 2'b01;

// ============================================================
// Clock and reset
// ============================================================
reg clk;
reg rst_n;

initial clk = 0;
always #5 clk = ~clk;  // 100 MHz, 10ns period

// ============================================================
// DUT ports
// ============================================================
reg  [ADDR_W-1:0]   s_addr;
reg  [LEN_W-1:0]    s_len_dw;
reg  [TC_W-1:0]     s_tc;
reg  [ATTR_W-1:0]   s_attr;
reg  [RTYPE_W-1:0]  s_reqtype;
reg                 s_valid;
wire                s_ready;

wire [ADDR_W-1:0]   m_addr;
wire [LEN_W-1:0]    m_len_dw;
wire [TC_W-1:0]     m_tc;
wire [ATTR_W-1:0]   m_attr;
wire [RTYPE_W-1:0]  m_reqtype;
wire                m_split_flag;
wire [1:0]          m_sub_index;
wire                m_valid;
reg                 m_ready;

wire                stat_split_occurred;

// ============================================================
// DUT instantiation
// ============================================================
ar_request_splitter #(
    .ADDR_W  (ADDR_W),
    .LEN_W   (LEN_W),
    .TC_W    (TC_W),
    .ATTR_W  (ATTR_W),
    .RTYPE_W (RTYPE_W)
) dut (
    .clk                (clk),
    .rst_n              (rst_n),
    .s_addr             (s_addr),
    .s_len_dw           (s_len_dw),
    .s_tc               (s_tc),
    .s_attr             (s_attr),
    .s_reqtype          (s_reqtype),
    .s_valid            (s_valid),
    .s_ready            (s_ready),
    .m_addr             (m_addr),
    .m_len_dw           (m_len_dw),
    .m_tc               (m_tc),
    .m_attr             (m_attr),
    .m_reqtype          (m_reqtype),
    .m_split_flag       (m_split_flag),
    .m_sub_index        (m_sub_index),
    .m_valid            (m_valid),
    .m_ready            (m_ready),
    .stat_split_occurred(stat_split_occurred)
);

// ============================================================
// Test tracking
// ============================================================
integer test_num;
integer pass_count;
integer fail_count;

// ============================================================
// Output capture — what came out of the DUT
// ============================================================
reg [ADDR_W-1:0]   cap_addr   [0:1];
reg [LEN_W-1:0]    cap_len_dw [0:1];
reg [TC_W-1:0]     cap_tc     [0:1];
reg [ATTR_W-1:0]   cap_attr   [0:1];
reg [RTYPE_W-1:0]  cap_reqtype[0:1];
reg                cap_split  [0:1];
reg [1:0]          cap_subidx [0:1];
integer            cap_count;

// ============================================================
// Task: reset
// ============================================================
task do_reset;
    begin
        rst_n   = 0;
        s_valid = 0;
        s_addr  = 0;
        s_len_dw= 0;
        s_tc    = 0;
        s_attr  = 0;
        s_reqtype = 0;
        m_ready = 1;
        repeat(4) @(posedge clk);
        @(negedge clk);
        rst_n = 1;
        @(posedge clk);
    end
endtask

// ============================================================
// Task: drive one descriptor and capture all outputs
// Waits until upstream accepts (s_ready+s_valid) then
// captures every m_valid output until no more come
// ============================================================
task drive_and_capture;
    input [ADDR_W-1:0]  addr;
    input [LEN_W-1:0]   len;
    input [TC_W-1:0]    tc;
    input [ATTR_W-1:0]  attr;
    input [RTYPE_W-1:0] reqtype;
    input               apply_backpressure; // hold m_ready low for 3 cycles on first output
    integer i;
    begin
        cap_count = 0;

        // drive descriptor
        @(negedge clk);
        s_addr    = addr;
        s_len_dw  = len;
        s_tc      = tc;
        s_attr    = attr;
        s_reqtype = reqtype;
        s_valid   = 1;

        // wait for acceptance
        @(posedge clk);
        while (!s_ready) @(posedge clk);

        // descriptor accepted — de-assert valid
        @(negedge clk);
        s_valid = 0;

        // capture outputs
        // wait up to 20 cycles for all outputs to appear
        for (i = 0; i < 20; i = i + 1) begin
            @(posedge clk);

            if (m_valid) begin
                // apply back-pressure if requested on first output
                if (apply_backpressure && cap_count == 0) begin
                    m_ready = 0;
                    repeat(3) @(posedge clk);
                    m_ready = 1;
                    @(posedge clk);
                end

                if (m_valid && m_ready) begin
                    cap_addr   [cap_count] = m_addr;
                    cap_len_dw [cap_count] = m_len_dw;
                    cap_tc     [cap_count] = m_tc;
                    cap_attr   [cap_count] = m_attr;
                    cap_reqtype[cap_count] = m_reqtype;
                    cap_split  [cap_count] = m_split_flag;
                    cap_subidx [cap_count] = m_sub_index;
                    cap_count = cap_count + 1;
                end
            end
        end
    end
endtask

// ============================================================
// Task: check pass-through (data in == data out)
// ============================================================
task check_passthrough;
    input [ADDR_W-1:0]  exp_addr;
    input [LEN_W-1:0]   exp_len;
    input [TC_W-1:0]    exp_tc;
    input [ATTR_W-1:0]  exp_attr;
    input [RTYPE_W-1:0] exp_reqtype;
    begin
        if (cap_count !== 1) begin
            $display("FAIL Test %0d: expected 1 output, got %0d", test_num, cap_count);
            fail_count = fail_count + 1;
        end else if (
            cap_addr[0]    !== exp_addr    ||
            cap_len_dw[0]  !== exp_len     ||
            cap_tc[0]      !== exp_tc      ||
            cap_attr[0]    !== exp_attr    ||
            cap_reqtype[0] !== exp_reqtype ||
            cap_split[0]   !== 1'b0        ||
            cap_subidx[0]  !== 2'd0
        ) begin
            $display("FAIL Test %0d: passthrough field mismatch", test_num);
            $display("  addr:    got %h, exp %h", cap_addr[0], exp_addr);
            $display("  len_dw:  got %0d, exp %0d", cap_len_dw[0], exp_len);
            $display("  tc:      got %0d, exp %0d", cap_tc[0], exp_tc);
            $display("  attr:    got %0d, exp %0d", cap_attr[0], exp_attr);
            $display("  reqtype: got %0d, exp %0d", cap_reqtype[0], exp_reqtype);
            $display("  split:   got %0d, exp 0", cap_split[0]);
            fail_count = fail_count + 1;
        end else begin
            $display("PASS Test %0d", test_num);
            pass_count = pass_count + 1;
        end
    end
endtask

// ============================================================
// Task: check split correctness
// ============================================================
task check_split;
    input [ADDR_W-1:0]  orig_addr;
    input [LEN_W-1:0]   orig_len;
    input [TC_W-1:0]    orig_tc;
    input [ATTR_W-1:0]  orig_attr;
    input [RTYPE_W-1:0] orig_reqtype;
    // expected sub0
    input [ADDR_W-1:0]  exp_addr0;
    input [LEN_W-1:0]   exp_len0;
    // expected sub1
    input [ADDR_W-1:0]  exp_addr1;
    input [LEN_W-1:0]   exp_len1;
    integer ok;
    begin
        ok = 1;

        if (cap_count !== 2) begin
            $display("FAIL Test %0d: expected 2 outputs for split, got %0d",
                     test_num, cap_count);
            fail_count = fail_count + 1;
            ok = 0;
        end

        if (ok) begin
            // check sub0
            if (cap_addr[0]    !== exp_addr0  ||
                cap_len_dw[0]  !== exp_len0   ||
                cap_tc[0]      !== orig_tc     ||
                cap_attr[0]    !== orig_attr   ||
                cap_reqtype[0] !== orig_reqtype||
                cap_split[0]   !== 1'b1        ||
                cap_subidx[0]  !== 2'd0) begin
                $display("FAIL Test %0d: sub0 mismatch", test_num);
                $display("  addr:   got %h, exp %h", cap_addr[0], exp_addr0);
                $display("  len_dw: got %0d, exp %0d", cap_len_dw[0], exp_len0);
                $display("  split:  got %0d, exp 1", cap_split[0]);
                $display("  subidx: got %0d, exp 0", cap_subidx[0]);
                fail_count = fail_count + 1;
                ok = 0;
            end

            // check sub1
            if (cap_addr[1]    !== exp_addr1  ||
                cap_len_dw[1]  !== exp_len1   ||
                cap_tc[1]      !== orig_tc     ||
                cap_attr[1]    !== orig_attr   ||
                cap_reqtype[1] !== orig_reqtype||
                cap_split[1]   !== 1'b1        ||
                cap_subidx[1]  !== 2'd1) begin
                $display("FAIL Test %0d: sub1 mismatch", test_num);
                $display("  addr:   got %h, exp %h", cap_addr[1], exp_addr1);
                $display("  len_dw: got %0d, exp %0d", cap_len_dw[1], exp_len1);
                $display("  split:  got %0d, exp 1", cap_split[1]);
                $display("  subidx: got %0d, exp 1", cap_subidx[1]);
                fail_count = fail_count + 1;
                ok = 0;
            end

            // check sub0+sub1 length == original
            if ((cap_len_dw[0] + cap_len_dw[1]) !== orig_len) begin
                $display("FAIL Test %0d: sub0_len + sub1_len (%0d) != orig_len (%0d)",
                         test_num,
                         cap_len_dw[0] + cap_len_dw[1],
                         orig_len);
                fail_count = fail_count + 1;
                ok = 0;
            end

            // check sub1 starts exactly at 4KB boundary
            if (cap_addr[1] & 12'hFFF) begin
                $display("FAIL Test %0d: sub1 addr %h is not 4KB aligned",
                         test_num, cap_addr[1]);
                fail_count = fail_count + 1;
                ok = 0;
            end

            if (ok) begin
                $display("PASS Test %0d", test_num);
                pass_count = pass_count + 1;
            end
        end
    end
endtask

// ============================================================
// Main test sequence
// ============================================================
initial begin
    $dumpfile("ar_request_splitter_tb.vcd");
    $dumpvars(0, ar_request_splitter_tb);

    pass_count = 0;
    fail_count = 0;
    m_ready    = 1;

    do_reset;

    // ----------------------------------------------------------
    // Test 1: MWr — must pass through unchanged, no split
    // addr=0x0000_1000, len=16 DW (64 bytes), no crossing
    // ----------------------------------------------------------
    test_num = 1;
    $display("\n--- Test %0d: MWr pass-through ---", test_num);
    drive_and_capture(
        64'h0000_0000_0000_1000,  // addr
        10'd16,                    // len_dw
        3'h2,                      // tc
        3'h1,                      // attr
        RTYPE_MWR,                 // reqtype
        0                          // no back-pressure
    );
    check_passthrough(
        64'h0000_0000_0000_1000,
        10'd16,
        3'h2,
        3'h1,
        RTYPE_MWR
    );

    // ----------------------------------------------------------
    // Test 2: MRd — no boundary crossing — pass through
    // addr=0x0000_2000, len=32 DW (128 bytes)
    // page boundary at 0x3000 — request ends at 0x2080, no cross
    // ----------------------------------------------------------
    test_num = 2;
    $display("\n--- Test %0d: MRd no crossing pass-through ---", test_num);
    drive_and_capture(
        64'h0000_0000_0000_2000,
        10'd32,
        3'h0,
        3'h0,
        RTYPE_MRD,
        0
    );
    check_passthrough(
        64'h0000_0000_0000_2000,
        10'd32,
        3'h0,
        3'h0,
        RTYPE_MRD
    );

    // ----------------------------------------------------------
    // Test 3: MRd — crosses 4KB boundary — must split
    // addr=0x0000_0F00, len=128 DW (512 bytes)
    // page boundary at 0x1000
    // sub0: addr=0x0F00, len=(0x1000-0x0F00)/4 = 256/4 = 64 DW
    // sub1: addr=0x1000, len=128-64 = 64 DW
    // ----------------------------------------------------------
    test_num = 3;
    $display("\n--- Test %0d: MRd crosses 4KB boundary ---", test_num);
    drive_and_capture(
        64'h0000_0000_0000_0F00,
        10'd128,
        3'h1,
        3'h2,
        RTYPE_MRD,
        0
    );
    check_split(
        64'h0000_0000_0000_0F00, 10'd128,
        3'h1, 3'h2, RTYPE_MRD,
        64'h0000_0000_0000_0F00, 10'd64,   // sub0
        64'h0000_0000_0000_1000, 10'd64    // sub1
    );

    // ----------------------------------------------------------
    // Test 4: MRd — exactly at 4KB boundary, length fits in page
    // addr=0x0000_1000, len=64 DW (256 bytes)
    // ends at 0x1100 — no crossing
    // ----------------------------------------------------------
    test_num = 4;
    $display("\n--- Test %0d: MRd at 4KB boundary no crossing ---", test_num);
    drive_and_capture(
        64'h0000_0000_0000_1000,
        10'd64,
        3'h3,
        3'h3,
        RTYPE_MRD,
        0
    );
    check_passthrough(
        64'h0000_0000_0000_1000,
        10'd64,
        3'h3,
        3'h3,
        RTYPE_MRD
    );

    // ----------------------------------------------------------
    // Test 5: MRd — crosses by exactly 1 DW (minimal split)
    // addr=0x0000_0FFC, len=2 DW (8 bytes)
    // page boundary at 0x1000
    // sub0: addr=0x0FFC, len=(0x1000-0x0FFC)/4 = 4/4 = 1 DW
    // sub1: addr=0x1000, len=2-1 = 1 DW
    // ----------------------------------------------------------
    test_num = 5;
    $display("\n--- Test %0d: MRd crosses by exactly 1 DW ---", test_num);
    drive_and_capture(
        64'h0000_0000_0000_0FFC,
        10'd2,
        3'h0,
        3'h0,
        RTYPE_MRD,
        0
    );
    check_split(
        64'h0000_0000_0000_0FFC, 10'd2,
        3'h0, 3'h0, RTYPE_MRD,
        64'h0000_0000_0000_0FFC, 10'd1,   // sub0: 1 DW
        64'h0000_0000_0000_1000, 10'd1    // sub1: 1 DW
    );

    // ----------------------------------------------------------
    // Test 6: MRd — large request crossing near end of page
    // addr=0x0000_0100, len=1024 DW (4096 bytes = full MRd max)
    // page boundary at 0x1000
    // sub0: addr=0x0100, len=(0x1000-0x0100)/4 = 3840/4 = 960 DW
    // sub1: addr=0x1000, len=1024-960 = 64 DW
    // ----------------------------------------------------------
    test_num = 6;
    $display("\n--- Test %0d: MRd large crossing ---", test_num);
    drive_and_capture(
        64'h0000_0000_0000_0100,
        10'd1024,
        3'h5,
        3'h6,
        RTYPE_MRD,
        0
    );
    check_split(
        64'h0000_0000_0000_0100, 10'd1024,
        3'h5, 3'h6, RTYPE_MRD,
        64'h0000_0000_0000_0100, 10'd960,  // sub0
        64'h0000_0000_0000_1000, 10'd64    // sub1
    );

    // ----------------------------------------------------------
    // Test 7: back-pressure during split
    // Same crossing as test 3, but m_ready held low for 3 cycles
    // between sub0 and sub1. Verify no data lost.
    // ----------------------------------------------------------
    test_num = 7;
    $display("\n--- Test %0d: split with back-pressure ---", test_num);
    drive_and_capture(
        64'h0000_0000_0000_0F00,
        10'd128,
        3'h1,
        3'h2,
        RTYPE_MRD,
        1   // apply back-pressure
    );
    check_split(
        64'h0000_0000_0000_0F00, 10'd128,
        3'h1, 3'h2, RTYPE_MRD,
        64'h0000_0000_0000_0F00, 10'd64,
        64'h0000_0000_0000_1000, 10'd64
    );

    // ----------------------------------------------------------
    // Test 8: two descriptors back to back — ordering check
    // First: MWr (pass-through)
    // Second: MRd crossing (split)
    // Verify first output belongs to MWr, next two to MRd
    // ----------------------------------------------------------
    test_num = 8;
    $display("\n--- Test %0d: back-to-back MWr then MRd split ---", test_num);

    // drive MWr
    @(negedge clk);
    s_addr    = 64'h0000_0000_0000_5000;
    s_len_dw  = 10'd8;
    s_tc      = 3'h0;
    s_attr    = 3'h0;
    s_reqtype = RTYPE_MWR;
    s_valid   = 1;
    @(posedge clk);
    while (!s_ready) @(posedge clk);
    @(negedge clk);
    // immediately drive MRd without de-asserting valid
    s_addr    = 64'h0000_0000_0000_0F80;
    s_len_dw  = 10'd64;
    s_tc      = 3'h2;
    s_attr    = 3'h1;
    s_reqtype = RTYPE_MRD;

    // capture 3 outputs total
    cap_count = 0;
    begin : capture_block
        integer i;
        for (i = 0; i < 40; i = i + 1) begin
            @(posedge clk);
            if (m_valid && m_ready) begin
                cap_addr   [cap_count] = m_addr;
                cap_len_dw [cap_count] = m_len_dw;
                cap_reqtype[cap_count] = m_reqtype;
                cap_split  [cap_count] = m_split_flag;
                cap_subidx [cap_count] = m_sub_index;
                cap_count = cap_count + 1;
                if (cap_count == 1) begin
                    @(negedge clk);
                    s_valid = 0;
                end
            end
            if (cap_count == 3) disable capture_block;
        end
    end

    begin : check_back2back
        integer ok;
        ok = 1;
        if (cap_count !== 3) begin
            $display("FAIL Test %0d: expected 3 outputs, got %0d",
                     test_num, cap_count);
            fail_count = fail_count + 1;
            ok = 0;
        end
        if (ok) begin
            // output 0: MWr pass-through
            if (cap_reqtype[0] !== RTYPE_MWR || cap_split[0] !== 0) begin
                $display("FAIL Test %0d: output 0 should be MWr passthrough", test_num);
                fail_count = fail_count + 1;
                ok = 0;
            end
            // output 1: MRd sub0
            if (cap_reqtype[1] !== RTYPE_MRD ||
                cap_split[1]   !== 1          ||
                cap_subidx[1]  !== 0) begin
                $display("FAIL Test %0d: output 1 should be MRd sub0", test_num);
                fail_count = fail_count + 1;
                ok = 0;
            end
            // output 2: MRd sub1
            if (cap_reqtype[2] !== RTYPE_MRD ||
                cap_split[2]   !== 1          ||
                cap_subidx[2]  !== 1) begin
                $display("FAIL Test %0d: output 2 should be MRd sub1", test_num);
                fail_count = fail_count + 1;
                ok = 0;
            end
            if (ok) begin
                $display("PASS Test %0d", test_num);
                pass_count = pass_count + 1;
            end
        end
    end

    // ----------------------------------------------------------
    // Summary
    // ----------------------------------------------------------
    repeat(5) @(posedge clk);
    $display("\n==============================");
    $display("Results: %0d PASSED, %0d FAILED", pass_count, fail_count);
    $display("==============================\n");

    if (fail_count == 0)
        $display("ALL TESTS PASSED");
    else
        $display("SOME TESTS FAILED — check output above");

    $finish;
end

// ============================================================
// Timeout watchdog — kills simulation if it hangs
// ============================================================
initial begin
    #50000;
    $display("FATAL: simulation timeout — possible deadlock");
    $finish;
end

endmodule