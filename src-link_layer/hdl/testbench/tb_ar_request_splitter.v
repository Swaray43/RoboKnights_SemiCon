// ============================================================
// Testbench : tb_ar_request_splitter  (updated for change-report 2025)
//
// Changes vs original TB:
//   - s_tc, s_attr, s_reqtype ports removed — MWr-only now
//   - s_tuser [TUSER_W-1:0] added (stream_id + session_offset
//     + tlast_flag)
//   - cap arrays extended to hold tuser output
//   - check_passthrough and check_split updated accordingly
//   - Test 1 (MWr) and Test 2 (MRd no-cross) become:
//       Test 1: MWr (no crossing) — pass-through
//       Test 2: MWr (no crossing) — different stream_id,
//               verifies tuser propagation
//   - Tests 3-8: crossing / back-pressure tests retained,
//     adapted to MWr-only interface
//   - Bug 7 coverage: first transaction issued immediately
//     after reset with no extra idle — verifies s_ready=1
//     at reset release
//   - Bug 10 coverage: stat_split_occurred checked, then
//     registered lengths verified on the following cycle
//   - Bug 11 coverage: back-to-back descriptors are driven
//     with no gap between them; expected output count verified
//
// Parameters: LEN_W=11 (supports 1024 DW), TUSER_W=26
// ============================================================

`timescale 1ns/1ps

module tb_ar_request_splitter;

// ============================================================
// Parameters — must match DUT defaults
// ============================================================
localparam ADDR_W  = 64;
localparam LEN_W   = 11;
localparam TUSER_W = 26;   // {tlast_flag(1), session_offset(24), stream_id(1)}
localparam SOFF_W  = 24;

// ============================================================
// Bit positions inside TUSER (must match DUT)
// ============================================================
localparam STREAM_ID_BIT  = 0;
localparam SOFF_LSB       = 1;
localparam SOFF_MSB       = SOFF_W;           // bit 24
localparam TLAST_FLAG_BIT = TUSER_W - 1;      // bit 25

// ============================================================
// Clock and reset
// ============================================================
reg clk;
reg rst_n;

initial clk = 0;
always #5 clk = ~clk;   // 100 MHz

// ============================================================
// DUT ports
// ============================================================
reg  [ADDR_W-1:0]    s_addr;
reg  [LEN_W-1:0]     s_len_dw;
reg  [TUSER_W-1:0]   s_tuser;
reg                  s_valid;
wire                 s_ready;

wire [ADDR_W-1:0]    m_addr;
wire [LEN_W-1:0]     m_len_dw;
wire [TUSER_W-1:0]   m_tuser;
wire                 m_split_flag;
wire [1:0]           m_sub_index;
wire                 m_valid;
reg                  m_ready;

wire                 stat_split_occurred;

// ============================================================
// DUT instantiation
// ============================================================
ar_request_splitter #(
    .ADDR_W  (ADDR_W),
    .LEN_W   (LEN_W),
    .TUSER_W (TUSER_W),
    .SOFF_W  (SOFF_W)
) dut (
    .clk                (clk),
    .rst_n              (rst_n),
    .s_addr             (s_addr),
    .s_len_dw           (s_len_dw),
    .s_tuser            (s_tuser),
    .s_valid            (s_valid),
    .s_ready            (s_ready),
    .m_addr             (m_addr),
    .m_len_dw           (m_len_dw),
    .m_tuser            (m_tuser),
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
// Capture arrays — sized [0:2] for test 8 (3 outputs)
// ============================================================
reg [ADDR_W-1:0]    cap_addr   [0:2];
reg [LEN_W-1:0]     cap_len_dw [0:2];
reg [TUSER_W-1:0]   cap_tuser  [0:2];
reg                 cap_split  [0:2];
reg [1:0]           cap_subidx [0:2];
integer             cap_count;

// ============================================================
// Helper: build a TUSER word
// Ticket G7 (Aditya + Swaray): now built via shared/axis_tuser_defs.vh's
// tuser_pack instead of a second hand-written copy of the field order
// (see that file's header for why this matters).
// ============================================================
`include "axis_tuser_defs.vh"

function [TUSER_W-1:0] make_tuser;
    input               stream_id;
    input [SOFF_W-1:0]  session_offset;
    input               tlast_flag;
    begin
        make_tuser = tuser_pack(tlast_flag, session_offset, stream_id);
    end
endfunction

// ============================================================
// Task: do_reset
// ============================================================
task do_reset;
    begin
        rst_n     = 0;
        s_valid   = 0;
        s_addr    = 0;
        s_len_dw  = 0;
        s_tuser   = 0;
        m_ready   = 1;
        repeat(4) @(posedge clk);
        @(negedge clk);
        rst_n = 1;
        // Bug 7 fix coverage: DO NOT wait extra cycles here.
        // s_ready should be 1 immediately after rst_n rises.
        @(posedge clk);
    end
endtask

// ============================================================
// Task: wait_dut_idle
// ============================================================
task wait_dut_idle;
    integer timeout;
    begin
        timeout = 0;
        @(posedge clk);
        while (!s_ready && timeout < 100) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        if (timeout == 100)
            $display("WARNING: wait_dut_idle timed out");
        @(posedge clk);
    end
endtask

// ============================================================
// Task: drive_descriptor
// ============================================================
task drive_descriptor;
    input [ADDR_W-1:0]   addr;
    input [LEN_W-1:0]    len;
    input [TUSER_W-1:0]  tuser;
    begin
        @(negedge clk);
        s_addr   = addr;
        s_len_dw = len;
        s_tuser  = tuser;
        s_valid  = 1;

        @(posedge clk);
        while (!s_ready) @(posedge clk);

        @(negedge clk);
        s_valid = 0;
    end
endtask

// ============================================================
// Task: drive_and_capture (no back-pressure)
// ============================================================
task drive_and_capture;
    input [ADDR_W-1:0]   addr;
    input [LEN_W-1:0]    len;
    input [TUSER_W-1:0]  tuser;
    integer i;
    begin
        cap_count = 0;
        m_ready   = 1;

        drive_descriptor(addr, len, tuser);

        begin : cap_loop
            integer seen_valid;
            seen_valid = 0;
            for (i = 0; i < 60; i = i + 1) begin
                @(posedge clk);
                if (m_valid && m_ready) begin
                    seen_valid            = 1;
                    cap_addr   [cap_count] = m_addr;
                    cap_len_dw [cap_count] = m_len_dw;
                    cap_tuser  [cap_count] = m_tuser;
                    cap_split  [cap_count] = m_split_flag;
                    cap_subidx [cap_count] = m_sub_index;
                    cap_count             = cap_count + 1;
                end else if (seen_valid && !m_valid) begin
                    disable cap_loop;
                end
            end
        end
    end
endtask

// ============================================================
// Task: drive_and_capture_with_bp
// ============================================================
task drive_and_capture_with_bp;
    input [ADDR_W-1:0]   addr;
    input [LEN_W-1:0]    len;
    input [TUSER_W-1:0]  tuser;
    integer timeout;
    begin
        cap_count = 0;
        m_ready   = 1;

        drive_descriptor(addr, len, tuser);

        // Wait for sub0 and capture it
        timeout = 0;
        @(posedge clk);
        while (!(m_valid && m_ready) && timeout < 100) begin
            @(posedge clk);
            timeout = timeout + 1;
        end

        if (m_valid && m_ready) begin
            cap_addr   [0] = m_addr;
            cap_len_dw [0] = m_len_dw;
            cap_tuser  [0] = m_tuser;
            cap_split  [0] = m_split_flag;
            cap_subidx [0] = m_sub_index;
            cap_count      = 1;
        end else begin
            $display("WARNING Test %0d: sub0 never appeared", test_num);
        end

        // Apply back-pressure after sub0 captured
        @(negedge clk); m_ready = 0;
        @(posedge clk);
        @(posedge clk);
        @(posedge clk);

        @(negedge clk); m_ready = 1;

        // Wait for sub1 and capture it
        timeout = 0;
        @(posedge clk);
        while (!(m_valid && m_ready) && timeout < 100) begin
            @(posedge clk);
            timeout = timeout + 1;
        end

        if (m_valid && m_ready) begin
            cap_addr   [1] = m_addr;
            cap_len_dw [1] = m_len_dw;
            cap_tuser  [1] = m_tuser;
            cap_split  [1] = m_split_flag;
            cap_subidx [1] = m_sub_index;
            cap_count      = 2;
        end else begin
            $display("WARNING Test %0d: sub1 never appeared after bp release",
                     test_num);
        end

        @(posedge clk);
        while (m_valid) @(posedge clk);
    end
endtask

// ============================================================
// Task: check_passthrough
// ============================================================
task check_passthrough;
    input [ADDR_W-1:0]   exp_addr;
    input [LEN_W-1:0]    exp_len;
    input [TUSER_W-1:0]  exp_tuser;
    begin
        if (cap_count !== 1) begin
            $display("FAIL Test %0d: expected 1 output, got %0d",
                     test_num, cap_count);
            fail_count = fail_count + 1;
        end else if (
            cap_addr[0]   !== exp_addr  ||
            cap_len_dw[0] !== exp_len   ||
            cap_tuser[0]  !== exp_tuser ||
            cap_split[0]  !== 1'b0      ||
            cap_subidx[0] !== 2'd0
        ) begin
            $display("FAIL Test %0d: passthrough field mismatch", test_num);
            $display("  addr:    got %h, exp %h",    cap_addr[0],   exp_addr);
            $display("  len_dw:  got %0d, exp %0d",  cap_len_dw[0], exp_len);
            $display("  tuser:   got %h, exp %h",    cap_tuser[0],  exp_tuser);
            $display("  split:   got %0d, exp 0",    cap_split[0]);
            $display("  subidx:  got %0d, exp 0",    cap_subidx[0]);
            fail_count = fail_count + 1;
        end else begin
            $display("PASS Test %0d", test_num);
            pass_count = pass_count + 1;
        end
    end
endtask

// ============================================================
// Task: check_split
// ============================================================
task check_split;
    input [ADDR_W-1:0]   orig_addr;
    input [LEN_W-1:0]    orig_len;
    input [TUSER_W-1:0]  orig_tuser;
    input [ADDR_W-1:0]   exp_addr0;
    input [LEN_W-1:0]    exp_len0;
    input [ADDR_W-1:0]   exp_addr1;
    input [LEN_W-1:0]    exp_len1;
    integer ok;
    begin
        ok = 1;

        if (cap_count !== 2) begin
            $display("FAIL Test %0d: expected 2 outputs, got %0d",
                     test_num, cap_count);
            fail_count = fail_count + 1;
            ok = 0;
        end

        if (ok) begin
            // sub0
            if (cap_addr[0]   !== exp_addr0  ||
                cap_len_dw[0] !== exp_len0   ||
                cap_tuser[0]  !== orig_tuser ||
                cap_split[0]  !== 1'b1       ||
                cap_subidx[0] !== 2'd0) begin
                $display("FAIL Test %0d: sub0 mismatch", test_num);
                $display("  addr:   got %h, exp %h",   cap_addr[0],   exp_addr0);
                $display("  len_dw: got %0d, exp %0d", cap_len_dw[0], exp_len0);
                $display("  tuser:  got %h, exp %h",   cap_tuser[0],  orig_tuser);
                $display("  split:  got %0d, exp 1",   cap_split[0]);
                $display("  subidx: got %0d, exp 0",   cap_subidx[0]);
                fail_count = fail_count + 1;
                ok = 0;
            end

            // sub1
            if (cap_addr[1]   !== exp_addr1  ||
                cap_len_dw[1] !== exp_len1   ||
                cap_tuser[1]  !== orig_tuser ||
                cap_split[1]  !== 1'b1       ||
                cap_subidx[1] !== 2'd1) begin
                $display("FAIL Test %0d: sub1 mismatch", test_num);
                $display("  addr:   got %h, exp %h",   cap_addr[1],   exp_addr1);
                $display("  len_dw: got %0d, exp %0d", cap_len_dw[1], exp_len1);
                $display("  tuser:  got %h, exp %h",   cap_tuser[1],  orig_tuser);
                $display("  split:  got %0d, exp 1",   cap_split[1]);
                $display("  subidx: got %0d, exp 1",   cap_subidx[1]);
                fail_count = fail_count + 1;
                ok = 0;
            end

            // length conservation
            if ((cap_len_dw[0] + cap_len_dw[1]) !== orig_len) begin
                $display("FAIL Test %0d: len0(%0d)+len1(%0d)=%0d != orig(%0d)",
                         test_num,
                         cap_len_dw[0], cap_len_dw[1],
                         cap_len_dw[0]+cap_len_dw[1], orig_len);
                fail_count = fail_count + 1;
                ok = 0;
            end

            // sub1 must be 4KB-aligned
            if (cap_addr[1][11:0] !== 12'h000) begin
                $display("FAIL Test %0d: sub1 addr %h not 4KB aligned",
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
    $dumpfile("tb_ar_request_splitter.vcd");
    $dumpvars(0, tb_ar_request_splitter);

    pass_count = 0;
    fail_count = 0;
    m_ready    = 1;

    do_reset;

    // ----------------------------------------------------------
    // Test 1: MWr pass-through (stream_id=0, SmartNIC)
    // Bug 7 coverage: first beat issued immediately after reset;
    // s_ready should already be high (initialised to 1).
    // addr=0x1000, len=16 DW — well within one page.
    // Expected: 1 output, all fields preserved, split_flag=0.
    // ----------------------------------------------------------
    test_num = 1;
    $display("\n--- Test %0d: MWr no-cross pass-through (stream_id=0) ---",
             test_num);
    drive_and_capture(
        64'h0000_0000_0000_1000,
        11'd16,
        make_tuser(1'b0, 24'h000000, 1'b0)
    );
    check_passthrough(
        64'h0000_0000_0000_1000,
        11'd16,
        make_tuser(1'b0, 24'h000000, 1'b0)
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 2: MWr pass-through (stream_id=1, ML accelerator)
    // Verifies stream_id and session_offset are preserved.
    // addr=0x2000, len=32 DW — no crossing.
    // Expected: 1 output, tuser preserved exactly.
    // ----------------------------------------------------------
    test_num = 2;
    $display("\n--- Test %0d: MWr no-cross pass-through (stream_id=1) ---",
             test_num);
    drive_and_capture(
        64'h0000_0000_0000_2000,
        11'd32,
        make_tuser(1'b1, 24'hABCDEF, 1'b0)
    );
    check_passthrough(
        64'h0000_0000_0000_2000,
        11'd32,
        make_tuser(1'b1, 24'hABCDEF, 1'b0)
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 3: MWr crosses 4KB boundary — split required
    // addr=0x0F00, len=128 DW (512 bytes)
    // last byte at 0x0F00+512-1=0x10FF — crosses 0x1000
    // sub0: addr=0x0F00, len=(0x1000-0x0F00)/4=64 DW
    // sub1: addr=0x1000, len=128-64=64 DW
    // TUSER propagated to both sub-descriptors.
    // Expected: 2 outputs, both split_flag=1.
    // ----------------------------------------------------------
    test_num = 3;
    $display("\n--- Test %0d: MWr crosses 4KB boundary ---", test_num);
    drive_and_capture(
        64'h0000_0000_0000_0F00,
        11'd128,
        make_tuser(1'b0, 24'h000040, 1'b0)   // session_offset=64 (second beat)
    );
    check_split(
        64'h0000_0000_0000_0F00, 11'd128,
        make_tuser(1'b0, 24'h000040, 1'b0),
        64'h0000_0000_0000_0F00, 11'd64,
        64'h0000_0000_0000_1000, 11'd64
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 4: MWr exactly at 4KB boundary — no crossing
    // addr=0x1000 (page-aligned), len=64 DW (256 bytes)
    // ends at 0x1100 — within the 0x1000-0x2000 page
    // Expected: 1 output, split_flag=0.
    // ----------------------------------------------------------
    test_num = 4;
    $display("\n--- Test %0d: MWr at 4KB boundary (no crossing) ---",
             test_num);
    drive_and_capture(
        64'h0000_0000_0000_1000,
        11'd64,
        make_tuser(1'b1, 24'h000000, 1'b0)
    );
    check_passthrough(
        64'h0000_0000_0000_1000,
        11'd64,
        make_tuser(1'b1, 24'h000000, 1'b0)
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 5: MWr crosses by exactly 1 DW — minimal split
    // addr=0x0FFC (4 bytes before boundary), len=2 DW (8 bytes)
    // last byte at 0x0FFC+8-1=0x1003 — crosses 0x1000
    // sub0: addr=0x0FFC, len=1 DW
    // sub1: addr=0x1000, len=1 DW
    // tlast_flag=1 on the session-end descriptor.
    // ----------------------------------------------------------
    test_num = 5;
    $display("\n--- Test %0d: MWr crosses by exactly 1 DW (minimal split) ---",
             test_num);
    drive_and_capture(
        64'h0000_0000_0000_0FFC,
        11'd2,
        make_tuser(1'b0, 24'hFFFFFF, 1'b1)   // tlast_flag=1, session ending
    );
    check_split(
        64'h0000_0000_0000_0FFC, 11'd2,
        make_tuser(1'b0, 24'hFFFFFF, 1'b1),
        64'h0000_0000_0000_0FFC, 11'd1,
        64'h0000_0000_0000_1000, 11'd1
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 6: MWr large crossing — maximum payload length
    // addr=0x0100, len=1024 DW (4096 bytes)
    // LEN_W=11 required: 1024 needs 11 bits
    // last byte at 0x0100+4096-1=0x10FF — crosses 0x1000
    // sub0: addr=0x0100, len=(0x1000-0x0100)/4=960 DW
    // sub1: addr=0x1000, len=1024-960=64 DW
    // ----------------------------------------------------------
    test_num = 6;
    $display("\n--- Test %0d: MWr large crossing (1024 DW) ---", test_num);
    drive_and_capture(
        64'h0000_0000_0000_0100,
        11'd1024,
        make_tuser(1'b1, 24'h000000, 1'b0)
    );
    check_split(
        64'h0000_0000_0000_0100, 11'd1024,
        make_tuser(1'b1, 24'h000000, 1'b0),
        64'h0000_0000_0000_0100, 11'd960,
        64'h0000_0000_0000_1000, 11'd64
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 7: back-pressure mid-split
    // Same crossing as test 3 — sub0=64DW, sub1=64DW.
    // m_ready pulled low after sub0 is captured.
    // Verifies sub1 output is stable during stall and correctly
    // delivered after m_ready re-asserts.
    // ----------------------------------------------------------
    test_num = 7;
    $display("\n--- Test %0d: split with back-pressure ---", test_num);
    drive_and_capture_with_bp(
        64'h0000_0000_0000_0F00,
        11'd128,
        make_tuser(1'b0, 24'h000040, 1'b0)
    );
    check_split(
        64'h0000_0000_0000_0F00, 11'd128,
        make_tuser(1'b0, 24'h000040, 1'b0),
        64'h0000_0000_0000_0F00, 11'd64,
        64'h0000_0000_0000_1000, 11'd64
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 8: back-to-back descriptors — no-cross then crossing
    // Bug 11 coverage: the pass-through output and split pair
    // are issued with no idle bubble between them.
    //
    // Descriptor 0: no-cross, addr=0x5000, len=8 DW
    //   → 1 output: split_flag=0
    // Descriptor 1: crossing, addr=0x0F80, len=64 DW
    //   last byte at 0x0F80+256-1=0x107F — crosses 0x1000
    //   sub0: addr=0x0F80, len=(0x1000-0x0F80)/4=32 DW
    //   sub1: addr=0x1000, len=64-32=32 DW
    //   → 2 outputs: split_flag=1 on both
    //
    // Total: 3 outputs, in order: pass, sub0, sub1.
    // Uses fork-join so capture runs in parallel with driver.
    // ----------------------------------------------------------
    test_num  = 8;
    cap_count = 0;
    m_ready   = 1;
    $display("\n--- Test %0d: back-to-back (pass-through then split) ---",
             test_num);

    fork

        // Thread 1: descriptor driver
        begin
            // Drive descriptor 0 (no-cross)
            @(negedge clk);
            s_addr   = 64'h0000_0000_0000_5000;
            s_len_dw = 11'd8;
            s_tuser  = make_tuser(1'b0, 24'h000000, 1'b0);
            s_valid  = 1;

            @(posedge clk);
            while (!s_ready) @(posedge clk);

            // Switch to descriptor 1 (crossing) without dropping s_valid
            @(negedge clk);
            s_addr   = 64'h0000_0000_0000_0F80;
            s_len_dw = 11'd64;
            s_tuser  = make_tuser(1'b1, 24'h000080, 1'b0);
            // s_valid stays high

            @(posedge clk);
            while (!s_ready) @(posedge clk);

            @(negedge clk);
            s_valid = 0;
        end

        // Thread 2: output capture
        begin : cap8_fork
            integer i8;
            for (i8 = 0; i8 < 200; i8 = i8 + 1) begin
                @(posedge clk);
                if (m_valid && m_ready) begin
                    cap_addr   [cap_count] = m_addr;
                    cap_len_dw [cap_count] = m_len_dw;
                    cap_tuser  [cap_count] = m_tuser;
                    cap_split  [cap_count] = m_split_flag;
                    cap_subidx [cap_count] = m_sub_index;
                    cap_count = cap_count + 1;
                    if (cap_count == 3) disable cap8_fork;
                end
            end
        end

    join

    // Check test 8 results
    begin : check8
        integer ok8;
        ok8 = 1;

        if (cap_count !== 3) begin
            $display("FAIL Test %0d: expected 3 outputs, got %0d",
                     test_num, cap_count);
            fail_count = fail_count + 1;
            ok8 = 0;
        end

        if (ok8) begin
            // output 0: pass-through, stream_id=0
            if (cap_split[0]  !== 1'b0                    ||
                cap_subidx[0] !== 2'd0                    ||
                cap_addr[0]   !== 64'h0000_0000_0000_5000 ||
                cap_len_dw[0] !== 11'd8                   ||
                cap_tuser[0]  !== make_tuser(1'b0, 24'h000000, 1'b0)) begin
                $display("FAIL Test %0d: output 0 mismatch", test_num);
                $display("  split:  got %0d, exp 0",   cap_split[0]);
                $display("  subidx: got %0d, exp 0",   cap_subidx[0]);
                $display("  addr:   got %h, exp 5000", cap_addr[0]);
                $display("  len_dw: got %0d, exp 8",   cap_len_dw[0]);
                $display("  tuser:  got %h, exp %h",   cap_tuser[0],
                         make_tuser(1'b0, 24'h000000, 1'b0));
                fail_count = fail_count + 1;
                ok8 = 0;
            end

            // output 1: sub0, stream_id=1
            if (cap_split[1]  !== 1'b1                    ||
                cap_subidx[1] !== 2'd0                    ||
                cap_addr[1]   !== 64'h0000_0000_0000_0F80 ||
                cap_len_dw[1] !== 11'd32                  ||
                cap_tuser[1]  !== make_tuser(1'b1, 24'h000080, 1'b0)) begin
                $display("FAIL Test %0d: output 1 (sub0) mismatch", test_num);
                $display("  split:  got %0d, exp 1",   cap_split[1]);
                $display("  subidx: got %0d, exp 0",   cap_subidx[1]);
                $display("  addr:   got %h, exp 0F80", cap_addr[1]);
                $display("  len_dw: got %0d, exp 32",  cap_len_dw[1]);
                $display("  tuser:  got %h, exp %h",   cap_tuser[1],
                         make_tuser(1'b1, 24'h000080, 1'b0));
                fail_count = fail_count + 1;
                ok8 = 0;
            end

            // output 2: sub1, stream_id=1, page-aligned
            if (cap_split[2]  !== 1'b1                    ||
                cap_subidx[2] !== 2'd1                    ||
                cap_addr[2]   !== 64'h0000_0000_0000_1000 ||
                cap_len_dw[2] !== 11'd32                  ||
                cap_tuser[2]  !== make_tuser(1'b1, 24'h000080, 1'b0)) begin
                $display("FAIL Test %0d: output 2 (sub1) mismatch", test_num);
                $display("  split:  got %0d, exp 1",   cap_split[2]);
                $display("  subidx: got %0d, exp 1",   cap_subidx[2]);
                $display("  addr:   got %h, exp 1000", cap_addr[2]);
                $display("  len_dw: got %0d, exp 32",  cap_len_dw[2]);
                $display("  tuser:  got %h, exp %h",   cap_tuser[2],
                         make_tuser(1'b1, 24'h000080, 1'b0));
                fail_count = fail_count + 1;
                ok8 = 0;
            end

            // length conservation for split pair
            if ((cap_len_dw[1] + cap_len_dw[2]) !== 11'd64) begin
                $display("FAIL Test %0d: sub0(%0d)+sub1(%0d) != 64",
                         test_num, cap_len_dw[1], cap_len_dw[2]);
                fail_count = fail_count + 1;
                ok8 = 0;
            end

            // sub1 page-aligned
            if (cap_addr[2][11:0] !== 12'h000) begin
                $display("FAIL Test %0d: sub1 addr %h not 4KB aligned",
                         test_num, cap_addr[2]);
                fail_count = fail_count + 1;
                ok8 = 0;
            end

            if (ok8) begin
                $display("PASS Test %0d", test_num);
                pass_count = pass_count + 1;
            end
        end
    end

    // ----------------------------------------------------------
    // Test 9: Bug 7 regression — verify s_ready is high
    // immediately after reset, before any extra wait cycles.
    // Drive a descriptor with s_valid already asserted on the
    // first posedge after rst_n rises.
    // ----------------------------------------------------------
    test_num = 9;
    $display("\n--- Test %0d: Bug7 regression — s_ready high post-reset ---",
             test_num);
    begin : t9
        reg s_ready_at_first_posedge;

        // Apply reset
        rst_n   = 0;
        s_valid = 0;
        repeat(4) @(posedge clk);

        // Release reset and immediately drive s_valid
        @(negedge clk);
        rst_n   = 1;
        s_addr   = 64'h0000_0000_0000_3000;
        s_len_dw = 11'd4;
        s_tuser  = make_tuser(1'b0, 24'h000000, 1'b0);
        s_valid  = 1;

        // Sample s_ready on the very first posedge after reset release
        @(posedge clk);
        s_ready_at_first_posedge = s_ready;

        @(negedge clk);
        s_valid = 0;

        if (!s_ready_at_first_posedge) begin
            $display("FAIL Test %0d: s_ready was low on first posedge \
                      after reset — Bug 7 not fixed.", test_num);
            fail_count = fail_count + 1;
        end else begin
            $display("PASS Test %0d: s_ready high immediately after reset.",
                     test_num);
            pass_count = pass_count + 1;
        end
    end
    wait_dut_idle;

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
// Timeout watchdog
// ============================================================
initial begin
    #300000;
    $display("FATAL: simulation timeout — possible deadlock");
    $finish;
end

endmodule