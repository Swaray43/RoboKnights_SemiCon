`timescale 1ns/1ps

module tb_address_translation;

localparam ADDR_W      = 64;
localparam LEN_W       = 11;
localparam TUSER_W     = 26;
localparam SOFF_W      = 24;
localparam NUM_STREAMS = 2;

reg clk;
reg rst_n;

initial clk = 0;
always #5 clk = ~clk;

// ============================================================
// DUT ports
// ============================================================
reg  [ADDR_W-1:0]              s_addr;
reg  [LEN_W-1:0]               s_len_dw;
reg  [TUSER_W-1:0]             s_tuser;
reg                            s_split_flag;
reg  [1:0]                     s_sub_index;
reg                            s_valid;
wire                           s_ready;

wire [ADDR_W-1:0]              m_pcie_addr;
wire [LEN_W-1:0]               m_len_dw;
wire                           m_stream_id;
wire                           m_split_flag_out;
wire [1:0]                     m_sub_index_out;
wire                           m_tlast_flag;
wire                           m_valid;
reg                            m_ready;

reg                            cfg_base_addr_wr;
reg  [$clog2(NUM_STREAMS)-1:0] cfg_base_addr_id;
reg  [ADDR_W-1:0]              cfg_base_addr_data;
reg                            cfg_base_addr_clr;

wire                           err_xlat_miss;
wire                           err_addr_ovf;
wire                           xlat_busy;

// ============================================================
// DUT
// ============================================================
address_translation #(
    .ADDR_W      (ADDR_W),
    .LEN_W       (LEN_W),
    .TUSER_W     (TUSER_W),
    .SOFF_W      (SOFF_W),
    .NUM_STREAMS (NUM_STREAMS)
) dut (
    .clk               (clk),
    .rst_n             (rst_n),
    .s_addr            (s_addr),
    .s_len_dw          (s_len_dw),
    .s_tuser           (s_tuser),
    .s_split_flag      (s_split_flag),
    .s_sub_index       (s_sub_index),
    .s_valid           (s_valid),
    .s_ready           (s_ready),
    .m_pcie_addr       (m_pcie_addr),
    .m_len_dw          (m_len_dw),
    .m_stream_id       (m_stream_id),
    .m_split_flag      (m_split_flag_out),
    .m_sub_index       (m_sub_index_out),
    .m_tlast_flag      (m_tlast_flag),
    .m_valid           (m_valid),
    .m_ready           (m_ready),
    .cfg_base_addr_wr  (cfg_base_addr_wr),
    .cfg_base_addr_id  (cfg_base_addr_id),
    .cfg_base_addr_data(cfg_base_addr_data),
    .cfg_base_addr_clr (cfg_base_addr_clr),
    .err_xlat_miss     (err_xlat_miss),
    .err_addr_ovf      (err_addr_ovf),
    .xlat_busy         (xlat_busy)
);

// ============================================================
// Test tracking
// ============================================================
integer test_num;
integer pass_count;
integer fail_count;

// ============================================================
// Sticky error flag registers
// Cleared by calling clear_sticky_errors task.
// Latched by the always block below on every posedge.
// This way any test can check them without worrying about
// which exact cycle the error fired.
// ============================================================
reg sticky_xlat_miss;
reg sticky_addr_ovf;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sticky_xlat_miss <= 0;
        sticky_addr_ovf  <= 0;
    end else begin
        if (err_xlat_miss) sticky_xlat_miss <= 1;
        if (err_addr_ovf)  sticky_addr_ovf  <= 1;
    end
end

task clear_sticky_errors;
    begin
        // pulse a register clear by temporarily forcing — not
        // synthesisable but fine in testbench
        @(negedge clk);
        force sticky_xlat_miss = 0;
        force sticky_addr_ovf  = 0;
        @(posedge clk);
        release sticky_xlat_miss;
        release sticky_addr_ovf;
    end
endtask

// ============================================================
// Capture registers
// ============================================================
reg [ADDR_W-1:0]  cap_pcie_addr;
reg [LEN_W-1:0]   cap_len_dw;
reg               cap_stream_id;
reg               cap_split;
reg [1:0]         cap_subidx;
reg               cap_tlast;
reg               cap_got;      // 1 = capture_output succeeded

// ============================================================
// Helper: make_tuser
// Ticket G7 (Aditya + Swaray): this used to hardcode the field order
// itself ({tlast, sess_offset, stream_id}), a second hand-written copy
// of the layout the DUT assumes -- exactly the kind of copy that can
// silently drift from the DUT's own. Now built via shared/
// axis_tuser_defs.vh's tuser_pack, the one place the layout is defined.
// ============================================================
`include "axis_tuser_defs.vh"

function [TUSER_W-1:0] make_tuser;
    input              stream_id;
    input [SOFF_W-1:0] sess_offset;
    input              tlast;
    begin
        make_tuser = tuser_pack(tlast, sess_offset, stream_id);
    end
endfunction

// ============================================================
// Task: do_reset
// ============================================================
task do_reset;
    begin
        rst_n              = 0;
        s_valid            = 0;
        s_addr             = 0;
        s_len_dw           = 0;
        s_tuser            = 0;
        s_split_flag       = 0;
        s_sub_index        = 0;
        m_ready            = 1;
        cfg_base_addr_wr   = 0;
        cfg_base_addr_id   = 0;
        cfg_base_addr_data = 0;
        cfg_base_addr_clr  = 0;
        repeat(4) @(posedge clk);
        @(negedge clk);
        rst_n = 1;
        @(posedge clk);
    end
endtask

// ============================================================
// Task: program_base_addr
// ============================================================
task program_base_addr;
    input integer       sid;
    input [ADDR_W-1:0]  base;
    begin
        @(negedge clk);
        cfg_base_addr_wr   = 1;
        cfg_base_addr_id   = sid[$clog2(NUM_STREAMS)-1:0];
        cfg_base_addr_data = base;
        @(posedge clk);
        @(negedge clk);
        cfg_base_addr_wr   = 0;
        cfg_base_addr_id   = 0;
        cfg_base_addr_data = 0;
        @(posedge clk);
    end
endtask

// ============================================================
// Task: clear_base_addrs
// ============================================================
task clear_base_addrs;
    begin
        @(negedge clk);
        cfg_base_addr_clr = 1;
        @(posedge clk);
        @(negedge clk);
        cfg_base_addr_clr = 0;
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
        while (!s_ready && timeout < 200) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        if (timeout == 200)
            $display("WARNING: wait_dut_idle timed out");
        @(posedge clk);
    end
endtask

// ============================================================
// Task: send_descriptor
//
// Drives one descriptor and waits for s_ready acceptance.
// De-asserts s_valid immediately after — no extra wait cycles.
// Does NOT sample error flags — use sticky registers instead.
// ============================================================
task send_descriptor;
    input [ADDR_W-1:0]   addr;
    input [LEN_W-1:0]    len;
    input [TUSER_W-1:0]  tuser;
    input                split_flag;
    input [1:0]          sub_index;
    begin
        @(negedge clk);
        s_addr       = addr;
        s_len_dw     = len;
        s_tuser      = tuser;
        s_split_flag = split_flag;
        s_sub_index  = sub_index;
        s_valid      = 1;

        @(posedge clk);
        while (!s_ready) @(posedge clk);

        // de-assert valid immediately — do not consume any
        // extra cycles that might swallow m_valid pulses
        @(negedge clk);
        s_valid = 0;
    end
endtask

// ============================================================
// Task: capture_output
//
// Waits up to 100 cycles for m_valid && m_ready.
// Records all output fields into cap_* registers.
// Sets cap_got=1 on success, 0 on timeout.
// Waits for m_valid to drop before returning so the
// next test starts cleanly.
// ============================================================
task capture_output;
    integer timeout;
    begin
        cap_got = 0;
        timeout = 0;
        @(posedge clk);
        while (!(m_valid && m_ready) && timeout < 100) begin
            @(posedge clk);
            timeout = timeout + 1;
        end
        if (timeout == 100) begin
            $display("WARNING Test %0d: capture_output timed out — m_valid never fired",
                     test_num);
        end else begin
            cap_pcie_addr = m_pcie_addr;
            cap_len_dw    = m_len_dw;
            cap_stream_id = m_stream_id;
            cap_split     = m_split_flag_out;
            cap_subidx    = m_sub_index_out;
            cap_tlast     = m_tlast_flag;
            cap_got       = 1;
        end
        // drain m_valid before returning
        if (m_valid) begin
            @(posedge clk);
            while (m_valid) @(posedge clk);
        end
    end
endtask

// ============================================================
// Task: send_and_capture
//
// Runs send_descriptor and capture_output in parallel using
// fork-join. This is REQUIRED because m_valid can appear in
// the same cycle as (or immediately after) s_ready — a
// sequential send-then-capture would miss the m_valid pulse
// while still inside send_descriptor.
// ============================================================
task send_and_capture;
    input [ADDR_W-1:0]   addr;
    input [LEN_W-1:0]    len;
    input [TUSER_W-1:0]  tuser;
    input                split_flag;
    input [1:0]          sub_index;
    begin
        fork
            send_descriptor(addr, len, tuser, split_flag, sub_index);
            capture_output;
        join
    end
endtask

// ============================================================
// Task: check_output
// ============================================================
task check_output;
    input [ADDR_W-1:0]  exp_addr;
    input [LEN_W-1:0]   exp_len;
    input               exp_sid;
    input               exp_split;
    input [1:0]         exp_subidx;
    input               exp_tlast;
    begin
        if (!cap_got) begin
            $display("FAIL Test %0d: no output received (capture timed out)",
                     test_num);
            fail_count = fail_count + 1;
        end else if (
            cap_pcie_addr !== exp_addr   ||
            cap_len_dw    !== exp_len    ||
            cap_stream_id !== exp_sid    ||
            cap_split     !== exp_split  ||
            cap_subidx    !== exp_subidx ||
            cap_tlast     !== exp_tlast
        ) begin
            $display("FAIL Test %0d: output mismatch", test_num);
            $display("  pcie_addr:  got %h, exp %h",   cap_pcie_addr, exp_addr);
            $display("  len_dw:     got %0d, exp %0d", cap_len_dw,    exp_len);
            $display("  stream_id:  got %0d, exp %0d", cap_stream_id, exp_sid);
            $display("  split_flag: got %0d, exp %0d", cap_split,     exp_split);
            $display("  sub_index:  got %0d, exp %0d", cap_subidx,    exp_subidx);
            $display("  tlast_flag: got %0d, exp %0d", cap_tlast,     exp_tlast);
            fail_count = fail_count + 1;
        end else begin
            $display("PASS Test %0d", test_num);
            pass_count = pass_count + 1;
        end
    end
endtask

// ============================================================
// Task: check_no_output
// Verifies m_valid stays low for N cycles.
// ============================================================
task check_no_output;
    input integer cycles;
    integer i;
    integer saw;
    begin
        saw = 0;
        for (i = 0; i < cycles; i = i + 1) begin
            @(posedge clk);
            if (m_valid) begin
                saw = 1;
                i   = cycles;
            end
        end
        if (saw) begin
            $display("FAIL Test %0d: unexpected m_valid after dropped descriptor",
                     test_num);
            fail_count = fail_count + 1;
        end else begin
            $display("PASS Test %0d", test_num);
            pass_count = pass_count + 1;
        end
    end
endtask

// ============================================================
// Main test sequence
// ============================================================
initial begin
    $dumpfile("tb_address_translation.vcd");
    $dumpvars(0, tb_address_translation);

    pass_count = 0;
    fail_count = 0;
    m_ready    = 1;

    do_reset;

    // ----------------------------------------------------------
    // Test 1: base address not programmed — xlat_miss
    // Neither stream has been programmed yet.
    // Expected: sticky_xlat_miss fires, no m_valid output.
    // ----------------------------------------------------------
    test_num = 1;
    $display("\n--- Test %0d: base addr not programmed (xlat_miss) ---",
             test_num);

    clear_sticky_errors;

    // send only — no output expected, no fork needed
    send_descriptor(
        64'h0,
        11'd32,
        make_tuser(1'b0, 24'h000100, 1'b0),
        1'b0,
        2'd0
    );

    // wait 3 cycles for sticky to latch
    repeat(3) @(posedge clk);

    if (!sticky_xlat_miss) begin
        $display("FAIL Test %0d: sticky_xlat_miss did not set", test_num);
        fail_count = fail_count + 1;
    end else begin
        check_no_output(5);
    end
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 2: stream_id=0 SmartNIC
    // base=0xDEAD_0000_0000_0000  offset=0x000200
    // expected pcie_addr=0xDEAD_0000_0000_0200
    // ----------------------------------------------------------
    test_num = 2;
    $display("\n--- Test %0d: stream_id=0 SmartNIC xlat ---", test_num);

    program_base_addr(0, 64'hDEAD_0000_0000_0000);

    send_and_capture(
        64'h0,
        11'd64,
        make_tuser(1'b0, 24'h000200, 1'b0),
        1'b0,
        2'd0
    );
    check_output(
        64'hDEAD_0000_0000_0200,
        11'd64,
        1'b0, 1'b0, 2'd0, 1'b0
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 3: stream_id=1 ML
    // base=0xCAFE_0000_0000_0000  offset=0x001000
    // expected pcie_addr=0xCAFE_0000_0000_1000
    // ----------------------------------------------------------
    test_num = 3;
    $display("\n--- Test %0d: stream_id=1 ML xlat ---", test_num);

    program_base_addr(1, 64'hCAFE_0000_0000_0000);

    send_and_capture(
        64'h0,
        11'd128,
        make_tuser(1'b1, 24'h001000, 1'b1),
        1'b0,
        2'd0
    );
    check_output(
        64'hCAFE_0000_0000_1000,
        11'd128,
        1'b1, 1'b0, 2'd0, 1'b1
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 4: session_offset=0
    // expected pcie_addr = base_addr[0] exactly
    // ----------------------------------------------------------
    test_num = 4;
    $display("\n--- Test %0d: session_offset=0 addr=base_addr ---", test_num);

    send_and_capture(
        64'h0,
        11'd16,
        make_tuser(1'b0, 24'h000000, 1'b0),
        1'b0,
        2'd0
    );
    check_output(
        64'hDEAD_0000_0000_0000,
        11'd16,
        1'b0, 1'b0, 2'd0, 1'b0
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 5: large session_offset
    // base=0xDEAD_0000_0000_0000  offset=0xFFFF00
    // expected=0xDEAD_0000_00FF_FF00
    // ----------------------------------------------------------
    test_num = 5;
    $display("\n--- Test %0d: large session_offset addition ---", test_num);

    send_and_capture(
        64'h0,
        11'd256,
        make_tuser(1'b0, 24'hFFFF00, 1'b1),
        1'b0,
        2'd0
    );
    check_output(
        64'hDEAD_0000_00FF_FF00,
        11'd256,
        1'b0, 1'b0, 2'd0, 1'b1
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 6: address overflow
    // base=0xFFFF_FFFF_FFFF_0000  offset=0x010000
    // sum overflows 64 bits — err_addr_ovf must fire
    // ----------------------------------------------------------
    test_num = 6;
    $display("\n--- Test %0d: address overflow (err_addr_ovf) ---", test_num);

    program_base_addr(0, 64'hFFFF_FFFF_FFFF_0000);
    clear_sticky_errors;

    send_descriptor(
        64'h0,
        11'd32,
        make_tuser(1'b0, 24'h010000, 1'b0),
        1'b0,
        2'd0
    );

    repeat(3) @(posedge clk);

    if (!sticky_addr_ovf) begin
        $display("FAIL Test %0d: sticky_addr_ovf did not set", test_num);
        fail_count = fail_count + 1;
    end else begin
        check_no_output(5);
    end
    wait_dut_idle;

    // restore base_addr[0]
    program_base_addr(0, 64'hDEAD_0000_0000_0000);

    // ----------------------------------------------------------
    // Test 7: split sub-descriptor — s_addr used directly
    // s_addr=0xDEAD_0000_0000_1000 (pre-computed by splitter)
    // session_offset=0x000800 (must be ignored when split=1)
    // expected pcie_addr = s_addr
    // ----------------------------------------------------------
    test_num = 7;
    $display("\n--- Test %0d: split sub-descriptor uses s_addr ---", test_num);

    send_and_capture(
        64'hDEAD_0000_0000_1000,
        11'd32,
        make_tuser(1'b0, 24'h000800, 1'b0),
        1'b1,
        2'd1
    );
    check_output(
        64'hDEAD_0000_0000_1000,
        11'd32,
        1'b0, 1'b1, 2'd1, 1'b0
    );
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 8: back-pressure from packetizer
    // m_ready=0 when descriptor is driven.
    // m_valid must stay high and output must be stable until
    // m_ready is released.
    // ----------------------------------------------------------
    test_num = 8;
    $display("\n--- Test %0d: back-pressure from packetizer ---", test_num);

    m_ready = 0;

    // send descriptor with m_ready=0
    send_descriptor(
        64'h0,
        11'd48,
        make_tuser(1'b0, 24'h000400, 1'b0),
        1'b0,
        2'd0
    );

    // wait for m_valid to appear (DUT should load output
    // even though m_ready=0)
    begin : t8_wait
        integer t8i;
        for (t8i = 0; t8i < 20; t8i = t8i + 1) begin
            @(posedge clk);
            if (m_valid) disable t8_wait;
        end
    end

    if (!m_valid) begin
        $display("FAIL Test %0d: m_valid never appeared under back-pressure",
                 test_num);
        fail_count = fail_count + 1;
    end else begin
        // hold back-pressure 5 more cycles — verify stability
        begin : t8_hold
            integer          t8j;
            reg [ADDR_W-1:0] t8_stable_addr;
            reg [LEN_W-1:0]  t8_stable_len;
            integer          t8_ok;
            t8_ok         = 1;
            t8_stable_addr = m_pcie_addr;
            t8_stable_len  = m_len_dw;

            for (t8j = 0; t8j < 5; t8j = t8j + 1) begin
                @(posedge clk);
                if (!m_valid ||
                    m_pcie_addr !== t8_stable_addr ||
                    m_len_dw    !== t8_stable_len) begin
                    $display("FAIL Test %0d: output unstable during bp cycle %0d",
                             test_num, t8j);
                    fail_count = fail_count + 1;
                    t8_ok = 0;
                end
            end

            if (t8_ok) begin
                // release back-pressure and verify correct value
                @(negedge clk); m_ready = 1;
                @(posedge clk);

                if (!m_valid || !m_ready) begin
                    $display("FAIL Test %0d: m_valid/m_ready not both high after release",
                             test_num);
                    fail_count = fail_count + 1;
                end else if (
                    m_pcie_addr !== 64'hDEAD_0000_0000_0400 ||
                    m_len_dw    !== 11'd48
                ) begin
                    $display("FAIL Test %0d: wrong value after bp release", test_num);
                    $display("  pcie_addr: got %h exp DEAD_0000_0000_0400",
                             m_pcie_addr);
                    $display("  len_dw:    got %0d exp 48", m_len_dw);
                    fail_count = fail_count + 1;
                end else begin
                    $display("PASS Test %0d", test_num);
                    pass_count = pass_count + 1;
                end
            end
        end
    end

    // drain m_valid
    @(posedge clk);
    while (m_valid) @(posedge clk);
    m_ready = 1;
    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 9: back-to-back — no idle bubble
    // Drive desc1 and desc2 without dropping s_valid between.
    // Capture both outputs and measure time between them.
    // Gap must be 1 clock (10ns) — no extra idle cycle.
    // ----------------------------------------------------------
    test_num = 9;
    $display("\n--- Test %0d: back-to-back no idle bubble ---", test_num);

    m_ready = 1;

    begin : t9_block
        reg [ADDR_W-1:0] t9_a1;
        reg [ADDR_W-1:0] t9_a2;
        time             t9_t1;
        time             t9_t2;
        integer          t9_gap;
        integer          t9_ok;
        t9_ok = 1;

        fork
            // thread 1: drive both descriptors back to back
            begin
                @(negedge clk);
                s_addr       = 64'h0;
                s_len_dw     = 11'd8;
                s_tuser      = make_tuser(1'b0, 24'h001000, 1'b0);
                s_split_flag = 1'b0;
                s_sub_index  = 2'd0;
                s_valid      = 1;

                @(posedge clk);
                while (!s_ready) @(posedge clk);

                // switch immediately — no gap
                @(negedge clk);
                s_addr   = 64'h0;
                s_len_dw = 11'd16;
                s_tuser  = make_tuser(1'b0, 24'h002000, 1'b1);

                @(posedge clk);
                while (!s_ready) @(posedge clk);

                @(negedge clk);
                s_valid = 0;
            end

            // thread 2: capture both outputs sequentially
            begin
                // capture output 1
                begin : t9_c1
                    integer t9i;
                    for (t9i = 0; t9i < 60; t9i = t9i + 1) begin
                        @(posedge clk);
                        if (m_valid && m_ready) begin
                            t9_a1 = m_pcie_addr;
                            t9_t1 = $time;
                            disable t9_c1;
                        end
                    end
                end

                // capture output 2
                begin : t9_c2
                    integer t9j;
                    for (t9j = 0; t9j < 60; t9j = t9j + 1) begin
                        @(posedge clk);
                        if (m_valid && m_ready) begin
                            t9_a2 = m_pcie_addr;
                            t9_t2 = $time;
                            disable t9_c2;
                        end
                    end
                end
            end
        join

        t9_gap = t9_t2 - t9_t1;

        if (t9_a1 !== 64'hDEAD_0000_0000_1000) begin
            $display("FAIL Test %0d: output1 addr got %h exp DEAD_0000_0000_1000",
                     test_num, t9_a1);
            fail_count = fail_count + 1;
            t9_ok = 0;
        end

        if (t9_a2 !== 64'hDEAD_0000_0000_2000) begin
            $display("FAIL Test %0d: output2 addr got %h exp DEAD_0000_0000_2000",
                     test_num, t9_a2);
            fail_count = fail_count + 1;
            t9_ok = 0;
        end

        if (t9_gap > 20) begin
            $display("FAIL Test %0d: idle bubble — gap=%0d ns (max 20ns)",
                     test_num, t9_gap);
            fail_count = fail_count + 1;
            t9_ok = 0;
        end

        if (t9_ok) begin
            $display("PASS Test %0d (gap=%0d ns)", test_num, t9_gap);
            pass_count = pass_count + 1;
        end
    end

    wait_dut_idle;

    // ----------------------------------------------------------
    // Test 10: cfg_base_addr_clr clears all entries
    // After clear, next descriptor must trigger xlat_miss.
    // ----------------------------------------------------------
    test_num = 10;
    $display("\n--- Test %0d: cfg_base_addr_clr resets all entries ---",
             test_num);

    clear_base_addrs;
    clear_sticky_errors;

    send_descriptor(
        64'h0,
        11'd32,
        make_tuser(1'b0, 24'h000100, 1'b0),
        1'b0,
        2'd0
    );

    repeat(3) @(posedge clk);

    if (!sticky_xlat_miss) begin
        $display("FAIL Test %0d: sticky_xlat_miss did not fire after clr",
                 test_num);
        fail_count = fail_count + 1;
    end else begin
        check_no_output(5);
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
    $display("FATAL: simulation timeout");
    $finish;
end

endmodule