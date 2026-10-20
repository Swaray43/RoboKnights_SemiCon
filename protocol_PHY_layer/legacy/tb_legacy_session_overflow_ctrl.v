`timescale 1ns/1ps
module tb_legacy_session_overflow_ctrl;

    localparam OFFSET_WIDTH = 16;
    localparam SLICE_SIZE   = 8;   // small override so boundary is reachable fast

    reg clk = 0;
    reg rst_n;

    reg session_valid;
    reg [OFFSET_WIDTH-1:0] session_offset;
    reg beat_fire;
    reg overflow_flag_clear;

    wire overflow_abort;
    wire session_overflow_sticky;

    integer errors = 0;

    legacy_session_overflow_ctrl #(
        .OFFSET_WIDTH(OFFSET_WIDTH), .SLICE_SIZE(SLICE_SIZE)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .session_valid(session_valid),
        .session_offset(session_offset),
        .beat_fire(beat_fire),
        .overflow_abort(overflow_abort),
        .session_overflow_sticky(session_overflow_sticky),
        .overflow_flag_clear(overflow_flag_clear)
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

    initial begin
        rst_n = 0; session_valid = 0; session_offset = 0;
        beat_fire = 0; overflow_flag_clear = 0;
        tick; tick;
        rst_n = 1;
        tick;

        // ---- Case 1: well below bound, no overflow ----
        session_valid  = 1;
        session_offset = 8'd2;   // SLICE_SIZE-1 == 7, well clear
        beat_fire      = 1;
        tick;
        check(overflow_abort == 0, "case1: no overflow well below bound");
        check(session_overflow_sticky == 0, "case1: sticky stays clear");

        // ---- Case 2: offset == SLICE_SIZE-2 (== 6), beat fires -- should NOT abort ----
        session_offset = SLICE_SIZE - 2;  // 6
        beat_fire      = 1;
        tick;
        check(overflow_abort == 0, "case2: offset==SLICE_SIZE-2 does NOT abort (still legal)");
        check(session_overflow_sticky == 0, "case2: sticky still clear");

        // ---- Case 3: offset == SLICE_SIZE-1 (== 7), beat fires -- SHOULD abort ----
        session_offset = SLICE_SIZE - 1;  // 7
        beat_fire      = 1;
        tick;
        check(overflow_abort == 1, "case3: offset==SLICE_SIZE-1 DOES abort (boundary beat)");
        check(session_overflow_sticky == 1, "case3: sticky latches on overflow");

        // ---- Case 4: sticky stays high after triggering condition clears ----
        session_offset = 8'd0;   // back to a "safe" offset
        beat_fire      = 0;      // no beat firing at all now
        tick;
        tick;
        tick;
        check(overflow_abort == 0, "case4: overflow_abort deasserts once condition clears (combinational)");
        check(session_overflow_sticky == 1, "case4: sticky REMAINS latched across subsequent clean cycles");

        // ---- Case 5: host W1C clears the sticky flag ----
        overflow_flag_clear = 1;
        tick;
        overflow_flag_clear = 0;
        tick;
        check(session_overflow_sticky == 0, "case5: sticky cleared by host W1C");

        // ---- Case 6: overflow_abort only fires when session_valid=1 ----
        session_valid  = 0;
        session_offset = SLICE_SIZE - 1;  // boundary condition on offset...
        beat_fire      = 1;                // ...and a beat claims to fire...
        tick;
        check(overflow_abort == 0, "case6: overflow_abort suppressed when session_valid=0");
        check(session_overflow_sticky == 0, "case6: sticky not set when session invalid");
	
	// ---- Case 7: simultaneous would_exceed + overflow_flag_clear same cycle ----
        // Per DUT priority (if would_exceed / else if clear), set wins over clear.
        session_valid        = 1;
        session_offset       = SLICE_SIZE - 1;  // boundary condition, will assert would_exceed
        beat_fire            = 1;
        overflow_flag_clear  = 1;               // clear asserted the SAME cycle
        tick;
        check(overflow_abort == 1, "case7: overflow_abort still fires despite simultaneous clear");
        check(session_overflow_sticky == 1, "case7: sticky SET wins over clear in same cycle (set has priority)");

        // Follow-up: clear on its own, on a later clean cycle, still works normally
        beat_fire            = 0;
        session_offset       = 8'd0;
        overflow_flag_clear  = 1;
        tick;
        overflow_flag_clear  = 0;
        tick;
        check(session_overflow_sticky == 0, "case7b: clear succeeds once would_exceed condition is gone");

        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== %0d TEST(S) FAILED ===", errors);

        $display(" RESULT: %s", (errors == 0) ? "PASS" : "FAIL"); $finish;
    end

endmodule
