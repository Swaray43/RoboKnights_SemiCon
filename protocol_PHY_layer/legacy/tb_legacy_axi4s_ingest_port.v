`timescale 1ns/1ps
module tb_legacy_axi4s_ingest_port;

    localparam DATA_WIDTH   = 512;
    localparam STREAM_ID    = 1;
    localparam OFFSET_WIDTH = 16;
    localparam SLICE_SIZE   = 16'hFFFF;

    reg clk = 0;
    reg rst_n;

    reg session_valid;
    wire session_end;

    reg overflow_abort;
    reg downstream_full;

    reg  [DATA_WIDTH-1:0] s_tdata;
    reg                   s_tvalid;
    reg                   s_tlast;
    wire                  s_tready;

    wire [DATA_WIDTH-1:0] m_tdata;
    wire                  m_tvalid;
    wire                  m_tlast;
    wire [0:0]            m_tuser_stream_id;
    wire [OFFSET_WIDTH-1:0] m_tuser_session_offset;
    wire [OFFSET_WIDTH-1:0] session_offset_dbg;

    integer errors = 0;

    legacy_axi4s_ingest_port #(
        .DATA_WIDTH(DATA_WIDTH), .STREAM_ID(STREAM_ID),
        .OFFSET_WIDTH(OFFSET_WIDTH), .SLICE_SIZE(SLICE_SIZE)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .session_valid(session_valid), .session_end(session_end),
        .overflow_abort(overflow_abort), .downstream_full(downstream_full),
        .s_tdata(s_tdata), .s_tvalid(s_tvalid), .s_tlast(s_tlast), .s_tready(s_tready),
        .m_tdata(m_tdata), .m_tvalid(m_tvalid), .m_tlast(m_tlast),
        .m_tuser_stream_id(m_tuser_stream_id),
        .m_tuser_session_offset(m_tuser_session_offset),
        .session_offset_dbg(session_offset_dbg)
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

    // Drive one beat and return after it's accepted (or give up after N cycles)
    task send_beat(input [DATA_WIDTH-1:0] data, input tlast);
        integer guard;
        begin
            s_tdata  = data;
            s_tvalid = 1;
            s_tlast  = tlast;
            guard = 0;
            while (!(s_tvalid && s_tready) && guard < 50) begin
                tick;
                guard = guard + 1;
            end
            tick; // consume the accepted cycle
            s_tvalid = 0;
            s_tlast  = 0;
        end
    endtask

    initial begin
        rst_n = 0; session_valid = 0; overflow_abort = 0; downstream_full = 0;
        s_tdata = 0; s_tvalid = 0; s_tlast = 0;
        tick; tick;
        rst_n = 1;
        tick;

        // ---- Case 2: s_tready low when session_valid=0 ----
        s_tvalid = 1; s_tdata = 64'hAAAA;
        tick;
        check(s_tready == 0, "case2: s_tready low when session not valid");
        s_tvalid = 0;
        tick;

        // ---- Case 1: normal streaming, offset increments, TUSER correct ----
        session_valid = 1;
        tick;
        check(session_offset_dbg == 0, "case1: offset starts at 0 on session start");

        send_beat(512'hDEAD_0001, 0);
        check(m_tvalid == 1, "case1: m_tvalid asserted after beat 0 accepted");
        check(m_tdata == 512'hDEAD_0001, "case1: m_tdata matches beat 0");
        check(m_tuser_stream_id == STREAM_ID[0], "case1: stream_id correct");
        check(m_tuser_session_offset == 16'd0, "case1: TUSER offset==0 for beat 0");

        send_beat(512'hDEAD_0002, 0);
        check(m_tuser_session_offset == 16'd1, "case1: TUSER offset==1 for beat 1");
        check(session_offset_dbg == 16'd2, "case1: internal offset counter now 2");

        // ---- Case 3: s_tready drops when downstream_full, mid-session ----
        downstream_full = 1;
        s_tdata = 512'hFEED; s_tvalid = 1; s_tlast = 0;
        tick;
        check(s_tready == 0, "case3: s_tready low when downstream_full asserted");
        downstream_full = 0;
        s_tvalid = 0;
        tick;

        // ---- Case 4: TLAST beat asserts session_end for exactly one cycle ----
        send_beat(512'hFACE_0003, 1'b1);   // TLAST beat
        check(session_end == 1, "case4: session_end asserted same cycle beat accepted w/ TLAST");
        tick;
        check(session_end == 0, "case4: session_end deasserts one cycle later");

        // ---- Case 6: offset resets to 0 when session_valid drops ----
        session_valid = 0;
        tick;
        check(session_offset_dbg == 16'd0, "case6: offset resets when session_valid drops");

        // ---- Case 5: overflow_abort forces session_end, blocks s_tready ----
        session_valid = 1;
        tick;
        s_tdata = 512'hB00; s_tvalid = 1; s_tlast = 0;
        overflow_abort = 1;
        tick;
        check(s_tready == 0, "case5: s_tready blocked by overflow_abort even though session_valid=1");
        check(session_end == 1, "case5: overflow_abort forces session_end");
        overflow_abort = 0;
        s_tvalid = 0;
        tick;
        check(session_end == 0, "case5: session_end clears after overflow_abort deasserts");

        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== %0d TEST(S) FAILED ===", errors);

        $stop;
    end

endmodule
