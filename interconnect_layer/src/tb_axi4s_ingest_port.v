`timescale 1ns/1ps

module tb_axi4s_ingest_port;

    localparam DATA_WIDTH = 64; // small for readability
    localparam KEEP_WIDTH = DATA_WIDTH/8;
    localparam OFFSET_WIDTH = 16;

    reg clk = 0;
    reg rstn;
    reg [DATA_WIDTH-1:0] s_tdata;
    reg [KEEP_WIDTH-1:0] s_tkeep;
    reg s_tlast, s_tvalid;
    wire s_tready;

    reg session_start;
    reg [OFFSET_WIDTH-1:0] session_limit;
    wire session_active, session_overflow;

    wire req;
    reg grant;
    reg ds_backpressure;

    wire [DATA_WIDTH-1:0] out_tdata;
    wire [KEEP_WIDTH-1:0] out_tkeep;
    wire out_tlast;
    wire [OFFSET_WIDTH-1:0] out_session_offset;
    wire accepted;

    integer errors = 0;

    axi4s_ingest_port #(
        .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH),
        .OFFSET_WIDTH(OFFSET_WIDTH), .CH_ID(0)
    ) dut (
        .clk(clk), .rstn(rstn),
        .s_tdata(s_tdata), .s_tkeep(s_tkeep), .s_tlast(s_tlast), .s_tvalid(s_tvalid), .s_tready(s_tready),
        .session_start(session_start), .session_limit(session_limit),
        .session_active(session_active), .session_overflow(session_overflow),
        .req(req), .grant(grant), .ds_backpressure(ds_backpressure),
        .out_tdata(out_tdata), .out_tkeep(out_tkeep), .out_tlast(out_tlast),
        .out_session_offset(out_session_offset), .accepted(accepted)
    );

    always #5 clk = ~clk;

    task do_reset;
        begin
            rstn = 0; s_tvalid = 0; s_tdata = 0; s_tkeep = {KEEP_WIDTH{1'b1}}; s_tlast = 0;
            session_start = 0; session_limit = 16'd64; grant = 0; ds_backpressure = 0;
            repeat (3) @(posedge clk);
            rstn = 1;
            @(posedge clk);
        end
    endtask

    integer i;
    reg [OFFSET_WIDTH-1:0] last_offset;

    initial begin
        $display("=== Test 1: session tracking across several beats ===");
        do_reset;
        session_start = 1;
        @(posedge clk); #1; session_start = 0;
        grant = 1;
        s_tvalid = 1; s_tdata = 64'hAAAA_AAAA_AAAA_AAAA; s_tkeep = {KEEP_WIDTH{1'b1}}; s_tlast = 0;

        for (i = 0; i < 3; i = i + 1) begin
            @(posedge clk);
            #1;
            $display("beat %0d: session_offset=%0d tready=%0d", i, out_session_offset, s_tready);
        end
        if (out_session_offset != (3 * KEEP_WIDTH)) begin
            $display("FAIL: expected session_offset=%0d after 3 accepted beats, got %0d", 3*KEEP_WIDTH, out_session_offset);
            errors = errors + 1;
        end else $display("PASS");

        $display("=== Test 2: TLAST ends the session ===");
        s_tlast = 1;
        @(posedge clk); #1;
        s_tvalid = 0; s_tlast = 0;
        @(posedge clk); #1;
        if (session_active !== 1'b0) begin
            $display("FAIL: session_active should drop after TLAST beat accepted");
            errors = errors + 1;
        end else $display("PASS");

        $display("=== Test 3: overflow guard aborts session before exceeding limit ===");
        do_reset;
        session_limit = 16'd12; // small limit: 1.5 beats at 8B/beat
        session_start = 1;
        @(posedge clk); #1; session_start = 0;
        grant = 1;
        s_tvalid = 1; s_tdata = 64'h1111_1111_1111_1111; s_tkeep = {KEEP_WIDTH{1'b1}}; s_tlast = 0;
        @(posedge clk); #1; // beat 1: offset 0->8, within limit(12)
        @(posedge clk); #1; // beat 2: would be 8->16, exceeds limit(12) -> overflow expected
        if (session_overflow !== 1'b1) begin
            $display("FAIL: expected session_overflow=1 when a beat would exceed session_limit");
            errors = errors + 1;
        end else $display("PASS (overflow=%0d, active=%0d, offset stayed at %0d)", session_overflow, session_active, out_session_offset);

        $display("=== Test 4: ds_backpressure holds TREADY low even when granted ===");
        do_reset;
        session_start = 1;
        @(posedge clk); #1; session_start = 0;
        grant = 1; ds_backpressure = 1;
        s_tvalid = 1;
        @(posedge clk); #1;
        if (s_tready !== 1'b0) begin
            $display("FAIL: TREADY should be held low while ds_backpressure asserted");
            errors = errors + 1;
        end else $display("PASS");

        if (errors == 0)
            $display("\nALL TESTS PASSED");
        else
            $display("\n%0d TEST(S) FAILED", errors);
        $finish;
    end

endmodule
