`timescale 1ns/1ps

module tb_cdc_fifo_async;

    localparam WIDTH = 16;
    localparam ADDR_WIDTH = 8; // depth 256 -- large enough that the write/read
                                // rate mismatch (7ns vs 13ns) never actually
                                // fills the FIFO across all 200 attempts, so
                                // every write succeeds instead of being
                                // blocked by wr_full partway through

    reg wr_clk = 0, rd_clk = 0;
    reg wr_rstn, rd_rstn;
    reg wr_en, rd_en;
    reg [WIDTH-1:0] wr_data;
    wire [WIDTH-1:0] rd_data;
    wire wr_full, wr_almost_full, rd_empty;

    // Two genuinely unrelated clock periods -- 7ns and 13ns, no common multiple nearby
    always #3.5 wr_clk = ~wr_clk;
    always #6.5 rd_clk = ~rd_clk;

    cdc_fifo_async #(.WIDTH(WIDTH), .ADDR_WIDTH(ADDR_WIDTH)) dut (
        .wr_clk(wr_clk), .wr_rstn(wr_rstn), .wr_en(wr_en), .wr_data(wr_data),
        .wr_full(wr_full), .wr_almost_full(wr_almost_full),
        .rd_clk(rd_clk), .rd_rstn(rd_rstn), .rd_en(rd_en), .rd_data(rd_data),
        .rd_empty(rd_empty)
    );

    integer i;
    integer errors = 0;
    reg [WIDTH-1:0] expected_q [0:255];
    integer push_idx = 0;
    integer pop_idx = 0;

    // Writer process
    initial begin
        wr_rstn = 0; wr_en = 0; wr_data = 0;
        repeat (5) @(posedge wr_clk);
        wr_rstn = 1;
        @(posedge wr_clk);

        for (i = 0; i < 200; i = i + 1) begin
            @(posedge wr_clk);
            #1; // settle after the edge -- avoid racing the DUT's own posedge sampling
            if (!wr_full) begin
                wr_en   = 1;
                wr_data = i[WIDTH-1:0];
                expected_q[push_idx] = i[WIDTH-1:0];
                push_idx = push_idx + 1;
            end else begin
                wr_en = 0;
            end
        end
        @(posedge wr_clk);
        #1; // settle BEFORE clearing wr_en -- without this, this blocking
            // assignment can race the DUT's own posedge sampling of wr_en
            // for the final write, silently dropping it
        wr_en = 0;
    end

    // Reader process
    initial begin
        rd_rstn = 0; rd_en = 0;
        repeat (5) @(posedge rd_clk);
        rd_rstn = 1;
        @(posedge rd_clk);

        forever begin
            @(posedge rd_clk);
            #1; // settle after the edge -- avoid racing the DUT's own posedge sampling
            // FWFT: rd_data is valid combinationally whenever !rd_empty,
            // BEFORE rd_en advances the pointer -- check in the same cycle.
            if (!rd_empty) begin
                rd_en = 1;
                if (rd_data !== expected_q[pop_idx]) begin
                    $display("FAIL at pop_idx=%0d: expected %0d got %0d", pop_idx, expected_q[pop_idx], rd_data);
                    errors = errors + 1;
                end
                pop_idx = pop_idx + 1;
            end else begin
                rd_en = 0;
            end
        end
    end

    initial begin
        #4700; // generous drain margin -- writer burst (~1.4us) plus time
                // for the reader to fully drain the worst-case backlog
                // (~90+ entries at the 13ns read rate) before checking
        $display("Pushed %0d words, popped %0d words", push_idx, pop_idx);
        if (push_idx != pop_idx) begin
            $display("FAIL: push/pop count mismatch (some data still in flight or lost)");
            errors = errors + 1;
        end
        if (errors == 0)
            $display("\nALL TESTS PASSED");
        else
            $display("\n%0d TEST(S) FAILED", errors);
        $finish;
    end

endmodule
