`timescale 1ns/1ps
module tb_cfg_base_addr_bridge;
    reg clk = 0, rst_n = 0;
    reg [1:0] wr_ch;
    reg [63:0] ch0, ch1;
    wire wr, id;
    wire [63:0] data;

    integer errors = 0;

    cfg_base_addr_bridge #(.ADDR_W(64)) dut (
        .clk(clk), .rst_n(rst_n),
        .cfg_base_addr_wr_ch(wr_ch),
        .cfg_base_addr_ch0(ch0),
        .cfg_base_addr_ch1(ch1),
        .cfg_base_addr_wr(wr),
        .cfg_base_addr_id(id),
        .cfg_base_addr_data(data)
    );

    always #5 clk = ~clk;

    task check(input exp_wr, input exp_id, input [63:0] exp_data, input [127:0] label);
        begin
            if (wr !== exp_wr || (exp_wr && (id !== exp_id || data !== exp_data))) begin
                $display("FAIL %s: wr=%b id=%b data=%h (expected wr=%b id=%b data=%h)",
                          label, wr, id, data, exp_wr, exp_id, exp_data);
                errors = errors + 1;
            end else begin
                $display("ok   %s: wr=%b id=%b data=%h", label, wr, id, data);
            end
        end
    endtask

    initial begin
        wr_ch = 2'b00; ch0 = 64'h0; ch1 = 64'h0;
        rst_n = 0;
        @(negedge clk); @(negedge clk);
        rst_n = 1;

        // ---- Case 1: channel 0 alone ----
        @(negedge clk);
        ch0 = 64'hAAAA_BBBB_0000_1000;
        wr_ch = 2'b01;
        @(negedge clk);           // posedge happened in between: dut captured wr_ch=01
        wr_ch = 2'b00;
        #1 check(1, 0, 64'hAAAA_BBBB_0000_1000, "ch0 alone");
        @(negedge clk); #1 check(0, 0, 64'hAAAA_BBBB_0000_1000, "ch0 alone - deasserts");

        // ---- Case 2: channel 1 alone ----
        ch1 = 64'h1111_2222_0000_2000;
        wr_ch = 2'b10;
        @(negedge clk);
        wr_ch = 2'b00;
        #1 check(1, 1, 64'h1111_2222_0000_2000, "ch1 alone");
        @(negedge clk); #1 check(0, 0, 64'h1111_2222_0000_2000, "ch1 alone - deasserts");

        // ---- Case 3: both channels arm in the same cycle ----
        ch0 = 64'hC0C0_C0C0_0000_3000;
        ch1 = 64'hD0D0_D0D0_0000_4000;
        wr_ch = 2'b11;
        @(negedge clk);
        wr_ch = 2'b00;
        #1 check(1, 0, 64'hC0C0_C0C0_0000_3000, "both - cycle N (ch0 first)");
        @(negedge clk);
        #1 check(1, 1, 64'hD0D0_D0D0_0000_4000, "both - cycle N+1 (ch1 queued)");
        @(negedge clk);
        #1 check(0, 0, 64'hD0D0_D0D0_0000_4000, "both - cycle N+2 (idle)");

        if (errors == 0) $display("RESULT: PASS");
        else              $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end
endmodule
