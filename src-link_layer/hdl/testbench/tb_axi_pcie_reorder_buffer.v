`timescale 1ns/1ps
// ============================================================
// tb_axi_pcie_reorder_buffer -- S7 (rank 9)
//
// File was tb_axi_pcie_reorder_buffer.v but its module was named
// tb_axi_backpressure, so scripted runs couldn't find the top.
// Renamed to match, and real checks added: out-of-order
// completions (received out of allocation order on the "wire")
// must still drain to the AXI master in tag/allocation order,
// with tlast correct on each slot's last beat, plus the original
// AXI backpressure (TREADY stall) coverage.
// ============================================================
module tb_axi_pcie_reorder_buffer;

    reg clk;
    reg rst_n;

    // Allocation
    reg        req_valid;
    reg [9:0]  req_tag;
    reg [9:0]  req_len_dw;
    wire       req_ready;

    // Ingress
    reg        rx_valid, rx_sof, rx_dvalid, rx_last_dw;
    reg [9:0]  rx_offset;
    reg [9:0]  rx_tag;
    reg [31:0] rx_data;

    // Egress AXI-S
    wire        m_axis_tvalid;
    wire [31:0] m_axis_tdata;
    wire        m_axis_tlast;
    wire [3:0]  m_axis_tkeep;
    wire        m_axis_tuser;
    reg         m_axis_tready;

    integer errors;

    axi_pcie_reorder_buffer dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_tag(req_tag), .req_len_dw(req_len_dw), .req_ready(req_ready),
        .rx_cpld_valid(rx_valid), .rx_cpld_sof(rx_sof), .rx_cpld_offset(rx_offset),
        .rx_cpld_tag(rx_tag), .rx_cpld_data(rx_data), .rx_cpld_dvalid(rx_dvalid),
        .rx_cpld_status(3'b0), .rx_cpld_last_dw(rx_last_dw),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tdata(m_axis_tdata),
        .m_axis_tlast(m_axis_tlast), .m_axis_tkeep(m_axis_tkeep),
        .m_axis_tuser(m_axis_tuser), .m_axis_tready(m_axis_tready)
    );

    always #5 clk = ~clk;

    task check_eq(input [31:0] got, input [31:0] exp, input [255:0] label);
        begin
            if (got !== exp) begin
                $display("FAIL %0s: got=0x%0h expected=0x%0h", label, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    // -------- capture every AXI output beat --------
    reg [31:0] cap_data [0:63];
    reg        cap_last [0:63];
    integer    cap_cnt;

    always @(posedge clk) begin
        if (m_axis_tvalid && m_axis_tready) begin
            cap_data[cap_cnt] = m_axis_tdata;
            cap_last[cap_cnt] = m_axis_tlast;
            cap_cnt = cap_cnt + 1;
        end
    end

    task alloc(input [9:0] tag, input [9:0] len);
        begin
            @(negedge clk);
            req_valid = 1; req_tag = tag; req_len_dw = len;
            @(posedge clk);
            while (!req_ready) @(posedge clk);
            @(negedge clk);
            req_valid = 0;
        end
    endtask

    // Deliver one completion DW for `tag` at dw offset `off`.
    task cpld_word(input [9:0] tag, input [9:0] off, input [31:0] data, input last);
        begin
            @(negedge clk);
            rx_valid = 1;
            rx_sof   = (off == 0);
            rx_offset = off;
            rx_tag   = tag;
            rx_data  = data;
            rx_dvalid = 1;
            rx_last_dw = last;
            @(negedge clk);
            rx_valid = 0; rx_sof = 0; rx_dvalid = 0; rx_last_dw = 0;
        end
    endtask

    initial begin
        clk = 0; rst_n = 0;
        req_valid = 0; req_tag = 0; req_len_dw = 0;
        rx_valid = 0; rx_sof = 0; rx_offset = 0; rx_tag = 0; rx_data = 0; rx_dvalid = 0; rx_last_dw = 0;
        m_axis_tready = 1;
        errors = 0; cap_cnt = 0;

        @(negedge clk); @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // ============================================================
        // Test 1: out-of-order wire arrival -> in-order AXI drain
        // ============================================================
        // Allocate tag A (2 DW) then tag B (2 DW), in that order.
        alloc(10'h0A0, 10'd2);
        alloc(10'h0B0, 10'd2);

        // Completion data for B arrives first on the wire...
        cpld_word(10'h0B0, 10'd0, 32'hB000_0000, 1'b0);
        cpld_word(10'h0B0, 10'd1, 32'hB000_0001, 1'b1);
        // ...then A's completion data arrives.
        cpld_word(10'h0A0, 10'd0, 32'hA000_0000, 1'b0);
        cpld_word(10'h0A0, 10'd1, 32'hA000_0001, 1'b1);

        repeat (10) @(negedge clk);

        // Expect A's 2 words first (allocation order), then B's 2 words,
        // even though B's data arrived on the wire first.
        check_eq(cap_cnt, 4, "T1: 4 completion beats drained");
        check_eq(cap_data[0], 32'hA000_0000, "T1: tag A word0 drains first (alloc order)");
        check_eq(cap_last[0], 0, "T1: tag A word0 not tlast");
        check_eq(cap_data[1], 32'hA000_0001, "T1: tag A word1");
        check_eq(cap_last[1], 1, "T1: tag A word1 is tlast (end of A)");
        check_eq(cap_data[2], 32'hB000_0000, "T1: tag B word0 drains second");
        check_eq(cap_last[2], 0, "T1: tag B word0 not tlast");
        check_eq(cap_data[3], 32'hB000_0001, "T1: tag B word1");
        check_eq(cap_last[3], 1, "T1: tag B word1 is tlast (end of B)");

        // ============================================================
        // Test 2: AXI backpressure (egress stall) mid-completion
        // ============================================================
        cap_cnt = 0;
        alloc(10'h020, 10'd4);
        cpld_word(10'h020, 10'd0, 32'hDDDD_0000, 1'b0);
        cpld_word(10'h020, 10'd1, 32'hDDDD_1111, 1'b0);
        cpld_word(10'h020, 10'd2, 32'hDDDD_2222, 1'b0);
        cpld_word(10'h020, 10'd3, 32'hDDDD_3333, 1'b1);

        wait (m_axis_tvalid == 1'b1);
        @(negedge clk);
        m_axis_tready = 0; // stall right as the completion starts draining
        repeat (4) @(negedge clk);
        m_axis_tready = 1;

        repeat (10) @(negedge clk);
        check_eq(cap_cnt, 4, "T2: all 4 beats eventually drained despite stall");
        check_eq(cap_data[0], 32'hDDDD_0000, "T2: beat0 intact after stall");
        check_eq(cap_data[3], 32'hDDDD_3333, "T2: beat3 intact after stall");
        check_eq(cap_last[3], 1, "T2: tlast correct after stall");

        if (errors == 0) $display("RESULT: PASS");
        else              $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #20000;
        $display("[ERROR] Watchdog timeout");
        $display("RESULT: FAIL (watchdog)");
        $finish;
    end

endmodule