`timescale 1ns/1ps
// ============================================================
// tb_tlp_packetizer -- S1 (rank 1)
//
// Revived against the current DW-stream interface:
//   req_valid/is_write/addr/length/rid/tag/ready,
//   wdata_tdata/tvalid/tready,
//   tlp_out_valid/data/sof/eof/ready.
//
// Checks DW0-DW3 header fields and payload for 1, 2, 16 and 64 DW
// writes, the FirstBE/LastBE rules for 1-DW writes at all four byte
// offsets, a 16-DW MRd (no data phase, EOF on HDR3), and random
// tlp_out_ready backpressure under a 16-DW write.
// ============================================================
module tb_tlp_packetizer;

    reg         clk, rst_n;
    reg         req_valid, req_is_write;
    reg  [63:0] req_addr;
    reg  [9:0]  req_length;
    reg  [15:0] req_rid;
    reg  [7:0]  req_tag;
    wire        req_ready;

    reg  [31:0] wdata_tdata;
    reg         wdata_tvalid;
    wire        wdata_tready;

    wire        tlp_out_valid;
    wire [31:0] tlp_out_data;
    wire        tlp_out_sof;
    wire        tlp_out_eof;
    reg         tlp_out_ready;

    integer errors;
    integer seed;

    tlp_packetizer dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_is_write(req_is_write),
        .req_addr(req_addr), .req_length(req_length),
        .req_rid(req_rid), .req_tag(req_tag), .req_ready(req_ready),
        .wdata_tdata(wdata_tdata), .wdata_tvalid(wdata_tvalid), .wdata_tready(wdata_tready),
        .tlp_out_valid(tlp_out_valid), .tlp_out_data(tlp_out_data),
        .tlp_out_sof(tlp_out_sof), .tlp_out_eof(tlp_out_eof), .tlp_out_ready(tlp_out_ready)
    );

    always #5 clk = ~clk;

    // -------- capture of every accepted output word --------
    reg [31:0] cap_data [0:255];
    reg        cap_sof  [0:255];
    reg        cap_eof  [0:255];
    integer    cap_cnt;

    always @(posedge clk) begin
        if (tlp_out_valid && tlp_out_ready) begin
            cap_data[cap_cnt] = tlp_out_data;
            cap_sof [cap_cnt] = tlp_out_sof;
            cap_eof [cap_cnt] = tlp_out_eof;
            cap_cnt = cap_cnt + 1;
        end
    end

    task reset_capture; begin cap_cnt = 0; end endtask

    task check32(input [31:0] got, input [31:0] exp, input [255:0] label);
        begin
            if (got !== exp) begin
                $display("FAIL %0s: got=0x%08h expected=0x%08h", label, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    task check1(input got, input exp, input [255:0] label);
        begin
            if (got !== exp) begin
                $display("FAIL %0s: got=%b expected=%b", label, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    // Send one data word: assert at negedge, then wait at posedge(s)
    // (read-only there -- never races the DUT's own posedge block)
    // for the cycle where wdata_tready is actually high.
    task send_data_word(input [31:0] data);
        begin
            @(negedge clk);
            wdata_tdata  = data;
            wdata_tvalid = 1;
            @(posedge clk);
            while (!wdata_tready) @(posedge clk);
        end
    endtask

    // Drive a write request + its data payload. Inputs change at
    // negedge so they never race the DUT's own posedge block.
    task do_write(input [63:0] addr, input [9:0] len, input [15:0] rid,
                  input [7:0] tag, input [31:0] d0, input [31:0] d1);
        integer k;
        begin
            @(negedge clk);
            req_valid = 1; req_is_write = 1;
            req_addr = addr; req_length = len; req_rid = rid; req_tag = tag;
            wait (req_ready);
            @(negedge clk);
            req_valid = 0;
            for (k = 0; k < len; k = k + 1) begin
                send_data_word((k == 0) ? d0 : (k == 1) ? d1 : (32'hC0DE_0000 + k));
            end
            @(negedge clk);
            wdata_tvalid = 0;
            @(negedge clk);
        end
    endtask

    task do_read(input [63:0] addr, input [9:0] len, input [15:0] rid, input [7:0] tag);
        begin
            @(negedge clk);
            req_valid = 1; req_is_write = 0;
            req_addr = addr; req_length = len; req_rid = rid; req_tag = tag;
            wait (req_ready);
            @(negedge clk);
            req_valid = 0;
            // No data phase on a read -- give the FSM enough cycles to
            // walk EVAL->HDR0->HDR1->HDR2->HDR3->NEXT->IDLE and emit
            // the 4 header words before we inspect the capture.
            repeat (10) @(negedge clk);
        end
    endtask

    reg [31:0] exp_dw0, exp_dw1, exp_dw2, exp_dw3;

    initial begin
        clk = 0; rst_n = 0; seed = 42;
        req_valid = 0; req_is_write = 0; req_addr = 0; req_length = 0;
        req_rid = 0; req_tag = 0;
        wdata_tdata = 0; wdata_tvalid = 0;
        tlp_out_ready = 1;
        errors = 0; cap_cnt = 0;

        @(negedge clk); @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // -------- Case A: 1-DW write, offset 00 --------
        reset_capture;
        do_write(64'h0000_0000_0000_1000, 10'd1, 16'h0100, 8'hAA, 32'hDEAD_BEEF, 32'h0);
        #1;
        exp_dw0 = 32'h6000_0001;                         // Fmt=011 Type=00000 Len=1
        exp_dw1 = {16'h0100, 8'hAA, 4'b0000, 4'b1111};    // last_be=0 (len==1), first_be (off00)
        exp_dw2 = 32'h0000_0000;
        exp_dw3 = 32'h0000_1000;
        check32(cap_data[0], exp_dw0, "A.DW0"); check1(cap_sof[0], 1, "A.DW0.sof");
        check32(cap_data[1], exp_dw1, "A.DW1");
        check32(cap_data[2], exp_dw2, "A.DW2");
        check32(cap_data[3], exp_dw3, "A.DW3");
        check32(cap_data[4], 32'hDEAD_BEEF, "A.data0"); check1(cap_eof[4], 1, "A.data0.eof");
        check1(cap_cnt == 5, 1, "A.wordcount");

        // -------- Case B: FirstBE at offset 01 (1-DW write) --------
        reset_capture;
        do_write(64'h0000_0000_0000_1001, 10'd1, 16'h0200, 8'hBB, 32'h1111_1111, 32'h0);
        #1;
        exp_dw1 = {16'h0200, 8'hBB, 4'b0000, 4'b1110};
        check32(cap_data[1], exp_dw1, "B.DW1(off01)");

        // -------- Case C: FirstBE at offset 10 --------
        reset_capture;
        do_write(64'h0000_0000_0000_1002, 10'd1, 16'h0300, 8'hCC, 32'h2222_2222, 32'h0);
        #1;
        exp_dw1 = {16'h0300, 8'hCC, 4'b0000, 4'b1100};
        check32(cap_data[1], exp_dw1, "C.DW1(off10)");

        // -------- Case D: FirstBE at offset 11 --------
        reset_capture;
        do_write(64'h0000_0000_0000_1003, 10'd1, 16'h0400, 8'hDD, 32'h3333_3333, 32'h0);
        #1;
        exp_dw1 = {16'h0400, 8'hDD, 4'b0000, 4'b1000};
        check32(cap_data[1], exp_dw1, "D.DW1(off11)");

        // -------- Case E: 2-DW write --------
        reset_capture;
        do_write(64'h0000_0000_0000_1004, 10'd2, 16'h0500, 8'hEE, 32'h1111_1111, 32'h2222_2222);
        #1;
        exp_dw0 = 32'h6000_0002;
        exp_dw1 = {16'h0500, 8'hEE, 4'b1111, 4'b1111}; // last_be=F (len!=1)
        exp_dw3 = 32'h0000_1004;
        check32(cap_data[0], exp_dw0, "E.DW0");
        check32(cap_data[1], exp_dw1, "E.DW1");
        check32(cap_data[3], exp_dw3, "E.DW3");
        check32(cap_data[4], 32'h1111_1111, "E.data0"); check1(cap_eof[4], 0, "E.data0.eof");
        check32(cap_data[5], 32'h2222_2222, "E.data1"); check1(cap_eof[5], 1, "E.data1.eof");
        check1(cap_cnt == 6, 1, "E.wordcount");

        // -------- Case F: 16-DW write --------
        reset_capture;
        do_write(64'h0000_0000_0000_2000, 10'd16, 16'h0600, 8'h11, 32'hA000_0000, 32'hA000_0001);
        #1;
        exp_dw0 = 32'h6000_0010;
        check32(cap_data[0], exp_dw0, "F.DW0(len16)");
        check1(cap_cnt == 4 + 16, 1, "F.wordcount");
        check1(cap_eof[4+16-1], 1, "F.lastdata.eof");

        // -------- Case G: 64-DW write (exact MAX_PAYLOAD_DW, no split) --------
        reset_capture;
        do_write(64'h0000_0000_0000_3000, 10'd64, 16'h0700, 8'h22, 32'hB000_0000, 32'hB000_0001);
        #1;
        exp_dw0 = 32'h6000_0040;
        check32(cap_data[0], exp_dw0, "G.DW0(len64)");
        check1(cap_cnt == 4 + 64, 1, "G.wordcount(no split)");
        check1(cap_eof[4+64-1], 1, "G.lastdata.eof");

        // -------- Case H: 16-DW MRd (read, no data phase, EOF on HDR3) --------
        reset_capture;
        do_read(64'h0000_0000_0000_4000, 10'd16, 16'h0800, 8'h33);
        #1;
        exp_dw0 = 32'h2000_0010; // Fmt=001 (4DW no data)
        check32(cap_data[0], exp_dw0, "H.DW0(read)");
        check1(cap_sof[0], 1, "H.DW0.sof");
        check1(cap_eof[3], 1, "H.DW3.eof(read terminates at header)");
        check1(cap_cnt == 4, 1, "H.wordcount(read, no data)");

        // -------- Case I: random tlp_out_ready backpressure on a 16-DW write --------
        reset_capture;
        fork
            do_write(64'h0000_0000_0000_5000, 10'd16, 16'h0900, 8'h44, 32'hC000_0000, 32'hC000_0001);
            begin
                repeat (60) begin
                    @(negedge clk);
                    tlp_out_ready = $random(seed) & 1'b1;
                end
                tlp_out_ready = 1;
            end
        join
        tlp_out_ready = 1;
        #1;
        check1(cap_cnt == 4 + 16, 1, "I.wordcount(backpressure, no drops/dupes)");
        check32(cap_data[0], 32'h6000_0010, "I.DW0(under backpressure)");
        check1(cap_eof[4+16-1], 1, "I.lastdata.eof(under backpressure)");

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