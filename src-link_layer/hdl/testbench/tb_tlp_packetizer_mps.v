`timescale 1ns/1ps
// ============================================================
// tb_tlp_packetizer_mps -- S1 (rank 1), MPS-splitting variant
//
// MAX_PAYLOAD_DW lowered to 32 for quick testing. A single 80-DW
// write request must split into three back-to-back TLPs of
// length 32, 32, 16, with addr advancing by the chunk size each
// time and payload words staying in order with nothing dropped.
// ============================================================
module tb_tlp_packetizer_mps;

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

    tlp_packetizer #(
        .MAX_PAYLOAD_DW(10'd32)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_is_write(req_is_write),
        .req_addr(req_addr), .req_length(req_length),
        .req_rid(req_rid), .req_tag(req_tag), .req_ready(req_ready),
        .wdata_tdata(wdata_tdata), .wdata_tvalid(wdata_tvalid), .wdata_tready(wdata_tready),
        .tlp_out_valid(tlp_out_valid), .tlp_out_data(tlp_out_data),
        .tlp_out_sof(tlp_out_sof), .tlp_out_eof(tlp_out_eof), .tlp_out_ready(tlp_out_ready)
    );

    always #5 clk = ~clk;

    // -------- capture every accepted output word --------
    reg [31:0] cap_data [0:511];
    reg        cap_sof  [0:511];
    reg        cap_eof  [0:511];
    integer    cap_cnt;

    always @(posedge clk) begin
        if (tlp_out_valid && tlp_out_ready) begin
            cap_data[cap_cnt] = tlp_out_data;
            cap_sof [cap_cnt] = tlp_out_sof;
            cap_eof [cap_cnt] = tlp_out_eof;
            cap_cnt = cap_cnt + 1;
        end
    end

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

    integer k;

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

    initial begin
        clk = 0; rst_n = 0;
        req_valid = 0; req_is_write = 0; req_addr = 0; req_length = 0;
        req_rid = 0; req_tag = 0;
        wdata_tdata = 0; wdata_tvalid = 0;
        tlp_out_ready = 1;
        errors = 0; cap_cnt = 0;

        @(negedge clk); @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // Issue a single 80-DW write request.
        // Expected split: 32, 32, 16 DW TLPs (MAX_PAYLOAD_DW=32).
        @(negedge clk);
        req_valid = 1; req_is_write = 1;
        req_addr  = 64'h0000_1111_2222_0000;
        req_length = 10'd80;
        req_rid    = 16'h0100;
        req_tag    = 8'h0A;
        wait (req_ready);
        @(negedge clk);
        req_valid = 0;

        for (k = 0; k < 80; k = k + 1) begin
            send_data_word(32'hD000_0000 + k);
        end
        @(negedge clk);
        wdata_tvalid = 0;
        @(negedge clk);
        #1;

        // Total words: (4 hdr + 32 data) + (4 hdr + 32 data) + (4 hdr + 16 data) = 92
        check1(cap_cnt == 92, 1, "total word count (3 TLPs, 32/32/16)");

        // TLP 1: header at [0..3], length=32, addr base
        check32(cap_data[0], 32'h6000_0020, "TLP1.DW0 (len=32)");
        check1(cap_sof[0], 1, "TLP1.DW0.sof");
        check32(cap_data[3], 64'h0000_1111_2222_0000, "TLP1.DW3 (addr low)");
        check32(cap_data[4], 32'hD000_0000, "TLP1.data0");
        check32(cap_data[4+31], 32'hD000_0000 + 31, "TLP1.data31");
        check1(cap_eof[4+31], 1, "TLP1.data31.eof");

        // TLP 2: header at [36..39], length=32, addr advanced by 32 DW (128 bytes)
        check32(cap_data[36], 32'h6000_0020, "TLP2.DW0 (len=32)");
        check1(cap_sof[36], 1, "TLP2.DW0.sof");
        check32(cap_data[39], 64'h0000_1111_2222_0080, "TLP2.DW3 (addr + 128 bytes)");
        check32(cap_data[40], 32'hD000_0000 + 32, "TLP2.data0");
        check32(cap_data[40+31], 32'hD000_0000 + 63, "TLP2.data31");
        check1(cap_eof[40+31], 1, "TLP2.data31.eof");

        // TLP 3: header at [72..75], length=16, addr advanced by another 32 DW
        check32(cap_data[72], 32'h6000_0010, "TLP3.DW0 (len=16)");
        check1(cap_sof[72], 1, "TLP3.DW0.sof");
        check32(cap_data[75], 64'h0000_1111_2222_0100, "TLP3.DW3 (addr + 256 bytes)");
        check32(cap_data[76], 32'hD000_0000 + 64, "TLP3.data0");
        check32(cap_data[76+15], 32'hD000_0000 + 79, "TLP3.data15");
        check1(cap_eof[76+15], 1, "TLP3.data15.eof");

        // req_ready should go back high only after all 3 TLPs are drained
        check1(req_ready, 1, "req_ready high again after full split completes");

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