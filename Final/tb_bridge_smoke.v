`timescale 1ns/1ps
module tb_bridge_smoke;

    localparam DATA_WIDTH = 512, KEEP_WIDTH = 64, OFFSET_WIDTH = 24;

    reg aclk = 0, arstn = 0, pcie_clk = 0, pcie_rstn = 0;
    always #1 aclk = ~aclk;
    always #1 pcie_clk = ~pcie_clk;   // zero-skew, matches project convention

    reg  [DATA_WIDTH-1:0] s0_tdata;
    reg  [KEEP_WIDTH-1:0] s0_tkeep;
    reg  s0_tlast, s0_tvalid;
    wire s0_tready;

    reg  [DATA_WIDTH-1:0] s1_tdata = 0;
    reg  [KEEP_WIDTH-1:0] s1_tkeep = 0;
    reg  s1_tlast = 0, s1_tvalid = 0;
    wire s1_tready;

    reg  [63:0] ch0_base_addr_in;
    reg  [2:0]  ch0_tc_in;
    reg         ch0_session_arm;
    reg  [63:0] ch1_base_addr_in = 0;
    reg  [2:0]  ch1_tc_in = 0;
    reg         ch1_session_arm = 0;

    wire [DATA_WIDTH-1:0] m_tdata;
    wire [KEEP_WIDTH-1:0] m_tkeep;
    wire m_tvalid, m_tlast;
    wire [OFFSET_WIDTH+1:0] m_tuser;
    reg  m_tready = 1'b1;

    bridge_top #(
        .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH), .OFFSET_WIDTH(OFFSET_WIDTH),
        .CH0_SESSION_LIMIT(24'hFFFFFF), .CH1_SESSION_LIMIT(24'hFFFFFF)
    ) dut (
        .aclk(aclk), .arstn(arstn), .pcie_clk(pcie_clk), .pcie_rstn(pcie_rstn),
        .s0_tdata(s0_tdata), .s0_tkeep(s0_tkeep), .s0_tlast(s0_tlast), .s0_tvalid(s0_tvalid), .s0_tready(s0_tready),
        .ch0_session_overflow(),
        .s1_tdata(s1_tdata), .s1_tkeep(s1_tkeep), .s1_tlast(s1_tlast), .s1_tvalid(s1_tvalid), .s1_tready(s1_tready),
        .ch1_session_overflow(),
        .weight0(8'd1), .weight1(8'd1),
        .ch0_base_addr_in(ch0_base_addr_in), .ch0_tc_in(ch0_tc_in), .ch0_session_arm(ch0_session_arm),
        .ch1_base_addr_in(ch1_base_addr_in), .ch1_tc_in(ch1_tc_in), .ch1_session_arm(ch1_session_arm),
        .m_tdata(m_tdata), .m_tkeep(m_tkeep), .m_tvalid(m_tvalid), .m_tlast(m_tlast), .m_tuser(m_tuser),
        .m_tready(m_tready),
        .err_tkeep_gap(), .err_burst_interleave(), .fifo_occupancy(), .fifo_full(), .fifo_hwm()
    );

    integer errors = 0;
    task check(input cond, input [255:0] msg);
        begin
            if (!cond) begin errors = errors + 1; $display("FAIL: %s", msg); end
            else $display("PASS: %s", msg);
        end
    endtask

    reg [DATA_WIDTH-1:0] beat0_data, beat1_data;

    initial begin
        s0_tdata = 0; s0_tkeep = 0; s0_tlast = 0; s0_tvalid = 0;
        ch0_base_addr_in = 64'hDEAD_BEEF_0000_1000; ch0_tc_in = 3'd0; ch0_session_arm = 0;
        beat0_data = {DATA_WIDTH{1'b1}} ^ {DATA_WIDTH{1'b0}}; // placeholder, set below
        beat0_data = 512'hAAAA_1111_2222_3333_4444_5555_6666_7777_8888_9999_AAAA_BBBB_CCCC_DDDD_EEEE_FFFF;
        beat1_data = 512'h1234_5678_9ABC_DEF0_0FED_CBA9_8765_4321_1122_3344_5566_7788_99AA_BBCC_DDEE_FF00;

        // reset
        repeat (4) @(posedge aclk);
        arstn = 1; pcie_rstn = 1;
        @(posedge aclk);

        // arm channel 0
        @(posedge pcie_clk); ch0_session_arm = 1;
        @(posedge pcie_clk); ch0_session_arm = 0;
        @(posedge pcie_clk); // LATCH -> ACTIVE lands here (session_valid should now be high)
        @(posedge pcie_clk);

        // beat 0: full tkeep, not last
        @(negedge aclk);
        s0_tdata = beat0_data; s0_tkeep = {KEEP_WIDTH{1'b1}}; s0_tlast = 0; s0_tvalid = 1;
        wait (s0_tready == 1'b1);
        @(posedge aclk); // beat 0 accepted here
        @(negedge aclk);

        // beat 1: full tkeep, TLAST
        s0_tdata = beat1_data; s0_tkeep = {KEEP_WIDTH{1'b1}}; s0_tlast = 1; s0_tvalid = 1;
        wait (s0_tready == 1'b1);
        @(posedge aclk); // beat 1 accepted here
        @(negedge aclk);
        s0_tvalid = 0;

        // capture both beats as they pop out of write_data_buffer, in order
        begin : capture
            integer seen;
            reg [DATA_WIDTH-1:0]     cap_data   [0:1];
            reg [OFFSET_WIDTH+1:0]   cap_tuser  [0:1];
            reg                      cap_tlast  [0:1];
            seen = 0;
            while (seen < 2) begin
                @(posedge pcie_clk);
                if (m_tvalid) begin
                    cap_data[seen]  = m_tdata;
                    cap_tuser[seen] = m_tuser;
                    cap_tlast[seen] = m_tlast;
                    seen = seen + 1;
                end
            end

            check(cap_data[0] === beat0_data, "beat0 data matches");
            check(cap_tuser[0][0] === 1'b0, "beat0 stream_id (bit0) == 0 for channel 0");
            check(cap_tuser[0][OFFSET_WIDTH:1] === 24'd0, "beat0 session_offset (bits[24:1]) == 0");
            check(cap_tuser[0][OFFSET_WIDTH+1] === 1'b0, "beat0 tlast_flag mirror (bit25) == 0");
            check(cap_tlast[0] === 1'b0, "beat0 m_tlast == 0");

            check(cap_data[1] === beat1_data, "beat1 data matches");
            check(cap_tuser[1][0] === 1'b0, "beat1 stream_id (bit0) == 0 for channel 0");
            check(cap_tuser[1][OFFSET_WIDTH:1] === 24'd64, "beat1 session_offset (bits[24:1]) == 64 (bytes from beat0)");
            check(cap_tuser[1][OFFSET_WIDTH+1] === 1'b1, "beat1 tlast_flag mirror (bit25) == 1");
            check(cap_tlast[1] === 1'b1, "beat1 m_tlast == 1");
        end

        repeat (10) @(posedge pcie_clk);

        if (errors == 0) $display("ALL PASS: bridge_top smoke test (interconnect_top -> write_data_buffer) clean end-to-end");
        else $display("%0d FAILURES", errors);
        $finish;
    end

    initial begin
        #2000;
        $display("TIMEOUT");
        $finish;
    end

endmodule
