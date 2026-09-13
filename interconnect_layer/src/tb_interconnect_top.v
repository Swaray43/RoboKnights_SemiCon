`timescale 1ns/1ps

module tb_interconnect_top;

    // ---- everything derived from these -- change once, applies everywhere ----
    localparam DATA_WIDTH    = 512;          // production width: 64 bytes/beat
    localparam KEEP_WIDTH    = DATA_WIDTH/8;
    localparam OFFSET_WIDTH  = 16;
    localparam CREDIT_WIDTH  = 8;
    localparam FIFO_ADDR_WIDTH = 4;          // matches dut instantiation below
    localparam NUM_BEATS     = 64;           // beats sent per channel -- the one number to change

    // ---- bit-field positions for tag/ctr, derived from DATA_WIDTH, not hardcoded ----
    localparam TAG_HI = DATA_WIDTH-1;
    localparam TAG_LO = DATA_WIDTH-8;
    localparam CTR_HI = DATA_WIDTH-9;
    localparam CTR_LO = DATA_WIDTH-40;
    localparam PAD_WIDTH = DATA_WIDTH-8-32;  // remaining bits after tag+ctr

    // ---- FIFO packed-word layout, derived from interconnect_top's own packing
    //      order {xbar_data, xbar_keep, xbar_id, xbar_offset, xbar_last} ----
    localparam FIFO_WORD_WIDTH = DATA_WIDTH + KEEP_WIDTH + 1 + OFFSET_WIDTH + 1;
    localparam FW_DATA_LO   = FIFO_WORD_WIDTH - DATA_WIDTH;
    localparam FW_KEEP_HI   = FW_DATA_LO - 1;
    localparam FW_KEEP_LO   = FW_DATA_LO - KEEP_WIDTH;
    localparam FW_ID_BIT    = FW_KEEP_LO - 1;
    localparam FW_OFFSET_LO = 1;
    localparam FW_LAST_BIT  = 0;

    reg aclk = 0, pcie_clk = 0;
    reg arstn, pcie_rstn;

    reg  [DATA_WIDTH-1:0] s0_tdata, s1_tdata;
    reg  [KEEP_WIDTH-1:0] s0_tkeep, s1_tkeep;
    reg  s0_tlast, s1_tlast, s0_tvalid, s1_tvalid;
    wire s0_tready, s1_tready;

    reg ch0_session_start, ch1_session_start;
    reg [OFFSET_WIDTH-1:0] ch0_session_limit, ch1_session_limit;
    wire ch0_session_active, ch1_session_active, ch0_session_overflow, ch1_session_overflow;

    reg [CREDIT_WIDTH-1:0] weight0, weight1;

    wire [DATA_WIDTH-1:0] m_tdata;
    wire [KEEP_WIDTH-1:0] m_tkeep;
    wire [OFFSET_WIDTH:0] m_tuser;
    wire m_stream_id = m_tuser[OFFSET_WIDTH];
    wire [OFFSET_WIDTH-1:0] m_session_offset = m_tuser[OFFSET_WIDTH-1:0];
    wire m_tlast, m_tvalid;
    reg  m_tready;

    integer errors = 0;
    integer i;

    interconnect_top #(
        .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH), .OFFSET_WIDTH(OFFSET_WIDTH),
        .FIFO_ADDR_WIDTH(FIFO_ADDR_WIDTH), .CREDIT_WIDTH(CREDIT_WIDTH), .STARVE_CYCLES(32)
    ) dut (
        .aclk(aclk), .arstn(arstn),
        .s0_tdata(s0_tdata), .s0_tkeep(s0_tkeep), .s0_tlast(s0_tlast), .s0_tvalid(s0_tvalid), .s0_tready(s0_tready),
        .ch0_session_start(ch0_session_start), .ch0_session_limit(ch0_session_limit),
        .ch0_session_active(ch0_session_active), .ch0_session_overflow(ch0_session_overflow),
        .s1_tdata(s1_tdata), .s1_tkeep(s1_tkeep), .s1_tlast(s1_tlast), .s1_tvalid(s1_tvalid), .s1_tready(s1_tready),
        .ch1_session_start(ch1_session_start), .ch1_session_limit(ch1_session_limit),
        .ch1_session_active(ch1_session_active), .ch1_session_overflow(ch1_session_overflow),
        .weight0(weight0), .weight1(weight1),
        .pcie_clk(pcie_clk), .pcie_rstn(pcie_rstn),
        .m_tdata(m_tdata), .m_tkeep(m_tkeep), .m_tuser(m_tuser),
        .m_tlast(m_tlast), .m_tvalid(m_tvalid), .m_tready(m_tready)
    );

    // ---- both clocks identical: 1ns period, zero skew, as requested ----
    always #0.5 aclk = ~aclk;
    always #0.5 pcie_clk = ~pcie_clk;

    integer sent0, sent1, recv0, recv1;
    integer expected_offset0, expected_offset1;

    // ---- full-detail per-channel ingestion tracking, sized off NUM_BEATS ----
    integer s0_present_t [0:NUM_BEATS-1], s0_accept_t [0:NUM_BEATS-1];
    reg [DATA_WIDTH-1:0] s0_data [0:NUM_BEATS-1];
    integer s1_present_t [0:NUM_BEATS-1], s1_accept_t [0:NUM_BEATS-1];
    reg [DATA_WIDTH-1:0] s1_data [0:NUM_BEATS-1];

    initial begin
        $dumpfile("startup_trace.vcd");
        $dumpvars(0, tb_interconnect_top);
    end

    initial begin
        arstn = 0; pcie_rstn = 0;
        s0_tvalid = 0; s1_tvalid = 0;
        s0_tkeep = {KEEP_WIDTH{1'b1}}; s1_tkeep = {KEEP_WIDTH{1'b1}};
        s0_tlast = 0; s1_tlast = 0;
        ch0_session_start = 0; ch1_session_start = 0;
        ch0_session_limit = {OFFSET_WIDTH{1'b1}}; ch1_session_limit = {OFFSET_WIDTH{1'b1}};
        weight0 = 8'd1; weight1 = 8'd1;
        m_tready = 1;
        sent0 = 0; sent1 = 0; recv0 = 0; recv1 = 0;
        expected_offset0 = 0; expected_offset1 = 0;

        repeat (5) @(posedge aclk);
        arstn = 1; pcie_rstn = 1;
        @(posedge aclk); #0.1;
        ch0_session_start = 1; ch1_session_start = 1;
        @(posedge aclk); #0.1;
        ch0_session_start = 0; ch1_session_start = 0;
    end

    // ---- ch0 driver: NUM_BEATS beats, tag data with channel + counter ----
    initial begin : ch0_drive
        wait (arstn == 1);
        @(posedge aclk); #0.1;
        s0_tvalid = 1;
        while (sent0 < NUM_BEATS) begin
            s0_tdata = {8'hC0, sent0[31:0], {PAD_WIDTH{1'b0}}};
            s0_present_t[sent0] = $time;
            @(posedge aclk);
            // NOTE: deliberately NO settle delay here, unlike elsewhere in
            // this file. This check must stay aligned with cdc_fifo_async's
            // own same-edge, pre-NBA sampling of grant/tready -- adding a
            // delay here (as we do for reading already-settled DUT outputs
            // elsewhere) made the driver see tready's post-NBA value one
            // delta-cycle before the FIFO's own write logic did, causing it
            // to advance to the next beat's data before that edge's write
            // actually committed -- silently corrupting exactly the first
            // beat after every grant transition. Confirmed via VCD trace
            // and fixed by removing this delay specifically.
            if (s0_tready) begin
                s0_accept_t[sent0] = $time;
                s0_data[sent0]     = s0_tdata;
                sent0 = sent0 + 1;
            end
        end
        s0_tvalid = 0;
    end

    // ---- ch1 driver: NUM_BEATS beats ----
    initial begin : ch1_drive
        wait (arstn == 1);
        @(posedge aclk); #0.1;
        s1_tvalid = 1;
        while (sent1 < NUM_BEATS) begin
            s1_tdata = {8'hC1, sent1[31:0], {PAD_WIDTH{1'b0}}};
            s1_present_t[sent1] = $time;
            @(posedge aclk);
            // see matching note in ch0_drive above
            if (s1_tready) begin
                s1_accept_t[sent1] = $time;
                s1_data[sent1]     = s1_tdata;
                sent1 = sent1 + 1;
            end
        end
        s1_tvalid = 0;
    end

    // ---- receiver on pcie_clk: check tagging, offsets, mutual exclusion ----
    reg [7:0] tag;
    reg [31:0] ctr;
    initial begin
        wait (pcie_rstn == 1);
        forever begin
            @(posedge pcie_clk);
            #0.1;
            if (m_tvalid && m_tready) begin
                tag = m_tdata[TAG_HI:TAG_LO];
                ctr = m_tdata[CTR_HI:CTR_LO];
                if (m_stream_id == 1'b0) begin
                    if (tag !== 8'hC0) begin
                        $display("FAIL: m_stream_id=0 but data tag=%02h (expected C0)", tag);
                        errors = errors + 1;
                    end
                    if (m_session_offset !== expected_offset0) begin
                        $display("FAIL ch0: expected session_offset=%0d got %0d", expected_offset0, m_session_offset);
                        errors = errors + 1;
                    end
                    expected_offset0 = expected_offset0 + KEEP_WIDTH;
                    recv0 = recv0 + 1;
                end else begin
                    if (tag !== 8'hC1) begin
                        $display("FAIL: m_stream_id=1 but data tag=%02h (expected C1)", tag);
                        errors = errors + 1;
                    end
                    if (m_session_offset !== expected_offset1) begin
                        $display("FAIL ch1: expected session_offset=%0d got %0d", expected_offset1, m_session_offset);
                        errors = errors + 1;
                    end
                    expected_offset1 = expected_offset1 + KEEP_WIDTH;
                    recv1 = recv1 + 1;
                end
            end
        end
    end

    // ---- MASTER LOG + counters, all derived from actual FIFO pointer state ----
    // Counting/logging is gated on dut.u_fifo.wr_ptr_bin / rd_ptr_bin ACTUALLY
    // changing value, not on the combinational xbar_valid/fifo_wr_en signal --
    // those can pulse transiently within a cycle without a real write ever
    // committing (found and confirmed via VCD trace in this same debugging
    // session), so gating on the pointer's own committed state is what makes
    // these counts trustworthy without needing a VCD cross-check every time.
    integer master_log;
    integer fifo_wr_log, fifo_rd_log;
    integer wr_log_idx = 0, rd_log_idx = 0;
    integer wr_time_arr [0:2*NUM_BEATS-1];
    reg [DATA_WIDTH-1:0] wr_data_arr [0:2*NUM_BEATS-1];
    integer wr_stream_arr [0:2*NUM_BEATS-1];
    reg [DATA_WIDTH-1:0] rd_data_arr [0:2*NUM_BEATS-1];
    integer rd_stream_arr [0:2*NUM_BEATS-1];
    integer rd_time_arr [0:2*NUM_BEATS-1];
    integer ch0_first_acc_t = -1, ch0_last_acc_t = -1, ch0_acc_cnt = 0;
    integer ch1_first_acc_t = -1, ch1_last_acc_t = -1, ch1_acc_cnt = 0;
    integer mvalid_first_high_t = -1, mvalid_last_drop_t = -1;
    reg m_tvalid_d = 0;
    integer prev_wr_ptr = 0, prev_rd_ptr = 0;
    integer wr_cnt = 0, rd_cnt = 0, accepted0_cnt = 0, accepted1_cnt = 0;
    reg [FIFO_WORD_WIDTH-1:0] committed_word;

    initial begin
        master_log = $fopen("interconnect_master_log.txt", "w");
        fifo_wr_log = $fopen("fifo_write_log.txt", "w");
        fifo_rd_log = $fopen("fifo_read_log.txt", "w");
        $fdisplay(master_log, "==== INTERCONNECT LAYER -- MASTER LOG ====");
        $fdisplay(master_log, "NUM_BEATS=%0d per channel, DATA_WIDTH=%0d (%0d bytes/beat), aclk=pcie_clk=1ns period.", NUM_BEATS, DATA_WIDTH, KEEP_WIDTH);
        $fdisplay(master_log, "Write/read counts are gated on the FIFO's own wr_ptr_bin/rd_ptr_bin actually changing,");
        $fdisplay(master_log, "not on the combinational write-enable signal, so these counts are trustworthy as-is.\n");
        $fdisplay(fifo_wr_log, "idx\ttime_ns\tstream_id\ttag\tctr\tsession_offset\ttlast\twr_ptr_after");
        $fdisplay(fifo_rd_log, "idx\ttime_ns\tstream_id\ttag\tctr\tsession_offset\ttlast\trd_ptr_after");
    end

    // ---- per-channel ingestion window (first/last real accepted beat) ----
    always @(posedge aclk) begin
        #0.1;
        if (dut.ch0_accepted) begin
            if (ch0_first_acc_t == -1) ch0_first_acc_t = $time;
            ch0_last_acc_t = $time;
            ch0_acc_cnt = ch0_acc_cnt + 1;
            accepted0_cnt = accepted0_cnt + 1;
        end
        if (dut.ch1_accepted) begin
            if (ch1_first_acc_t == -1) ch1_first_acc_t = $time;
            ch1_last_acc_t = $time;
            ch1_acc_cnt = ch1_acc_cnt + 1;
            accepted1_cnt = accepted1_cnt + 1;
        end
    end

    // ---- output bus (m_tdata/m_tvalid) true validity window ----
    always @(posedge pcie_clk) begin
        #0.1;
        if (m_tvalid && !m_tvalid_d && mvalid_first_high_t == -1)
            mvalid_first_high_t = $time;
        if (m_tvalid_d && !m_tvalid)
            mvalid_last_drop_t = $time;
        m_tvalid_d <= m_tvalid;
    end

    // ---- write-side: log only when wr_ptr_bin genuinely advances ----
    always @(posedge aclk) begin
        #0.1;
        if (dut.u_fifo.wr_ptr_bin !== prev_wr_ptr) begin
            wr_cnt = wr_cnt + 1;
            wr_log_idx = wr_log_idx + 1;
            // read back the slot that was JUST written (prev_wr_ptr's address) --
            // this is the true committed data, immune to any transient glitch
            // on the combinational xbar_data/xbar_valid bus.
            committed_word = dut.u_fifo.mem[prev_wr_ptr[FIFO_ADDR_WIDTH-1:0]];
            wr_time_arr[wr_log_idx]   = $time;
            wr_data_arr[wr_log_idx]   = committed_word[FIFO_WORD_WIDTH-1 -: DATA_WIDTH];
            wr_stream_arr[wr_log_idx] = committed_word[FW_ID_BIT];
            $fdisplay(fifo_wr_log, "%0d\t%0t\t%0d\t0x%02h\t%0d\t%0d\t%0d\t%0d",
                wr_log_idx, $time, committed_word[FW_ID_BIT],
                committed_word[FIFO_WORD_WIDTH-1 -: 8],
                committed_word[FIFO_WORD_WIDTH-9 -: 32],
                committed_word[FW_ID_BIT-1 -: OFFSET_WIDTH],
                committed_word[FW_LAST_BIT],
                dut.u_fifo.wr_ptr_bin);
            prev_wr_ptr = dut.u_fifo.wr_ptr_bin;
        end
    end

    // ---- read-side: log only when rd_ptr_bin genuinely advances ----
    always @(posedge pcie_clk) begin
        #0.1;
        if (dut.u_fifo.rd_ptr_bin !== prev_rd_ptr) begin
            rd_cnt = rd_cnt + 1;
            rd_log_idx = rd_log_idx + 1;
            rd_time_arr[rd_log_idx]   = $time;
            rd_data_arr[rd_log_idx]   = m_tdata;
            rd_stream_arr[rd_log_idx] = m_stream_id;
            $fdisplay(fifo_rd_log, "%0d\t%0t\t%0d\t0x%02h\t%0d\t%0d\t%0d\t%0d",
                rd_log_idx, $time, m_stream_id, m_tdata[TAG_HI:TAG_LO], m_tdata[CTR_HI:CTR_LO],
                m_session_offset, m_tlast, dut.u_fifo.rd_ptr_bin);
            $fdisplay(master_log, "BEAT\t%0d\tch%0d\tctr=%0d\twrite_t_ns=%0d\tread_t_ns=%0d\tcdc_latency_ns=%0d\tsession_offset=%0d\ttlast=%0d",
                rd_log_idx, m_stream_id, m_tdata[CTR_HI:CTR_LO],
                wr_time_arr[rd_log_idx], $time, $time - wr_time_arr[rd_log_idx],
                m_session_offset, m_tlast);
            prev_rd_ptr = dut.u_fifo.rd_ptr_bin;
        end
    end

    initial begin
        wait (sent0 >= NUM_BEATS && sent1 >= NUM_BEATS);
        repeat (500) @(posedge pcie_clk);
        $display("sent0=%0d sent1=%0d recv0=%0d recv1=%0d", sent0, sent1, recv0, recv1);
        $display("Pointer-derived ground truth: accepted0_cnt=%0d accepted1_cnt=%0d wr_cnt=%0d rd_cnt=%0d", accepted0_cnt, accepted1_cnt, wr_cnt, rd_cnt);

        if (wr_cnt != rd_cnt) begin
            $display("FAIL: FIFO write count vs read count mismatch");
            errors = errors + 1;
        end
        if ((recv0 + recv1) != rd_cnt) begin
            $display("FAIL: received beats vs FIFO read count mismatch -- data lost or duplicated inside the pipe");
            errors = errors + 1;
        end
        if (recv0 < NUM_BEATS/3 || recv1 < NUM_BEATS/3) begin
            $display("FAIL: expected roughly balanced 50/50 traffic, got recv0=%0d recv1=%0d", recv0, recv1);
            errors = errors + 1;
        end
        if (errors == 0)
            $display("\nALL TESTS PASSED");
        else
            $display("\n%0d TEST(S) FAILED", errors);

        $fdisplay(master_log, "\n==== SECTION 1: S0 (SmartNIC) INGESTION -- %0d beats ====", sent0);
        $fdisplay(master_log, "idx\tpresent_t_ns\taccept_t_ns\tdwell_on_bus_ns\ts0_tdata_full_hex");
        for (i = 0; i < sent0; i = i + 1) begin
            $fdisplay(master_log, "%0d\t%0d\t%0d\t%0d\t%h",
                i, s0_present_t[i], s0_accept_t[i], s0_accept_t[i] - s0_present_t[i], s0_data[i]);
        end

        $fdisplay(master_log, "\n==== SECTION 2: S1 (ML) INGESTION -- %0d beats ====", sent1);
        $fdisplay(master_log, "idx\tpresent_t_ns\taccept_t_ns\tdwell_on_bus_ns\ts1_tdata_full_hex");
        for (i = 0; i < sent1; i = i + 1) begin
            $fdisplay(master_log, "%0d\t%0d\t%0d\t%0d\t%h",
                i, s1_present_t[i], s1_accept_t[i], s1_accept_t[i] - s1_present_t[i], s1_data[i]);
        end

        $fdisplay(master_log, "\n==== SECTION 3: FIFO WRITE (IN) -- %0d events, true committed order ====", wr_log_idx);
        $fdisplay(master_log, "idx\tstream\twrite_t_ns\tnext_write_t_ns\ttdata_full_hex");
        for (i = 1; i <= wr_log_idx; i = i + 1) begin
            if (i < wr_log_idx)
                $fdisplay(master_log, "%0d\tch%0d\t%0d\t%0d\t%h",
                    i, wr_stream_arr[i], wr_time_arr[i], wr_time_arr[i+1], wr_data_arr[i]);
            else
                $fdisplay(master_log, "%0d\tch%0d\t%0d\t(final write)\t%h",
                    i, wr_stream_arr[i], wr_time_arr[i], wr_data_arr[i]);
        end

        $fdisplay(master_log, "\n==== SECTION 4: FIFO READ (EXIT to Swaray's layer) -- %0d events, true committed order ====", rd_log_idx);
        $fdisplay(master_log, "idx\tstream\texit_t_ns\tvalid_until_ns\ttdata_full_hex");
        for (i = 1; i <= rd_log_idx; i = i + 1) begin
            if (i < rd_log_idx)
                $fdisplay(master_log, "%0d\tch%0d\t%0d\t%0d\t%h",
                    i, rd_stream_arr[i], rd_time_arr[i], rd_time_arr[i+1], rd_data_arr[i]);
            else
                $fdisplay(master_log, "%0d\tch%0d\t%0d\t%0d (mvalid_last_drop_t)\t%h",
                    i, rd_stream_arr[i], rd_time_arr[i], mvalid_last_drop_t, rd_data_arr[i]);
        end

        $fdisplay(master_log, "\n==== SESSION SUMMARY (all times in ns) ====");
        $fdisplay(master_log, "ch0 (SmartNIC): first_accept_t_ns=%0d  last_accept_t_ns=%0d  accepted_beats=%0d  ingestion_window_ns=%0d",
            ch0_first_acc_t, ch0_last_acc_t, ch0_acc_cnt, ch0_last_acc_t - ch0_first_acc_t);
        $fdisplay(master_log, "ch1 (ML):       first_accept_t_ns=%0d  last_accept_t_ns=%0d  accepted_beats=%0d  ingestion_window_ns=%0d",
            ch1_first_acc_t, ch1_last_acc_t, ch1_acc_cnt, ch1_last_acc_t - ch1_first_acc_t);
        $fdisplay(master_log, "output bus (m_tdata): first_valid_t_ns=%0d  last_drop_t_ns=%0d",
            mvalid_first_high_t, mvalid_last_drop_t);
        $fdisplay(master_log, "totals: wr_cnt=%0d rd_cnt=%0d accepted0_cnt=%0d accepted1_cnt=%0d recv0=%0d recv1=%0d errors=%0d",
            wr_cnt, rd_cnt, accepted0_cnt, accepted1_cnt, recv0, recv1, errors);

        $fclose(fifo_wr_log);
        $fclose(fifo_rd_log);
        $fclose(master_log);
        $finish;
    end

endmodule
