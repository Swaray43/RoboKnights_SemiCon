`timescale 1ns/1ps
// ============================================================
// tb_stream_path -- self-checking loopback bench for stream_tx_path_top
//
//   * Two AXI4-Stream channels, interleaved beat by beat, random valid.
//   * PHY TX looped to PHY RX (the link ACKs its own Flits).
//   * Optional: random PHY ready, random RX tready, bit-error injection
//     (single flips = FEC-correctable, double flips in one byte =
//     uncorrectable -> NAK -> replay).
//   * Reference model: every beat the TB drives is predicted into the
//     exact MWr TLP(s) it must become (4KB split included); MSI-X
//     requests are predicted at the point the DUT takes them. Received
//     TLPs (m_rx_axis_*) are compared DW by DW, in order, exactly once.
//
// plusargs: +NBEATS=n +READY_PCT=p +RXREADY_PCT=p +ERR1=ppm +ERR2=ppm +SEED=s
// ============================================================
module tb_stream_path #(parameter IDLE_FLUSH = 8);

    localparam DATA_W = 512, KEEP_W = 64, TUSER_W = 26, SYS = 256;

    reg clk = 0, rst_n = 0;
    always #2 clk = ~clk;

    integer NBEATS, READY_PCT, RXREADY_PCT, ERR1, ERR2, SEED, NOBASE1;

    // ---------------- DUT I/O ----------------
    reg  [DATA_W-1:0]  s_tdata;
    reg  [KEEP_W-1:0]  s_tkeep;
    reg  [TUSER_W-1:0] s_tuser;
    reg                s_tlast;
    reg                s_tvalid;
    wire               s_tready;

    reg                cfg_wr = 0;
    reg                cfg_id = 0;
    reg  [63:0]        cfg_data = 0;

    reg                msix_valid = 0;
    reg  [63:0]        msix_addr  = 0;
    reg  [31:0]        msix_data  = 0;
    wire               msix_ready;
    wire [1:0]         session_done;

    wire               ptx_valid, ptx_sof, ptx_eof;
    wire [SYS-1:0]     ptx_data;
    reg                ptx_ready;

    reg                prx_valid, prx_sof, prx_eof;
    reg  [SYS-1:0]     prx_data;

    wire               rx_tvalid, rx_tlast;
    wire [511:0]       rx_tdata;
    wire [63:0]        rx_tkeep;
    reg                rx_tready;

    wire err_xlat_miss, err_addr_ovf, stat_split, err_tkeep_gap;
    wire crc_err, fec_err, fec_corr, replay_ev;

    stream_tx_path_top #(.REPLAY_TIMEOUT(4096), .FLIT_IDLE_FLUSH(IDLE_FLUSH)) dut (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tdata(s_tdata), .s_axis_tkeep(s_tkeep), .s_axis_tuser(s_tuser),
        .s_axis_tlast(s_tlast), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
        .cfg_base_addr_wr(cfg_wr), .cfg_base_addr_id(cfg_id),
        .cfg_base_addr_data(cfg_data), .cfg_base_addr_clr(1'b0),
        .msix_req_valid(msix_valid), .msix_req_addr(msix_addr), .msix_req_data(msix_data),
        .msix_req_ready(msix_ready), .session_done(session_done),
        .phy_tx_valid(ptx_valid), .phy_tx_data(ptx_data), .phy_tx_sof(ptx_sof),
        .phy_tx_eof(ptx_eof), .phy_tx_ready(ptx_ready),
        .phy_rx_valid(prx_valid), .phy_rx_data(prx_data), .phy_rx_sof(prx_sof), .phy_rx_eof(prx_eof),
        .m_rx_axis_tvalid(rx_tvalid), .m_rx_axis_tdata(rx_tdata), .m_rx_axis_tkeep(rx_tkeep),
        .m_rx_axis_tlast(rx_tlast), .m_rx_axis_tready(rx_tready),
        .err_xlat_miss(err_xlat_miss), .err_addr_ovf(err_addr_ovf),
        .stat_split_occurred(stat_split), .err_tkeep_gap(err_tkeep_gap),
        .crc_err(crc_err), .fec_err(fec_err), .fec_corrected(fec_corr), .replay_event(replay_ev)
    );

    // ---------------- PHY loopback with error injection ----------------
    integer n_flip1 = 0, n_flip2 = 0;
    reg [SYS-1:0] flip;
    integer bpos;
    always @(posedge clk) begin
        ptx_ready <= ($unsigned($random) % 100) < READY_PCT;
        rx_tready <= ($unsigned($random) % 100) < RXREADY_PCT;
    end
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prx_valid <= 0; prx_sof <= 0; prx_eof <= 0; prx_data <= 0;
        end else begin
            prx_valid <= ptx_valid && ptx_ready;
            prx_sof   <= ptx_sof;
            prx_eof   <= ptx_eof;
            flip = 0;
            if (ptx_valid && ptx_ready) begin
                if (($unsigned($random) % 1000000) < ERR1) begin
                    flip[$unsigned($random) % SYS] = 1'b1; n_flip1 = n_flip1 + 1;
                end else if (($unsigned($random) % 1000000) < ERR2) begin
                    bpos = ($unsigned($random) % (SYS/8)) * 8;
                    flip[bpos] = 1'b1; flip[bpos+3] = 1'b1; n_flip2 = n_flip2 + 1;
                end
            end
            prx_data <= ptx_data ^ flip;
        end
    end

    // ---------------- reference model ----------------
    localparam QMAX = 1 << 16;
    reg [31:0] exp_dw  [0:QMAX*8-1];   // flattened expected TLP DWs
    integer    exp_len [0:QMAX-1];     // DWs per expected TLP
    integer    exp_base[0:QMAX-1];
    integer    exp_wr = 0, exp_rd = 0, exp_dw_wr = 0;

    reg [63:0] base [0:1];

    task push_tlp(input [63:0] addr, input integer ndw, input [511:0] data, input integer first_dw);
        integer k;
        begin
            exp_base[exp_wr] = exp_dw_wr;
            exp_len [exp_wr] = 4 + ndw;
            exp_dw[exp_dw_wr+0] = {3'b011, 5'b00000, 14'd0, ndw[9:0]};
            exp_dw[exp_dw_wr+1] = {16'h0200, 8'h00, (ndw == 1) ? 4'b0000 : 4'b1111, 4'b1111};
            exp_dw[exp_dw_wr+2] = addr[63:32];
            exp_dw[exp_dw_wr+3] = {addr[31:2], 2'b00};
            for (k = 0; k < ndw; k = k + 1)
                exp_dw[exp_dw_wr+4+k] = data[(first_dw+k)*32 +: 32];
            exp_dw_wr = exp_dw_wr + 4 + ndw;
            exp_wr = exp_wr + 1;
        end
    endtask

    // Beat-level FIFO check of write_data_buffer, and TLP prediction at
    // the point the DUT takes a beat / MSI (dut.src_fire).
    reg [DATA_W+KEEP_W+TUSER_W:0] in_q [0:QMAX-1];
    integer in_wr = 0, in_rd = 0;
    integer n_wdb_mismatch = 0;
    integer n_msix_sent = 0;

    always @(posedge clk) if (rst_n) begin
        if (s_tvalid && s_tready) begin
            in_q[in_wr] = {s_tlast, s_tuser, s_tkeep, s_tdata};
            in_wr = in_wr + 1;
        end
        if (dut.src_fire) begin
            if (dut.src_is_msix) begin
                push_tlp(dut.msix_addr_r, 1, {480'd0, dut.msix_data_r}, 0);
                n_msix_sent = n_msix_sent + 1;
            end else begin : beat
                reg [DATA_W+KEEP_W+TUSER_W:0] e;
                reg [63:0] a, nxt;
                integer ndw, n0;
                e = in_q[in_rd]; in_rd = in_rd + 1;
                if (e !== {dut.wdb_tlast, dut.wdb_tuser, dut.wdb_tkeep, dut.wdb_tdata}) begin
                    n_wdb_mismatch = n_wdb_mismatch + 1;
                    if (n_wdb_mismatch < 5)
                        $display("[%0t] WDB MISMATCH: beat %0d out of write_data_buffer is not the beat that went in", $time, in_rd-1);
                end
                ndw = 0;
                for (n0 = 0; n0 < 16; n0 = n0 + 1) if (e[DATA_W + n0*4]) ndw = ndw + 1;
                if (ndw > 0 && !(NOBASE1 && e[DATA_W+KEEP_W])) begin
                    a = base[e[DATA_W+KEEP_W]] + e[DATA_W+KEEP_W+24:DATA_W+KEEP_W+1];
                    nxt = {a[63:12], 12'h000} + 64'h1000;
                    if (a + ndw*4 > nxt) begin
                        n0 = (nxt - a) >> 2;
                        push_tlp(a,   n0,       e[DATA_W-1:0], 0);
                        push_tlp(nxt, ndw - n0, e[DATA_W-1:0], n0);
                    end else
                        push_tlp(a, ndw, e[DATA_W-1:0], 0);
                end
            end
        end
    end

    // ---------------- receive checker ----------------
    reg  [31:0] got [0:2047];
    integer got_n = 0, n_tlp_ok = 0, n_tlp_bad = 0, n_extra = 0;
    integer k2;
    always @(posedge clk) if (rst_n && rx_tvalid && rx_tready) begin
        for (k2 = 0; k2 < 16; k2 = k2 + 1)
            if (rx_tkeep[k2*4]) begin got[got_n] = rx_tdata[k2*32 +: 32]; got_n = got_n + 1; end
        if (rx_tlast) begin : cmp
            integer m; reg bad;
            if (exp_rd >= exp_wr) begin
                n_extra = n_extra + 1;
                if (n_extra < 5) $display("[%0t] EXTRA TLP received (nothing expected), %0d DW, DW0=%h", $time, got_n, got[0]);
            end else begin
                bad = (got_n != exp_len[exp_rd]);
                for (m = 0; m < got_n && m < exp_len[exp_rd]; m = m + 1)
                    if (got[m] !== exp_dw[exp_base[exp_rd]+m]) bad = 1;
                if (bad) begin
                    n_tlp_bad = n_tlp_bad + 1;
                    if (n_tlp_bad < 5) begin
                        $display("[%0t] TLP %0d MISMATCH: got %0d DW, expected %0d", $time, exp_rd, got_n, exp_len[exp_rd]);
                        for (m = 0; m < 6; m = m + 1)
                            $display("      DW%0d got %h exp %h", m, got[m], exp_dw[exp_base[exp_rd]+m]);
                    end
                end else n_tlp_ok = n_tlp_ok + 1;
                exp_rd = exp_rd + 1;
            end
            got_n = 0;
        end
    end

    // ---------------- counters ----------------
    integer n_crc = 0, n_fec = 0, n_cor = 0, n_replay = 0, n_flits = 0;
    always @(posedge clk) if (rst_n) begin
        if (crc_err) n_crc = n_crc + 1;
        if (fec_err) n_fec = n_fec + 1;
        if (fec_corr) n_cor = n_cor + 1;
        if (replay_ev) n_replay = n_replay + 1;
        if (ptx_valid && ptx_ready && ptx_sof) n_flits = n_flits + 1;
    end

    // ---------------- stimulus ----------------
    integer sent = 0, last_progress;
    reg [23:0] off [0:1];
    reg        ch;
    reg        msix_armed;
    integer    w;
    integer    cyc = 0;
    reg [6:0]  sent7;
    reg [15:0] off16;
    always @(posedge clk) cyc <= cyc + 1;

    initial begin
        if (!$value$plusargs("NBEATS=%d", NBEATS)) NBEATS = 400;
        if (!$value$plusargs("READY_PCT=%d", READY_PCT)) READY_PCT = 100;
        if (!$value$plusargs("RXREADY_PCT=%d", RXREADY_PCT)) RXREADY_PCT = 100;
        if (!$value$plusargs("ERR1=%d", ERR1)) ERR1 = 0;
        if (!$value$plusargs("ERR2=%d", ERR2)) ERR2 = 0;
        if (!$value$plusargs("SEED=%d", SEED)) SEED = 1;
        NOBASE1 = $test$plusargs("NOBASE1");
        w = $random(SEED);

        base[0] = 64'h0000_0001_0000_0000;
        base[1] = 64'h0000_0002_0000_0020;   // 32-byte skew: some beats cross 4KB
        off[0] = 0; off[1] = 0;
        s_tvalid = 0; s_tdata = 0; s_tkeep = 0; s_tuser = 0; s_tlast = 0;
        msix_armed = 0;

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);
        @(negedge clk); cfg_wr = 1; cfg_id = 0; cfg_data = base[0];
        @(negedge clk); cfg_wr = !NOBASE1; cfg_id = 1; cfg_data = base[1];
        @(negedge clk); cfg_wr = 0;

        last_progress = cyc;
        while (sent < NBEATS) begin
            @(negedge clk);
            if (s_tvalid && s_tready_q) begin
                off[ch] = off[ch] + 64;
                sent = sent + 1;
                s_tvalid = 0;
                last_progress = cyc;
            end
            if (!s_tvalid && sent < NBEATS && ($unsigned($random) % 100) < 70) begin
                ch = $unsigned($random) % 2;
                sent7 = sent; off16 = off[ch] >> 6;
                for (w = 0; w < 16; w = w + 1) s_tdata[w*32 +: 32] = {ch, sent7, w[7:0], off16};
                // every 25th beat on a channel ends its session with a short beat
                s_tlast = (((off[ch] >> 6) % 25) == 24) || (sent >= NBEATS - 2);
                s_tkeep = s_tlast ? (64'hFFFF_FFFF_FF >> 0) : {64{1'b1}};   // 40 B = 10 DW
                s_tuser = {s_tlast, off[ch], ch};
                s_tvalid = 1;
            end
            if (cyc - last_progress > 20000) begin
                $display("FAIL: input stalled (sent %0d of %0d)", sent, NBEATS);
                finish_report;
            end
        end
        @(negedge clk); s_tvalid = 0;

        // drain
        last_progress = cyc;
        while (exp_rd < exp_wr || in_rd < in_wr || dut.msix_pend) begin
            @(posedge clk);
            if (cyc - last_progress > 200000) begin
                $display("FAIL: drain timeout");
                finish_report;
            end
        end
        repeat (2000) @(posedge clk);
        finish_report;
    end

    // registered view of ready at the sampling edge
    reg s_tready_q;
    always @(posedge clk) s_tready_q <= s_tready;

    // MSI-X: one request after every session_done
    always @(posedge clk) begin
        if (msix_valid && msix_ready) msix_valid <= 0;
        if (|session_done) begin
            msix_valid <= 1;
            msix_addr  <= 64'h0000_0000_FEE0_0000 + (session_done[1] ? 16 : 0);
            msix_data  <= session_done[1] ? 32'h0000_00A1 : 32'h0000_00A0;
        end
    end

    task finish_report;
        begin
            $display("------------------------------------------------------------");
            $display(" beats sent         : %0d   (WDB in/out mismatches: %0d)", sent, n_wdb_mismatch);
            $display(" TLPs expected      : %0d   (incl. %0d MSI-X)", exp_wr, n_msix_sent);
            $display(" TLPs received ok   : %0d   bad: %0d   extra: %0d   missing: %0d",
                     n_tlp_ok, n_tlp_bad, n_extra, exp_wr - exp_rd);
            $display(" Flits sent         : %0d", n_flits);
            $display(" bit flips injected : %0d single, %0d double", n_flip1, n_flip2);
            $display(" FEC corrected      : %0d   FEC uncorrectable: %0d   CRC err: %0d   replays: %0d",
                     n_cor, n_fec, n_crc, n_replay);
            if (n_tlp_bad == 0 && n_extra == 0 && exp_rd == exp_wr && n_wdb_mismatch == 0 &&
                sent == NBEATS && exp_wr > 0)
                $display(" RESULT: PASS");
            else
                $display(" RESULT: FAIL");
            $display("------------------------------------------------------------");
            $finish;
        end
    endtask

endmodule
