`timescale 1ns/1ps
// ============================================================
// tb_bridge_top -- end-to-end: fabric AXI4-Stream -> interconnect -> link ->
// PHY -> PAM4 levels -> (wire loopback) -> PHY RX -> link RX -> TLPs out.
//
// Reference model is built ONLY from what the fabric drives (no DUT internals):
//   per channel, per session: addr = base[ch][sess] + running byte offset,
//   one MWr per beat, payload = the beat's valid DWs.
// Received TLPs are matched per channel (by address window), in order.
// MSI-X MWrs are checked for address/data and for arriving after every data
// TLP of the session they report.
//
// plusargs: +BEATS=n  +SESS=n  +GAP=pct  +REARM_FAST=0/1  +SEED=s
// ============================================================
module tb_bridge_ovf;
    localparam DATA_W = 512, KEEP_W = 64;

    reg clk = 0, aclk = 0, perst_n = 0;
    always #2   clk  = ~clk;     // 250 MHz
    always #2.7 aclk = ~aclk;    // ~185 MHz, unrelated

    integer BEATS, SESS, GAP, REARM_FAST, SEED;

    reg  [DATA_W-1:0] s0_tdata, s1_tdata;
    reg  [KEEP_W-1:0] s0_tkeep, s1_tkeep;
    reg  s0_tlast = 0, s1_tlast = 0, s0_tvalid = 0, s1_tvalid = 0;
    wire s0_tready, s1_tready;
    wire [1:0] session_active, session_overflow;

    wire rx_tvalid, rx_tlast; wire [DATA_W-1:0] rx_tdata; wire [KEEP_W-1:0] rx_tkeep;

    reg ts_byte_valid = 0; reg [7:0] ts_byte_l0 = 0, ts_byte_l1 = 0, ts_byte_l2 = 0, ts_byte_l3 = 0;
    reg updatefc_received = 0;
    wire link_trained, datapath_rst_n, axi_rst_n;

    reg bar0_wr_en = 0; reg [15:0] bar0_addr = 0; reg [31:0] bar0_wdata = 0;
    reg cfg_wr_en = 0; reg [11:0] cfg_addr = 0; reg [31:0] cfg_wdata = 0;
    reg tbl_wr_en = 0; reg [2:0] tbl_wr_vec = 0; reg [63:0] tbl_wr_addr = 0; reg [31:0] tbl_wr_data = 0;

    wire [11:0] lvl; wire [3:0] lvl_v;
    wire err_xlat_miss, err_addr_ovf, crc_err, fec_err, phy_framing_err, phy_lane_err;

    bridge_top dut (
        .clk(clk), .perst_n(perst_n), .aclk(aclk),
        .s0_tdata(s0_tdata), .s0_tkeep(s0_tkeep), .s0_tlast(s0_tlast), .s0_tvalid(s0_tvalid), .s0_tready(s0_tready),
        .s1_tdata(s1_tdata), .s1_tkeep(s1_tkeep), .s1_tlast(s1_tlast), .s1_tvalid(s1_tvalid), .s1_tready(s1_tready),
        .session_active(session_active), .session_overflow(session_overflow),
        .m_rx_axis_tvalid(rx_tvalid), .m_rx_axis_tdata(rx_tdata), .m_rx_axis_tkeep(rx_tkeep),
        .m_rx_axis_tlast(rx_tlast), .m_rx_axis_tready(1'b1),
        .ts_byte_valid(ts_byte_valid), .ts_byte_l0(ts_byte_l0), .ts_byte_l1(ts_byte_l1),
        .ts_byte_l2(ts_byte_l2), .ts_byte_l3(ts_byte_l3), .link_trained(link_trained),
        .hot_reset_request(1'b0), .flr_request(1'b0), .updatefc_received(updatefc_received),
        .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n),
        .cfg_wr_en(cfg_wr_en), .cfg_rd_en(1'b0), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
        .cfg_rdata(), .cfg_cpl_valid(),
        .bar0_wr_en(bar0_wr_en), .bar0_rd_en(1'b0), .bar0_addr(bar0_addr), .bar0_wdata(bar0_wdata),
        .bar0_rdata(), .bar0_rd_valid(),
        .tbl_wr_en(tbl_wr_en), .tbl_wr_vec(tbl_wr_vec), .tbl_wr_addr(tbl_wr_addr),
        .tbl_wr_data(tbl_wr_data), .tbl_wr_mask(1'b0),
        .tx_level(lvl), .tx_level_valid(lvl_v), .rx_level(lvl), .rx_level_valid(lvl_v),   // SerDes loopback
        .err_xlat_miss(err_xlat_miss), .err_addr_ovf(err_addr_ovf), .crc_err(crc_err), .fec_err(fec_err),
        .phy_framing_err(phy_framing_err), .phy_lane_err(phy_lane_err)
    );

    // ---------------- reference model ----------------
    localparam Q = 4096;
    reg [63:0] exp_addr [0:1][0:Q-1];
    integer    exp_ndw  [0:1][0:Q-1];
    integer    exp_sess [0:1][0:Q-1];
    reg [511:0] exp_data [0:1][0:Q-1];
    integer exp_wr [0:1], exp_rd [0:1];
    integer msix_seen = 0, msix_bad = 0, msix_early = 0;
    integer last_sess_sent [0:1];     // highest session whose data the fabric has finished sending

    function [63:0] base_of(input integer ch, input integer s);
        base_of = (ch == 0 ? 64'h0000_00AB_1000_0000 : 64'h0000_00CD_2000_0000) + s * 64'h10_0000;
    endfunction
    function [63:0] msix_addr_of(input integer ch);
        msix_addr_of = 64'h0000_0000_FEE0_0000 + (ch == 0 ? 2 : 6) * 16;
    endfunction
    function [31:0] msix_data_of(input integer ch);
        msix_data_of = 32'h0000_A000 + (ch == 0 ? 2 : 6);
    endfunction

    // ---------------- RX checker ----------------
    reg  [31:0] got [0:63];
    integer got_n = 0, ok = 0, bad = 0, extra = 0, k;
    always @(posedge clk) if (rx_tvalid) begin
        for (k = 0; k < 16; k = k + 1)
            if (rx_tkeep[k*4]) begin got[got_n] = rx_tdata[k*32 +: 32]; got_n = got_n + 1; end
        if (rx_tlast) begin : chk
            reg [63:0] a; integer ch, ndw, m, i; reg b;
            a   = {got[2], got[3]};
            ndw = got[0][9:0];
            if (ndw == 1 && (a == msix_addr_of(0) || a == msix_addr_of(1))) begin
                ch = (a == msix_addr_of(0)) ? 0 : 1;
                msix_seen = msix_seen + 1;
                if (got[4] !== msix_data_of(ch)) msix_bad = msix_bad + 1;
                // every data TLP of sessions already completed on this channel must be in
                for (i = exp_rd[ch]; i < exp_wr[ch]; i = i + 1)
                    if (exp_sess[ch][i] < msix_seen_ch(ch)) msix_early = msix_early + 1;
                bump_msix(ch);
            end else begin
                ch = (a[39:32] == 8'hAB) ? 0 : (a[39:32] == 8'hCD) ? 1 : -1;
                if (ch < 0 || exp_rd[ch] >= exp_wr[ch]) begin
                    extra = extra + 1;
                    if (extra < 5) $display("[%0t] UNEXPECTED TLP addr=%h ndw=%0d", $time, a, ndw);
                end else begin
                    i = exp_rd[ch];
                    b = (a !== exp_addr[ch][i]) || (ndw != exp_ndw[ch][i]) || (got_n != 4 + ndw)
                        || (got[0][31:29] !== 3'b011);
                    for (m = 0; m < ndw && m < 16; m = m + 1)
                        if (got[4+m] !== exp_data[ch][i][m*32 +: 32]) b = 1;
                    if (b) begin
                        bad = bad + 1;
                        if (bad < 6) $display("[%0t] ch%0d TLP %0d MISMATCH: got addr %h ndw %0d / exp addr %h ndw %0d (sess %0d)",
                                              $time, ch, i, a, ndw, exp_addr[ch][i], exp_ndw[ch][i], exp_sess[ch][i]);
                    end else ok = ok + 1;
                    exp_rd[ch] = i + 1;
                end
            end
            got_n = 0;
        end
    end
    integer msix_cnt [0:1];
    integer leaked = 0, LIMIT, OVF_BEATS, stuck = 0;
    function integer msix_seen_ch(input integer ch); msix_seen_ch = msix_cnt[ch] + 1; endfunction
    task bump_msix(input integer ch); msix_cnt[ch] = msix_cnt[ch] + 1; endtask

    // ---------------- stimulus helpers ----------------
    task send_ts1; integer j; begin
        @(negedge clk); ts_byte_valid = 1;
        {ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3} = {4{8'hBC}}; @(negedge clk);
        {ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3} = 32'h0; @(negedge clk);
        ts_byte_l0 = 0; ts_byte_l1 = 1; ts_byte_l2 = 2; ts_byte_l3 = 3; @(negedge clk);
        for (j = 3; j <= 9; j = j + 1) begin {ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3} = 32'h0; @(negedge clk); end
        for (j = 10; j <= 15; j = j + 1) begin {ts_byte_l0, ts_byte_l1, ts_byte_l2, ts_byte_l3} = {4{8'h4A}}; @(negedge clk); end
        ts_byte_valid = 0;
    end endtask
    task cfg_write(input [11:0] a, input [31:0] d); begin
        @(negedge clk); cfg_addr = a; cfg_wdata = d; cfg_wr_en = 1; @(negedge clk); cfg_wr_en = 0;
    end endtask
    task bar0_write(input [15:0] a, input [31:0] d); begin
        @(negedge clk); bar0_addr = a; bar0_wdata = d; bar0_wr_en = 1; @(negedge clk); bar0_wr_en = 0;
    end endtask
    task msix_entry(input [2:0] v, input [63:0] a, input [31:0] d); begin
        @(negedge clk); tbl_wr_vec = v; tbl_wr_addr = a; tbl_wr_data = d; tbl_wr_en = 1; @(negedge clk); tbl_wr_en = 0;
    end endtask

    function automatic [511:0] beat_data(input integer ch, input integer s, input integer b);
        integer d; begin
            for (d = 0; d < 16; d = d + 1) beat_data[d*32 +: 32] = {ch[3:0], s[3:0], b[11:0], d[11:0]};
        end
    endfunction

    // one channel's session, on aclk; predicts TLPs as beats are ACCEPTED
    task automatic run_channel(input integer ch, input integer s, input integer nbeats);
        integer b, nbytes, d, w; reg [63:0] off; reg [63:0] kp; reg [511:0] dat; reg stop; begin
            off = 0; stop = 0;
            for (b = 0; b < nbeats && !stop; b = b + 1) begin
                kp  = (b == nbeats - 1) ? 64'h0000_00FF_FFFF_FFFF : {64{1'b1}};
                dat = beat_data(ch, s, b);
                @(negedge aclk);
                if (ch == 0) begin s0_tdata = dat; s0_tkeep = kp; s0_tlast = (b == nbeats-1); s0_tvalid = 1; end
                else         begin s1_tdata = dat; s1_tkeep = kp; s1_tlast = (b == nbeats-1); s1_tvalid = 1; end
                nbytes = 0; for (d = 0; d < 64; d = d + 1) nbytes = nbytes + kp[d];
                @(posedge aclk); w = 0;
                while (!(ch == 0 ? s0_tready : s1_tready) && !stop) begin
                    if (session_overflow[ch]) stop = 1;     // aborted: fabric gives up on this session
                    else begin @(posedge aclk); w = w + 1; end
                end
                if (!stop) begin
                    if (off + nbytes > LIMIT) leaked = leaked + 1;   // handshake beyond the limit: must never happen
                    else begin
                        exp_addr[ch][exp_wr[ch]] = base_of(ch, s) + off;
                        exp_ndw [ch][exp_wr[ch]] = nbytes / 4;
                        exp_sess[ch][exp_wr[ch]] = s;
                        exp_data[ch][exp_wr[ch]] = dat;
                        exp_wr[ch] = exp_wr[ch] + 1;
                    end
                    off = off + nbytes;
                end
                @(negedge aclk);
                if (ch == 0) s0_tvalid = 0; else s1_tvalid = 0;
            end
            if (stop) $display("[%0t] ch%0d sess %0d: overflow abort after %0d bytes accepted (limit %0d)", $time, ch, s, off, LIMIT);
        end
    endtask

    // ---------------- main ----------------
    integer s, t0, cyc;
    initial begin
        if (!$value$plusargs("BEATS=%d", BEATS)) BEATS = 24;
        if (!$value$plusargs("LIMIT=%d", LIMIT)) LIMIT = 1000;
        if (!$value$plusargs("OVF_BEATS=%d", OVF_BEATS)) OVF_BEATS = 24;
        if (!$value$plusargs("SESS=%d", SESS)) SESS = 3;
        if (!$value$plusargs("GAP=%d", GAP)) GAP = 20;
        if (!$value$plusargs("REARM_FAST=%d", REARM_FAST)) REARM_FAST = 1;
        if (!$value$plusargs("SEED=%d", SEED)) SEED = 1;
        exp_wr[0] = 0; exp_wr[1] = 0; exp_rd[0] = 0; exp_rd[1] = 0; msix_cnt[0] = 0; msix_cnt[1] = 0;

        repeat (5) @(negedge clk); perst_n = 1;
        // Link trains by itself over the SerDes loopback (LTSSM: Detect -> Polling/TS1
        // -> Configuration/TS2 -> L0, ~1700 clk); no TS1 injection needed any more.
        wait (link_trained === 1'b1);
        repeat (3) @(negedge clk); updatefc_received = 1; @(negedge clk); updatefc_received = 0;
        wait (axi_rst_n === 1'b1);
        // What a host driver does first: Memory Space + Bus Master, then MSI-X Enable.
        cfg_write(12'h004, 32'h0000_0006);
        cfg_write(12'h050, 32'h8000_0000);
        repeat (20) @(negedge clk);
        $display("[%0t] link_trained=%b datapath_rst_n=%b axi_rst_n=%b", $time, link_trained, datapath_rst_n, axi_rst_n);

        msix_entry(2, msix_addr_of(0), msix_data_of(0));
        msix_entry(6, msix_addr_of(1), msix_data_of(1));
        bar0_write(12'h010, 32'h0000_0602);       // MSI-X vector ch0=2 ch1=6
        bar0_write(12'h008, 32'h0000_0101);       // weights 1:1
        bar0_write(12'h00C, 32'h00FF_FFFF);       // limit

        for (s = 0; s < 2; s = s + 1) begin
            bar0_write(12'h00C, LIMIT);
            bar0_write(12'h020, base_of(0, s));  bar0_write(12'h024, base_of(0, s) >> 32);
            bar0_write(12'h028, base_of(1, s));  bar0_write(12'h02C, base_of(1, s) >> 32);
            bar0_write(12'h000, 32'h3);
            t0 = 0; while (session_active !== 2'b11 && t0 < 2000) begin @(posedge aclk); t0 = t0 + 1; end
            if (session_active !== 2'b11) $display("[%0t] FAIL: session %0d never became active (active=%b)", $time, s, session_active);
            fork
                run_channel(0, s, (s == 0) ? OVF_BEATS : 8);
                run_channel(1, s, 8);
            join
            // host: wait for BAR0 STATUS to show both channels idle before re-arming
            cyc = 0;
            while (dut.u_lpb.u_pcie_phy_top.session_valid !== 2'b00 && cyc < 30000) begin @(posedge clk); cyc = cyc + 1; end
            if (dut.u_lpb.u_pcie_phy_top.session_valid !== 2'b00) begin
                stuck = stuck + 1;
                $display("[%0t] HANG: STATUS session_valid=%b stays set after session %0d -- host can never re-arm", $time,
                         dut.u_lpb.u_pcie_phy_top.session_valid, s);
            end
            $display("[%0t] after session %0d: STATUS.overflow(sync)=%b  msix ch0=%0d ch1=%0d  idle after %0d clk",
                     $time, s, dut.u_lpb.u_pcie_phy_top.ovf_s2, msix_cnt[0], msix_cnt[1], cyc);
            repeat (40) @(posedge clk);
        end
        cyc = 0;
        while ((exp_rd[0] < exp_wr[0] || exp_rd[1] < exp_wr[1] || msix_seen < 4) && cyc < 400000) begin @(posedge clk); cyc = cyc + 1; end
        repeat (2000) @(posedge clk);

        $display("------------------------------------------------------------");
        $display(" session 0: ch0 %0d beats vs limit %0d B (overflows), ch1 8 beats; session 1: 8 + 8 beats", OVF_BEATS, LIMIT);
        $display(" data TLPs expected     : ch0 %0d  ch1 %0d", exp_wr[0], exp_wr[1]);
        $display(" data TLPs received ok  : %0d   mismatched: %0d   unexpected: %0d   still missing: %0d",
                 ok, bad, extra, (exp_wr[0]-exp_rd[0]) + (exp_wr[1]-exp_rd[1]));
        $display(" MSI-X received         : ch0 %0d (sessions: 2)  ch1 %0d (sessions: 2)  bad data: %0d  early: %0d",
                 msix_cnt[0], msix_cnt[1], msix_bad, msix_early);
        $display(" beats handshaked beyond the limit (leaked): %0d    host hangs: %0d", leaked, stuck);
        $display(" errors: xlat_miss=%b addr_ovf=%b crc=%b fec=%b framing=%b lane=%b overflow=%b",
                 err_xlat_miss, err_addr_ovf, crc_err, fec_err, phy_framing_err, phy_lane_err, session_overflow);
        if (bad == 0 && extra == 0 && exp_rd[0] == exp_wr[0] && exp_rd[1] == exp_wr[1]
            && msix_cnt[0] == 2 && msix_cnt[1] == 2 && msix_bad == 0 && msix_early == 0 && leaked == 0 && stuck == 0)
            $display(" RESULT: PASS");
        else
            $display(" RESULT: FAIL");
        $finish;
    end

    // sticky error catchers
    always @(posedge clk) begin
        if (err_xlat_miss) $display("[%0t] err_xlat_miss pulse", $time);
        if (err_addr_ovf)  $display("[%0t] err_addr_ovf pulse", $time);
    end
    initial begin #20_000_000; $display("GLOBAL TIMEOUT"); $display(" RESULT: FAIL"); $finish; end
endmodule
