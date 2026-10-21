`timescale 1ns/1ps
// ============================================================
// tb_bridge_rand -- randomized full-stack test (Phase 2, Step 3 B)
//
// Same DUT, loopback and checker style as tb_bridge_top (the reference model is
// built only from what the fabric drives), but every run is a different random
// scenario, reproducible from +SEED.
//
// Features (Step 3 B plan, one per commit):
//   F1  random session count, beats per session, last-beat size (1..16 DW)
//       and per-beat gaps on each channel
//   F2  random DW-aligned base addresses (not 4 KB aligned), so beats cross
//       4 KB boundaries and the address splitter is exercised end to end
//   F3  random arbiter weights (1..7 : 1..7) per session, and sessions where
//       only one channel is armed
//   F4  PAM4 channel noise: +NOISE_PPM=n flips a lane symbol to an adjacent
//       level (a 1-bit Gray error) with probability n per million per lane per
//       clock, once traffic starts; +BURST=k makes each hit k symbols long.
//       FEC must correct single errors, CRC + replay must recover the rest:
//       every TLP must still arrive exactly once, in order, bit-exact.
//   (F5..F9 -- overflow, lane reversal, resets, MSI-X masking, host-driven
//    BAR0 -- come in later commits)
//
// Reference model, per accepted beat at address A with N bytes:
//   one MWr of N bytes, or -- if [A, A+N) crosses a 4 KB boundary -- two MWrs:
//   (4096 - A%4096) bytes at A, then the rest at the boundary.
//
// plusargs: +SEED=n (default 1)  +SESS_MAX=n (6)  +BEATS_MAX=n (40)
//           +GAP_MAX=pct (60)  +NOISE_PPM=n (0)  +BURST=k (1)  +VERBOSE=1
// Ends with RESULT: PASS / RESULT: FAIL and prints the seed to replay it.
// ============================================================
module tb_bridge_rand;
    localparam DATA_W = 512, KEEP_W = 64;

    reg clk = 0, aclk = 0, perst_n = 0;
    always #2   clk  = ~clk;     // 250 MHz
    always #2.7 aclk = ~aclk;    // ~185 MHz, unrelated

    integer SEED, SEED0, SESS_MAX, BEATS_MAX, GAP_MAX, VERBOSE, NOISE_PPM, BURST;

    reg  [DATA_W-1:0] s0_tdata = 0, s1_tdata = 0;
    reg  [KEEP_W-1:0] s0_tkeep = 0, s1_tkeep = 0;
    reg  s0_tlast = 0, s1_tlast = 0, s0_tvalid = 0, s1_tvalid = 0;
    wire s0_tready, s1_tready;
    wire [1:0] session_active, session_overflow;

    wire rx_tvalid, rx_tlast; wire [DATA_W-1:0] rx_tdata; wire [KEEP_W-1:0] rx_tkeep;

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
        .ts_byte_valid(1'b0), .ts_byte_l0(8'd0), .ts_byte_l1(8'd0), .ts_byte_l2(8'd0), .ts_byte_l3(8'd0),
        .link_trained(link_trained),
        .hot_reset_request(1'b0), .flr_request(1'b0), .updatefc_received(updatefc_received),
        .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n),
        .cfg_wr_en(cfg_wr_en), .cfg_rd_en(1'b0), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
        .cfg_rdata(), .cfg_cpl_valid(),
        .bar0_wr_en(bar0_wr_en), .bar0_rd_en(1'b0), .bar0_addr(bar0_addr), .bar0_wdata(bar0_wdata),
        .bar0_rdata(), .bar0_rd_valid(),
        .tbl_wr_en(tbl_wr_en), .tbl_wr_vec(tbl_wr_vec), .tbl_wr_addr(tbl_wr_addr),
        .tbl_wr_data(tbl_wr_data), .tbl_wr_mask(1'b0),
        .tx_level(lvl), .tx_level_valid(lvl_v), .rx_level(rx_lvl), .rx_level_valid(lvl_v),   // SerDes loopback + F4 noise
        .err_xlat_miss(err_xlat_miss), .err_addr_ovf(err_addr_ovf), .crc_err(crc_err), .fec_err(fec_err),
        .phy_framing_err(phy_framing_err), .phy_lane_err(phy_lane_err)
    );

    // ---------------- F4: PAM4 channel noise ----------------
    // Each lane carries a signed 3-bit level (-3,-1,+1,+3). A hit moves it to
    // an adjacent level, i.e. one Gray bit flips -- the common real-world error.
    reg  [3:0]  noise_hit = 0;       // per lane, decided at negedge (race-free)
    reg  [3:0]  noise_dir = 0;
    integer     burst_left [0:3];
    reg         noise_on = 0;
    integer     n_noise = 0, nl;
    function [2:0] adj_level(input [2:0] l, input up);
        reg signed [3:0] v; begin
            v = $signed({l[2], l});
            if (v >= 3)       v = 1;          // edges can only move inwards
            else if (v <= -3) v = -1;
            else              v = up ? v + 2 : v - 2;
            adj_level = v[2:0];
        end
    endfunction
    wire [11:0] rx_lvl;
    genvar gl;
    generate for (gl = 0; gl < 4; gl = gl + 1) begin : g_noise
        assign rx_lvl[gl*3 +: 3] = noise_hit[gl] ? adj_level(lvl[gl*3 +: 3], noise_dir[gl]) : lvl[gl*3 +: 3];
    end endgenerate
    initial for (nl = 0; nl < 4; nl = nl + 1) burst_left[nl] = 0;
    always @(negedge clk) begin
        for (nl = 0; nl < 4; nl = nl + 1) begin
            noise_hit[nl] = 1'b0;
            if (noise_on && lvl_v[nl]) begin
                if (burst_left[nl] > 0) begin
                    noise_hit[nl] = 1'b1; burst_left[nl] = burst_left[nl] - 1;
                end else if (urand(0, 999_999) < NOISE_PPM) begin
                    noise_hit[nl] = 1'b1; noise_dir[nl] = urand(0, 1); burst_left[nl] = BURST - 1;
                    n_noise = n_noise + 1;
                end
            end
        end
    end
    integer n_fec_fix = 0, n_fec_bad = 0, n_crc = 0, n_replay = 0;
    always @(posedge clk) begin
        if (dut.u_lpb.fec_corrected) n_fec_fix = n_fec_fix + 1;
        if (fec_err) n_fec_bad = n_fec_bad + 1;
        if (crc_err) n_crc = n_crc + 1;
        if (dut.u_lpb.replay_event) n_replay = n_replay + 1;
    end

    // ---------------- random helpers ----------------
    function integer urand(input integer lo, input integer hi);   // inclusive
        integer r; begin
            r = $random(SEED);
            if (r < 0) r = -r;
            urand = lo + (r % (hi - lo + 1));
        end
    endfunction

    // ---------------- scenario state ----------------
    localparam Q = 8192;
    reg [63:0]  exp_addr [0:1][0:Q-1];
    integer     exp_ndw  [0:1][0:Q-1];
    integer     exp_sess [0:1][0:Q-1];
    reg [511:0] exp_data [0:1][0:Q-1];
    integer exp_wr [0:1], exp_rd [0:1];
    reg [63:0] base [0:1];                 // current session's base per channel
    integer cur_sess [0:1];                // session index per channel
    integer msix_seen = 0, msix_bad = 0, msix_early = 0, msix_expected = 0;
    integer msix_cnt [0:1];

    // coverage-ish counters (printed at the end)
    integer n_split = 0, n_single_ch = 0, n_beats = 0, n_sessions = 0;
    integer n_last_sz [1:16];
    integer w_used [1:7];

    function [63:0] msix_addr_of(input integer ch);
        msix_addr_of = 64'h0000_0000_FEE0_0000 + (ch == 0 ? 2 : 6) * 16;
    endfunction
    function [31:0] msix_data_of(input integer ch);
        msix_data_of = 32'h0000_A000 + (ch == 0 ? 2 : 6);
    endfunction

    // F2: random DW-aligned base, kept inside the channel's checker window
    // (bits [39:32] = AB for ch0, CD for ch1) and never 4 KB aligned on purpose
    function [63:0] rand_base(input integer ch);
        reg [31:0] lo; begin
            lo = $random(SEED);
            lo[1:0] = 2'b00;
            if (lo[11:2] == 0) lo[11:2] = 10'd1;
            rand_base = {24'h0, (ch == 0 ? 8'hAB : 8'hCD), lo};
        end
    endfunction

    // push one expected MWr (ndw DWs of dat starting at DW index dw0)
    task push_exp(input integer ch, input [63:0] a, input integer ndw, input [511:0] dat, input integer dw0);
        begin
            exp_addr[ch][exp_wr[ch]] = a;
            exp_ndw [ch][exp_wr[ch]] = ndw;
            exp_sess[ch][exp_wr[ch]] = cur_sess[ch];
            exp_data[ch][exp_wr[ch]] = dat >> (dw0 * 32);
            exp_wr[ch] = exp_wr[ch] + 1;
        end
    endtask

    // ---------------- RX checker (same matching as tb_bridge_top) ----------------
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
                for (i = exp_rd[ch]; i < exp_wr[ch]; i = i + 1)
                    if (exp_sess[ch][i] <= msix_cnt[ch]) msix_early = msix_early + 1;
                msix_cnt[ch] = msix_cnt[ch] + 1;
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

    // ---------------- host helpers ----------------
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

    // one channel's session on aclk: random beat count, last-beat size, gaps (F1)
    task automatic run_channel(input integer ch, input integer s, input integer nbeats,
                               input integer last_dw, input integer gap);
        integer b, nbytes, first, ndw; reg [63:0] off, a; reg [63:0] kp; reg [511:0] dat; begin
            off = 0;
            for (b = 0; b < nbeats; b = b + 1) begin
                kp  = (b == nbeats - 1) ? ({64{1'b1}} >> (64 - 4*last_dw)) : {64{1'b1}};
                dat = beat_data(ch, s, b);
                while (urand(0, 99) < gap) @(posedge aclk);
                @(negedge aclk);
                if (ch == 0) begin s0_tdata = dat; s0_tkeep = kp; s0_tlast = (b == nbeats-1); s0_tvalid = 1; end
                else         begin s1_tdata = dat; s1_tkeep = kp; s1_tlast = (b == nbeats-1); s1_tvalid = 1; end
                @(posedge aclk);
                while (!(ch == 0 ? s0_tready : s1_tready)) @(posedge aclk);
                nbytes = (b == nbeats - 1) ? 4*last_dw : 64;
                a = base[ch] + off;
                // F2: 4 KB boundary inside this beat -> the splitter emits two MWrs
                if ((a % 4096) + nbytes > 4096) begin
                    first = 4096 - (a % 4096);
                    push_exp(ch, a,         first / 4,            dat, 0);
                    push_exp(ch, a + first, (nbytes - first) / 4, dat, first / 4);
                    n_split = n_split + 1;
                end else
                    push_exp(ch, a, nbytes / 4, dat, 0);
                off = off + nbytes;
                n_beats = n_beats + 1;
                @(negedge aclk);
                if (ch == 0) s0_tvalid = 0; else s1_tvalid = 0;
            end
            if (VERBOSE) $display("[%0t] ch%0d sess %0d: %0d beats, last %0d DW, gap %0d%%, base %h",
                                  $time, ch, s, nbeats, last_dw, gap, base[ch]);
        end
    endtask

    // ---------------- main ----------------
    integer s, t0, cyc, nsess, w0, w1, arm, nb0, nb1, ld0, ld1, g0, g1, i;
    initial begin
        if (!$value$plusargs("SEED=%d", SEED)) SEED = 1;
        SEED0 = SEED;
        if (!$value$plusargs("SESS_MAX=%d", SESS_MAX)) SESS_MAX = 6;
        if (!$value$plusargs("BEATS_MAX=%d", BEATS_MAX)) BEATS_MAX = 40;
        if (!$value$plusargs("GAP_MAX=%d", GAP_MAX)) GAP_MAX = 60;
        if (!$value$plusargs("VERBOSE=%d", VERBOSE)) VERBOSE = 0;
        if (!$value$plusargs("NOISE_PPM=%d", NOISE_PPM)) NOISE_PPM = 0;
        if (!$value$plusargs("BURST=%d", BURST)) BURST = 1;
        // decorrelate neighbouring seeds ($random's first draws track the seed)
        SEED = SEED * 1103515245 + 12345;
        for (i = 0; i < 8; i = i + 1) t0 = $random(SEED);
        exp_wr[0] = 0; exp_wr[1] = 0; exp_rd[0] = 0; exp_rd[1] = 0;
        msix_cnt[0] = 0; msix_cnt[1] = 0; cur_sess[0] = -1; cur_sess[1] = -1;
        for (i = 1; i <= 16; i = i + 1) n_last_sz[i] = 0;
        for (i = 1; i <= 7; i = i + 1) w_used[i] = 0;
        $display("tb_bridge_rand: SEED=%0d SESS_MAX=%0d BEATS_MAX=%0d GAP_MAX=%0d NOISE_PPM=%0d BURST=%0d",
                 SEED0, SESS_MAX, BEATS_MAX, GAP_MAX, NOISE_PPM, BURST);

        repeat (5) @(negedge clk); perst_n = 1;
        wait (link_trained === 1'b1);
        repeat (3) @(negedge clk); updatefc_received = 1; @(negedge clk); updatefc_received = 0;
        wait (axi_rst_n === 1'b1);
        cfg_write(12'h004, 32'h0000_0006);
        cfg_write(12'h050, 32'h8000_0000);
        repeat (20) @(negedge clk);

        msix_entry(2, msix_addr_of(0), msix_data_of(0));
        msix_entry(6, msix_addr_of(1), msix_data_of(1));
        bar0_write(16'h0010, 32'h0000_0602);       // MSI-X vector ch0=2 ch1=6
        bar0_write(16'h000C, 32'h00FF_FFFF);       // limit: never reached here (overflow is F5)

        noise_on = (NOISE_PPM > 0);          // F4: noise from here on (link is up, config done)
        nsess = urand(1, SESS_MAX);
        for (s = 0; s < nsess; s = s + 1) begin
            // F3: random weights, and sometimes only one channel
            w0 = urand(1, 7); w1 = urand(1, 7);
            w_used[w0] = w_used[w0] + 1; w_used[w1] = w_used[w1] + 1;
            arm = urand(0, 9);
            arm = (arm < 7) ? 3 : (arm < 8 ? 1 : 2);
            if (arm != 3) n_single_ch = n_single_ch + 1;
            nb0 = urand(1, BEATS_MAX); nb1 = urand(1, BEATS_MAX);
            ld0 = urand(1, 16);       ld1 = urand(1, 16);
            g0  = urand(0, GAP_MAX);  g1  = urand(0, GAP_MAX);
            if (arm[0]) n_last_sz[ld0] = n_last_sz[ld0] + 1;
            if (arm[1]) n_last_sz[ld1] = n_last_sz[ld1] + 1;
            n_sessions = n_sessions + 1;

            bar0_write(16'h0008, {16'h0, w1[7:0], w0[7:0]});
            if (arm[0]) begin
                base[0] = rand_base(0); cur_sess[0] = cur_sess[0] + 1;
                bar0_write(16'h0020, base[0][31:0]); bar0_write(16'h0024, base[0][63:32]);
            end
            if (arm[1]) begin
                base[1] = rand_base(1); cur_sess[1] = cur_sess[1] + 1;
                bar0_write(16'h0028, base[1][31:0]); bar0_write(16'h002C, base[1][63:32]);
            end
            bar0_write(16'h0000, arm);
            msix_expected = msix_expected + arm[0] + arm[1];
            t0 = 0; while ((session_active & arm[1:0]) !== arm[1:0] && t0 < 2000) begin @(posedge aclk); t0 = t0 + 1; end
            if ((session_active & arm[1:0]) !== arm[1:0])
                $display("[%0t] FAIL: session %0d never became active (arm=%b active=%b)", $time, s, arm[1:0], session_active);
            fork
                if (arm[0]) run_channel(0, cur_sess[0], nb0, ld0, g0);
                if (arm[1]) run_channel(1, cur_sess[1], nb1, ld1, g1);
            join
            t0 = 0; while (session_active !== 2'b00 && t0 < 4000) begin @(posedge aclk); t0 = t0 + 1; end
            // host polls BAR0 STATUS until both channels report idle before re-arming
            // No short cap here: under heavy channel noise a session can take
            // milliseconds to drain (replays). The global watchdog catches a real hang.
            while (dut.u_lpb.u_pcie_phy_top.session_valid !== 2'b00) @(posedge clk);
            repeat (urand(1, 200)) @(posedge clk);
        end
        cyc = 0;
        while ((exp_rd[0] < exp_wr[0] || exp_rd[1] < exp_wr[1] || msix_seen < msix_expected) && cyc < 5000000) begin
            @(posedge clk); cyc = cyc + 1;
        end
        repeat (2000) @(posedge clk);

        $display("------------------------------------------------------------");
        $display(" seed %0d: %0d sessions (%0d single-channel), %0d beats, %0d beats split at 4 KB",
                 SEED0, n_sessions, n_single_ch, n_beats, n_split);
        $display(" data TLPs expected     : ch0 %0d  ch1 %0d", exp_wr[0], exp_wr[1]);
        $display(" data TLPs received ok  : %0d   mismatched: %0d   unexpected: %0d   still missing: %0d",
                 ok, bad, extra, (exp_wr[0]-exp_rd[0]) + (exp_wr[1]-exp_rd[1]));
        $display(" MSI-X received         : %0d of %0d   bad data: %0d   arrived before its data: %0d",
                 msix_seen, msix_expected, msix_bad, msix_early);
        $display(" channel: %0d symbol errors injected -> FEC corrected %0d, uncorrectable %0d, CRC fails %0d, replays %0d",
                 n_noise, n_fec_fix, n_fec_bad, n_crc, n_replay);
        $display(" errors now: framing=%b lane=%b overflow=%b", phy_framing_err, phy_lane_err, session_overflow);
        if (bad == 0 && extra == 0 && exp_rd[0] == exp_wr[0] && exp_rd[1] == exp_wr[1]
            && msix_seen == msix_expected && msix_bad == 0 && msix_early == 0)
            $display(" RESULT: PASS (seed %0d)", SEED0);
        else
            $display(" RESULT: FAIL (seed %0d -- replay with +SEED=%0d)", SEED0, SEED0);
        $finish;
    end

    // watchdog: 50 ms of simulated time (clean runs need < 3 ms; heavy noise up to ~20 ms)
    initial begin #50_000_000; $display("GLOBAL TIMEOUT (seed %0d): exp_rd %0d/%0d of %0d/%0d, session_valid %b",
                 SEED0, exp_rd[0], exp_rd[1], exp_wr[0], exp_wr[1], dut.u_lpb.u_pcie_phy_top.session_valid);
                 $display(" RESULT: FAIL"); $finish; end
endmodule
