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
//   F5  session-limit overflow at a random point (+OVF_PCT, default 20% of
//       sessions): the limit is set below one channel's session size; the
//       fabric gives up when session_overflow rises. No beat may be accepted
//       past the limit, everything accepted must still arrive, and the
//       session must still end with its MSI-X.
//   F6  lane reversal (+REVERSE=0/1, default random 25%): the loopback swaps
//       lanes 0<->3, 1<->2; training must detect it and data must still flow.
//   F7  hot reset in the middle of a session (+HOTRESET_PCT, default 10% of
//       sessions): in-flight data is lost by design, but the link must
//       retrain, the host re-configures, nothing stale arrives afterwards and
//       every later session must be clean.
//   F8  MSI-X masking (+MASK_PCT, default 20% of sessions): one channel's
//       vector is masked (vector-control bit) or the whole function is masked
//       (Message Control bit 30) before the session. When it ends, no MSI-X
//       may come out and the PBA bit must be set; after unmasking, exactly one
//       MSI-X must come out and the PBA bit must clear.
//   F9  host-driven BAR0 (+HOSTWR_PCT, default 30% of sessions): one register
//       the session depends on (ch1 base low, arbiter weights or TC) is written
//       by an inbound MWr through the link -- a 1-DW DMA on ch0 aimed at BAR0
//       (BAR0 = 0x8000_0000) -- instead of the test port, then read back. For
//       ch1's base the data TLPs only land at the right address if it worked.
//   F7b link drop without hot reset (K6; +LINKDROP_PCT, default 10% of
//       sessions), at a random point of a session, one of:
//         cable pull  rx_level_valid low on all lanes for 200..5000 clk. The
//                     LTSSM must leave L0 (signal loss) while the cable is
//                     out; after it is back the link retrains, the host
//                     re-configures, nothing stale arrives, later sessions
//                     are clean. A pull that does not drop the link FAILS.
//         glitch      rx_level_valid low for 2..40 clk (< LOSS_CLKS). Either
//                     outcome is legal (survive with replays, or lost framing
//                     -> link down -> retrain), but data must never be wrong.
//       Any link drop with no pull/glitch/hot reset in progress FAILS (e.g.
//       channel noise must not take the link down).
//
// Step 3 C coverage: every run prints one "COV:" line of bin counters
// (BAR0 register writes/reads, 4 KB split / no split, last-beat sizes,
// contested arbiter wins per weight, credit stalls, FEC/CRC/replay, MSI-X
// masks, hot resets, link drops by reason, overflow, lane order, inbound BAR0
// writes). ci/coverage_report.py merges them over all seeds and fails if a
// required bin stayed empty.
//
// Reference model, per accepted beat at address A with N bytes:
//   one MWr of N bytes, or -- if [A, A+N) crosses a 4 KB boundary -- two MWrs:
//   (4096 - A%4096) bytes at A, then the rest at the boundary.
//
// plusargs: +SEED=n (default 1)  +SESS_MAX=n (6)  +BEATS_MAX=n (40)
//           +GAP_MAX=pct (60)  +NOISE_PPM=n (0)  +BURST=k (1)  +OVF_PCT=p (20)
//           +REVERSE=0/1 (random)  +HOTRESET_PCT=p (10)  +MASK_PCT=p (20)
//           +HOSTWR_PCT=p (30)  +LINKDROP_PCT=p (10)  +VERBOSE=1
// Ends with RESULT: PASS / RESULT: FAIL and prints the seed to replay it.
// ============================================================
module tb_bridge_rand;
    localparam DATA_W = 512, KEEP_W = 64;

    reg clk = 0, aclk = 0, perst_n = 0;
    always #2   clk  = ~clk;     // 250 MHz
    always #2.7 aclk = ~aclk;    // ~185 MHz, unrelated

    integer SEED, SEED0, SESS_MAX, BEATS_MAX, GAP_MAX, VERBOSE, NOISE_PPM, BURST, OVF_PCT, REVERSE, HOTRESET_PCT, MASK_PCT, HOSTWR_PCT, LINKDROP_PCT;
    localparam [31:0] BAR0 = 32'h8000_0000;
    reg bar0_rd_en = 0; wire [31:0] bar0_rdata; wire bar0_rd_valid; reg tbl_wr_mask = 0;
    reg hot_reset_request = 0;

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

    localparam LVL_W = 384, SPL = 32, LPW = 96;   // wide PHY level bus (bridge_top PHY_WIDE=1)
    wire [LVL_W-1:0] lvl; wire [3:0] lvl_v;
    reg  cable_cut = 0;                        // F7b: receiver sees no signal on any lane
    wire [3:0] rx_v = cable_cut ? 4'b0000 : lvl_v;
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
        .hot_reset_request(hot_reset_request), .flr_request(1'b0), .updatefc_received(updatefc_received),
        .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n),
        .cfg_wr_en(cfg_wr_en), .cfg_rd_en(1'b0), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
        .cfg_rdata(), .cfg_cpl_valid(),
        .bar0_wr_en(bar0_wr_en), .bar0_rd_en(bar0_rd_en), .bar0_addr(bar0_addr), .bar0_wdata(bar0_wdata),
        .bar0_rdata(bar0_rdata), .bar0_rd_valid(bar0_rd_valid),
        .tbl_wr_en(tbl_wr_en), .tbl_wr_vec(tbl_wr_vec), .tbl_wr_addr(tbl_wr_addr),
        .tbl_wr_data(tbl_wr_data), .tbl_wr_mask(tbl_wr_mask),
        .tx_level(lvl), .tx_level_valid(lvl_v), .rx_level(rx_lvl), .rx_level_valid(rx_v),   // SerDes loopback + F4 noise + F7b pull
        .err_xlat_miss(err_xlat_miss), .err_addr_ovf(err_addr_ovf), .crc_err(crc_err), .fec_err(fec_err),
        .phy_framing_err(phy_framing_err), .phy_lane_err(phy_lane_err)
    );

    // ---------------- F4: PAM4 channel noise ----------------
    // Each lane carries a signed 3-bit level (-3,-1,+1,+3). A hit moves it to
    // an adjacent level, i.e. one Gray bit flips -- the common real-world error.
    reg         lane_rev = 0;        // F6
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
    // Step 4 item 3: the wide PHY carries SPL symbols per lane per clock
    // (lane gl symbol gs at [LPW*gl + 3*gs +: 3]). Noise is per symbol at
    // NOISE_PPM per million, as before; a burst runs over consecutive symbols
    // (into the next clock if needed).
    wire [LVL_W-1:0] rx_lvl;
    reg  [4*SPL-1:0] sym_hit;          // lane gl symbol gs at [SPL*gl + gs]
    reg  [4*SPL-1:0] sym_dir;
    genvar gl, gs;
    generate for (gl = 0; gl < 4; gl = gl + 1) begin : g_noise
        for (gs = 0; gs < SPL; gs = gs + 1) begin : g_sym
            // F6: with lane_rev the receiver's lane gl is the transmitter's lane 3-gl
            wire [2:0] tx_l = lane_rev ? lvl[LPW*(3-gl) + 3*gs +: 3] : lvl[LPW*gl + 3*gs +: 3];
            assign rx_lvl[LPW*gl + 3*gs +: 3] = sym_hit[SPL*gl + gs] ? adj_level(tx_l, sym_dir[SPL*gl + gs]) : tx_l;
        end
    end endgenerate
    initial for (nl = 0; nl < 4; nl = nl + 1) burst_left[nl] = 0;
    always @(negedge clk) begin : noise_gen
        integer si, dir0;
        sym_hit = {4*SPL{1'b0}};
        for (nl = 0; nl < 4; nl = nl + 1) begin
            noise_hit[nl] = 1'b0;
            if (noise_on && lvl_v[nl]) begin
                si = 0;
                if (burst_left[nl] == 0 && urand(0, 999_999) < NOISE_PPM * SPL) begin
                    si = urand(0, SPL - 1); burst_left[nl] = BURST; noise_dir[nl] = urand(0, 1);
                    n_noise = n_noise + 1;
                end
                while (burst_left[nl] > 0 && si < SPL) begin
                    sym_hit[SPL*nl + si] = 1'b1; sym_dir[SPL*nl + si] = noise_dir[nl];
                    noise_hit[nl] = 1'b1;
                    burst_left[nl] = burst_left[nl] - 1; si = si + 1;
                end
            end
        end
    end
    integer n_fec_fix = 0, n_fec_bad = 0, n_crc = 0, n_replay = 0, n_nak = 0;
    integer n_ovf_ch [0:1]; integer exp_cnt [0:9];
    reg [1:0] ovf_q2 = 2'b00;
    initial begin n_ovf_ch[0] = 0; n_ovf_ch[1] = 0; end
    always @(posedge aclk) begin
        if (session_overflow[0] && !ovf_q2[0]) n_ovf_ch[0] = n_ovf_ch[0] + 1;
        if (session_overflow[1] && !ovf_q2[1]) n_ovf_ch[1] = n_ovf_ch[1] + 1;
        ovf_q2 = session_overflow;
    end
    always @(posedge clk) begin
        if (dut.u_lpb.ll_nak_evt) n_nak = n_nak + 1;
        if (dut.u_lpb.fec_corrected) n_fec_fix = n_fec_fix + 1;
        if (fec_err) n_fec_bad = n_fec_bad + 1;
        if (crc_err) n_crc = n_crc + 1;
        if (dut.u_lpb.replay_event) n_replay = n_replay + 1;
    end

    // ---------------- Step 3 C: coverage monitors ----------------
    integer cv_wr [0:15], cv_rd [0:15];       // BAR0 0x000-0x02C by DW index (test port + inbound MWr)
    integer cv_win0 [1:7], cv_win1 [1:7];     // contested arbiter grants won, by the winner's weight
    integer cv_credit_stall = 0, cv_stall_ep = 0, cv_framing = 0, cv_lane = 0, cv_pba_rd = 0, cv_xlat = 0;
    integer cv_hw [1:3];                      // F9 inbound BAR0 writes by kind (weights, TC, ch1 base)
    reg     stall_q = 0, fr_q = 0, ln_q = 0;
    initial begin
        for (nl = 0; nl < 16; nl = nl + 1) begin cv_wr[nl] = 0; cv_rd[nl] = 0; end
        for (nl = 1; nl <= 7; nl = nl + 1) begin cv_win0[nl] = 0; cv_win1[nl] = 0; end
        for (nl = 1; nl <= 3; nl = nl + 1) cv_hw[nl] = 0;
    end
    always @(posedge clk) begin : cov_clk
        reg st;
        if (dut.u_lpb.u_pcie_phy_top.u_bar0.bar0_wr_en === 1'b1 && dut.u_lpb.u_pcie_phy_top.u_bar0.bar0_addr[11:6] == 0)
            cv_wr[dut.u_lpb.u_pcie_phy_top.u_bar0.bar0_addr[5:2]] = cv_wr[dut.u_lpb.u_pcie_phy_top.u_bar0.bar0_addr[5:2]] + 1;
        if (dut.u_lpb.u_pcie_phy_top.u_bar0.bar0_rd_en === 1'b1 && dut.u_lpb.u_pcie_phy_top.u_bar0.bar0_addr[11:6] == 0)
            cv_rd[dut.u_lpb.u_pcie_phy_top.u_bar0.bar0_addr[5:2]] = cv_rd[dut.u_lpb.u_pcie_phy_top.u_bar0.bar0_addr[5:2]] + 1;
        st = (dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.credit_alloc_valid === 1'b1)
          && (dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.credit_grant !== 1'b1);
        if (st) cv_credit_stall = cv_credit_stall + 1;
        if (st && !stall_q) cv_stall_ep = cv_stall_ep + 1;
        stall_q = st;
        if (phy_framing_err === 1'b1 && !fr_q) cv_framing = cv_framing + 1;
        if (phy_lane_err === 1'b1 && !ln_q) cv_lane = cv_lane + 1;
        fr_q = (phy_framing_err === 1'b1); ln_q = (phy_lane_err === 1'b1);
        if (err_xlat_miss === 1'b1) begin
            cv_xlat = cv_xlat + 1;
            if (VERBOSE && cv_xlat < 4) $display("[%0t] translation miss (err_xlat_miss) #%0d", $time, cv_xlat);
        end
    end
    always @(posedge aclk) begin : cov_arb
        integer w;
        if (dut.u_ic.u_arb.req0 === 1'b1 && dut.u_ic.u_arb.req1 === 1'b1) begin
            if (dut.u_ic.u_arb.grant0 === 1'b1) begin
                w = dut.u_ic.u_arb.weight0; if (w >= 1 && w <= 7) cv_win0[w] = cv_win0[w] + 1;
            end
            if (dut.u_ic.u_arb.grant1 === 1'b1) begin
                w = dut.u_ic.u_arb.weight1; if (w >= 1 && w <= 7) cv_win1[w] = cv_win1[w] + 1;
            end
        end
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
    // Step 4 item 2: the reference model is a DW stream per channel. The bridge
    // may cut it into TLPs anywhere (merging up to Max Payload, splits at 4 KB),
    // so a received TLP is checked DW by DW against the stream, plus the legality
    // rules: length <= MPS, no 4 KB crossing, never two sessions in one TLP.
    localparam QD = Q * 16;
    localparam MPS_DW = 64;
    reg [63:0]  exp_addr [0:1][0:QD-1];
    reg [31:0]  exp_dw   [0:1][0:QD-1];
    integer     exp_sess [0:1][0:QD-1];
    integer     exp_t    [0:1][0:QD-1];     // $time the beat was accepted (latency)
    integer exp_wr [0:1], exp_rd [0:1];     // DW indices
    integer n_tlp_rx = 0, max_tlp_dw = 0;
    // latency samples (ns, per TLP: last DW accepted -> TLP received)
    localparam LS = 65536;
    integer lat_s [0:LS-1];
    integer lat_n = 0;
    integer lat_c [0:2*LS-1];               // per channel (ch*LS + n)
    integer lat_cn [0:1];
    integer rxb [0:1], rx_t0 [0:1], rx_t1 [0:1], n_tlp_ch [0:1];
    initial begin lat_cn[0] = 0; lat_cn[1] = 0; rxb[0] = 0; rxb[1] = 0; rx_t0[0] = 0; rx_t0[1] = 0;
                  rx_t1[0] = 0; rx_t1[1] = 0; n_tlp_ch[0] = 0; n_tlp_ch[1] = 0; end
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
        integer pe; begin
            for (pe = 0; pe < ndw; pe = pe + 1) begin
                exp_addr[ch][exp_wr[ch] % QD] = a + 4 * pe;
                exp_dw  [ch][exp_wr[ch] % QD] = dat[(dw0 + pe) * 32 +: 32];
                exp_sess[ch][exp_wr[ch] % QD] = cur_sess[ch];
                exp_t   [ch][exp_wr[ch] % QD] = $time;
                exp_wr[ch] = exp_wr[ch] + 1;
            end
        end
    endtask

    // Step 5: per-session (packet) latency = MSI-X arrival - first beat accepted
    integer sess_t0 [0:1][0:4095];
    integer pl_s [0:2*LS-1];
    integer pl_n [0:1], msix_t1 [0:1], msix_n [0:1];
    integer si_;
    initial begin
        for (si_ = 0; si_ < 4096; si_ = si_ + 1) begin sess_t0[0][si_] = -1; sess_t0[1][si_] = -1; end
        pl_n[0] = 0; pl_n[1] = 0; msix_t1[0] = 0; msix_t1[1] = 0; msix_n[0] = 0; msix_n[1] = 0;
    end

    // ---------------- RX checker (same matching as tb_bridge_top) ----------------
    reg  [31:0] got [0:127];
    integer got_n = 0, ok = 0, bad = 0, extra = 0, k;
    integer n_bar0_tlp = 0;          // F9
    integer xl_n = 0, xl_b0 = 0;     // Step 3 C: inbound TLPs since XLAT_MISS was last cleared (other / BAR0)
    reg rx_ignore = 0;               // F7: between a hot reset and the end of re-configuration
    integer n_ignored = 0;
    always @(posedge clk) if (rx_tvalid && rx_ignore) begin
        if (rx_tlast) n_ignored = n_ignored + 1;
        got_n = 0;
    end else if (rx_tvalid) begin
        for (k = 0; k < 16; k = k + 1)
            if (rx_tkeep[k*4]) begin got[got_n] = rx_tdata[k*32 +: 32]; got_n = got_n + 1; end
        if (rx_tlast) begin : chk
            reg [63:0] a; integer ch, ndw, m, i; reg b;
            a   = {got[2], got[3]};
            ndw = got[0][9:0];
            if (a[63:16] == {32'h0, BAR0[31:16]}) xl_b0 = xl_b0 + 1; else xl_n = xl_n + 1;
            if (ndw == 1 && (a == msix_addr_of(0) || a == msix_addr_of(1))) begin
                ch = (a == msix_addr_of(0)) ? 0 : 1;
                msix_seen = msix_seen + 1;
                if (got[4] !== msix_data_of(ch)) msix_bad = msix_bad + 1;
                if (exp_rd[ch] < exp_wr[ch] && exp_sess[ch][exp_rd[ch] % QD] <= msix_cnt[ch]) msix_early = msix_early + 1;
                if (sess_t0[ch][msix_cnt[ch] % 4096] >= 0 && pl_n[ch] < LS) begin
                    pl_s[ch*LS + pl_n[ch]] = $time - sess_t0[ch][msix_cnt[ch] % 4096]; pl_n[ch] = pl_n[ch] + 1;
                end
                msix_t1[ch] = $time; msix_n[ch] = msix_n[ch] + 1;
                msix_cnt[ch] = msix_cnt[ch] + 1;
            end else if (a[63:16] == {32'h0, BAR0[31:16]}) begin
                n_bar0_tlp = n_bar0_tlp + 1;            // F9: our own BAR0 write, looped back
            end else begin
                ch = (a[39:32] == 8'hAB) ? 0 : (a[39:32] == 8'hCD) ? 1 : -1;
                if (ch < 0 || exp_rd[ch] + ndw > exp_wr[ch]) begin
                    extra = extra + 1;
                    if (extra < 5) $display("[%0t] UNEXPECTED TLP addr=%h ndw=%0d", $time, a, ndw);
                end else begin
                    i = exp_rd[ch];
                    b = (got_n != 4 + ndw) || (got[0][31:29] !== 3'b011) || (ndw > MPS_DW) || (ndw < 1)
                        || ((a % 4096) + 4 * ndw > 4096);
                    for (m = 0; m < ndw && m < 124; m = m + 1)
                        if (got[4+m] !== exp_dw[ch][(i+m) % QD] || (a + 4*m) !== exp_addr[ch][(i+m) % QD]
                            || exp_sess[ch][(i+m) % QD] != exp_sess[ch][i % QD]) b = 1;
                    if (b) begin
                        bad = bad + 1;
                        if (bad < 6) $display("[%0t] ch%0d DW %0d MISMATCH: got TLP addr %h ndw %0d / exp addr %h (sess %0d)",
                                              $time, ch, i, a, ndw, exp_addr[ch][i % QD], exp_sess[ch][i % QD]);
                    end else begin
                        ok = ok + 1;
                        if (lat_n < LS) begin lat_s[lat_n] = $time - exp_t[ch][(i + ndw - 1) % QD]; lat_n = lat_n + 1; end
                        if (lat_cn[ch] < LS) begin lat_c[ch*LS + lat_cn[ch]] = $time - exp_t[ch][(i + ndw - 1) % QD]; lat_cn[ch] = lat_cn[ch] + 1; end
                        if (rxb[ch] == 0) rx_t0[ch] = $time;
                        rxb[ch] = rxb[ch] + 4 * ndw; rx_t1[ch] = $time; n_tlp_ch[ch] = n_tlp_ch[ch] + 1;
                    end
                    if (ndw > max_tlp_dw) max_tlp_dw = ndw;
                    exp_rd[ch] = i + ndw;
                end
            end
            got_n = 0;
        end
    end

    // ---------------- host helpers ----------------
    task cfg_write(input [11:0] a, input [31:0] d); begin
        @(negedge clk); cfg_addr = a; cfg_wdata = d; cfg_wr_en = 1; @(negedge clk); cfg_wr_en = 0;
    end endtask
    reg [31:0] mir0 = 0, mir1 = 0; reg mir0_ok = 0, mir1_ok = 0;   // Step 3 C: last base written
    task bar0_write(input [15:0] a, input [31:0] d); begin
        if (a == 16'h0020) begin mir0 = d; mir0_ok = 1; end
        if (a == 16'h0028) begin mir1 = d; mir1_ok = 1; end
        @(negedge clk); bar0_addr = a; bar0_wdata = d; bar0_wr_en = 1; @(negedge clk); bar0_wr_en = 0;
    end endtask
    task msix_entry(input [2:0] v, input [63:0] a, input [31:0] d); begin
        msix_entry_m(v, a, d, 1'b0);
    end endtask
    task msix_entry_m(input [2:0] v, input [63:0] a, input [31:0] d, input m); begin
        @(negedge clk); tbl_wr_vec = v; tbl_wr_addr = a; tbl_wr_data = d; tbl_wr_mask = m; tbl_wr_en = 1;
        @(negedge clk); tbl_wr_en = 0; tbl_wr_mask = 0;
    end endtask
    task bar0_read(input [15:0] a, output [31:0] d); integer n; begin
        @(negedge clk); bar0_addr = a; bar0_rd_en = 1; @(negedge clk); bar0_rd_en = 0;
        n = 0; while (!bar0_rd_valid && n < 50) begin @(posedge clk); n = n + 1; end
        d = bar0_rdata;
    end endtask

    function automatic [511:0] beat_data(input integer ch, input integer s, input integer b);
        integer d; begin
            for (d = 0; d < 16; d = d + 1) beat_data[d*32 +: 32] = {ch[3:0], s[3:0], b[11:0], d[11:0]};
        end
    endfunction

    // one channel's session on aclk: random beat count, last-beat size, gaps (F1)
    integer leaked = 0, n_ovf = 0, n_hotreset = 0;
    integer n_mask = 0, n_fmask = 0, mask_err = 0, n_hostwr = 0, hostwr_err = 0, reg_err = 0;
    reg     abort_all = 0;            // F7: hot reset -> fabric stops driving
    integer limit_now = 24'hFF_FFFF;  // F5: current session byte limit (ch0)
    integer limit_ch1 = 24'hFF_FFFF;  // Step 4 item 7: ch1's own limit (BAR0 0x34)
    integer n_lim1 = 0;
    task automatic run_channel(input integer ch, input integer s, input integer nbeats,
                               input integer last_dw, input integer gap);
        integer b, nbytes, first, ndw; reg [63:0] off, a; reg [63:0] kp; reg [511:0] dat; reg stop; begin
            off = 0; stop = 0;
            for (b = 0; b < nbeats && !stop && !abort_all; b = b + 1) begin
                kp  = (b == nbeats - 1) ? ({64{1'b1}} >> (64 - 4*last_dw)) : {64{1'b1}};
                dat = beat_data(ch, s, b);
                while (urand(0, 99) < gap) @(posedge aclk);
                @(negedge aclk);
                if (ch == 0) begin s0_tdata = dat; s0_tkeep = kp; s0_tlast = (b == nbeats-1); s0_tvalid = 1; end
                else         begin s1_tdata = dat; s1_tkeep = kp; s1_tlast = (b == nbeats-1); s1_tvalid = 1; end
                @(posedge aclk);
                // F5/F7: the fabric gives up on an overflowed or reset session
                while (!(ch == 0 ? s0_tready : s1_tready) && !stop && !abort_all) begin
                    if (session_overflow[ch]) stop = 1;
                    else @(posedge aclk);
                end
                if (stop || abort_all) begin
                    @(negedge aclk);
                    if (ch == 0) s0_tvalid = 0; else s1_tvalid = 0;
                    if (stop) begin
                        n_ovf = n_ovf + 1;
                        if (VERBOSE) $display("[%0t] ch%0d sess %0d: overflow after %0d B (limit %0d)", $time, ch, s, off, ch == 0 ? limit_now : limit_ch1);
                    end
                end else begin
                nbytes = (b == nbeats - 1) ? 4*last_dw : 64;
                if (off + nbytes > (ch == 0 ? limit_now : limit_ch1)) leaked = leaked + 1;   // F5: accepted past the limit: must never happen
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
            end
            if (VERBOSE) $display("[%0t] ch%0d sess %0d: %0d beats, last %0d DW, gap %0d%%, base %h",
                                  $time, ch, s, nbeats, last_dw, gap, base[ch]);
        end
    endtask


    // =================================================================
    // Step 5: network and ML workloads (+WL=1..5). Same DUT, loopback,
    // checker and host model as the random test; the random features
    // are off. Rates are computed at the 250 MHz design clock.
    //   WL=1  SmartNIC on ch0: IMIX 64 / 596 / 1520 B (7:4:1), bursty gaps
    //   WL=2  ML on ch1: tensors of +WL_KB KB (default 1024) from unaligned bases
    //   WL=3  Mixed: ML saturating ch1 while the NIC mix runs on ch0,
    //         arbiter weights +W0 (NIC) : +W1 (ML)
    //   WL=4  SmartNIC worst case: back-to-back 64 B packets
    //   WL=5  ML, 4 KB-crossing heavy: 64 KB tensors, bases 64 B below a 4 KB line
    // Packet sizes are rounded up to whole DWs (594 -> 596, 1518 -> 1520).
    // =================================================================
    integer WL, WL_PKTS, WL_KB, WL_TENSORS, W0, W1;
    reg     bar0_lock = 0;
    reg     nic_done = 0;
    integer wl_t0, wl_nic_t1, wl_rxb1_at0, wl_rxb1_at1, wl_rxb0_at1, pk_cdc, pk_wdb, pk_rpl, pk_rxf, n_pkts_sent;
    integer wl_bytes_sent [0:1];
    integer nic_slot;

    always @(posedge clk) if (WL != 0) begin : peaks
        integer v;
        v = dut.u_ic.u_fifo.occupancy;                                                                   if (v > pk_cdc) pk_cdc = v;
        v = dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.u_write_data_buffer.occupancy_r;            if (v > pk_wdb) pk_wdb = v;
        v = (dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.u_data_link_layer_top.u_replay_buffer.head_seq -
             dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.u_data_link_layer_top.u_replay_buffer.tail_seq) & 12'hFFF;
                                                                                                         if (v > pk_rpl) pk_rpl = v;
        v = dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.u_data_link_layer_top.rxf_cnt;              if (v > pk_rxf) pk_rxf = v;
    end

    // back-to-back beats on one channel (no idle cycle between beats)
    task automatic wl_send(input integer ch, input integer nbeats, input integer last_dw);
        integer b, nbytes; reg [63:0] off, a, kp; reg [511:0] dat; begin
            off = 0;
            for (b = 0; b < nbeats; b = b + 1) begin
                kp  = (b == nbeats - 1) ? ({64{1'b1}} >> (64 - 4*last_dw)) : {64{1'b1}};
                dat = beat_data(ch, cur_sess[ch], b);
                @(negedge aclk);
                if (ch == 0) begin s0_tdata = dat; s0_tkeep = kp; s0_tlast = (b == nbeats-1); s0_tvalid = 1; end
                else         begin s1_tdata = dat; s1_tkeep = kp; s1_tlast = (b == nbeats-1); s1_tvalid = 1; end
                @(posedge aclk);
                while (!(ch == 0 ? s0_tready : s1_tready)) @(posedge aclk);
                if (b == 0) sess_t0[ch][cur_sess[ch] % 4096] = $time;
                nbytes = (b == nbeats - 1) ? 4*last_dw : 64;
                a = base[ch] + off;
                push_exp(ch, a, nbytes / 4, dat, 0);
                if (((a % 4096) + nbytes) > 4096) n_split = n_split + 1;
                off = off + nbytes;
                n_beats = n_beats + 1;
                wl_bytes_sent[ch] = wl_bytes_sent[ch] + nbytes;
            end
            @(negedge aclk);
            if (ch == 0) s0_tvalid = 0; else s1_tvalid = 0;
        end
    endtask

    // one session (= one packet or one tensor): wait for the channel to be free,
    // program base, arm, stream it
    task automatic wl_session(input integer ch, input integer bytes, input [63:0] b_addr);
        integer dw, nb, ld, t; begin
            dw = (bytes + 3) / 4; nb = (dw + 15) / 16; ld = dw - 16 * (nb - 1);
            while (dut.u_lpb.u_pcie_phy_top.session_valid[ch] !== 1'b0 || session_active[ch] !== 1'b0) @(posedge clk);
            while (bar0_lock) @(negedge clk);
            bar0_lock = 1;
            base[ch] = b_addr;
            cur_sess[ch] = cur_sess[ch] + 1;
            if (ch == 0) begin bar0_write(16'h0020, b_addr[31:0]); bar0_write(16'h0024, b_addr[63:32]); end
            else         begin bar0_write(16'h0028, b_addr[31:0]); bar0_write(16'h002C, b_addr[63:32]); end
            bar0_write(16'h0000, ch == 0 ? 32'h1 : 32'h2);
            msix_expected = msix_expected + 1;
            n_sessions = n_sessions + 1;
            bar0_lock = 0;
            t = 0; while (session_active[ch] !== 1'b1 && t < 4000) begin @(posedge aclk); t = t + 1; end
            wl_send(ch, nb, ld);
        end
    endtask

    function integer imix_bytes(input integer dummy);
        integer r; begin
            r = urand(0, 11);
            imix_bytes = (r < 7) ? 64 : (r < 11) ? 596 : 1520;
        end
    endfunction

    task automatic wl_nic(input integer npk, input integer b2b);
        integer pk, g; begin
            for (pk = 0; pk < npk; pk = pk + 1) begin
                // bursty: bursts of 1..16 packets back to back, then 0..2 us idle
                if (!b2b && urand(0, 7) == 0) begin g = urand(0, 500); repeat (g) @(posedge clk); end
                nic_slot = (nic_slot + 1) % 4096;
                wl_session(0, b2b ? 64 : imix_bytes(0), 64'h0000_00AB_1000_0000 + nic_slot * 2048);
                n_pkts_sent = n_pkts_sent + 1;
            end
        end
    endtask

    function [63:0] ml_base(input integer near4k);
        reg [31:0] lo; begin
            lo = $random(SEED);
            lo[1:0] = 2'b00;
            lo[31:28] = lo[31:28] & 4'h7;             // leave room for a 16 MB tensor
            if (near4k) lo[11:0] = 12'hFC0;            // 64 B below a 4 KB line
            else if (lo[11:2] == 0) lo[11:2] = 10'd1;  // never 4 KB aligned
            ml_base = {24'h0, 8'hCD, lo};
        end
    endfunction

    task automatic wl_ml(input integer ntens, input integer kb, input integer near4k, input integer until_nic);
        integer t; begin
            t = 0;
            while ((until_nic && !nic_done) || (!until_nic && t < ntens)) begin
                wl_session(1, kb * 1024, ml_base(near4k));
                t = t + 1;
            end
        end
    endtask

    // ---- percentile helper (heap sort of a slice of an integer array) ----
    integer hs [0:LS-1];
    task hsort(input integer n);
        integer i2, j2, k2, tmp2, st; begin
            for (st = n/2 - 1; st >= 0; st = st - 1) begin
                k2 = st;
                while (2*k2 + 1 < n) begin
                    j2 = 2*k2 + 1; if (j2 + 1 < n && hs[j2+1] > hs[j2]) j2 = j2 + 1;
                    if (hs[k2] >= hs[j2]) k2 = n; else begin tmp2 = hs[k2]; hs[k2] = hs[j2]; hs[j2] = tmp2; k2 = j2; end
                end
            end
            for (i2 = n - 1; i2 > 0; i2 = i2 - 1) begin
                tmp2 = hs[0]; hs[0] = hs[i2]; hs[i2] = tmp2;
                k2 = 0;
                while (2*k2 + 1 < i2) begin
                    j2 = 2*k2 + 1; if (j2 + 1 < i2 && hs[j2+1] > hs[j2]) j2 = j2 + 1;
                    if (hs[k2] >= hs[j2]) k2 = i2; else begin tmp2 = hs[k2]; hs[k2] = hs[j2]; hs[j2] = tmp2; k2 = j2; end
                end
            end
        end
    endtask
    integer p50_, p99_, pmax_;
    task pct_lat(input integer ch); integer n2, q2; begin
        n2 = lat_cn[ch]; for (q2 = 0; q2 < n2; q2 = q2 + 1) hs[q2] = lat_c[ch*LS + q2];
        if (n2 > 0) begin hsort(n2); p50_ = hs[n2/2]; p99_ = hs[(n2*99)/100]; pmax_ = hs[n2-1]; end
        else begin p50_ = 0; p99_ = 0; pmax_ = 0; end
    end endtask
    task pct_pkt(input integer ch); integer n2, q2; begin
        n2 = pl_n[ch]; for (q2 = 0; q2 < n2; q2 = q2 + 1) hs[q2] = pl_s[ch*LS + q2];
        if (n2 > 0) begin hsort(n2); p50_ = hs[n2/2]; p99_ = hs[(n2*99)/100]; pmax_ = hs[n2-1]; end
        else begin p50_ = 0; p99_ = 0; pmax_ = 0; end
    end endtask

    task run_workload;
        real ns, bpc0, bpc1, sh0, sh1, tg0, tg1, x0, x1, jain;
        integer c, tl0, tl1, tl99_0, tl99_1, pk50, pk99, cyc2;
        begin
            OVF_PCT = 0; HOTRESET_PCT = 0; LINKDROP_PCT = 0; MASK_PCT = 0; HOSTWR_PCT = 0;
            pk_cdc = 0; pk_wdb = 0; pk_rpl = 0; pk_rxf = 0; n_pkts_sent = 0; nic_slot = 0;
            wl_bytes_sent[0] = 0; wl_bytes_sent[1] = 0;
            bar0_write(16'h0008, {16'h0, W1[7:0], W0[7:0]});
            bar0_write(16'h000C, 32'h00FF_FFFF);
            noise_on = (NOISE_PPM > 0);
            wl_t0 = $time;
            case (WL)
                1: wl_nic(WL_PKTS, 0);
                4: wl_nic(WL_PKTS, 1);
                2: wl_ml(WL_TENSORS, WL_KB, 0, 0);
                5: wl_ml(WL_TENSORS, 64, 1, 0);
                3: fork
                       begin wl_nic(WL_PKTS, 0); nic_done = 1; end
                       wl_ml(0, WL_KB, 0, 1);
                   join
            endcase
            cyc2 = 0;
            while ((exp_rd[0] < exp_wr[0] || exp_rd[1] < exp_wr[1] || msix_seen < msix_expected) && cyc2 < 20000000) begin
                @(posedge clk); cyc2 = cyc2 + 1;
                if (WL == 3 && exp_rd[0] >= exp_wr[0] && wl_nic_t1 == 0 && nic_done) begin
                    wl_nic_t1 = $time; wl_rxb1_at1 = rxb[1]; wl_rxb0_at1 = rxb[0];
                end
            end
            if (WL == 3 && wl_nic_t1 == 0) begin wl_nic_t1 = $time; wl_rxb1_at1 = rxb[1]; wl_rxb0_at1 = rxb[0]; end
            repeat (200) @(posedge clk);

            ns   = ($time) - wl_t0;
            bpc0 = (rx_t1[0] > rx_t0[0]) ? rxb[0] * 4.0 / (rx_t1[0] - rx_t0[0]) : 0.0;
            bpc1 = (rx_t1[1] > rx_t0[1]) ? rxb[1] * 4.0 / (rx_t1[1] - rx_t0[1]) : 0.0;
            $display("------------------------------------------------------------");
            $display(" Step 5 workload WL=%0d  seed %0d  noise %0d ppm  weights NIC:ML = %0d:%0d", WL, SEED0, NOISE_PPM, W0, W1);
            $display(" sessions %0d (NIC packets %0d), beats %0d, 4 KB crossings %0d, run %0.1f us", n_sessions, n_pkts_sent, n_beats, n_split, ns / 1000.0);
            pct_lat(0); tl0 = p50_; tl99_0 = p99_;
            pct_lat(1); tl1 = p50_; tl99_1 = p99_;
            pct_pkt(0); pk50 = p50_; pk99 = p99_;
            $display(" ch0 (NIC): %0d B, %0.2f B/clk = %0.2f GB/s, %0.3f Mpps, TLP latency p50 %0d ns p99 %0d ns, packet latency p50 %0d ns p99 %0d ns",
                     rxb[0], bpc0, bpc0 * 0.25, (rx_t1[0] > rx_t0[0]) ? msix_n[0] * 1000.0 / (rx_t1[0] - rx_t0[0] + 1) : 0.0, tl0, tl99_0, pk50, pk99);
            $display(" ch1 (ML) : %0d B, %0.2f B/clk = %0.2f GB/s, TLP latency p50 %0d ns p99 %0d ns",
                     rxb[1], bpc1, bpc1 * 0.25, tl1, tl99_1);
            $display(" interrupts: %0d MSI-X, %0.3f M/s   TLPs ch0 %0d ch1 %0d", msix_seen, msix_seen * 1000.0 / ns, n_tlp_ch[0], n_tlp_ch[1]);
            $display(" peaks: CDC FIFO %0d, write buffer %0d, replay outstanding %0d, RX Flit FIFO %0d   replays %0d  FEC fixed %0d  credit stall clk %0d",
                     pk_cdc, pk_wdb, pk_rpl, pk_rxf, n_replay, n_fec_fix, cv_credit_stall);
            if (WL == 3) begin
                sh0 = wl_rxb0_at1 * 1.0 / (wl_rxb0_at1 + wl_rxb1_at1 + 1);
                sh1 = 1.0 - sh0;
                tg0 = W0 * 1.0 / (W0 + W1); tg1 = 1.0 - tg0;
                x0 = sh0 / tg0; x1 = sh1 / tg1;
                jain = (x0 + x1) * (x0 + x1) / (2.0 * (x0*x0 + x1*x1));
                $display(" mixed: bandwidth split while both active NIC %0.1f%% / ML %0.1f%% (weights %0.1f%% / %0.1f%%), Jain fairness %0.3f",
                         sh0 * 100.0, sh1 * 100.0, tg0 * 100.0, tg1 * 100.0, jain);
            end
            // machine-readable line for ci/load (Step 5)
            $write("LOAD: wl=%0d seed=%0d noise=%0d w0=%0d w1=%0d run_ns=%0.0f", WL, SEED0, NOISE_PPM, W0, W1, ns);
            $write(" ch0_bpc=%0.3f ch1_bpc=%0.3f ch0_gbps=%0.3f ch1_gbps=%0.3f", bpc0, bpc1, bpc0 * 0.25, bpc1 * 0.25);
            $write(" ch0_mpps=%0.4f msix_mps=%0.4f", (rx_t1[0] > rx_t0[0]) ? msix_n[0] * 1000.0 / (rx_t1[0] - rx_t0[0] + 1) : 0.0, msix_seen * 1000.0 / ns);
            $write(" ch0_lat50=%0d ch0_lat99=%0d ch1_lat50=%0d ch1_lat99=%0d pkt_lat50=%0d pkt_lat99=%0d", tl0, tl99_0, tl1, tl99_1, pk50, pk99);
            $write(" pk_cdc=%0d pk_wdb=%0d pk_rpl=%0d pk_rxf=%0d replays=%0d fec_fix=%0d credit_stall=%0d tlps0=%0d tlps1=%0d",
                   pk_cdc, pk_wdb, pk_rpl, pk_rxf, n_replay, n_fec_fix, cv_credit_stall, n_tlp_ch[0], n_tlp_ch[1]);
            if (WL == 3) $write(" share0=%0.4f share1=%0.4f jain=%0.4f", sh0, sh1, jain);
            $write("\n");
            $display(" data TLPs received ok  : %0d   mismatched: %0d   unexpected: %0d   still missing DWs: %0d",
                     ok, bad, extra, (exp_wr[0]-exp_rd[0]) + (exp_wr[1]-exp_rd[1]));
            $display(" MSI-X received         : %0d of %0d   bad data: %0d   arrived before its data: %0d", msix_seen, msix_expected, msix_bad, msix_early);
            if (bad == 0 && extra == 0 && exp_rd[0] == exp_wr[0] && exp_rd[1] == exp_wr[1] &&
                msix_seen == msix_expected && msix_bad == 0 && msix_early == 0 && drop_err == 0)
                $display(" RESULT: PASS (workload %0d seed %0d)", WL, SEED0);
            else
                $display(" RESULT: FAIL (workload %0d seed %0d)", WL, SEED0);
        end
    endtask

    // ---------------- F7b: link-drop watcher ----------------
    // A falling link_trained outside a hot reset is only legal while a cable
    // pull or glitch is in progress (drop_ok). Then the fabric stops and the
    // session recovers like F7; otherwise it is a FAIL.
    reg     drop_ok = 0, watch_on = 0, lt_q = 0;
    integer n_cut = 0, n_glitch = 0, n_glitch_drop = 0, n_drop = 0, drop_err = 0;
    integer n_exit [0:3];
    initial for (nl = 0; nl < 4; nl = nl + 1) n_exit[nl] = 0;
    always @(posedge clk) begin
        if (watch_on && lt_q === 1'b1 && link_trained === 1'b0 && !abort_all) begin
            if (!drop_ok) begin
                drop_err = drop_err + 1;
                $display("[%0t] F7b FAIL: link went down with no cable pull, glitch or hot reset", $time);
            end
            n_drop = n_drop + 1;
            rx_ignore = 1; abort_all = 1;
            if (VERBOSE) $display("[%0t] F7b: link down, fabric stops", $time);
        end
        lt_q = link_trained;
        if (dut.u_lpb.u_pcie_phy_top.u_ltssm.l0_exit === 1'b1)
            n_exit[dut.u_lpb.u_pcie_phy_top.u_ltssm.exit_reason] = n_exit[dut.u_lpb.u_pcie_phy_top.u_ltssm.exit_reason] + 1;
    end

    // ---------------- main ----------------
    integer s, t0, cyc, nsess, w0, w1, arm, nb0, nb1, ld0, ld1, g0, g1, i, hr, hr_at;
    integer mk, mk_ch, mk_fn, hw, msix_before, hr_hit; reg [31:0] pba;
    integer ld, ld_at, ld_kind, ld_len, ld_t, rg, n_s13 = 0; reg [31:0] rv; reg [7:0] rb8;

    // F9: write one BAR0 register through the link: a 1-beat, 1-DW session on
    // ch0 whose DMA address is BAR0 + off. Its own MSI-X is accounted for.
    task host_mwr(input [15:0] off, input [31:0] val);
        reg [31:0] rb; integer t; begin
            n_hostwr = n_hostwr + 1;
            cur_sess[0] = cur_sess[0] + 1;
            bar0_write(16'h000C, 24'hFF_FFFF);
            bar0_write(16'h0020, BAR0 + off); bar0_write(16'h0024, 32'h0);
            bar0_write(16'h0000, 32'h1);
            t = 0; while (session_active[0] !== 1'b1 && t < 2000) begin @(posedge aclk); t = t + 1; end
            @(negedge aclk);
            s0_tdata = {480'h0, val}; s0_tkeep = 64'hF; s0_tlast = 1; s0_tvalid = 1;
            @(posedge aclk); while (!s0_tready) @(posedge aclk);
            @(negedge aclk); s0_tvalid = 0; s0_tlast = 0;
            msix_expected = msix_expected + 1;
            if (off == 16'h0028) begin mir1 = val; mir1_ok = 1; end
            while (dut.u_lpb.u_pcie_phy_top.session_valid[0] !== 1'b0) @(posedge clk);
            // the write lands when its TLP comes back through the link (slow under noise):
            // wait until the checker has seen it, then let the RX decode finish
            t = 0; while (n_bar0_tlp < n_hostwr && t < 2000000) begin @(posedge clk); t = t + 1; end
            repeat (100) @(posedge clk);
            bar0_read(off, rb);
            if (rb !== val) begin
                hostwr_err = hostwr_err + 1;
                $display("[%0t] F9 FAIL: BAR0+%h written %h by inbound MWr, reads back %h", $time, off, val, rb);
            end else if (VERBOSE) $display("[%0t] F9: BAR0+%h <= %h via inbound MWr, read back OK", $time, off, val);
        end
    endtask

    // host bring-up (power-on, and again after a hot reset): wait for link,
    // first FC, then what a driver does: Memory Space + Bus Master, MSI-X on,
    // MSI-X table entries, vectors
    task host_bringup; begin
        wait (link_trained === 1'b1);
        repeat (3) @(negedge clk); updatefc_received = 1; @(negedge clk); updatefc_received = 0;
        wait (axi_rst_n === 1'b1);
        cfg_write(12'h004, 32'h0000_0006);
        cfg_write(12'h010, BAR0);                  // F9: BAR0 at 0x8000_0000 (64 KB)
        cfg_write(12'h050, 32'h8000_0000);
        repeat (20) @(negedge clk);
        msix_entry(2, msix_addr_of(0), msix_data_of(0));
        msix_entry(6, msix_addr_of(1), msix_data_of(1));
        bar0_write(16'h0010, 32'h0000_0602);       // MSI-X vector ch0=2 ch1=6
        bar0_write(16'h000C, 32'h00FF_FFFF);
        bar0_write(16'h001C, 32'h0);               // XLAT_MISS: start counting from 0
        xl_n = 0; xl_b0 = 0;
    end endtask
    initial begin
        if (!$value$plusargs("SEED=%d", SEED)) SEED = 1;
        SEED0 = SEED;
        if (!$value$plusargs("SESS_MAX=%d", SESS_MAX)) SESS_MAX = 6;
        if (!$value$plusargs("BEATS_MAX=%d", BEATS_MAX)) BEATS_MAX = 40;
        if (!$value$plusargs("GAP_MAX=%d", GAP_MAX)) GAP_MAX = 60;
        if (!$value$plusargs("VERBOSE=%d", VERBOSE)) VERBOSE = 0;
        if (!$value$plusargs("NOISE_PPM=%d", NOISE_PPM)) NOISE_PPM = 0;
        if (!$value$plusargs("BURST=%d", BURST)) BURST = 1;
        if (!$value$plusargs("OVF_PCT=%d", OVF_PCT)) OVF_PCT = 20;
        if (!$value$plusargs("HOTRESET_PCT=%d", HOTRESET_PCT)) HOTRESET_PCT = 10;
        if (!$value$plusargs("REVERSE=%d", REVERSE)) REVERSE = -1;
        if (!$value$plusargs("MASK_PCT=%d", MASK_PCT)) MASK_PCT = 20;
        if (!$value$plusargs("HOSTWR_PCT=%d", HOSTWR_PCT)) HOSTWR_PCT = 30;
        if (!$value$plusargs("LINKDROP_PCT=%d", LINKDROP_PCT)) LINKDROP_PCT = 10;
        if (!$value$plusargs("WL=%d", WL)) WL = 0;                    // Step 5 workloads
        if (!$value$plusargs("WL_PKTS=%d", WL_PKTS)) WL_PKTS = 200;
        if (!$value$plusargs("WL_KB=%d", WL_KB)) WL_KB = 1024;
        if (!$value$plusargs("WL_TENSORS=%d", WL_TENSORS)) WL_TENSORS = 2;
        if (!$value$plusargs("W0=%d", W0)) W0 = 1;
        if (!$value$plusargs("W1=%d", W1)) W1 = 1;
        wl_nic_t1 = 0;
        if (WL != 0) REVERSE = 0;
        // decorrelate neighbouring seeds ($random's first draws track the seed)
        SEED = SEED * 1103515245 + 12345;
        for (i = 0; i < 8; i = i + 1) t0 = $random(SEED);
        lane_rev = (REVERSE < 0) ? (urand(0, 3) == 0) : (REVERSE != 0);   // F6, fixed before training
        exp_wr[0] = 0; exp_wr[1] = 0; exp_rd[0] = 0; exp_rd[1] = 0;
        msix_cnt[0] = 0; msix_cnt[1] = 0; cur_sess[0] = -1; cur_sess[1] = -1;
        for (i = 1; i <= 16; i = i + 1) n_last_sz[i] = 0;
        for (i = 1; i <= 7; i = i + 1) w_used[i] = 0;
        $display("tb_bridge_rand: SEED=%0d SESS_MAX=%0d BEATS_MAX=%0d GAP_MAX=%0d NOISE_PPM=%0d BURST=%0d OVF_PCT=%0d HOTRESET_PCT=%0d LINKDROP_PCT=%0d lanes %s",
                 SEED0, SESS_MAX, BEATS_MAX, GAP_MAX, NOISE_PPM, BURST, OVF_PCT, HOTRESET_PCT, LINKDROP_PCT, lane_rev ? "REVERSED" : "straight");

        repeat (5) @(negedge clk); perst_n = 1;
        host_bringup;
        if (lane_rev && dut.u_lpb.lane_reversed_locked !== 1'b1)
            $display("[%0t] FAIL: lanes are reversed but lane_reversed_locked=%b", $time, dut.u_lpb.lane_reversed_locked);
        watch_on = 1;                        // F7b: from here every link drop must be explained
        if (WL != 0) begin run_workload; $finish; end

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

            // F5: sometimes set the limit below one channel's session size
            if (urand(0, 99) < OVF_PCT) begin
                limit_now = 64 * urand(1, (nb0 > nb1 ? nb0 : nb1)) - urand(0, 63);
                if (limit_now < 1) limit_now = 1;
            end else limit_now = 24'hFF_FFFF;
            limit_ch1 = limit_now;
            // Step 4 item 7: sometimes ch1 gets its own limit
            if (urand(0, 99) < OVF_PCT) begin limit_ch1 = 64 * urand(1, nb1) - urand(0, 63); if (limit_ch1 < 1) limit_ch1 = 1; n_lim1 = n_lim1 + 1; end
            // F9: sometimes one register goes through the link instead of the test port
            hw = (urand(0, 99) < HOSTWR_PCT && !abort_all) ? urand(1, 3) : 0;
            if (hw != 0) cv_hw[hw] = cv_hw[hw] + 1;
            if (hw == 1) host_mwr(16'h0008, {16'h0, w1[7:0], w0[7:0]});
            else         bar0_write(16'h0008, {16'h0, w1[7:0], w0[7:0]});
            if (hw == 2) host_mwr(16'h0018, {21'h0, urand(0,7) & 3'h7, 5'h0, urand(0,7) & 3'h7});
            bar0_write(16'h000C, limit_now);
            bar0_write(16'h0034, limit_ch1);
            if (arm[0]) begin
                base[0] = rand_base(0); cur_sess[0] = cur_sess[0] + 1;
                bar0_write(16'h0020, base[0][31:0]); bar0_write(16'h0024, base[0][63:32]);
            end
            if (arm[1]) begin
                base[1] = rand_base(1); cur_sess[1] = cur_sess[1] + 1;
                if (hw == 3) host_mwr(16'h0028, base[1][31:0]);   // F9: data only lands right if this worked
                else         bar0_write(16'h0028, base[1][31:0]);
                bar0_write(16'h002C, base[1][63:32]);
            end
            // ch0's base registers may have been used by host_mwr: (re)write them last
            if (arm[0]) begin bar0_write(16'h0020, base[0][31:0]); bar0_write(16'h0024, base[0][63:32]); end
            bar0_write(16'h000C, limit_now);
            bar0_write(16'h0034, limit_ch1);
            // F8: mask one channel's vector, or the whole function, for this session
            mk = (urand(0, 99) < MASK_PCT);
            if (mk) begin   // F8 needs a quiet start: every earlier MSI-X must have arrived
                t0 = 0; while (msix_seen < msix_expected && t0 < 400000) begin @(posedge clk); t0 = t0 + 1; end
            end
            mk_ch = arm[0] ? (arm[1] ? urand(0, 1) : 0) : 1;
            mk_fn = mk && (urand(0, 2) == 0);
            if (mk) begin
                if (mk_fn) begin cfg_write(12'h050, 32'hC000_0000); n_fmask = n_fmask + 1; end
                else begin
                    msix_entry_m(mk_ch == 0 ? 2 : 6, msix_addr_of(mk_ch), msix_data_of(mk_ch), 1'b1);
                    n_mask = n_mask + 1;
                end
            end
            msix_before = msix_seen;
            bar0_write(16'h0000, arm);
            msix_expected = msix_expected + arm[0] + arm[1];
            t0 = 0; while ((session_active & arm[1:0]) !== arm[1:0] && t0 < 2000) begin @(posedge aclk); t0 = t0 + 1; end
            if ((session_active & arm[1:0]) !== arm[1:0])
                $display("[%0t] FAIL: session %0d never became active (arm=%b active=%b)", $time, s, arm[1:0], session_active);
            hr = (urand(0, 99) < HOTRESET_PCT);
            hr_at = urand(20, 3000);
            // F7b: or a cable pull / glitch instead (never both in one session)
            ld      = !hr && (urand(0, 99) < LINKDROP_PCT);
            ld_at   = urand(20, 3000);
            ld_kind = urand(0, 1);                       // 0 cable pull, 1 glitch
            ld_len  = ld_kind == 0 ? urand(200, 5000) : urand(2, 40);
            fork
                if (arm[0]) run_channel(0, cur_sess[0], nb0, ld0, g0);
                if (arm[1]) run_channel(1, cur_sess[1], nb1, ld1, g1);
                if (hr) begin   // F7: hot reset at a random point of the session
                    repeat (hr_at) @(posedge clk);
                    rx_ignore = 1; abort_all = 1; n_hotreset = n_hotreset + 1;
                    if (VERBOSE) $display("[%0t] F7: hot reset in session %0d", $time, s);
                    @(negedge clk); hot_reset_request = 1; @(negedge clk); hot_reset_request = 0;
                end
                if (ld) begin   // F7b: link drop without hot reset
                    repeat (ld_at) @(posedge clk);
                    drop_ok = 1;
                    if (VERBOSE) $display("[%0t] F7b: %s of %0d clk in session %0d", $time,
                                          ld_kind == 0 ? "cable pull" : "glitch", ld_len, s);
                    @(negedge clk); cable_cut = 1;
                    repeat (ld_len) @(posedge clk);
                    if (ld_kind == 0) begin
                        n_cut = n_cut + 1;
                        if (!abort_all) begin
                            drop_err = drop_err + 1; rx_ignore = 1; abort_all = 1;
                            $display("[%0t] F7b FAIL: cable pulled for %0d clk but link_trained stayed %b", $time, ld_len, link_trained);
                        end
                        @(negedge clk); cable_cut = 0;
                    end else begin
                        n_glitch = n_glitch + 1;
                        @(negedge clk); cable_cut = 0;
                        // lost framing is detected after GAP_ERR_MAX unframed gap cycles
                        ld_t = 0; while (!abort_all && ld_t < 60000) begin @(posedge clk); ld_t = ld_t + 1; end
                        if (abort_all) n_glitch_drop = n_glitch_drop + 1;
                    end
                    drop_ok = 0;
                end
            join
            hr_hit = abort_all;              // F7/F7b happened in this session: F8 checks don't apply
            if (abort_all) begin
                // F7/F7b recovery: everything in flight is gone; the host re-trains and re-configures
                wait (link_trained === 1'b0);                     // link really went down
                host_bringup;
                exp_rd[0] = exp_wr[0]; exp_rd[1] = exp_wr[1];     // in-flight data was lost by design
                msix_cnt[0] = cur_sess[0] + 1; msix_cnt[1] = cur_sess[1] + 1;
                msix_expected = msix_seen;
                mir0_ok = 0; mir1_ok = 0;                         // registers were reset; re-learn
                repeat (200) @(posedge clk);
                got_n = 0;                  // drop a TLP cut in half by the reset (first beat seen, tail lost)
                rx_ignore = 0; abort_all = 0;
            end
            t0 = 0; while (session_active !== 2'b00 && t0 < 4000) begin @(posedge aclk); t0 = t0 + 1; end
            // host polls BAR0 STATUS until both channels report idle before re-arming
            // No short cap here: under heavy channel noise a session can take
            // milliseconds to drain (replays). The global watchdog catches a real hang.
            while (dut.u_lpb.u_pcie_phy_top.session_valid !== 2'b00) @(posedge clk);
            if (mk && !hr_hit) begin : f8
                integer want;
                // masked: the other channel's MSI-X (if armed and not masked) may come; ours must not
                want = mk_fn ? 0 : (arm[0] + arm[1] - 1);
                // wait for the unmasked channel's MSI-X (it trails its data by ~10k clk),
                // then a further window in which the masked one must stay silent
                t0 = 0; while (msix_seen - msix_before < want && t0 < 400000) begin @(posedge clk); t0 = t0 + 1; end
                repeat (20000) @(posedge clk);
                if (msix_seen - msix_before != want) begin
                    mask_err = mask_err + 1;
                    $display("[%0t] F8 FAIL: %0d MSI-X while %s masked (expected %0d)", $time,
                             msix_seen - msix_before, (mk_fn ? "function" : "vector"), want);
                end
                bar0_read(16'h1800, pba);
                if (!mk_fn && pba[mk_ch == 0 ? 2 : 6] !== 1'b1) begin
                    mask_err = mask_err + 1;
                    $display("[%0t] F8 FAIL: PBA=%h, vector %0d pending bit not set", $time, pba, mk_ch == 0 ? 2 : 6);
                end
                if (mk_fn) cfg_write(12'h050, 32'h8000_0000);
                else       msix_entry_m(mk_ch == 0 ? 2 : 6, msix_addr_of(mk_ch), msix_data_of(mk_ch), 1'b0);
                // released MSI-X may wait behind data and replays under noise: allow time
                t0 = 0; while (msix_seen < msix_expected && t0 < 400000) begin @(posedge clk); t0 = t0 + 1; end
                if (msix_seen != msix_expected) begin
                    mask_err = mask_err + 1;
                    $display("[%0t] F8 FAIL: after unmask %0d of %0d MSI-X", $time, msix_seen, msix_expected);
                end
                bar0_read(16'h1800, pba);
                if (pba[7:0] !== 8'h00) begin
                    mask_err = mask_err + 1;
                    $display("[%0t] F8 FAIL: PBA=%h after unmask (expected 0)", $time, pba);
                end
                if (VERBOSE) $display("[%0t] F8: %s mask on ch%0d checked", $time, mk_fn ? "function" : "vector", mk_ch);
            end
            repeat (urand(1, 200)) @(posedge clk);
        end
        cyc = 0;
        while ((exp_rd[0] < exp_wr[0] || exp_rd[1] < exp_wr[1] || msix_seen < msix_expected) && cyc < 5000000) begin
            @(posedge clk); cyc = cyc + 1;
        end
        repeat (2000) @(posedge clk);

        // Step 3 C: read every DMA register once; XLAT_MISS must be 0 (all
        // addresses are inside the windows), the bases must hold what was last
        // written, and a write to 0x1C clears the counter
        for (rg = 0; rg < 12; rg = rg + 1) begin
            bar0_read(rg * 4, rv);
            // In loopback every DMA write and MSI-X comes back inbound and misses
            // the RX translation window, so each must be counted (saturating at
            // 255). A looped-back BAR0 write is claimed by BAR0 and is not a
            // miss (S13).
            if (rg == 7) begin
                if (rv == ((xl_n > 255) ? 255 : xl_n)) ;
                else begin
                    if (rv == ((xl_n + xl_b0 > 255) ? 255 : xl_n + xl_b0)) n_s13 = n_s13 + 1;
                    reg_err = reg_err + 1;
                    $display("[%0t] FAIL: XLAT_MISS=%0d, expected %0d inbound misses (+%0d BAR0 writes)", $time, rv, xl_n, xl_b0);
                end
            end
            if (rg == 8 && mir0_ok && rv !== mir0) begin
                reg_err = reg_err + 1; $display("[%0t] FAIL: BASE_CH0_LO=%h, last written %h", $time, rv, mir0);
            end
            if (rg == 10 && mir1_ok && rv !== mir1) begin
                reg_err = reg_err + 1; $display("[%0t] FAIL: BASE_CH1_LO=%h, last written %h", $time, rv, mir1);
            end
        end
        bar0_write(16'h001C, 32'h0);
        bar0_read(16'h1800, rv); cv_pba_rd = cv_pba_rd + 1;
        // Step 4 item 6: the sticky error counters must match what the bench saw
        exp_cnt[0] = n_crc; exp_cnt[1] = n_fec_fix; exp_cnt[2] = n_fec_bad; exp_cnt[3] = n_replay;
        exp_cnt[4] = n_nak; exp_cnt[5] = 0; exp_cnt[6] = n_exit[1] + n_exit[2] + n_exit[3];
        exp_cnt[7] = n_hotreset; exp_cnt[8] = n_ovf_ch[0]; exp_cnt[9] = n_ovf_ch[1];
        for (rg = 0; rg < 10; rg = rg + 1) begin
            bar0_read(16'h0040 + rg * 4, rv);
            if (rv !== (exp_cnt[rg] > 65535 ? 65535 : exp_cnt[rg])) begin
                reg_err = reg_err + 1;
                $display("[%0t] FAIL: error counter at BAR0 0x%h = %0d, bench counted %0d", $time, 16'h0040 + rg * 4, rv, exp_cnt[rg]);
            end
        end
        bar0_write(16'h0040, 32'h0);
        bar0_read(16'h0040, rv);
        if (rv !== 32'h0) begin reg_err = reg_err + 1; $display("[%0t] FAIL: CRC counter not cleared by a write", $time); end
        if (rv[7:0] !== 8'h00) begin reg_err = reg_err + 1; $display("[%0t] FAIL: PBA=%h at end", $time, rv); end

        $display("------------------------------------------------------------");
        $display(" seed %0d: %0d sessions (%0d single-channel), %0d beats, %0d beats split at 4 KB",
                 SEED0, n_sessions, n_single_ch, n_beats, n_split);
        $display(" data DWs expected      : ch0 %0d  ch1 %0d   (largest TLP received: %0d DW)", exp_wr[0], exp_wr[1], max_tlp_dw);
        $display(" data TLPs received ok  : %0d   mismatched: %0d   unexpected: %0d   still missing DWs: %0d",
                 ok, bad, extra, (exp_wr[0]-exp_rd[0]) + (exp_wr[1]-exp_rd[1]));
        $display(" MSI-X received         : %0d of %0d   bad data: %0d   arrived before its data: %0d",
                 msix_seen, msix_expected, msix_bad, msix_early);
        $display(" channel: %0d symbol errors injected -> FEC corrected %0d, uncorrectable %0d, CRC fails %0d, replays %0d",
                 n_noise, n_fec_fix, n_fec_bad, n_crc, n_replay);
        $display(" F5 overflow aborts: %0d (beats accepted past the limit: %0d)   F6 lanes %s   F7 hot resets: %0d (%0d TLPs dropped in reset windows)",
                 n_ovf, leaked, lane_rev ? "reversed" : "straight", n_hotreset, n_ignored);
        $display(" F8 masked sessions: %0d vector + %0d function (errors %0d)   F9 inbound BAR0 writes: %0d (%0d TLPs, errors %0d)",
                 n_mask, n_fmask, mask_err, n_hostwr, n_bar0_tlp, hostwr_err);
        $display(" F7b link drops: %0d cable pulls, %0d glitches (%0d dropped the link); L0 exits: loss %0d framing %0d TS1 %0d; unexplained drops %0d",
                 n_cut, n_glitch, n_glitch_drop, n_exit[1], n_exit[2], n_exit[3], drop_err);
        $display(" errors now: framing=%b lane=%b overflow=%b   register check errors: %0d", phy_framing_err, phy_lane_err, session_overflow, reg_err);
        if (n_s13) $display(" S13 REGRESSION: XLAT_MISS also counted %0d inbound BAR0 write(s) as translation misses", xl_b0);
        // Step 3 C: one machine-readable line, merged over seeds by ci/coverage_report.py
        $write("COV: seed=%0d noise=%0d", SEED0, NOISE_PPM > 0);
        for (rg = 0; rg < 12; rg = rg + 1) begin rb8 = rg * 4; $write(" wr_%h=%0d rd_%h=%0d", rb8, cv_wr[rg], rb8, cv_rd[rg]); end
        $write(" rd_1800=%0d split=%0d nosplit=%0d single_ch=%0d", cv_pba_rd, n_split, n_beats - n_split, n_single_ch);
        for (rg = 1; rg <= 16; rg = rg + 1) $write(" last_%0d=%0d", rg, n_last_sz[rg]);
        for (rg = 1; rg <= 7; rg = rg + 1) $write(" arb0_w%0d=%0d arb1_w%0d=%0d", rg, cv_win0[rg], rg, cv_win1[rg]);
        $write(" credit_stall_clk=%0d credit_stall_ep=%0d fec_fix=%0d fec_bad=%0d crc=%0d replay=%0d framing=%0d lane_err=%0d xlat_err=%0d xlat_cnt=%0d",
               cv_credit_stall, cv_stall_ep, n_fec_fix, n_fec_bad, n_crc, n_replay, cv_framing, cv_lane, cv_xlat, xl_n);
        $write(" ovf=%0d mask_vec=%0d mask_fn=%0d hotreset=%0d cable=%0d glitch=%0d glitch_drop=%0d exit_loss=%0d exit_framing=%0d exit_ts1=%0d",
               n_ovf, n_mask, n_fmask, n_hotreset, n_cut, n_glitch, n_glitch_drop, n_exit[1], n_exit[2], n_exit[3]);
        $write(" lanes_rev=%0d lanes_straight=%0d hw_weights=%0d hw_tc=%0d hw_base1=%0d msix=%0d\n",
               lane_rev, !lane_rev, cv_hw[1], cv_hw[2], cv_hw[3], msix_seen);
        if (bad == 0 && extra == 0 && exp_rd[0] == exp_wr[0] && exp_rd[1] == exp_wr[1]
            && msix_seen == msix_expected && msix_bad == 0 && msix_early == 0 && leaked == 0
            && mask_err == 0 && hostwr_err == 0 && n_bar0_tlp == n_hostwr && drop_err == 0 && reg_err == 0)
            $display(" RESULT: PASS (seed %0d)", SEED0);
        else
            $display(" RESULT: FAIL (seed %0d -- replay with +SEED=%0d)", SEED0, SEED0);
        $finish;
    end

    // watchdog: 50 ms of simulated time (clean runs need < 3 ms; heavy noise up to ~20 ms)
    reg [63:0] WL_WATCHDOG_NS;
    initial begin WL_WATCHDOG_NS = 64'd50_000_000; if ($test$plusargs("WL=")) WL_WATCHDOG_NS = 64'd2_000_000_000; end
    initial begin #1; #(WL_WATCHDOG_NS); $display("GLOBAL TIMEOUT (seed %0d): exp_rd %0d/%0d of %0d/%0d, session_valid %b",
                 SEED0, exp_rd[0], exp_rd[1], exp_wr[0], exp_wr[1], dut.u_lpb.u_pcie_phy_top.session_valid);
                 $display(" RESULT: FAIL"); $finish; end
endmodule
