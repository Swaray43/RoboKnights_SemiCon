`timescale 1ns/1ps
// ============================================================
// tb_credit_gate_proof -- proves credit_manager's Posted pool,
// now wired into stream_tx_path_top (S6), actually gates TLP
// issuance and isn't just inert plumbing.
//
// Sequence:
//   0. G2: credits come from FC DLLPs over the self-loop. Wait for
//      FC_INIT1/2 to finish and check the Posted pool came up
//      infinite from the looped-back InitFC-P (HdrFC = DataFC = 0).
//   1. Normal operation: send a beat, confirm it completes.
//   2. Force the Posted header field finite with ph_limit ==
//      ph_consumed (0 available) -- a real finite advertisement.
//      Drive another beat and confirm the TX path genuinely stalls:
//      split_valid stays high, credit_grant stays low, no new TLP,
//      for many cycles.
//   3. Return one header credit (ph_limit = ph_consumed + 1, still
//      finite) and confirm the stalled beat proceeds and completes.
// ============================================================
module tb_credit_gate_proof;

    localparam DATA_W = 512, KEEP_W = 64, TUSER_W = 26, SYS = 256;

    reg clk = 0, rst_n = 0;
    always #2 clk = ~clk;

    reg  [DATA_W-1:0]  s_tdata;
    reg  [KEEP_W-1:0]  s_tkeep;
    reg  [TUSER_W-1:0] s_tuser;
    reg                s_tlast;
    reg                s_tvalid;
    wire               s_tready;

    reg                cfg_wr = 0;
    reg                cfg_id = 0;
    reg  [63:0]        cfg_data = 0;

    wire               ptx_valid, ptx_sof, ptx_eof;
    wire [SYS-1:0]     ptx_data;
    reg                ptx_ready = 1'b1;

    reg                prx_valid, prx_sof, prx_eof;
    reg  [SYS-1:0]     prx_data;

    wire               rx_tvalid, rx_tlast;
    wire [511:0]       rx_tdata;
    wire [63:0]        rx_tkeep;
    reg                rx_tready = 1'b1;

    wire err_xlat_miss, err_addr_ovf, stat_split, err_tkeep_gap;
    wire crc_err, fec_err, fec_corr, replay_ev;
    wire [1:0] session_done;

    stream_tx_path_top #(.REPLAY_TIMEOUT(4096)) dut (
        .clk(clk), .rst_n(rst_n),
        .s_axis_tdata(s_tdata), .s_axis_tkeep(s_tkeep), .s_axis_tuser(s_tuser),
        .s_axis_tlast(s_tlast), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
        .cfg_base_addr_wr(cfg_wr), .cfg_base_addr_id(cfg_id),
        .cfg_base_addr_data(cfg_data), .cfg_base_addr_clr(1'b0),
        .msix_req_valid(1'b0), .msix_req_addr(64'd0), .msix_req_data(32'd0),
        .msix_req_ready(), .session_done(session_done),
        .phy_tx_valid(ptx_valid), .phy_tx_data(ptx_data), .phy_tx_sof(ptx_sof),
        .phy_tx_eof(ptx_eof), .phy_tx_ready(ptx_ready),
        .phy_rx_valid(prx_valid), .phy_rx_data(prx_data), .phy_rx_sof(prx_sof), .phy_rx_eof(prx_eof),
        .m_rx_axis_tvalid(rx_tvalid), .m_rx_axis_tdata(rx_tdata), .m_rx_axis_tkeep(rx_tkeep),
        .m_rx_axis_tlast(rx_tlast), .m_rx_axis_tready(rx_tready),
        .err_xlat_miss(err_xlat_miss), .err_addr_ovf(err_addr_ovf),
        .stat_split_occurred(stat_split), .err_tkeep_gap(err_tkeep_gap),
        .crc_err(crc_err), .fec_err(fec_err), .fec_corrected(fec_corr), .replay_event(replay_ev)
    );

    // self-loop PHY (no error injection -- not what this test is about)
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            prx_valid <= 0; prx_sof <= 0; prx_eof <= 0; prx_data <= 0;
        end else begin
            prx_valid <= ptx_valid && ptx_ready;
            prx_sof   <= ptx_sof;
            prx_eof   <= ptx_eof;
            prx_data  <= ptx_data;
        end
    end

    integer n_tlp_out = 0;
    always @(posedge clk) if (rst_n && rx_tvalid && rx_tready && rx_tlast) n_tlp_out = n_tlp_out + 1;

    task send_beat(input [63:0] pattern);
        integer w;
        begin
            @(negedge clk);
            for (w = 0; w < 16; w = w + 1) s_tdata[w*32 +: 32] = pattern[31:0] + w;
            s_tkeep  = {64{1'b1}};
            s_tuser  = {1'b1, 24'd0, 1'b0};   // tlast_flag=1, stream 0
            s_tlast  = 1'b1;
            s_tvalid = 1'b1;
            @(posedge clk);
            while (!s_tready) @(posedge clk);
            @(negedge clk);
            s_tvalid = 0;
            s_tlast  = 0;
        end
    endtask

    integer pass = 1;
    integer stall_cycles;
    reg [7:0] lim_one;

    initial begin
        s_tvalid = 0; s_tdata = 0; s_tkeep = 0; s_tuser = 0; s_tlast = 0;

        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);

        @(negedge clk); cfg_wr = 1; cfg_id = 0; cfg_data = 64'h0000_0001_0000_0000;
        @(negedge clk); cfg_wr = 0;

        // ---- Phase 0: FC init over the self-loop (G2) ----
        begin : fcwait
            integer t;
            for (t = 0; t < 5000 && !dut.fc_init_done; t = t + 1) @(posedge clk);
        end
        if (!dut.fc_init_done || !dut.fc_credits_valid) begin
            $display("FAIL Phase0: FC init did not complete over the loopback");
            pass = 0;
        end else if (!dut.u_credit_manager.p_init ||
                     !dut.u_credit_manager.is_ph_inf || !dut.u_credit_manager.is_pd_inf) begin
            $display("FAIL Phase0: Posted pool not initialised infinite from InitFC-P");
            pass = 0;
        end else
            $display("Phase0 OK: FC_INIT1/2 done, Posted pool infinite from InitFC-P (HdrFC=DataFC=0)");

        // ---- Phase 1: normal operation. ----
        send_beat(64'hAAAA_0000);
        repeat (300) @(posedge clk);
        if (n_tlp_out != 1) begin
            $display("FAIL Phase1: expected 1 TLP out under infinite credit, got %0d", n_tlp_out);
            pass = 0;
        end else
            $display("Phase1 OK: beat completed normally (%0d TLP out), ph_consumed=%0d",
                      n_tlp_out, dut.u_credit_manager.ph_consumed);

        // ---- Phase 2: force a finite, exhausted Posted-header limit
        // (as if a real UpdateFC DLLP had advertised ph_limit ==
        // ph_consumed, i.e. zero credits available) and confirm the
        // TX path for a fresh beat genuinely stalls -- no TLP egress,
        // for a sustained window, with split_valid asserted and
        // credit_grant held low the whole time. ----
        @(negedge clk);
        force dut.u_credit_manager.ph_inf_r = 1'b0;
        force dut.u_credit_manager.ph_limit = dut.u_credit_manager.ph_consumed;
        @(posedge clk);

        send_beat(64'hBBBB_0000);

        stall_cycles = 0;
        repeat (200) begin
            @(posedge clk);
            if (dut.credit_alloc_valid) begin
                stall_cycles = stall_cycles + 1;
                if (dut.credit_grant) begin
                    $display("FAIL Phase2: credit_grant asserted with ph credits exhausted (cyc %0d)", stall_cycles);
                    pass = 0;
                end
            end
        end

        if (n_tlp_out != 1) begin
            $display("FAIL Phase2: a TLP escaped while Posted header credit was exhausted (n_tlp_out=%0d)", n_tlp_out);
            pass = 0;
        end else if (stall_cycles < 10) begin
            $display("FAIL Phase2: split_valid was not held pending long enough to prove a real stall (%0d cycles)", stall_cycles);
            pass = 0;
        end else
            $display("Phase2 OK: TX path genuinely stalled for %0d cycles with Posted credit exhausted, no TLP escaped", stall_cycles);

        // ---- Phase 3: return exactly one header credit (still a
        // finite pool) and confirm the stalled beat proceeds. ----
        @(negedge clk);
        release dut.u_credit_manager.ph_limit;
        lim_one = dut.u_credit_manager.ph_consumed + 8'd1;
        force dut.u_credit_manager.ph_limit = lim_one;

        repeat (300) @(posedge clk);
        if (dut.u_credit_manager.ph_consumed != lim_one) begin
            $display("FAIL Phase3: expected exactly one header credit used (consumed=%0d limit=%0d)",
                      dut.u_credit_manager.ph_consumed, lim_one);
            pass = 0;
        end
        release dut.u_credit_manager.ph_limit;
        release dut.u_credit_manager.ph_inf_r;

        if (n_tlp_out != 2) begin
            $display("FAIL Phase3: stalled beat did not complete after credit was restored (n_tlp_out=%0d, expected 2)", n_tlp_out);
            pass = 0;
        end else
            $display("Phase3 OK: stalled beat completed once credit was restored (n_tlp_out=%0d)", n_tlp_out);

        repeat (20) @(posedge clk);

        $display("------------------------------------------------------------");
        if (pass) $display("RESULT: PASS");
        else      $display("RESULT: FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end

    initial begin
        #500000;
        $display("FATAL: simulation timeout");
        $display("RESULT: FAIL");
        $finish;
    end

endmodule