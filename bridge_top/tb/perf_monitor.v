`timescale 1ns/1ps
// ============================================================
// perf_monitor -- Phase 2 Step 2 baseline counters (simulation only)
//
// Instantiated inside tb_bridge_top under `ifdef PERF; reads the DUT through
// absolute hierarchical references.
//   iverilog -g2012 -DSIMULATION -DPERF -s tb_bridge_top -o p.vvp -c bridge_top/bridge_top.f \
//            -c bridge_top/bridge_top_tb.f bridge_top/tb/perf_monitor.v && vvp p.vvp
//   Vivado: bridge_top/vivado_baseline.tcl does this in XSim.
// Override the bench name with +define+PERF_TB=tb_bridge_ovf.
//
// Reports (in a final block, after the bench's RESULT line):
//   * bytes per clock at each stage, over that stage's active window
//     (first to last transfer): fabric in, interconnect out, TLP packetizer
//     out (32-bit, headers included), Flits to PHY, PHY lane symbols, RX TLPs
//   * latency per data beat: fabric handshake -> its TLP's first beat on
//     the receive side (loopback), min / median / p99 / max
//   * peak fill: CDC FIFO, write buffer, replay buffer (un-ACKed Flits), DLL RX FIFO
//   * replay events
// Matching assumes 1 fabric data beat -> 1 data TLP, in order (true for
// tb_bridge_top: 4 KB-aligned bases, 64 B beats). RX TLPs of <= 20 bytes
// (header + 1 DW) are MSI-X and are not matched.
// ============================================================
`ifndef PERF_TB
`define PERF_TB tb_bridge_top
`endif

module perf_monitor;
    localparam real CLK_NS = 4.0;     // pcie_clk period used by the bench
    localparam QD = 8192;

    function integer popc64; input [63:0] k; integer i; begin
        popc64 = 0; for (i = 0; i < 64; i = i + 1) popc64 = popc64 + k[i]; end endfunction
    function integer popc4; input [3:0] k; begin popc4 = k[0] + k[1] + k[2] + k[3]; end endfunction

    // ---------------- stage byte counters ----------------
    // index: 0 fabric, 1 ic out, 2 packetizer, 3 flits, 4 lanes, 5 rx tlps
    real    t_first [0:5];
    real    t_last  [0:5];
    real    bytes   [0:5];
    reg     seen    [0:5];
    integer s;
    initial for (s = 0; s < 6; s = s + 1) begin t_first[s] = 0; t_last[s] = 0; bytes[s] = 0; seen[s] = 0; end

    task automatic hit(input integer st, input integer nb);
        begin
            if (!seen[st]) begin seen[st] = 1; t_first[st] = $realtime; end
            t_last[st] = $realtime;
            bytes[st]  = bytes[st] + nb;
        end
    endtask

    // ---------------- latency matching ----------------
    real    q0 [0:QD-1];  integer q0w = 0, q0r = 0;     // fabric ch0 timestamps
    real    q1 [0:QD-1];  integer q1w = 0, q1r = 0;     // fabric ch1
    real    qi [0:QD-1];  integer qiw = 0, qir = 0;     // in interconnect-out order
    real    lat [0:QD-1]; integer nlat = 0;
    integer unmatched = 0;

    // fabric side (aclk)
    always @(posedge `PERF_TB.dut.aclk) begin
        if (`PERF_TB.dut.s0_tvalid && `PERF_TB.dut.s0_tready && |`PERF_TB.dut.s0_tkeep) begin
            hit(0, popc64(`PERF_TB.dut.s0_tkeep)); q0[q0w % QD] = $realtime; q0w = q0w + 1;
        end
        if (`PERF_TB.dut.s1_tvalid && `PERF_TB.dut.s1_tready && |`PERF_TB.dut.s1_tkeep) begin
            hit(0, popc64(`PERF_TB.dut.s1_tkeep)); q1[q1w % QD] = $realtime; q1w = q1w + 1;
        end
    end

    // pcie_clk side
    integer rx_tlp_bytes = 0;
    real    rx_tlp_t0 = 0;
    reg     rx_in_tlp = 0;
    integer replays = 0;
    integer pk_cdc = 0, pk_wbuf = 0, pk_replay = 0, pk_rxf = 0;
    integer v;

    always @(posedge `PERF_TB.dut.clk) begin
        // interconnect out (data beats only: an abort marker has tkeep 0)
        if (`PERF_TB.dut.ic_tvalid && `PERF_TB.dut.ic_tready && |`PERF_TB.dut.ic_tkeep) begin
            hit(1, popc64(`PERF_TB.dut.ic_tkeep));
            if (`PERF_TB.dut.ic_tuser[0] == 1'b0) begin
                if (q0r < q0w) begin qi[qiw % QD] = q0[q0r % QD]; q0r = q0r + 1; qiw = qiw + 1; end
            end else begin
                if (q1r < q1w) begin qi[qiw % QD] = q1[q1r % QD]; q1r = q1r + 1; qiw = qiw + 1; end
            end
        end
        // packetizer output, 32-bit DW stream incl. headers
        if (`PERF_TB.dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.tlp_out_valid &&
            `PERF_TB.dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.tlp_out_ready)
            hit(2, 4);
        // Flits to the PHY (256-bit words)
        if (`PERF_TB.dut.u_lpb.phy_tx_valid && `PERF_TB.dut.u_lpb.phy_tx_ready)
            hit(3, 32);
        // PAM4 lane symbols: 2 bits per symbol per lane
        if (|`PERF_TB.dut.u_lpb.tx_level_valid)
            hit(4, 0);
        if (|`PERF_TB.dut.u_lpb.tx_level_valid)
            bytes[4] = bytes[4] + popc4(`PERF_TB.dut.u_lpb.tx_level_valid) * 0.25;
        // receive side TLPs (loopback)
        if (`PERF_TB.dut.m_rx_axis_tvalid) begin
            hit(5, popc64(`PERF_TB.dut.m_rx_axis_tkeep));
            if (!rx_in_tlp) begin rx_in_tlp = 1; rx_tlp_t0 = $realtime; rx_tlp_bytes = 0; end
            rx_tlp_bytes = rx_tlp_bytes + popc64(`PERF_TB.dut.m_rx_axis_tkeep);
            if (`PERF_TB.dut.m_rx_axis_tlast) begin
                rx_in_tlp = 0;
                if (rx_tlp_bytes > 20) begin
                    if (qir < qiw) begin lat[nlat % QD] = rx_tlp_t0 - qi[qir % QD]; nlat = nlat + 1; qir = qir + 1; end
                    else unmatched = unmatched + 1;
                end
            end
        end
        if (`PERF_TB.dut.u_lpb.replay_event) replays = replays + 1;
        // peak fills
        v = `PERF_TB.dut.u_ic.u_fifo.occupancy;                                                    if (v > pk_cdc)    pk_cdc = v;
        v = `PERF_TB.dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.u_write_data_buffer.occupancy_r; if (v > pk_wbuf)   pk_wbuf = v;
        v = (`PERF_TB.dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.u_data_link_layer_top.u_replay_buffer.head_seq -
             `PERF_TB.dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.u_data_link_layer_top.u_replay_buffer.tail_seq) & 12'hFFF;
                                                                                                   if (v > pk_replay) pk_replay = v;
        v = `PERF_TB.dut.u_lpb.u_link_layer_top.u_stream_tx_path_top.u_data_link_layer_top.rxf_cnt; if (v > pk_rxf)    pk_rxf = v;
    end

    // ---------------- report ----------------
    function [8*28-1:0] stage_name; input integer st; begin
        case (st)
            0: stage_name = "fabric in (both ch)";
            1: stage_name = "interconnect out";
            2: stage_name = "packetizer out (incl hdr)";
            3: stage_name = "Flits to PHY";
            4: stage_name = "PHY lanes (PAM4 symbols)";
            default: stage_name = "RX TLPs (loopback)";
        endcase end endfunction

    integer i, j, n;
    real    tmp, win, bpc;
    final begin
        $display("PERF ================= baseline counters (perf_monitor) =================");
        for (s = 0; s < 6; s = s + 1) begin
            win = (t_last[s] - t_first[s]) / CLK_NS + 1.0;
            bpc = seen[s] ? bytes[s] / win : 0.0;
            $display("PERF  %-28s %10.0f B  over %8.0f clk  = %6.2f B/clk  (%6.2f GB/s at 250 MHz)",
                     stage_name(s), bytes[s], seen[s] ? win : 0.0, bpc, bpc * 0.25);
        end
        n = (nlat < QD) ? nlat : QD;
        for (i = 1; i < n; i = i + 1) begin           // insertion sort
            tmp = lat[i]; j = i - 1;
            while (j >= 0 && lat[j] > tmp) begin lat[j + 1] = lat[j]; j = j - 1; end
            lat[j + 1] = tmp;
        end
        if (n > 0)
            $display("PERF  latency fabric beat -> RX TLP (%0d beats): min %0.0f  median %0.0f  p99 %0.0f  max %0.0f clk",
                     n, lat[0] / CLK_NS, lat[n / 2] / CLK_NS, lat[(n * 99) / 100] / CLK_NS, lat[n - 1] / CLK_NS);
        $display("PERF  unmatched data TLPs: %0d   fabric beats not yet seen on RX: %0d", unmatched, qiw - qir);
        $display("PERF  peak fill: CDC FIFO %0d   write buffer %0d   replay (un-ACKed Flits) %0d   DLL RX FIFO %0d",
                 pk_cdc, pk_wbuf, pk_replay, pk_rxf);
        $display("PERF  replay events: %0d", replays);
        $display("PERF ======================================================================");
    end
endmodule
