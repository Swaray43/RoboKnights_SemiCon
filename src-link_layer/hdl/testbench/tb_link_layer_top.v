`timescale 1ns/1ps
// ============================================================
// tb_link_layer_top -- smoke test for the new link_layer_top module
//
//   Drives a single-session MWr write in on s_axis (same TX framing
//   convention as tb_stream_path.v), loops the PHY TX Flit back into
//   PHY RX (self-ACKing loopback, same trick tb_stream_path.v uses),
//   and checks that the inbound decode chain this module adds
//   (unpacker -> tlp_header_parser -> pcie_rx_engine) produces the
//   expected AXI BAR0 write: m_axi_awvalid with the translated
//   address, derived from the same PCIe address the TX side used.
// ============================================================
module tb_link_layer_top;
 
    localparam DATA_W = 512, KEEP_W = 64, TUSER_W = 26, SYS = 256;
    localparam AXI_ADDR_W = 32;
 
    reg clk = 0, rst_n = 0;
    always #2 clk = ~clk;
 
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
 
    wire               ptx_valid, ptx_sof, ptx_eof;
    wire [SYS-1:0]     ptx_data;
    reg                ptx_ready;
 
    reg                prx_valid, prx_sof, prx_eof;
    reg  [SYS-1:0]     prx_data;
 
    wire                   m_axi_awvalid;
    wire [AXI_ADDR_W-1:0]  m_axi_awaddr;
    wire                   m_axi_awerr;
 
    wire crc_err, fec_err, fec_corr, replay_ev;
    wire malformed;
 
    // PCIe address of our write: base + 0 offset.
    localparam [63:0] PCIE_BASE  = 64'h0000_0001_0000_0000;
    // RX-decode window: upper 32 bits fixed == PCIE_BASE's upper 32
    // bits, lower 32 bits free as in-window offset.
    localparam [63:0] WIN_BASE   = PCIE_BASE & 64'hFFFF_FFFF_0000_0000;
    localparam [63:0] WIN_MASK   = 64'hFFFF_FFFF_0000_0000;
    localparam [31:0] AXI_BASE   = 32'h2000_0000;
    localparam [31:0] EXP_AWADDR = AXI_BASE; // offset 0 into the window
 
    link_layer_top #(
        .REPLAY_TIMEOUT(4096)
    ) dut (
        .clk(clk), .rst_n(rst_n),
 
        .s_axis_tdata(s_tdata), .s_axis_tkeep(s_tkeep), .s_axis_tuser(s_tuser),
        .s_axis_tlast(s_tlast), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
 
        .cfg_base_addr_wr(cfg_wr), .cfg_base_addr_id(cfg_id),
        .cfg_base_addr_data(cfg_data), .cfg_base_addr_clr(1'b0),
 
        .cfg_window_base(WIN_BASE), .cfg_window_mask(WIN_MASK), .cfg_axi_base(AXI_BASE),
        .cfg_excl_en(1'b0), .cfg_excl_base(64'h0), .cfg_excl_mask(64'h0),
 
        .msix_req_valid(1'b0), .msix_req_addr(64'd0), .msix_req_data(32'd0),
        .msix_req_ready(), .session_done(),
 
        .phy_tx_valid(ptx_valid), .phy_tx_data(ptx_data), .phy_tx_sof(ptx_sof),
        .phy_tx_eof(ptx_eof), .phy_tx_ready(ptx_ready),
        .phy_rx_valid(prx_valid), .phy_rx_data(prx_data), .phy_rx_sof(prx_sof), .phy_rx_eof(prx_eof),
 
        .m_axi_awvalid(m_axi_awvalid), .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awerr(m_axi_awerr), .m_axi_awready(1'b1),
 
        .cpl_tvalid(), .cpl_tdata(), .cpl_tlast(), .cpl_tkeep(), .cpl_tuser(),
        .cpl_tready(1'b1),
 
        .err_xlat_miss(), .err_addr_ovf(), .stat_split_occurred(), .err_tkeep_gap(),
        .crc_err(crc_err), .fec_err(fec_err), .fec_corrected(fec_corr), .replay_event(replay_ev),
        .malformed_tlp_err(malformed)
    );
 
    // ---------------- PHY self-loopback (no error injection) ----------------
    always @(posedge clk) ptx_ready <= 1'b1;
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
 
    // ---------------- result tracking ----------------
    integer n_aw = 0;
    reg [31:0] seen_awaddr;
    reg        seen_awerr;
    always @(posedge clk) if (rst_n && m_axi_awvalid) begin
        n_aw = n_aw + 1;
        seen_awaddr = m_axi_awaddr;
        seen_awerr  = m_axi_awerr;
    end
 
    integer n_crc = 0, n_fec = 0, n_malformed = 0;
    always @(posedge clk) if (rst_n) begin
        if (crc_err) n_crc = n_crc + 1;
        if (fec_err) n_fec = n_fec + 1;
        if (malformed) n_malformed = n_malformed + 1;
    end
 
    integer cyc = 0, last_progress;
    always @(posedge clk) cyc <= cyc + 1;
 
    initial begin
        s_tvalid = 0; s_tdata = 0; s_tkeep = 0; s_tuser = 0; s_tlast = 0;
 
        repeat (5) @(posedge clk);
        rst_n = 1;
        repeat (3) @(posedge clk);
 
        // Program the TX-side session base address (stream 0) and
        // send one short MWr beat: 4 DW (16 B) at offset 0, tlast=1.
        @(negedge clk); cfg_wr = 1; cfg_id = 0; cfg_data = PCIE_BASE;
        @(negedge clk); cfg_wr = 0;
 
        @(negedge clk);
        s_tdata  = {DATA_W{1'b0}};
        s_tdata[31:0] = 32'hA5A5_1234;   // DW0 of the write payload
        s_tkeep  = {60'd0, 4'hF};     // 16 bytes = 4 DW valid
        s_tuser  = {1'b1, 24'd0, 1'b0}; // {tlast_flag, session_offset=0, stream_id=0}
        s_tlast  = 1;
        s_tvalid = 1;
        @(negedge clk);
        while (!s_tready) @(negedge clk);
        s_tvalid = 0; s_tlast = 0;
 
        last_progress = cyc;
        while (n_aw == 0) begin
            @(posedge clk);
            if (cyc - last_progress > 20000) begin
                $display("FAIL: timed out waiting for m_axi_awvalid");
                $display("RESULT: FAIL");
                $finish;
            end
        end
 
        repeat (50) @(posedge clk);
 
        $display("------------------------------------------------------------");
        $display(" AXI BAR0 writes seen : %0d", n_aw);
        $display(" awaddr (last)        : %h  (expected %h)", seen_awaddr, EXP_AWADDR);
        $display(" awerr (last)         : %0d", seen_awerr);
        $display(" crc_err: %0d  fec_err: %0d  malformed_tlp_err: %0d", n_crc, n_fec, n_malformed);
        if (n_aw >= 1 && seen_awaddr == EXP_AWADDR && !seen_awerr &&
            n_crc == 0 && n_fec == 0 && n_malformed == 0)
            $display(" RESULT: PASS");
        else
            $display(" RESULT: FAIL");
        $display("------------------------------------------------------------");
        $finish;
    end
 
    initial begin
        #2000000;
        $display("FAIL: global watchdog timeout");
        $display("RESULT: FAIL");
        $finish;
    end
 
endmodule
 