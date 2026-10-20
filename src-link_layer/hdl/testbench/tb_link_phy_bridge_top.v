`timescale 1ns/1ps
// ============================================================
// tb_link_phy_bridge_top -- integration smoke test for the new
// link_phy_bridge_top glue module (bridge_top.v's "u_lpb" instance).
//
// Since interconnect_top.v does not exist anywhere in the project,
// this bench plays its role directly: it drives link_phy_bridge_top's
// top-level s_axis_* inputs (TUSER in link-native order) exactly as bridge_top.v would
// feed them from interconnect_top, and the bar0_*/session_* ports
// exactly as pcie_config_space/host software would via bridge_top.v's
// own pass-through ports.
//
// Path under test (everything downstream of interconnect_top):
//   s_axis (one MWr beat) -> link_layer_top TX chain -> Flit DLL
//     -> pcie_phy_top TX datapath -> tx_level/tx_level_valid
//     -> [self-loopback, registered 1 cycle]
//     -> rx_level/rx_level_valid -> pcie_phy_top RX datapath
//     -> link_layer_top RX decode (unpacker -> tlp_header_parser
//        -> pcie_rx_engine) -> m_axi_awvalid/awaddr
//
// Session base address is armed the same way host software would:
// bar0_dma_regs regs (base_addr_ch0 lo/hi, then arm_pulse[0]) ->
// session_arm_ctrl -> this module's cfg_base_addr arbiter ->
// link_layer_top's cfg_base_addr_wr/id/data port.
//
// Link training is intentionally NOT exercised here: datapath_rst_n
// (pcie_reset_controller) releases on perst_n alone, independent of
// link_trained, and lane_reversed_locked defaults to 0 (identity,
// no swap) until training completes -- which is exactly the mode
// this loopback needs. Training itself is already covered by
// pcie_phy_top's own lane-reversal-focused testbenches; re-deriving
// a full TS1/TS2 sequence here would test infrastructure already
// proven elsewhere, not this module's new integration logic.
// ============================================================
module tb_link_phy_bridge_top;
 
    localparam DATA_W     = 512;
    localparam KEEP_W     = 64;
    localparam SOFF_W     = 24;
    localparam TUSER_W    = SOFF_W + 2; // 26
    localparam NUM_STREAMS = 2;
    localparam SYS_BUS_W  = 256;
    localparam AXI_ADDR_W = 32;
 
    reg clk = 0;
    always #2 clk = ~clk;
    wire aclk = clk;
 
    reg perst_n = 0;
    wire aclk_rst_n = perst_n;
 
    // ---------------- DUT I/O ----------------
    reg  [DATA_W-1:0] s_tdata;
    reg  [KEEP_W-1:0] s_tkeep;
    reg  [SOFF_W+1:0] s_tuser;       // {tlast_flag, session_offset[23:0], stream_id} -- link-native, as interconnect_top emits it
    reg               s_tlast;
    reg               s_tvalid;
    wire              s_tready;
 
    wire                    m_rx_axis_tvalid;
    wire [DATA_W-1:0]       m_rx_axis_tdata;
    wire [KEEP_W-1:0]       m_rx_axis_tkeep;
    wire                    m_rx_axis_tlast;
 
    reg  ts_byte_valid = 0;
    reg  [7:0] ts_byte_l0 = 0, ts_byte_l1 = 0, ts_byte_l2 = 0, ts_byte_l3 = 0;
    wire link_trained, lane_reversed_locked;
    reg  hot_reset_request = 0, flr_request = 0, updatefc_received = 1;
    wire datapath_rst_n, axi_rst_n, flr_active;
 
    reg  cfg_wr_en = 0, cfg_rd_en = 0;
    reg  [11:0] cfg_addr = 0;
    reg  [31:0] cfg_wdata = 0;
    wire [31:0] cfg_rdata;
    wire cfg_cpl_valid;
 
    reg  tbl_wr_en = 0, tbl_rd_en = 0;
    reg  [2:0] tbl_wr_vec = 0, tbl_rd_vec = 0;
    reg  [63:0] tbl_wr_addr = 0;
    reg  [31:0] tbl_wr_data = 0;
    reg  tbl_wr_mask = 0;
    wire [63:0] tbl_rd_addr;
    wire [31:0] tbl_rd_data;
    wire tbl_rd_mask;
    wire [7:0] pba;
    reg  dma_complete_pulse = 0;
    reg  [2:0] dma_complete_vector = 0;
 
    reg  bar0_wr_en = 0, bar0_rd_en = 0;
    reg  [15:0] bar0_addr = 0;     // 0x0000 DMA regs, 0x1000 MSI-X table, 0x1800 PBA
    reg  [31:0] bar0_wdata = 0;
    wire [31:0] bar0_rdata;
    wire bar0_rd_valid;
 
    wire [11:0] tx_level;
    wire [3:0]  tx_level_valid;
    reg  [11:0] rx_level;
    reg  [3:0]  rx_level_valid;
 
    wire [NUM_STREAMS-1:0] session_start_aclk;
    reg  [NUM_STREAMS-1:0] session_end_aclk = 0;
    reg  [NUM_STREAMS-1:0] session_overflow_aclk = 0;
    wire [7:0] arb_weight_ch0, arb_weight_ch1;
    wire [23:0] session_limit;
 
    wire err_xlat_miss, err_addr_ovf, stat_split_occurred, err_tkeep_gap;
    wire crc_err, fec_err, fec_corrected, replay_event;
    wire phy_tx_idle, phy_framing_err, phy_lane_err;
 
    wire                  m_axi_awvalid;
    wire [AXI_ADDR_W-1:0] m_axi_awaddr;
    wire                  m_axi_awerr;
    wire                  malformed_tlp_err;
 
    // PCIe address of our write: base + 0 offset, matching the window
    // link_phy_bridge_top's default WINDOW_BASE/WINDOW_MASK/AXI_BASE_DEFAULT.
    localparam [63:0] PCIE_BASE  = 64'h0000_0001_0000_0000;
    localparam [31:0] AXI_BASE   = 32'h2000_0000;
    localparam [31:0] EXP_AWADDR = AXI_BASE; // offset 0 into the window
 
    link_phy_bridge_top #(
        .DATA_W(DATA_W), .KEEP_W(KEEP_W), .SOFF_W(SOFF_W), .TUSER_W(TUSER_W),
        .NUM_STREAMS(NUM_STREAMS), .SYS_BUS_W(SYS_BUS_W), .AXI_ADDR_W(AXI_ADDR_W)
    ) dut (
        .clk(clk), .perst_n(perst_n), .aclk(aclk), .aclk_rst_n(aclk_rst_n),
 
        .s_axis_tdata(s_tdata), .s_axis_tkeep(s_tkeep), .s_axis_tuser(s_tuser),
        .s_axis_tlast(s_tlast), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready),
 
        .m_rx_axis_tvalid(m_rx_axis_tvalid), .m_rx_axis_tdata(m_rx_axis_tdata),
        .m_rx_axis_tkeep(m_rx_axis_tkeep), .m_rx_axis_tlast(m_rx_axis_tlast),
        .m_rx_axis_tready(1'b1),
 
        .ts_byte_valid(ts_byte_valid),
        .ts_byte_l0(ts_byte_l0), .ts_byte_l1(ts_byte_l1),
        .ts_byte_l2(ts_byte_l2), .ts_byte_l3(ts_byte_l3),
        .link_trained(link_trained), .lane_reversed_locked(lane_reversed_locked),
        .hot_reset_request(hot_reset_request), .flr_request(flr_request),
        .updatefc_received(updatefc_received),
        .datapath_rst_n(datapath_rst_n), .axi_rst_n(axi_rst_n), .flr_active(flr_active),
 
        .cfg_wr_en(cfg_wr_en), .cfg_rd_en(cfg_rd_en), .cfg_addr(cfg_addr),
        .cfg_wdata(cfg_wdata), .cfg_rdata(cfg_rdata), .cfg_cpl_valid(cfg_cpl_valid),
 
        .tbl_wr_en(tbl_wr_en), .tbl_wr_vec(tbl_wr_vec), .tbl_wr_addr(tbl_wr_addr),
        .tbl_wr_data(tbl_wr_data), .tbl_wr_mask(tbl_wr_mask),
        .tbl_rd_en(tbl_rd_en), .tbl_rd_vec(tbl_rd_vec),
        .tbl_rd_addr(tbl_rd_addr), .tbl_rd_data(tbl_rd_data), .tbl_rd_mask(tbl_rd_mask),
        .pba(pba),
        .dma_complete_pulse(dma_complete_pulse), .dma_complete_vector(dma_complete_vector),
 
        .bar0_wr_en(bar0_wr_en), .bar0_rd_en(bar0_rd_en), .bar0_addr(bar0_addr),
        .bar0_wdata(bar0_wdata), .bar0_rdata(bar0_rdata), .bar0_rd_valid(bar0_rd_valid),
 
        .tx_level(tx_level), .tx_level_valid(tx_level_valid),
        .rx_level(rx_level), .rx_level_valid(rx_level_valid),
 
        .session_start_aclk(session_start_aclk),
        .session_end_aclk(session_end_aclk), .session_overflow_aclk(session_overflow_aclk),
        .arb_weight_ch0(arb_weight_ch0), .arb_weight_ch1(arb_weight_ch1),
        .session_limit(session_limit),
 
        .err_xlat_miss(err_xlat_miss), .err_addr_ovf(err_addr_ovf),
        .stat_split_occurred(stat_split_occurred), .err_tkeep_gap(err_tkeep_gap),
        .crc_err(crc_err), .fec_err(fec_err), .fec_corrected(fec_corrected),
        .replay_event(replay_event),
        .phy_tx_idle(phy_tx_idle), .phy_framing_err(phy_framing_err), .phy_lane_err(phy_lane_err),
 
        .m_axi_awvalid(m_axi_awvalid), .m_axi_awaddr(m_axi_awaddr), .m_axi_awerr(m_axi_awerr),
        .malformed_tlp_err(malformed_tlp_err)
    );
 
    // ---------------- PHY self-loopback (no error injection, no training) ----------------
    // Identity lane order is the reset default (lane_reversed_locked==0 until
    // training completes), so a straight registered loopback of the PAM4
    // symbol-level bus exercises the real TX serializer + RX deserializer
    // chain without needing to re-derive a TS1/TS2 training sequence here.
    always @(posedge clk or negedge perst_n) begin
        if (!perst_n) begin
            rx_level       <= 12'd0;
            rx_level_valid <= 4'd0;
        end else begin
            rx_level       <= tx_level;
            rx_level_valid <= tx_level_valid;
        end
    end
 
    // ---------------- result tracking ----------------
    integer n_aw = 0;
    reg [31:0] seen_awaddr;
    reg        seen_awerr;
    always @(posedge clk) if (perst_n && m_axi_awvalid) begin
        n_aw = n_aw + 1;
        seen_awaddr = m_axi_awaddr;
        seen_awerr  = m_axi_awerr;
    end
 
    integer n_crc = 0, n_fec = 0, n_malformed = 0, n_lane_err = 0, n_framing_err = 0;
    always @(posedge clk) if (perst_n) begin
        if (crc_err)          n_crc = n_crc + 1;
        if (fec_err)          n_fec = n_fec + 1;
        if (malformed_tlp_err) n_malformed = n_malformed + 1;
        if (phy_lane_err)     n_lane_err = n_lane_err + 1;
        if (phy_framing_err)  n_framing_err = n_framing_err + 1;
    end
 
    integer cyc = 0, last_progress;
    always @(posedge clk) cyc <= cyc + 1;
 
    // ---------------- session-arm via bar0 regs (host-software path) ----------------
    task cfg_write(input [11:0] a, input [31:0] d);
        begin
            @(negedge clk); cfg_addr = a; cfg_wdata = d; cfg_wr_en = 1;
            @(negedge clk); cfg_wr_en = 0;
        end
    endtask

    task bar0_write(input [15:0] addr, input [31:0] data);
        begin
            @(negedge clk);
            bar0_wr_en = 1; bar0_addr = addr; bar0_wdata = data;
            @(negedge clk);
            bar0_wr_en = 0;
        end
    endtask
 
    initial begin
        s_tvalid = 0; s_tdata = 0; s_tkeep = 0; s_tuser = 0; s_tlast = 0;
 
        repeat (5) @(posedge clk);
        perst_n = 1;
        // PHY LTSSM trains over the loopback by itself (~1700 clk), then the
        // host enables Memory Space (BAR0 is ignored until it is set).
        wait (link_trained === 1'b1);
        wait (axi_rst_n === 1'b1);
        cfg_write(12'h004, 32'h0000_0006);
        repeat (5) @(posedge clk);
 
        // Program base_addr_ch0 (lo/hi words) then arm channel 0.
        bar0_write(16'h0020, PCIE_BASE[31:0]);   // addr word 0x008 -> byte addr 0x020
        bar0_write(16'h0024, PCIE_BASE[63:32]);  // addr word 0x009 -> byte addr 0x024
        bar0_write(16'h0000, 32'h0000_0001);     // arm_pulse[0] = 1
 
        // Let the arm pulse drain through session_arm_ctrl -> this module's
        // cfg_base_addr arbiter -> link_layer_top's base register.
        repeat (10) @(posedge clk);
 
        // Send one short MWr beat: 4 DW (16 B) at offset 0, stream 0, tlast=1.
        @(negedge clk);
        s_tdata        = {DATA_W{1'b0}};
        s_tdata[31:0]  = 32'hA5A5_1234;
        s_tkeep        = {{(KEEP_W-4){1'b0}}, 4'hF};
        s_tuser        = {1'b1, 24'd0, 1'b0}; // {tlast_flag=1, session_offset=0, stream_id=0}
        s_tlast        = 1;
        s_tvalid       = 1;
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
        $display(" crc_err: %0d  fec_err: %0d  malformed_tlp_err: %0d  lane_err: %0d  framing_err: %0d",
                  n_crc, n_fec, n_malformed, n_lane_err, n_framing_err);
        if (n_aw >= 1 && seen_awaddr == EXP_AWADDR && !seen_awerr &&
            n_crc == 0 && n_fec == 0 && n_malformed == 0 &&
            n_lane_err == 0 && n_framing_err == 0)
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