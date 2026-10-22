`timescale 1ns/1ps
// =============================================================================
// bar0_dma_regs - BAR0 DMA/session register block (fixes P4 / Seam B)
//
// Holds everything the host programs before a session that the review found
// had no home: per-channel base addresses, arm bits, arbiter weights, the
// session limit and the MSI-X vector per channel.
//
// Access port: bar0_wr_en / bar0_rd_en with a byte offset into BAR0. There is
// no inbound TLP path yet (host MWr/MRd -> BAR0), so for now cocotb drives this
// port directly, the same way it drives cfg_*. When Swaray's RX path decodes
// MWr/MRd hitting BAR0, it connects here unchanged (offset = addr - BAR0 base).
//
// Register map (byte offsets, 32-bit registers):
//   0x00 CTRL          W1S, reads 0   [0] arm ch0  [1] arm ch1   (1-cycle pulses)
//   0x04 STATUS        RO             [0] session_valid ch0  [1] session_valid ch1
//                                         session_valid clears once the session's data has
//                                         LEFT THE LINK LAYER (session_done) -> safe to re-arm.
//                                         An arm while it is still 1 is ignored and counted
//                                         in ARM_DEBUG.
//                                     [5:4] interconnect finished accepting ch0/ch1, data
//                                         still draining (debug; from session_end_aclk)
//                                     [9:8] status_in (overflow flags from the interconnect)
//   0x08 ARB_WEIGHTS   RW             [WW-1:0] weight ch0   [8+WW-1:8] weight ch1
//                                     reset 1:1
//   0x0C SESSION_LIMIT RW             [LW-1:0] session byte limit, reset all-ones
//   0x10 MSIX_VEC      RW             [2:0] vector ch0  [10:8] vector ch1, reset 0 / 1
//   0x14 ARM_DEBUG     RO             [7:0] arm-while-active count ch0, [15:8] ch1
//   0x18 TC            RW             [2:0] traffic class ch0  [10:8] ch1
//   0x1C XLAT_MISS      RO             [7:0] sticky count of inbound-MWr
//                                      address-translation window misses
//                                      (G3/P2-3), saturates at 0xFF. Any
//                                      write to this offset clears it.
//   0x20 BASE_CH0_LO   RW             base_addr_ch0[31:0]
//   0x24 BASE_CH0_HI   RW             base_addr_ch0[63:32]
//   0x28 BASE_CH1_LO   RW             base_addr_ch1[31:0]
//   0x2C BASE_CH1_HI   RW             base_addr_ch1[63:32]
//
// Write base/limit/weights FIRST, then set the arm bit: session_arm_ctrl
// snapshots base_addr/tc on arm, and weights/limit are treated as quasi-static
// by the aclk-domain interconnect (they must not change during a session).
// =============================================================================
module bar0_dma_regs #(
    parameter WW = 8,     // arbiter weight width (match interconnect_top), 1..8
    parameter LW = 24     // session limit width  (match interconnect session_offset), 1..32
)(
    input  wire          clk,
    input  wire          rst_n,

    input  wire          bar0_wr_en,
    input  wire          bar0_rd_en,
    input  wire [11:0]   bar0_addr,
    input  wire [31:0]   bar0_wdata,
    output reg  [31:0]   bar0_rdata,
    output reg           bar0_rd_valid,

    // ---- programmed values ----
    output reg  [1:0]    arm_pulse,
    output reg  [63:0]   base_addr_ch0,
    output reg  [63:0]   base_addr_ch1,
    output reg  [WW-1:0] weight_ch0,
    output reg  [WW-1:0] weight_ch1,
    output reg  [LW-1:0] session_limit,
    output reg  [2:0]    msix_vec_ch0,
    output reg  [2:0]    msix_vec_ch1,
    output reg  [2:0]    tc_ch0,
    output reg  [2:0]    tc_ch1,

    // ---- status readback ----
    input  wire [1:0]    session_valid,
    input  wire [1:0]    status_in,
    input  wire [1:0]    ic_end_seen,
    input  wire [7:0]    arm_dbg_ch0,
    input  wire [7:0]    arm_dbg_ch1,

    // ---- G3 (P2-3): inbound address-translation miss count ----
    input  wire [7:0]    xlat_miss_cnt,
    output reg           xlat_miss_clr
);

// zero-extended views for readback (WW <= 8, LW <= 32)
wire [7:0]  w0_x  = weight_ch0;
wire [7:0]  w1_x  = weight_ch1;
wire [31:0] lim_x = session_limit;

always @(posedge clk) begin
    if (!rst_n) begin
        arm_pulse     <= 2'b00;
        base_addr_ch0 <= 64'h0;
        base_addr_ch1 <= 64'h0;
        weight_ch0    <= {{(WW-1){1'b0}}, 1'b1};
        weight_ch1    <= {{(WW-1){1'b0}}, 1'b1};
        session_limit <= {LW{1'b1}};
        msix_vec_ch0  <= 3'd0;
        msix_vec_ch1  <= 3'd1;
        tc_ch0        <= 3'd0;
        tc_ch1        <= 3'd0;
        bar0_rdata    <= 32'h0;
        bar0_rd_valid <= 1'b0;
    end else begin
        arm_pulse     <= 2'b00;
        bar0_rd_valid <= 1'b0;
        xlat_miss_clr <= 1'b0;

        if (bar0_wr_en) begin
            case (bar0_addr[11:2])
                10'h000: arm_pulse <= bar0_wdata[1:0];
                10'h002: begin weight_ch0 <= bar0_wdata[WW-1:0]; weight_ch1 <= bar0_wdata[8+WW-1:8]; end
                10'h003: session_limit <= bar0_wdata[LW-1:0];
                10'h004: begin msix_vec_ch0 <= bar0_wdata[2:0]; msix_vec_ch1 <= bar0_wdata[10:8]; end
                10'h006: begin tc_ch0 <= bar0_wdata[2:0]; tc_ch1 <= bar0_wdata[10:8]; end
                10'h007: xlat_miss_clr <= 1'b1;   // G3: any write clears the sticky counter
                10'h008: base_addr_ch0[31:0]  <= bar0_wdata;
                10'h009: base_addr_ch0[63:32] <= bar0_wdata;
                10'h00A: base_addr_ch1[31:0]  <= bar0_wdata;
                10'h00B: base_addr_ch1[63:32] <= bar0_wdata;
                default: ; // RO / reserved
            endcase
        end

        if (bar0_rd_en) begin
            bar0_rd_valid <= 1'b1;
            case (bar0_addr[11:2])
                10'h000: bar0_rdata <= 32'h0;
                10'h001: bar0_rdata <= {22'h0, status_in, 2'h0, ic_end_seen, 2'h0, session_valid};
                10'h002: bar0_rdata <= {16'h0, w1_x, w0_x};
                10'h003: bar0_rdata <= lim_x;
                10'h004: bar0_rdata <= {21'h0, msix_vec_ch1, 5'h0, msix_vec_ch0};
                10'h005: bar0_rdata <= {16'h0, arm_dbg_ch1, arm_dbg_ch0};
                10'h006: bar0_rdata <= {21'h0, tc_ch1, 5'h0, tc_ch0};
                10'h007: bar0_rdata <= {24'h0, xlat_miss_cnt};
                10'h008: bar0_rdata <= base_addr_ch0[31:0];
                10'h009: bar0_rdata <= base_addr_ch0[63:32];
                10'h00A: bar0_rdata <= base_addr_ch1[31:0];
                10'h00B: bar0_rdata <= base_addr_ch1[63:32];
                default: bar0_rdata <= 32'h0;
            endcase
        end
    end
end

endmodule
