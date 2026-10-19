`timescale 1ns/1ps

// ============================================================
// Module : cfg_base_addr_bridge
// Layer  : Link layer / PHY seam -- NEW (wiring fix for the PHY drop)
//
// Kshitij's bar0_dma_regs + session_arm_ctrl (pcie_phy_top) expose the two
// channel base addresses as two always-driven 64-bit buses plus a 2-bit
// per-channel write pulse:
//
//     cfg_base_addr_ch0[63:0], cfg_base_addr_ch1[63:0], cfg_base_addr_wr[1:0]
//
// address_translation (inside stream_tx_path_top, unchanged from before the
// PHY existed) wants a single muxed register-file bus instead:
//
//     cfg_base_addr_wr (1 pulse), cfg_base_addr_id (which stream),
//     cfg_base_addr_data (64-bit value), cfg_base_addr_clr
//
// These two shapes don't connect directly -- this module is the adapter.
// It also covers the case bar0_dma_regs allows and address_translation
// cannot see on its own: software arming both channels in the same BAR0
// CTRL write (arm_pulse[1:0] = 2'b11), which makes cfg_base_addr_wr[1:0]
// pulse 2'b11 in the same clk cycle. Only one id can be driven per cycle,
// so channel 1's write is queued and issued the next cycle (channel 0 is
// never delayed). No write is ever dropped.
//
// There is no PHY-side equivalent of cfg_base_addr_clr (nothing in the
// review ever asked for a "clear all base addresses" register); it is
// tied low here. address_translation already treats a 0 base address as
// "unprogrammed" from its own reset state, so this does not reintroduce
// the base_unprog error path.
// ============================================================

module cfg_base_addr_bridge #(
    parameter ADDR_W = 64
)(
    input  wire              clk,
    input  wire              rst_n,

    // from pcie_phy_top
    input  wire [1:0]        cfg_base_addr_wr_ch,   // per-channel 1-cycle pulse
    input  wire [ADDR_W-1:0] cfg_base_addr_ch0,
    input  wire [ADDR_W-1:0] cfg_base_addr_ch1,

    // to address_translation (via stream_tx_path_top)
    output reg                cfg_base_addr_wr,
    output reg                cfg_base_addr_id,
    output reg  [ADDR_W-1:0]  cfg_base_addr_data
);

    reg               pend1_v;
    reg [ADDR_W-1:0]  pend1_data;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cfg_base_addr_wr   <= 1'b0;
            cfg_base_addr_id   <= 1'b0;
            cfg_base_addr_data <= {ADDR_W{1'b0}};
            pend1_v            <= 1'b0;
            pend1_data         <= {ADDR_W{1'b0}};
        end else begin
            cfg_base_addr_wr <= 1'b0;

            if (cfg_base_addr_wr_ch[0]) begin
                // ch0 always goes out now; queue ch1 if it collided with it
                cfg_base_addr_wr   <= 1'b1;
                cfg_base_addr_id   <= 1'b0;
                cfg_base_addr_data <= cfg_base_addr_ch0;
                if (cfg_base_addr_wr_ch[1]) begin
                    pend1_v    <= 1'b1;
                    pend1_data <= cfg_base_addr_ch1;
                end
            end else if (cfg_base_addr_wr_ch[1]) begin
                cfg_base_addr_wr   <= 1'b1;
                cfg_base_addr_id   <= 1'b1;
                cfg_base_addr_data <= cfg_base_addr_ch1;
            end else if (pend1_v) begin
                cfg_base_addr_wr   <= 1'b1;
                cfg_base_addr_id   <= 1'b1;
                cfg_base_addr_data <= pend1_data;
                pend1_v            <= 1'b0;
            end
        end
    end

endmodule
