`timescale 1ns/1ps

// ============================================================
// G3 (P2-3): two bugs fixed here.
//   * A window miss only pulsed translation_err for one cycle and left
//     nothing for software to read later -- a miss that happened between
//     polls of BAR0 STATUS was invisible. err_xlat_miss_cnt is now a
//     sticky, saturating (0xFF) count of misses, cleared by
//     err_xlat_miss_clr (link_phy_bridge_top pulses it on a write to the
//     BAR0 XLAT_MISS register).
//   * A request that arrived while axi_addr_valid was already asserted
//     and axi_addr_ready was low (translated address not yet taken) was
//     silently dropped: the old code's stall branch never looked at
//     req_valid at all. Latent today (m_axi_awready is tied high
//     end-to-end), but a real fix, not a note for later: one request is
//     now held in a 1-deep queue while stalled and serviced the cycle
//     the stall clears (no bubble). A second request arriving while that
//     queue slot is already full is still dropped (unavoidable with one
//     slot and no upstream back-pressure port) and flagged in
//     simulation.
// ============================================================
module pcie_address_translator #(
    parameter PCIE_ADDR_WIDTH = 64,
    parameter AXI_ADDR_WIDTH  = 32
)(
    input  wire                       clk,
    input  wire                       rst_n,

    input  wire [PCIE_ADDR_WIDTH-1:0] trans_window_base,
    input  wire [PCIE_ADDR_WIDTH-1:0] trans_window_mask,
    input  wire [AXI_ADDR_WIDTH-1:0]  trans_axi_base,

    input  wire                       req_valid,
    input  wire [PCIE_ADDR_WIDTH-1:0] req_pcie_addr,

    input  wire                       axi_addr_ready, // AXI4 Handshake addition
    output reg                        axi_addr_valid,
    output reg  [AXI_ADDR_WIDTH-1:0]  axi_addr,
    output reg                        translation_err,

    // G3: sticky miss counter (destined for BAR0 STATUS) + clear strobe
    output reg  [7:0]                 err_xlat_miss_cnt,
    input  wire                       err_xlat_miss_clr
);

    // Request actually being serviced this cycle: the queued one while a
    // stall is still in effect, else whatever req_valid presents now.
    reg                        pend_valid;
    reg [PCIE_ADDR_WIDTH-1:0]  pend_addr;
    wire                       stalled = axi_addr_valid && !axi_addr_ready;

    // Serviced only when !stalled (see below): the request that was
    // queued during a stall takes priority over a newly-arriving one.
    wire                       proc_valid = pend_valid || req_valid;
    wire [PCIE_ADDR_WIDTH-1:0] proc_addr  = pend_valid ? pend_addr : req_pcie_addr;
    wire [PCIE_ADDR_WIDTH-1:0] proc_masked= proc_addr & trans_window_mask;
    wire                       proc_hit   = (proc_masked == trans_window_base);
    wire [AXI_ADDR_WIDTH-1:0]  proc_offset= proc_addr[AXI_ADDR_WIDTH-1:0] &
                                            ~trans_window_mask[AXI_ADDR_WIDTH-1:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            axi_addr_valid    <= 1'b0;
            axi_addr          <= 0;
            translation_err   <= 1'b0;
            pend_valid        <= 1'b0;
            pend_addr         <= 0;
            err_xlat_miss_cnt <= 8'h0;
        end else begin
            if (stalled) begin
                // Hold valid/data; queue a new request instead of
                // dropping it (1-deep -- a second arrival while the
                // queue is already full has nowhere to go).
                axi_addr_valid <= 1'b1;
                if (req_valid && !pend_valid) begin
                    pend_valid <= 1'b1;
                    pend_addr  <= req_pcie_addr;
                end
                // synthesis translate_off
                if (req_valid && pend_valid)
                    $error("[pcie_address_translator] request dropped: 1-deep queue full while AXI slave stalled");
                // synthesis translate_on
            end else if (proc_valid) begin
                if (proc_hit) begin
                    axi_addr_valid  <= 1'b1;
                    axi_addr        <= trans_axi_base + proc_offset;
                    translation_err <= 1'b0;
                end else begin
                    axi_addr_valid  <= 1'b0;
                    translation_err <= 1'b1;
                    if (err_xlat_miss_cnt != 8'hFF)
                        err_xlat_miss_cnt <= err_xlat_miss_cnt + 8'd1;
                end
                pend_valid <= 1'b0;
            end else begin
                axi_addr_valid  <= 1'b0;
                translation_err <= 1'b0;
            end

            if (err_xlat_miss_clr) err_xlat_miss_cnt <= 8'h0;
        end
    end

endmodule