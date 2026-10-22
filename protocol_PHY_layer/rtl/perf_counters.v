`timescale 1ns/1ps
// ------------------------------------------------------------------
// perf_counters: the performance-counter set from work_distribution.md,
// minus tag utilization (sourced directly from pcie_tag_manager's
// tags_in_use, not duplicated here) and minus the cocotb coverage
// model / MWr scoreboard (those require BFM infrastructure, which
// remains an explicit Stage 1 deferral -- not modeled in RTL).
//
// All counters are 32-bit saturating, CfgRd-readable, independently
// W1C-clearable so a host can read+clear one without disturbing others.
// Credit types follow standard PCIe credit classes: Posted Header (PH),
// Posted Data (PD), Non-Posted Header (NPH), Non-Posted Data (NPD).
// ------------------------------------------------------------------
module perf_counters #(
    parameter CNT_WIDTH      = 32,
    parameter REORDER_WIDTH  = 8    // width of reorder buffer occupancy input
)(
    input  wire clk,
    input  wire rst_n,

    // ---- TLP TX/RX counts: one pulse per TLP sent/received ----
    input  wire tlp_tx_pulse,
    input  wire tlp_rx_pulse,
    output reg [CNT_WIDTH-1:0] tlp_tx_count,
    output reg [CNT_WIDTH-1:0] tlp_rx_count,
    input  wire clear_tlp_tx,
    input  wire clear_tlp_rx,

    // ---- Credit stall cycles, per credit type. Level signals: high
    //      for every cycle a TX is stalled waiting on that credit type ----
    input  wire credit_stall_ph,
    input  wire credit_stall_pd,
    input  wire credit_stall_nph,
    input  wire credit_stall_npd,
    output reg [CNT_WIDTH-1:0] credit_stall_ph_cycles,
    output reg [CNT_WIDTH-1:0] credit_stall_pd_cycles,
    output reg [CNT_WIDTH-1:0] credit_stall_nph_cycles,
    output reg [CNT_WIDTH-1:0] credit_stall_npd_cycles,
    input  wire clear_credit_stall,   // single clear for all four; grouped since they're one logical stat family

    // ---- Reorder buffer occupancy: peak/current. Current is a
    //      pass-through (owned by the reorder buffer itself, not
    //      counted here); peak is tracked in this module ----
    input  wire [REORDER_WIDTH-1:0] reorder_occupancy_current,
    output reg  [REORDER_WIDTH-1:0] reorder_occupancy_peak,
    input  wire clear_reorder_peak,

    // ---- FLIT NAK count: one pulse per NAK received ----
    input  wire flit_nak_pulse,
    output reg [CNT_WIDTH-1:0] flit_nak_count,
    input  wire clear_flit_nak,

    // ---- FEC error counts: corrected (recovered in-line) vs
    //      uncorrectable (link-layer must request retransmit/retrain) ----
    input  wire fec_corrected_pulse,
    input  wire fec_uncorrectable_pulse,
    output reg [CNT_WIDTH-1:0] fec_corrected_count,
    output reg [CNT_WIDTH-1:0] fec_uncorrectable_count,
    input  wire clear_fec_corrected,
    input  wire clear_fec_uncorrectable
);

    // All pulse-driven counters follow the same registered-edge-detect
    // pattern established for err_tkeep_gap in swaray_fifo_scoreboard:
    // register the incoming pulse once before counting its rising edge,
    // so this module doesn't assume the source is glitch-free or
    // exactly one cycle wide.
    reg tlp_tx_d, tlp_rx_d, flit_nak_d, fec_corr_d, fec_uncorr_d;

    always @(posedge clk) begin
        if (!rst_n) begin
            tlp_tx_d     <= 1'b0;
            tlp_rx_d     <= 1'b0;
            flit_nak_d   <= 1'b0;
            fec_corr_d   <= 1'b0;
            fec_uncorr_d <= 1'b0;
        end else begin
            tlp_tx_d     <= tlp_tx_pulse;
            tlp_rx_d     <= tlp_rx_pulse;
            flit_nak_d   <= flit_nak_pulse;
            fec_corr_d   <= fec_corrected_pulse;
            fec_uncorr_d <= fec_uncorrectable_pulse;
        end
    end

    wire tlp_tx_rise     = tlp_tx_pulse     & ~tlp_tx_d;
    wire tlp_rx_rise     = tlp_rx_pulse     & ~tlp_rx_d;
    wire flit_nak_rise   = flit_nak_pulse   & ~flit_nak_d;
    wire fec_corr_rise   = fec_corrected_pulse   & ~fec_corr_d;
    wire fec_uncorr_rise = fec_uncorrectable_pulse & ~fec_uncorr_d;

    // ---- TLP TX/RX ----
    always @(posedge clk) begin
        if (!rst_n) tlp_tx_count <= 0;
        else if (clear_tlp_tx) tlp_tx_count <= 0;
        else if (tlp_tx_rise && !(&tlp_tx_count)) tlp_tx_count <= tlp_tx_count + 1'b1;
    end
    always @(posedge clk) begin
        if (!rst_n) tlp_rx_count <= 0;
        else if (clear_tlp_rx) tlp_rx_count <= 0;
        else if (tlp_rx_rise && !(&tlp_rx_count)) tlp_rx_count <= tlp_rx_count + 1'b1;
    end

    // ---- Credit stall cycles (level-driven: no edge-detect needed,
    //      these count CYCLES asserted, not events) ----
    always @(posedge clk) begin
        if (!rst_n) credit_stall_ph_cycles <= 0;
        else if (clear_credit_stall) credit_stall_ph_cycles <= 0;
        else if (credit_stall_ph && !(&credit_stall_ph_cycles)) credit_stall_ph_cycles <= credit_stall_ph_cycles + 1'b1;
    end
    always @(posedge clk) begin
        if (!rst_n) credit_stall_pd_cycles <= 0;
        else if (clear_credit_stall) credit_stall_pd_cycles <= 0;
        else if (credit_stall_pd && !(&credit_stall_pd_cycles)) credit_stall_pd_cycles <= credit_stall_pd_cycles + 1'b1;
    end
    always @(posedge clk) begin
        if (!rst_n) credit_stall_nph_cycles <= 0;
        else if (clear_credit_stall) credit_stall_nph_cycles <= 0;
        else if (credit_stall_nph && !(&credit_stall_nph_cycles)) credit_stall_nph_cycles <= credit_stall_nph_cycles + 1'b1;
    end
    always @(posedge clk) begin
        if (!rst_n) credit_stall_npd_cycles <= 0;
        else if (clear_credit_stall) credit_stall_npd_cycles <= 0;
        else if (credit_stall_npd && !(&credit_stall_npd_cycles)) credit_stall_npd_cycles <= credit_stall_npd_cycles + 1'b1;
    end

    // ---- Reorder buffer occupancy peak ----
    always @(posedge clk) begin
        if (!rst_n) reorder_occupancy_peak <= 0;
        else if (clear_reorder_peak) reorder_occupancy_peak <= 0;
        else if (reorder_occupancy_current > reorder_occupancy_peak)
            reorder_occupancy_peak <= reorder_occupancy_current;
    end

    // ---- FLIT NAK ----
    always @(posedge clk) begin
        if (!rst_n) flit_nak_count <= 0;
        else if (clear_flit_nak) flit_nak_count <= 0;
        else if (flit_nak_rise && !(&flit_nak_count)) flit_nak_count <= flit_nak_count + 1'b1;
    end

    // ---- FEC corrected/uncorrectable ----
    always @(posedge clk) begin
        if (!rst_n) fec_corrected_count <= 0;
        else if (clear_fec_corrected) fec_corrected_count <= 0;
        else if (fec_corr_rise && !(&fec_corrected_count)) fec_corrected_count <= fec_corrected_count + 1'b1;
    end
    always @(posedge clk) begin
        if (!rst_n) fec_uncorrectable_count <= 0;
        else if (clear_fec_uncorrectable) fec_uncorrectable_count <= 0;
        else if (fec_uncorr_rise && !(&fec_uncorrectable_count)) fec_uncorrectable_count <= fec_uncorrectable_count + 1'b1;
    end

endmodule
