// ------------------------------------------------------------------
// fifo_scoreboard: counters + edge-detection for Module 5's
// FIFO status/error signals. Single clock domain, shared with
// the module (confirmed per handoff note).
// CfgRd-readable, same pattern as your other performance counters.
// ------------------------------------------------------------------
module fifo_scoreboard #(
    parameter OCC_WIDTH = 5,   // matches his fifo_occupancy width
    parameter CNT_WIDTH = 32   // saturating counters
)(
    input  wire                   clk,
    input  wire                   rst_n,            // sync active-low, per his note

    // Signals FROM module
    input  wire [OCC_WIDTH-1:0]   fifo_occupancy,
    input  wire                   fifo_full,
    input  wire                   fifo_hwm,
    input  wire                   err_tkeep_gap,     // 1-cycle pulse, not level

    // CfgRd-readable outputs
    output reg [CNT_WIDTH-1:0]    err_tkeep_gap_count,
    output reg [OCC_WIDTH-1:0]    fifo_occupancy_peak,
    output reg [CNT_WIDTH-1:0]    fifo_hwm_cycles,
    output reg [CNT_WIDTH-1:0]    fifo_full_cycles,

    // W1C clears for all four, host-driven, independent bits so one
    // counter can be read+cleared without disturbing the others
    input  wire                   clear_err_count,
    input  wire                   clear_occ_peak,
    input  wire                   clear_hwm_cycles,
    input  wire                   clear_full_cycles
);

    // -------- err_tkeep_gap: registered-prior-value edge detect --------
    // Per project learning: same-cycle NBA races require comparing
    // against a REGISTERED prior value, not gating directly off the
    // raw incoming pulse combinationally. Since notes say
    // this is already a clean 1-cycle pulse (not a level), we still
    // register it once to decouple our counter's timing from
    // whatever combinational logic produces it on his side.
    reg err_tkeep_gap_d;
    always @(posedge clk) begin
        if (!rst_n) err_tkeep_gap_d <= 1'b0;
        else        err_tkeep_gap_d <= err_tkeep_gap;
    end

    wire err_tkeep_gap_rise = err_tkeep_gap & ~err_tkeep_gap_d;
    // NOTE: since the source is already documented as a single-cycle
    // pulse, err_tkeep_gap_rise will equal err_tkeep_gap_d's inverse
    // AND the raw signal for exactly one cycle -- this is intentionally
    // defensive in case the pulse is ever widened upstream (e.g. if his
    // module changes to hold the pulse for >1 cycle under some
    // condition, we still count ONE event, not N cycles of assertion).

    always @(posedge clk) begin
        if (!rst_n) begin
            err_tkeep_gap_count <= {CNT_WIDTH{1'b0}};
        end else if (clear_err_count) begin
            err_tkeep_gap_count <= {CNT_WIDTH{1'b0}};
        end else if (err_tkeep_gap_rise && !(&err_tkeep_gap_count)) begin
            err_tkeep_gap_count <= err_tkeep_gap_count + 1'b1;
        end
    end

    // -------- fifo_occupancy: peak tracker --------
    always @(posedge clk) begin
        if (!rst_n) begin
            fifo_occupancy_peak <= {OCC_WIDTH{1'b0}};
        end else if (clear_occ_peak) begin
            fifo_occupancy_peak <= {OCC_WIDTH{1'b0}};
        end else if (fifo_occupancy > fifo_occupancy_peak) begin
            fifo_occupancy_peak <= fifo_occupancy;
        end
    end

    // -------- fifo_hwm: level-based cycle counter --------
    always @(posedge clk) begin
        if (!rst_n) begin
            fifo_hwm_cycles <= {CNT_WIDTH{1'b0}};
        end else if (clear_hwm_cycles) begin
            fifo_hwm_cycles <= {CNT_WIDTH{1'b0}};
        end else if (fifo_hwm && !(&fifo_hwm_cycles)) begin
            fifo_hwm_cycles <= fifo_hwm_cycles + 1'b1;
        end
    end

    // -------- fifo_full: level-based cycle counter (should stay 0) --------
    always @(posedge clk) begin
        if (!rst_n) begin
            fifo_full_cycles <= {CNT_WIDTH{1'b0}};
        end else if (clear_full_cycles) begin
            fifo_full_cycles <= {CNT_WIDTH{1'b0}};
        end else if (fifo_full && !(&fifo_full_cycles)) begin
            fifo_full_cycles <= fifo_full_cycles + 1'b1;
        end
    end

endmodule
