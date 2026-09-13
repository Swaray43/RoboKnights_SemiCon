//=============================================================================
// rr_arbiter_2ch.v
// Weighted round-robin arbiter, 2 channels, with starvation guard.
// Layer: Interconnect layer -- selects which AXI4-Stream source (ch0 =
// SmartNIC, ch1 = ML) drives the crossbar output this cycle.
//
// Design:
//   - Each channel has a credit counter, refilled to its runtime-
//     programmable weight (weight0/weight1) whenever BOTH credits have
//     been exhausted (a "round" completes). Weight ratio sets the
//     long-run service ratio -- default 50/50 via weight0==weight1, but
//     tunable at runtime.
//   - A channel is eligible only while it has a request AND nonzero
//     credit. Among eligible channels, ties broken by round-robin
//     (whichever channel was NOT granted last cycle wins).
//   - Starvation guard: if a channel has an outstanding request that has
//     gone ungranted for STARVE_CYCLES consecutive cycles, it is granted
//     unconditionally next cycle regardless of credit -- guarantees
//     forward progress even at weight 0 or after repeatedly losing ties.
//   - grant0/grant1 are REGISTERED outputs: the decision for cycle N is
//     computed combinationally from state at cycle N and registered, so
//     it drives the downstream crossbar mux select at cycle N+1 with no
//     combinational path from req/weight through the 512-bit data mux.
//
// Known simplification: credit-based WRR grants a channel's full weight
// worth of consecutive beats before switching (e.g. weight 3:1 -> ch0,
// ch0, ch0, ch1), not an interleaved/smoothed pattern. Fine for coarse
// bandwidth-share tuning; flag if a smoother (deficit/interleaved) WRR
// is actually needed.
//=============================================================================

module rr_arbiter_2ch #(
    parameter CREDIT_WIDTH  = 8,   // weight/credit register width
    parameter STARVE_WIDTH  = 16,  // wait-counter width
    parameter STARVE_CYCLES = 64   // cycles waiting before starvation override fires
)(
    input  wire                      clk,
    input  wire                      rstn,

    input  wire                      req0,      // ch0 (SmartNIC) has a beat ready
    input  wire                      req1,      // ch1 (ML) has a beat ready

    input  wire [CREDIT_WIDTH-1:0]   weight0,   // runtime-programmable weight, ch0
    input  wire [CREDIT_WIDTH-1:0]   weight1,   // runtime-programmable weight, ch1

    output reg                       grant0,    // registered: valid for the NEXT cycle's mux select
    output reg                       grant1
);

    reg [CREDIT_WIDTH-1:0]  credit0, credit1;
    reg                     last_grant;          // 0 = ch0 was last granted, 1 = ch1
    reg [STARVE_WIDTH-1:0]  wait0, wait1;         // consecutive cycles req asserted w/o grant

    wire starved0 = req0 && (wait0 >= STARVE_CYCLES);
    wire starved1 = req1 && (wait1 >= STARVE_CYCLES);

    // Per-channel refill trigger: a channel's credit is due for refill once
    // it's exhausted AND the other channel is either not competing for
    // credit (no request) or is itself exhausted. Using OWN exhaustion plus
    // the OTHER channel's state (not a single joint "both zero" flag) is
    // what prevents deadlock when only one channel is ever requesting --
    // an idle channel's credit would otherwise never decrement, so a joint
    // "both must hit zero" condition would never fire again.
    wire round_done0 = (credit0 == {CREDIT_WIDTH{1'b0}}) && (!req1 || credit1 == {CREDIT_WIDTH{1'b0}});
    wire round_done1 = (credit1 == {CREDIT_WIDTH{1'b0}}) && (!req0 || credit0 == {CREDIT_WIDTH{1'b0}});

    // Credit "available this cycle" folds in an about-to-refill channel so
    // eligibility doesn't stall for a cycle waiting on the registered
    // refill to land -- avoids a one-cycle bubble every round.
    wire cred_avail0 = (credit0 != {CREDIT_WIDTH{1'b0}}) || round_done0;
    wire cred_avail1 = (credit1 != {CREDIT_WIDTH{1'b0}}) || round_done1;

    wire eligible0 = req0 && cred_avail0;
    wire eligible1 = req1 && cred_avail1;

    // ---- next-grant decision (combinational) ----
    reg next_g0, next_g1;
    always @(*) begin
        next_g0 = 1'b0;
        next_g1 = 1'b0;
        if (starved0 && !starved1) begin
            next_g0 = 1'b1;
        end else if (starved1 && !starved0) begin
            next_g1 = 1'b1;
        end else if (starved0 && starved1) begin
            if (last_grant == 1'b0) next_g1 = 1'b1;
            else                    next_g0 = 1'b1;
        end else if (eligible0 && eligible1) begin
            if (last_grant == 1'b0) next_g1 = 1'b1;
            else                    next_g0 = 1'b1;
        end else if (eligible0) begin
            next_g0 = 1'b1;
        end else if (eligible1) begin
            next_g1 = 1'b1;
        end
    end

    always @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            grant0     <= 1'b0;
            grant1     <= 1'b0;
            credit0    <= {CREDIT_WIDTH{1'b0}};
            credit1    <= {CREDIT_WIDTH{1'b0}};
            last_grant <= 1'b0;
            wait0      <= {STARVE_WIDTH{1'b0}};
            wait1      <= {STARVE_WIDTH{1'b0}};
        end else begin
            grant0 <= next_g0;
            grant1 <= next_g1;

            // credit0: refill (if due) then consume (if granted) -- independent of ch1
            if (round_done0) begin
                credit0 <= next_g0 ? ((weight0 != 0) ? weight0 - 1'b1 : {CREDIT_WIDTH{1'b0}})
                                    : weight0;
            end else if (next_g0) begin
                credit0 <= (credit0 != 0) ? credit0 - 1'b1 : {CREDIT_WIDTH{1'b0}};
            end

            // credit1: same, independent of ch0
            if (round_done1) begin
                credit1 <= next_g1 ? ((weight1 != 0) ? weight1 - 1'b1 : {CREDIT_WIDTH{1'b0}})
                                    : weight1;
            end else if (next_g1) begin
                credit1 <= (credit1 != 0) ? credit1 - 1'b1 : {CREDIT_WIDTH{1'b0}};
            end

            if (next_g0) begin
                last_grant <= 1'b0;
                wait0      <= {STARVE_WIDTH{1'b0}};
                wait1      <= (req1) ? wait1 + 1'b1 : {STARVE_WIDTH{1'b0}};
            end else if (next_g1) begin
                last_grant <= 1'b1;
                wait1      <= {STARVE_WIDTH{1'b0}};
                wait0      <= (req0) ? wait0 + 1'b1 : {STARVE_WIDTH{1'b0}};
            end else begin
                wait0 <= (req0) ? wait0 + 1'b1 : {STARVE_WIDTH{1'b0}};
                wait1 <= (req1) ? wait1 + 1'b1 : {STARVE_WIDTH{1'b0}};
            end
        end
    end

endmodule
