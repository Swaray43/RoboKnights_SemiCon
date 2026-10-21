//=============================================================================
// cdc_fifo_async.v
// Generic gray-code asynchronous FIFO for crossing between two unrelated
// clock domains. Standard Cummings-style dual-pointer design: binary write/
// read pointers, gray-coded before crossing, 2-flop synchronizers on the
// receiving side of each pointer.
//
// full uses a LOOKAHEAD pointer (wr_ptr_bin + 1, unconditional -- not
// gated by wr_en or by full itself) compared against the synchronized read
// pointer. Making the lookahead unconditional is what avoids a
// combinational self-reference (full depending on a signal that depends on
// full) -- the actual conditional pointer advance is a separate register
// update gated by the already-resolved combinational full value, which is
// safe because full's own formula never references that gated advance.
// empty is a direct current-pointer comparison (no lookahead needed --
// "is there data available right now" doesn't require prediction).
//
// almost_full is derived from a gray->binary conversion of the already-
// synchronized pointer -- safe to do AFTER synchronization, never on a raw
// crossing signal.
//
// XDC/SDC: the gray-coded pointer crossings need multicycle path
// constraints in both the Vivado and OpenLane flows -- see
// interconnect_constraints.xdc / .sdc.
//=============================================================================

module cdc_fifo_async #(
    parameter WIDTH           = 512,
    parameter ADDR_WIDTH      = 4,          // depth = 2**ADDR_WIDTH
    parameter ALMOST_FULL_PCT = 75          // assert almost_full at this % occupancy
)(
    // ---- write side ----
    input  wire                  wr_clk,
    input  wire                  wr_rstn,
    input  wire                  wr_en,
    input  wire [WIDTH-1:0]      wr_data,
    output wire                  wr_full,
    output wire                  wr_almost_full,

    // ---- read side ----
    input  wire                  rd_clk,
    input  wire                  rd_rstn,
    input  wire                  rd_en,
    output wire [WIDTH-1:0]      rd_data,
    output wire                  rd_empty
);

    localparam DEPTH = (1 << ADDR_WIDTH);
    localparam ALMOST_FULL_THRESH = (DEPTH * ALMOST_FULL_PCT) / 100;

    (* ram_style = "distributed" *) reg [WIDTH-1:0] mem [0:DEPTH-1];

    // ---- write-side pointers ----
    reg  [ADDR_WIDTH:0] wr_ptr_bin;
    reg  [ADDR_WIDTH:0] wr_ptr_gray;

    // Unconditional lookahead -- does NOT depend on wr_en or wr_full, so
    // wr_full (derived from this) cannot form a loop back through itself.
    wire [ADDR_WIDTH:0] wr_ptr_bin_lookahead  = wr_ptr_bin + 1'b1;
    wire [ADDR_WIDTH:0] wr_ptr_gray_lookahead = (wr_ptr_bin_lookahead >> 1) ^ wr_ptr_bin_lookahead;

    // ---- read-side pointers ----
    reg  [ADDR_WIDTH:0] rd_ptr_bin;
    reg  [ADDR_WIDTH:0] rd_ptr_gray;

    // ---- synchronizers: 2-flop, gray-coded pointer crossing ----
    reg [ADDR_WIDTH:0] rd_ptr_gray_wr1, rd_ptr_gray_wr2; // read ptr synced into wr_clk domain
    reg [ADDR_WIDTH:0] wr_ptr_gray_rd1, wr_ptr_gray_rd2; // write ptr synced into rd_clk domain

    always @(posedge wr_clk or negedge wr_rstn) begin
        if (!wr_rstn) begin
            rd_ptr_gray_wr1 <= {(ADDR_WIDTH+1){1'b0}};
            rd_ptr_gray_wr2 <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            rd_ptr_gray_wr1 <= rd_ptr_gray;
            rd_ptr_gray_wr2 <= rd_ptr_gray_wr1;
        end
    end

    always @(posedge rd_clk or negedge rd_rstn) begin
        if (!rd_rstn) begin
            wr_ptr_gray_rd1 <= {(ADDR_WIDTH+1){1'b0}};
            wr_ptr_gray_rd2 <= {(ADDR_WIDTH+1){1'b0}};
        end else begin
            wr_ptr_gray_rd1 <= wr_ptr_gray;
            wr_ptr_gray_rd2 <= wr_ptr_gray_rd1;
        end
    end

    // ---- full: lookahead gray vs synced read gray, top two bits inverted ----
    // Purely a function of two registers (wr_ptr_bin, rd_ptr_gray_wr2) --
    // no self-reference.
    assign wr_full = (wr_ptr_gray_lookahead == {~rd_ptr_gray_wr2[ADDR_WIDTH:ADDR_WIDTH-1],
                                                  rd_ptr_gray_wr2[ADDR_WIDTH-2:0]});

    // ---- empty: direct current-pointer comparison, no lookahead needed ----
    assign rd_empty = (rd_ptr_gray == wr_ptr_gray_rd2);

    // ---- write logic: actual advance gated by the resolved wr_full ----
    always @(posedge wr_clk or negedge wr_rstn) begin
        if (!wr_rstn) begin
            wr_ptr_bin  <= {(ADDR_WIDTH+1){1'b0}};
            wr_ptr_gray <= {(ADDR_WIDTH+1){1'b0}};
        end else if (wr_en && !wr_full) begin
            wr_ptr_bin  <= wr_ptr_bin + 1'b1;
            wr_ptr_gray <= (wr_ptr_bin_lookahead >> 1) ^ wr_ptr_bin_lookahead;
        end
    end

    // ---- memory write: its own clocked block, NO reset (ticket #5) ----
    // A memory array inside an async-reset always block cannot map to a RAM
    // primitive (RAM has no reset), so Vivado builds it from flip-flops.
    // Contents need no reset: rd_empty/wr_full gate every access. Same
    // enable (wr_en && !wr_full) as the pointer advance, so behaviour is
    // identical. Read stays combinational (FWFT), so synthesis maps this to
    // distributed (LUT) RAM, the right fit at depth 16; a block-RAM mapping
    // would need a registered read port (extra latency) and is only worth
    // it for much deeper FIFOs.
    always @(posedge wr_clk) begin
        if (wr_en && !wr_full)
            mem[wr_ptr_bin[ADDR_WIDTH-1:0]] <= wr_data;
    end

    // ---- read logic: first-word-fall-through ----
    // rd_data is combinational off the CURRENT read pointer, so valid data
    // is present on rd_data whenever !rd_empty -- BEFORE rd_en is asserted,
    // matching AXI4-Stream TVALID/TDATA semantics (data is presented, then
    // consumed on TVALID&&TREADY). rd_en only advances the pointer to the
    // next entry; it does not gate what's currently visible on rd_data.
    always @(posedge rd_clk or negedge rd_rstn) begin
        if (!rd_rstn) begin
            rd_ptr_bin  <= {(ADDR_WIDTH+1){1'b0}};
            rd_ptr_gray <= {(ADDR_WIDTH+1){1'b0}};
        end else if (rd_en && !rd_empty) begin
            rd_ptr_bin  <= rd_ptr_bin + 1'b1;
            rd_ptr_gray <= ((rd_ptr_bin + 1'b1) >> 1) ^ (rd_ptr_bin + 1'b1);
        end
    end
    assign rd_data = mem[rd_ptr_bin[ADDR_WIDTH-1:0]];

    // ---- almost_full: gray->binary AFTER synchronization, safe to convert ----
    function [ADDR_WIDTH:0] gray2bin;
        input [ADDR_WIDTH:0] g;
        integer i;
        begin
            gray2bin[ADDR_WIDTH] = g[ADDR_WIDTH];
            for (i = ADDR_WIDTH-1; i >= 0; i = i - 1)
                gray2bin[i] = gray2bin[i+1] ^ g[i];
        end
    endfunction

    wire [ADDR_WIDTH:0] rd_ptr_bin_synced_to_wr = gray2bin(rd_ptr_gray_wr2);
    wire [ADDR_WIDTH:0] occupancy = wr_ptr_bin - rd_ptr_bin_synced_to_wr;

    assign wr_almost_full = (occupancy >= ALMOST_FULL_THRESH);

endmodule
