// =============================================================================
// msix_controller
//
// CHANGES (Seam D):
//   - New input mwr_ready. mwr_valid now HOLDS (with addr/data stable) until
//     the link layer accepts it with mwr_ready, instead of being a 1-cycle
//     pulse nobody could stall. Tie mwr_ready=1 for the old pulse behaviour.
//   - A completion that arrives while a message is still waiting for
//     mwr_ready is no longer lost: it sets the vector's PBA bit.
//   - Pending (PBA) vectors are now sent once their mask bit is cleared, and
//     the PBA bit is cleared when the message goes out (PCIe MSI-X rule).
//     Before, a masked interrupt stayed pending forever.
//
// Table entries hold {msg_addr[63:0], msg_data[31:0]} per work_distribution.md
// (vector-control/mask is tracked separately below as its own array, since
// the doc's entry description names only address+data - masking is its own
// explicit bullet, not folded into a full 16-byte hardware table entry here).
//
// Table/PBA memory access is modeled as its own simple word interface,
// separate from cfg_rd/cfg_wr above: in real PCIe the MSI-X table and PBA
// live in device MEMORY space (a BAR), reached via MWr/MRd, NOT in config
// space - only the Capability structure pointing at them lives in config
// space. Keeping that split here rather than folding table access into
// cfg_wr/cfg_rd, since collapsing the two would misrepresent the address
// space split the doc's own bullet list draws (separate "Config space
// block" vs MSI-X table/PBA under "MSI-X controller").
// =============================================================================
module msix_controller #(
    parameter NUM_VECTORS = 8   // reduced from the spec's 2048 for simulation size; MAX_VECTORS=2048 is the architectural limit, table depth here is a testbench-scale instance
)(
    input  wire        clk,
    input  wire        rst_n,

    // ---- Host-side table/PBA memory-mapped write access ----
    input  wire                        tbl_wr_en,
    input  wire [$clog2(NUM_VECTORS)-1:0] tbl_wr_vec,
    input  wire [63:0]                 tbl_wr_addr,
    input  wire [31:0]                 tbl_wr_data,
    input  wire                        tbl_wr_mask,       // per-vector mask bit, W alongside addr/data

    // ---- Host-side table/PBA read access ----
    input  wire                        tbl_rd_en,
    input  wire [$clog2(NUM_VECTORS)-1:0] tbl_rd_vec,
    output reg  [63:0]                 tbl_rd_addr,
    output reg  [31:0]                 tbl_rd_data,
    output reg                         tbl_rd_mask,
    output wire [NUM_VECTORS-1:0]      pba,               // Pending Bit Array, read-only to host

    // ---- DMA completion signal from Swaray's TX engine ----
    input  wire                        dma_complete_pulse,
    input  wire [$clog2(NUM_VECTORS)-1:0] dma_complete_vector,

    // ---- 1-DW MWr TLP issued to Swaray for injection (valid/ready) ----
    output reg                         mwr_valid,
    input  wire                        mwr_ready,
    output reg  [63:0]                 mwr_addr,
    output reg  [31:0]                 mwr_data
);

localparam VW = $clog2(NUM_VECTORS);

reg [63:0] table_addr [0:NUM_VECTORS-1];
reg [31:0] table_data [0:NUM_VECTORS-1];
reg        table_mask [0:NUM_VECTORS-1];
reg [NUM_VECTORS-1:0] pba_reg;

assign pba = pba_reg;

// Output slot is free this cycle if nothing is held, or the held message is
// being accepted right now.
wire slot_free = !mwr_valid || mwr_ready;

// Lowest-numbered pending vector whose mask is now clear
reg          drain_hit;
reg [VW-1:0] drain_vec;
integer d;
always @(*) begin
    drain_hit = 1'b0;
    drain_vec = {VW{1'b0}};
    for (d = 0; d < NUM_VECTORS; d = d + 1)
        if (!drain_hit && pba_reg[d] && !table_mask[d]) begin
            drain_hit = 1'b1;
            drain_vec = d[VW-1:0];
        end
end

wire new_fires = dma_complete_pulse && !table_mask[dma_complete_vector] && slot_free;

integer j;
always @(posedge clk) begin
    if (!rst_n) begin
        mwr_valid <= 1'b0;
        mwr_addr  <= 64'h0;
        mwr_data  <= 32'h0;
        pba_reg   <= {NUM_VECTORS{1'b0}};
        for (j = 0; j < NUM_VECTORS; j = j + 1) begin
            table_addr[j] <= 64'h0;
            table_data[j] <= 32'h0;
            table_mask[j] <= 1'b1; // masked by default at reset - safe default, no spurious interrupts before host programs the table
        end
    end else begin
        if (mwr_valid && mwr_ready)
            mwr_valid <= 1'b0;   // accepted; may be refilled below in the same cycle

        // ---- Host writes a table entry (address/data/mask together) ----
        if (tbl_wr_en) begin
            table_addr[tbl_wr_vec] <= tbl_wr_addr;
            table_data[tbl_wr_vec] <= tbl_wr_data;
            table_mask[tbl_wr_vec] <= tbl_wr_mask;
        end

        // ---- New completion: fire now, or pend if masked / output busy ----
        if (dma_complete_pulse) begin
            if (new_fires) begin
                mwr_valid <= 1'b1;
                mwr_addr  <= table_addr[dma_complete_vector];
                mwr_data  <= table_data[dma_complete_vector];
            end else begin
                pba_reg[dma_complete_vector] <= 1'b1;
            end
        end

        // ---- Drain one pending, now-unmasked vector when the slot is free ----
        if (!new_fires && slot_free && drain_hit) begin
            mwr_valid <= 1'b1;
            mwr_addr  <= table_addr[drain_vec];
            mwr_data  <= table_data[drain_vec];
            if (!(dma_complete_pulse && dma_complete_vector == drain_vec))
                pba_reg[drain_vec] <= 1'b0;
        end
    end
end

// ---- Table/PBA read path (combinational lookup, registered output) ----
always @(posedge clk) begin
    if (!rst_n) begin
        tbl_rd_addr <= 64'h0;
        tbl_rd_data <= 32'h0;
        tbl_rd_mask <= 1'b0;
    end else if (tbl_rd_en) begin
        tbl_rd_addr <= table_addr[tbl_rd_vec];
        tbl_rd_data <= table_data[tbl_rd_vec];
        tbl_rd_mask <= table_mask[tbl_rd_vec];
    end
end

endmodule
