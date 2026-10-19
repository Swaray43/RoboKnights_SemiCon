// ------------------------------------------------------------------
// pcie_tag_manager: allocates/frees PCIe tags for outstanding MRd
// requests. Free-list based, supports MAX_OUTSTANDING concurrent
// reads. Tag width per PCIe base spec is 8 bits (or 5 if extended
// tag not negotiated) -- parameterized here, default 5-bit/32-tag
// since extended-tag negotiation is a config-space capability we're
// not modeling (config space is already a documented deferral).
// ------------------------------------------------------------------
module pcie_tag_manager #(
    parameter TAG_WIDTH       = 5,
    parameter MAX_OUTSTANDING = 8    // <= 2**TAG_WIDTH
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // Allocation request (from axi_mrd_generator, one per accepted AR)
    input  wire                   tag_alloc_req,
    output wire                   tag_alloc_ready,   // low if pool exhausted
    output reg  [TAG_WIDTH-1:0]   tag_alloc_val,

    // Free (from mrd_completion_handler, on final split completion)
    input  wire                   tag_free_req,
    input  wire [TAG_WIDTH-1:0]   tag_free_val,

    output wire [$clog2(MAX_OUTSTANDING+1)-1:0] tags_in_use
);

    // Free-list as a bit-vector: 1 = free, 0 = in-use
    reg [MAX_OUTSTANDING-1:0] free_mask;

    integer i;

    // Priority-encode lowest free tag
    reg [TAG_WIDTH-1:0] next_free_tag;
    reg                 any_free;

    always @(*) begin
        next_free_tag = {TAG_WIDTH{1'b0}};
        any_free       = 1'b0;
        for (i = 0; i < MAX_OUTSTANDING; i = i + 1) begin
            if (!any_free && free_mask[i]) begin
                next_free_tag = i[TAG_WIDTH-1:0];
                any_free       = 1'b1;
            end
        end
    end
    integer p;
    reg [$clog2(MAX_OUTSTANDING+1)-1:0] in_use_count;
    always @(*) begin
        in_use_count = 0;
        for (p = 0; p < MAX_OUTSTANDING; p = p + 1)
            if (!free_mask[p]) in_use_count = in_use_count + 1;
    end
    assign tags_in_use = in_use_count;
    assign tag_alloc_ready = any_free;

    always @(posedge clk) begin
        if (!rst_n) begin
            free_mask   <= {MAX_OUTSTANDING{1'b1}};  // all free at reset
            tag_alloc_val <= {TAG_WIDTH{1'b0}};
        end else begin
            // Allocate: clear the bit for the granted tag
            if (tag_alloc_req && tag_alloc_ready) begin
                free_mask[next_free_tag] <= 1'b0;
                tag_alloc_val            <= next_free_tag;
            end
            // Free: set the bit back. Independent of alloc in the same
            // cycle -- alloc and free can coincide (one tag frees while
            // another allocates) without conflict since they're
            // different bit indices in the common case.
            if (tag_free_req) begin
                free_mask[tag_free_val] <= 1'b1;
            end
        end
    end

    // Simultaneous alloc+free of the SAME tag in the same cycle is not
    // a real hazard here (free sets a bit that alloc's combinational
    // priority-encode already latched from BEFORE this cycle's free),
    // but flagging it: if tag_free_val == next_free_tag on the same
    // cycle as an alloc, the freed tag becomes eligible next cycle,
    // not this one -- alloc always uses the pre-existing free_mask.
    // This is correct (no reuse-before-truly-free), just noting it's
    // not a coincidence, it's a deliberate consequence of registered
    // free_mask updates.

endmodule
