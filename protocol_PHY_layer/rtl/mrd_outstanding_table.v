// ------------------------------------------------------------------
// mrd_outstanding_table: tag-indexed storage for in-flight MRd
// requests. Written once by axi_mrd_generator on issue, read/updated
// by mrd_completion_handler as completions arrive. received_dw
// tracks cumulative DW received so far for split-completion
// reassembly; per PCIe base spec, completions for a SINGLE request
// are returned in address order relative to each other (no
// reordering within one tag's completion sequence), so a running
// received_dw compare against total_dw correctly identifies the
// final completion without needing to track per-completion sequence
// numbers.
// ------------------------------------------------------------------
module mrd_outstanding_table #(
    parameter ADDR_WIDTH      = 64,
    parameter AXI_ID_WIDTH    = 4,
    parameter TAG_WIDTH       = 5,
    parameter MAX_OUTSTANDING = 8,
    parameter LEN_WIDTH       = 10
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     wr_en,
    input  wire [TAG_WIDTH-1:0]     wr_tag,
    input  wire [ADDR_WIDTH-1:0]    wr_addr,
    input  wire [LEN_WIDTH-1:0]     wr_total_dw,
    input  wire [AXI_ID_WIDTH-1:0]  wr_axi_id,

    input  wire                     upd_en,
    input  wire [TAG_WIDTH-1:0]     upd_tag,
    input  wire [LEN_WIDTH-1:0]     upd_dw_received_this_cpl,

    // explicit retire, driven by completion handler the cycle
    // after it observes rd_is_final_cpl and frees the tag. Separated
    // from upd_en so accumulate and retire can never be conflated.
    input  wire                     inv_en,
    input  wire [TAG_WIDTH-1:0]     inv_tag,

    input  wire [TAG_WIDTH-1:0]     rd_tag,
    output wire [ADDR_WIDTH-1:0]    rd_addr,
    output wire [LEN_WIDTH-1:0]     rd_total_dw,
    output wire [LEN_WIDTH-1:0]     rd_received_dw,
    output wire [AXI_ID_WIDTH-1:0]  rd_axi_id,
    output wire                     rd_valid,
    output wire                     rd_is_final_cpl
);

    reg [ADDR_WIDTH-1:0]   addr_mem   [0:MAX_OUTSTANDING-1];
    reg [LEN_WIDTH-1:0]    total_mem  [0:MAX_OUTSTANDING-1];
    reg [LEN_WIDTH-1:0]    recvd_mem  [0:MAX_OUTSTANDING-1];
    reg [AXI_ID_WIDTH-1:0] axiid_mem  [0:MAX_OUTSTANDING-1];
    reg                    valid_mem  [0:MAX_OUTSTANDING-1];

    integer k;

    always @(posedge clk) begin
        if (!rst_n) begin
            for (k = 0; k < MAX_OUTSTANDING; k = k + 1) begin
                valid_mem[k] <= 1'b0;
                recvd_mem[k] <= {LEN_WIDTH{1'b0}};
            end
        end else begin
            if (wr_en) begin
                addr_mem[wr_tag]  <= wr_addr;
                total_mem[wr_tag] <= wr_total_dw;
                recvd_mem[wr_tag] <= {LEN_WIDTH{1'b0}};
                axiid_mem[wr_tag] <= wr_axi_id;
                valid_mem[wr_tag] <= 1'b1;
            end
            if (upd_en) begin
                recvd_mem[upd_tag] <= recvd_mem[upd_tag] + upd_dw_received_this_cpl;
            end
            if (inv_en) begin
                valid_mem[inv_tag] <= 1'b0;
            end
        end
    end

    assign rd_addr        = addr_mem[rd_tag];
    assign rd_total_dw    = total_mem[rd_tag];
    assign rd_received_dw = recvd_mem[rd_tag];
    assign rd_axi_id      = axiid_mem[rd_tag];
    assign rd_valid       = valid_mem[rd_tag];

    assign rd_is_final_cpl = valid_mem[rd_tag] &&
        ((recvd_mem[rd_tag] + upd_dw_received_this_cpl) >= total_mem[rd_tag]);

endmodule
