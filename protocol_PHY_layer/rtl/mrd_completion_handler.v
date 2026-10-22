`timescale 1ns/1ps
// ------------------------------------------------------------------
// mrd_completion_handler: consumes Cpl TLPs (from RX/
// deframer path), looks up the outstanding-request entry by tag,
// accumulates received DW count for split-completion reassembly,
// and drives the AXI4 Master read-DATA channel (R*) back to the
// original requester, with correct RID/RLAST/RRESP.
//
// Split-completion handling: a single MRd can legally be satisfied
// by multiple Cpl TLPs (RCB-boundary splits, or completer choice).
// This handler treats every accepted Cpl as "more data for this
// tag's burst," and asserts RLAST only when the outstanding table
// confirms received_dw (after this Cpl) >= total_dw for that tag --
// NOT on any per-TLP flag, since PCIe completions don't carry an
// explicit "this is the last one" bit; it's inferred from byte count.
// ------------------------------------------------------------------
module mrd_completion_handler #(
    parameter ADDR_WIDTH      = 64,
    parameter AXI_ID_WIDTH    = 4,
    parameter TAG_WIDTH       = 5,
    parameter MAX_OUTSTANDING = 8,
    parameter LEN_WIDTH       = 10,
    parameter DATA_WIDTH      = 512   // matches your FLIT/beat width
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // Inbound Cpl TLP (from RX path / deframer)
    input  wire                     cpl_valid,
    output wire                     cpl_ready,
    input  wire [DATA_WIDTH-1:0]    cpl_data,
    input  wire [LEN_WIDTH-1:0]     cpl_length_dw,     // this completion's payload length
    input  wire [TAG_WIDTH-1:0]     cpl_tag,
    input  wire [1:0]               cpl_status,        // 2'b00=SC, 2'b01=UR, 2'b10=CRS, 2'b11=CA (per spec encoding)

    // Outstanding-table interface (shared instance with axi_mrd_generator)
    output wire [TAG_WIDTH-1:0]     ot_rd_tag,
    input  wire [ADDR_WIDTH-1:0]    ot_rd_addr,
    input  wire [LEN_WIDTH-1:0]     ot_rd_total_dw,
    input  wire [LEN_WIDTH-1:0]     ot_rd_received_dw,
    input  wire [AXI_ID_WIDTH-1:0]  ot_rd_axi_id,
    input  wire                     ot_rd_valid,
    input  wire                     ot_rd_is_final_cpl,

    output reg                      ot_upd_en,
    output reg  [TAG_WIDTH-1:0]     ot_upd_tag,
    output reg  [LEN_WIDTH-1:0]     ot_upd_dw_this_cpl,

    output reg                      ot_inv_en,
    output reg  [TAG_WIDTH-1:0]     ot_inv_tag,

    // Tag free-back to pcie_tag_manager, on final completion
    output reg                      tag_free_req,
    output reg  [TAG_WIDTH-1:0]     tag_free_val,

    // Malformed/unexpected-tag debug counter -- a Cpl arriving for a
    // tag with ot_rd_valid==0 means either a completer bug, a
    // duplicate/replayed completion, or a tag-management bug on our
    // side. Per project principle (visibility over silent failure),
    // this MUST be counted, not dropped silently.
    output reg [15:0]               unexpected_cpl_tag_count,

    // AXI4 Master read DATA channel (drives back to requester)
    output reg  [DATA_WIDTH-1:0]    m_rdata,
    output reg  [AXI_ID_WIDTH-1:0]  m_rid,
    output reg  [1:0]               m_rresp,
    output reg                      m_rlast,
    output reg                      m_rvalid,
    input  wire                     m_rready
);

    localparam CPL_SC  = 2'b00;

    // Combinational lookup: point the outstanding table's read port at
    // the tag carried by the CURRENT Cpl being considered.
    assign ot_rd_tag = cpl_tag;

    // Only accept a new Cpl if: the AXI R channel isn't stalled on a
    // previous beat (m_rvalid && !m_rready), AND the tag it references
    // is actually a valid outstanding entry.
    wire tag_is_valid = ot_rd_valid;
    assign cpl_ready = (!m_rvalid || m_rready);

    wire accept_cpl = cpl_valid && cpl_ready;

    always @(posedge clk) begin
        if (!rst_n) begin
            m_rvalid                  <= 1'b0;
            m_rlast                   <= 1'b0;
            ot_upd_en                 <= 1'b0;
            ot_inv_en                 <= 1'b0;
            tag_free_req              <= 1'b0;
            unexpected_cpl_tag_count  <= 16'd0;
        end else begin
            ot_upd_en    <= 1'b0;
            ot_inv_en    <= 1'b0;
            tag_free_req <= 1'b0;

            if (accept_cpl) begin
                if (!tag_is_valid) begin
                    // Unexpected/duplicate/malformed -- count it, do NOT
                    // drive the R channel with garbage, do NOT update
                    // the table (there's nothing valid to update).
                    if (!(&unexpected_cpl_tag_count))
                        unexpected_cpl_tag_count <= unexpected_cpl_tag_count + 1'b1;
                    m_rvalid <= 1'b0;
                end else begin
                    // Legitimate completion: drive R channel this cycle
                    m_rdata  <= cpl_data;
                    m_rid    <= ot_rd_axi_id;
                    m_rresp  <= (cpl_status == CPL_SC) ? 2'b00 : 2'b10; // OKAY vs SLVERR
                    m_rvalid <= 1'b1;
                    m_rlast  <= ot_rd_is_final_cpl;

                    // Update the running received-count for this tag
                    ot_upd_en           <= 1'b1;
                    ot_upd_tag          <= cpl_tag;
                    ot_upd_dw_this_cpl  <= cpl_length_dw;

                    // If this was the final split completion, retire the
                    // table entry and return the tag to the free pool
                    // NEXT cycle (both ot_inv_en and tag_free_req fire
                    // one cycle after ot_upd_en, since ot_rd_is_final_cpl
                    // is computed off the PRE-update received_dw + this
                    // cpl's length -- it's correct THIS cycle, so the
                    // invalidate/free can actually also fire THIS cycle;
                    // no extra delay needed since rd_is_final_cpl already
                    // looks ahead by adding upd_dw_this_cpl combinationally).
                    if (ot_rd_is_final_cpl) begin
                        ot_inv_en    <= 1'b1;
                        ot_inv_tag   <= cpl_tag;
                        tag_free_req <= 1'b1;
                        tag_free_val <= cpl_tag;
                    end
                end
            end else if (m_rvalid && m_rready) begin
                // Previous beat consumed, no new Cpl this cycle
                m_rvalid <= 1'b0;
            end
        end
    end

endmodule
