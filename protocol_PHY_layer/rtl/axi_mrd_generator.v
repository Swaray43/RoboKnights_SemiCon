`timescale 1ns/1ps
// ------------------------------------------------------------------
// axi_mrd_generator: AXI4 Master read-address-channel FSM.
// Consumes AR*, allocates a tag, forms an MRd TLP header, and
// records an outstanding-request entry keyed by tag for the
// completion handler to consume later.
//
// One outstanding-request table entry per tag (not per AXI ID --
// PCIe tag is the completion-matching key per spec, AXI ID is
// carried alongside purely to route the eventual R-channel response
// back to the correct AXI requester).
// ------------------------------------------------------------------
module axi_mrd_generator #(
    parameter ADDR_WIDTH      = 64,
    parameter AXI_ID_WIDTH    = 4,
    parameter TAG_WIDTH       = 5,
    parameter MAX_OUTSTANDING = 8,
    parameter LEN_WIDTH       = 10   // PCIe MRd length field: DW count, max 1024 DW (spec-defined)
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // AXI4 Master read address channel (slave-facing: we ARE the master,
    // this is what we drive OUT to the AXI interconnect / requester side.
    // Naming here reflects OUR role: we receive AR from upstream logic
    // that wants to issue a read).
    input  wire [ADDR_WIDTH-1:0]    s_araddr,
    input  wire [7:0]               s_arlen,      // AXI burst length (beats-1)
    input  wire [2:0]               s_arsize,     // bytes per beat = 2^arsize
    input  wire [AXI_ID_WIDTH-1:0]  s_arid,
    input  wire                     s_arvalid,
    output wire                     s_arready,

    // Tag manager interface
    output reg                      tag_alloc_req,
    input  wire                     tag_alloc_ready,
    input  wire [TAG_WIDTH-1:0]     tag_alloc_val,

    // Outstanding-request table write port (one entry per accepted AR)
    output reg                              ot_wr_en,
    output reg  [TAG_WIDTH-1:0]             ot_wr_tag,
    output reg  [ADDR_WIDTH-1:0]            ot_wr_addr,
    output reg  [LEN_WIDTH-1:0]             ot_wr_total_dw,
    output reg  [AXI_ID_WIDTH-1:0]          ot_wr_axi_id,

    // Outbound MRd TLP header (to TLP packetizer / Module 5)
    output reg                      mrd_valid,
    input  wire                     mrd_ready,
    output reg  [ADDR_WIDTH-1:0]    mrd_addr,
    output reg  [LEN_WIDTH-1:0]     mrd_length_dw,   // TLP length field, in DW
    output reg  [TAG_WIDTH-1:0]     mrd_tag,
    output reg  [1:0]               mrd_fmt,          // 3DW/4DW addr, no-data (fixed for MRd)
    output reg  [4:0]               mrd_type          // MRd type per PCIe base spec
);

    localparam FMT_MRD_4DW = 2'b01;   // 4DW header (64-bit addressing), no data
    localparam TYPE_MRD    = 5'b00000;

    // Beats -> DW conversion: AXI beat size (2^arsize bytes) * (arlen+1) beats,
    // converted to 32-bit DW count for the TLP length field. Assumes
    // arsize yields a byte count that's DW-aligned (true for our
    // 512-bit/64B datapath width; flagging the assumption rather than
    // silently handling misaligned sizes, since misalignment would need
    // byte-enable/first-DW-BE, last-DW-BE fields we are not modeling --
    // that's a real spec corner (sub-DW read alignment) being scoped out
    // explicitly, not silently dropped).
    wire [15:0] beat_bytes   = (16'd1 << s_arsize);
    wire [23:0] total_bytes  = beat_bytes * (s_arlen + 1);
    wire [LEN_WIDTH-1:0] total_dw = total_bytes[LEN_WIDTH+1:2]; // /4, truncated to field width

    localparam IDLE = 1'b0, ISSUE = 1'b1;
    reg state;

    assign s_arready = (state == IDLE) && tag_alloc_ready && mrd_ready;

    always @(posedge clk) begin
        if (!rst_n) begin
            state          <= IDLE;
            tag_alloc_req  <= 1'b0;
            ot_wr_en       <= 1'b0;
            mrd_valid      <= 1'b0;
        end else begin
            tag_alloc_req <= 1'b0;
            ot_wr_en      <= 1'b0;

            case (state)
                IDLE: begin
                    mrd_valid <= 1'b0;
                    if (s_arvalid && s_arready) begin
                        // Accept: allocate tag, latch MRd header, write OT entry
                        tag_alloc_req <= 1'b1;

                        mrd_valid      <= 1'b1;
                        mrd_addr       <= s_araddr;
                        mrd_length_dw  <= total_dw;
                        mrd_tag        <= tag_alloc_val;  // valid same cycle since tag_alloc_ready gated arready
                        mrd_fmt        <= FMT_MRD_4DW;
                        mrd_type       <= TYPE_MRD;

                        ot_wr_en       <= 1'b1;
                        ot_wr_tag      <= tag_alloc_val;
                        ot_wr_addr     <= s_araddr;
                        ot_wr_total_dw <= total_dw;
                        ot_wr_axi_id   <= s_arid;

                        state <= ISSUE;
                    end
                end
                ISSUE: begin
                    if (mrd_valid && mrd_ready) begin
                        mrd_valid <= 1'b0;
                        state     <= IDLE;
                    end
                end
            endcase
        end
    end

endmodule
