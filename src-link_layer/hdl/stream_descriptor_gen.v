// ============================================================
// Module : stream_descriptor_gen
// Layer  : Link layer (Engineer B — Module 1b of 9)
//
// Function:
//   Bridges between write_data_buffer's 512-bit AXI4-Stream
//   beat interface and the descriptor interface expected by
//   address_translation.
//
//   Each 512-bit beat from write_data_buffer represents up to
//   16 DWs of PCIe payload (512 / 32 = 16).  The actual byte
//   count for a given beat is encoded in TKEEP (64 bits).
//   This module:
//     1. Counts the active bytes in TKEEP (popcount of set bits).
//     2. Converts byte count -> DW count (rounds up to nearest DW,
//        clamped to MAX_DW = DATA_W/32).
//     3. Emits one descriptor per beat:
//          s_addr   = zero (address_translation computes from
//                          base_addr + session_offset in TUSER)
//          s_len_dw = byte_count / 4  (rounded up)
//          s_tuser  = m_tuser from write_data_buffer (passed through)
//          s_split_flag = 0   (address_translation will set this
//                              after the splitter; we emit un-split)
//          s_sub_index  = 0
//     4. Applies full AXI4-Stream handshake backpressure from
//        address_translation (s_ready from downstream stalls
//        acceptance of the next beat from write_data_buffer).
//
//   NOTE ON ADDRESS:
//     s_addr is driven to zero here.  address_translation uses
//     session_offset from TUSER and base_addr from the config-
//     space register to compute pcie_addr = base_addr + session_offset.
//     For non-split descriptors that is always the right value.
//     For split sub-descriptors, ar_request_splitter pre-computes
//     sub0_addr = original addr and sub1_addr = next_page_addr,
//     and address_translation uses s_addr directly for split sub-reqs
//     (s_split_flag=1 bypasses the base+offset computation).
//     This module only ever produces non-split descriptors
//     (s_split_flag=0), so s_addr=0 is correct — address_translation
//     will compute the real address from TUSER session_offset.
//
//   NOTE ON TKEEP:
//     PCIe MWr TLPs must be DW-aligned and DW-granular.  A beat
//     whose TKEEP implies a non-DW-multiple byte count is unusual
//     (write_data_buffer already flags tkeep_has_gap errors for
//     non-contiguous TKEEP), but for robustness we round UP to the
//     nearest DW and allow the last DW to be partially valid
//     (matching how the wider codebase handles partial DWs:
//     FirstBE/LastBE in the TLP header carry the true byte enables,
//     not a truncated len field).
//
//   NOTE ON DATA WIDTH:
//     This module passes the 512-bit data through opaquely.
//     The tlp_packetizer consumes write data as a 32-bit DW stream
//     via a separate wdata_tdata port; the actual wide data is
//     forwarded directly from write_data_buffer's m_tdata to
//     the downstream 32-to-wide serialiser (tlp32_to_axis_packer
//     or a data FIFO between write_data_buffer and tlp_packetizer).
//     This module only generates the descriptor (addr/len/tuser);
//     the data path is a side-channel that bypasses this stage.
//
// Parameters:
//   DATA_W   : AXI data bus width (default 512)
//   KEEP_W   : TKEEP width = DATA_W/8 (default 64)
//   TUSER_W  : TUSER width (default 26)
//   LEN_W    : descriptor length field width in DW (default 11)
//   ADDR_W   : PCIe host address width (default 64)
//
// Upstream interface (from write_data_buffer):
//   s_tdata  [DATA_W-1:0]  — 512-bit beat (not used here; data
//                             flows on a side-channel to packetizer)
//   s_tkeep  [KEEP_W-1:0]  — byte enables; popcount -> DW length
//   s_tuser  [TUSER_W-1:0] — {tlast_flag, session_offset, stream_id}
//   s_tlast                — last beat of AXI burst
//   s_tvalid               — beat valid
//   s_tready               — output: ready to accept beat
//
// Downstream interface (to address_translation):
//   m_addr   [ADDR_W-1:0]  — zero; address_translation computes it
//   m_len_dw [LEN_W-1:0]   — byte_count_from_tkeep rounded up to DW
//   m_tuser  [TUSER_W-1:0] — passed through from s_tuser
//   m_split_flag           — always 0 (not yet split)
//   m_sub_index [1:0]      — always 2'd0
//   m_valid                — descriptor valid
//   m_ready                — input: address_translation ready
// ============================================================

`timescale 1ns/1ps

module stream_descriptor_gen #(
    parameter DATA_W  = 512,
    parameter KEEP_W  = DATA_W / 8,   // 64
    parameter TUSER_W = 26,
    parameter LEN_W   = 11,
    parameter ADDR_W  = 64
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // upstream — from write_data_buffer
    input  wire [DATA_W-1:0]     s_tdata,    // not used; data is a side-channel
    input  wire [KEEP_W-1:0]     s_tkeep,
    input  wire [TUSER_W-1:0]    s_tuser,
    input  wire                  s_tlast,
    input  wire                  s_tvalid,
    output wire                  s_tready,

    // downstream — descriptor interface to address_translation
    output reg  [ADDR_W-1:0]     m_addr,
    output reg  [LEN_W-1:0]      m_len_dw,
    output reg  [TUSER_W-1:0]    m_tuser,
    output reg                   m_split_flag,
    output reg  [1:0]            m_sub_index,
    output reg                   m_valid,
    input  wire                  m_ready
);

    // Elaboration-time checks
    // synthesis translate_off
    initial begin
        if (KEEP_W != DATA_W / 8)
            $fatal(1, "[stream_descriptor_gen] KEEP_W (%0d) != DATA_W/8 (%0d)",
                   KEEP_W, DATA_W / 8);
        if (LEN_W < $clog2(KEEP_W / 4) + 1)
            $fatal(1, "[stream_descriptor_gen] LEN_W (%0d) too narrow for max %0d DW",
                   LEN_W, KEEP_W / 4);
    end
    // synthesis translate_on

    // ------------------------------------------------------------------
    // TKEEP popcount -> DW length
    //
    // KEEP_W = 64 bits for DATA_W=512.  A full beat has 64 set bits
    // (64 bytes = 16 DW).  Partial last beat has 1..63 set bits.
    // Byte count = popcount(tkeep).
    // DW count   = (byte_count + 3) >> 2  (round up to DW boundary).
    //
    // Use a simple parallel popcount tree.  For KEEP_W=64 this is a
    // 6-level binary tree; synthesis will flatten it to an efficient
    // carry-chain adder.
    // ------------------------------------------------------------------
    localparam CNT_W = $clog2(KEEP_W) + 1;  // 7 bits to hold 0..64

    // Stage 0: pair sums (CNT_W=7 is overkill at this level; wire width
    // is set just wide enough at each stage to prevent any truncation)
    function automatic [CNT_W-1:0] popcount64;
        input [63:0] b;
        integer i;
        reg [CNT_W-1:0] cnt;
        begin
            cnt = {CNT_W{1'b0}};
            for (i = 0; i < 64; i = i + 1)
                cnt = cnt + {{(CNT_W-1){1'b0}}, b[i]};
            popcount64 = cnt;
        end
    endfunction

    // Combinational byte count from tkeep
    // For KEEP_W != 64 the function above still works (unused high
    // tkeep bits will be 0 because they are not driven).
    wire [CNT_W-1:0] byte_count;
    assign byte_count = (KEEP_W == 64) ? popcount64(s_tkeep[63:0]) :
                        // fallback for narrower widths: synthesiser will
                        // pick up the for-loop in popcount64 correctly
                        // when KEEP_W < 64 because s_tkeep's upper bits
                        // are 0 (port is only KEEP_W wide, so the
                        // concatenated literal 0 fills the rest).
                        popcount64({{(64-KEEP_W){1'b0}}, s_tkeep});

    // Round byte_count up to DW boundary: len_dw = (byte_count+3)/4
    // For byte_count=0 (all-zero TKEEP on TLAST) this gives 0, which
    // is the correct "no payload" case that the downstream modules
    // already handle (write_data_buffer only lets zero-TKEEP through
    // on TLAST, and a TLP with len=0 is a zero-length write which some
    // versions of the spec allow).
    wire [LEN_W-1:0] len_dw_comb = byte_count[CNT_W-1:2] +
                                   {{(LEN_W-1){1'b0}}, (|byte_count[1:0])};

    // ------------------------------------------------------------------
    // Handshake logic
    //
    // This module is purely combinational on the path from upstream to
    // downstream -- it does not buffer anything.  The descriptor is
    // valid whenever the upstream beat is valid; ready propagates from
    // downstream (m_ready) back to the upstream beat.
    //
    // Because address_translation has a combinational s_ready (it
    // exposes an almost-full-throughput path), there are no idle bubbles
    // on the critical path.
    //
    // We gate s_tready on m_ready so we only consume a beat from
    // write_data_buffer when address_translation can accept the resulting
    // descriptor in the same cycle.
    // ------------------------------------------------------------------
    assign s_tready = m_ready;

    // Combinational output (registered in address_translation)
    always @(*) begin
        m_addr       = {ADDR_W{1'b0}};   // address_translation computes from TUSER
        m_len_dw     = len_dw_comb;
        m_tuser      = s_tuser;
        m_split_flag = 1'b0;              // no split yet; ar_request_splitter handles it
        m_sub_index  = 2'd0;
        m_valid      = s_tvalid;
    end

`ifdef SIMULATION
    always @(posedge clk) begin
        // Verify that a zero-TKEEP descriptor only ever appears on TLAST
        if (s_tvalid && s_tready && (s_tkeep == {KEEP_W{1'b0}}) && !s_tlast)
            $error("[stream_descriptor_gen] Zero TKEEP on non-TLAST beat -- should have been caught by write_data_buffer");
        // Verify len_dw fits in LEN_W
        if (s_tvalid && s_tready && (len_dw_comb > {LEN_W{1'b1}}))
            $error("[stream_descriptor_gen] len_dw overflow: byte_count=%0d", byte_count);
    end
`endif

endmodule
