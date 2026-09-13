// ============================================================
// Module : stream_tx_path_top
// Layer  : Link layer (Engineer B — Top-level stream TX path)
//
// Function:
//   Wires the complete MWr stream TX chain:
//
//     write_data_buffer
//          |
//          | 512-bit beat + TUSER(stream_id, session_offset, tlast_flag)
//          v
//     stream_descriptor_gen          [NEW -- this session]
//          |
//          | descriptor: {addr=0, len_dw, tuser, split_flag=0, sub_index=0}
//          |   addr=0 because address_translation computes from TUSER
//          v
//     address_translation            [Engineer B -- Module 3]
//          |
//          | descriptor: {pcie_addr, len_dw, stream_id, split_flag, sub_index, tlast_flag}
//          v
//     ar_request_splitter            [Engineer B -- Module 2]
//          |
//          | descriptor(s): up to 2 sub-descriptors if 4KB boundary crossed
//          | (sub0_addr = pcie_addr; sub1_addr = next_page_addr pre-computed by splitter)
//          v
//     tlp_packetizer                 [Engineer B -- Module 5]
//          |
//          | 32-bit TLP DW stream
//          v
//     tlp32_to_axis_packer           [Engineer B -- width adapter]
//          |
//          | 512-bit AXIS beat
//          v
//     data_link_layer_top            [Engineer B -- DLL]
//          |
//          v
//     PHY
//
//   WRITE DATA SIDE CHANNEL:
//     tlp_packetizer needs the raw 32-bit DW payload via its
//     wdata_tdata/wdata_tvalid/wdata_tready port.  write_data_buffer
//     outputs 512-bit beats, so we need a 512-to-32-bit serialiser.
//     write_data_buffer's m_tvalid controls entry into the serialiser;
//     wdata_tready from tlp_packetizer drives the serialiser's output
//     side.  The serialiser holds one 512-bit beat at a time and emits
//     one 32-bit DW per cycle.
//
//   CIRCULAR-DEPENDENCY RESOLUTION:
//     The earlier analysis flagged a potential circularity:
//       - ar_request_splitter needs a translated address to do 4KB math
//       - address_translation was documented as running after the splitter
//     Resolution (verified from the RTL, not guessed):
//       address_translation's header comment says:
//         "For non-split, base_addr is still needed"
//         "For split sub-requests: uses s_addr directly"
//       So address_translation runs FIRST (order: xlat -> split), and
//       for the second sub-descriptor (sub1, at the next page boundary),
//       ar_request_splitter pre-computes sub1_addr = next_page_addr
//       from the already-translated pcie_addr it received for sub0.
//       address_translation recognises s_split_flag=1 and uses s_addr
//       as-is (i.e. the splitter's pre-computed sub1_addr).
//     Therefore the correct order IS: xlat -> splitter (as implemented).
//
//   ASSUMPTION (inherited from ar_request_splitter.v):
//     4KB boundary crossing detection uses the translated PCIe host
//     address.  This is correct as long as base_addr is programmed
//     before any session data starts -- a reasonable operational
//     requirement already implied by address_translation flagging
//     xlat_miss when base_addr is zero.
//
//   TAG MANAGER:
//     The stream path is always MWr (posted), so no tag is needed for
//     the stream path itself.  tag_manager is instantiated here for
//     completeness (the top level connects it) but its allocation port
//     is not asserted for the stream path.  Non-posted tag allocation
//     from the legacy tx_engine path goes through pcie_core_top.v and
//     is unchanged.
//     tag_manager's wdata_tready / cpl_valid ports for completions are
//     wired to the RX completion path from pcie_core_top.
//
//   CREDIT MANAGER:
//     Credit checking for the stream MWr TLPs happens inside tx_engine
//     (which already instantiates credit_manager for the legacy path).
//     The stream path's tlp_packetizer feeds its output through
//     tlp32_to_axis_packer -> data_link_layer_top exactly as the legacy
//     tx_engine does.  If a separate credit check is needed for the
//     stream path, instantiate credit_manager with alloc_req_type=2'b00
//     (Posted/MWr) and wire alloc_req_valid = (tlp_packetizer enters S_HDR0
//     for the first chunk of each descriptor).  That is left as an
//     extension; for now the legacy credit_manager in tx_engine covers
//     all outbound traffic through a shared credit pool.
//
// Parameters — match write_data_buffer and downstream modules:
//   DATA_W         : AXI data width (default 512)
//   KEEP_W         : TKEEP width (default 64)
//   TUSER_W        : TUSER width (default 26)
//   SOFF_W         : session_offset field width (default 24)
//   ADDR_W         : PCIe host address width (default 64)
//   LEN_W          : length field width in DW (default 11)
//   NUM_STREAMS    : number of DMA streams (default 2)
//   MAX_PAYLOAD_DW : tlp_packetizer max payload DW per TLP (default 64)
//   REQ_ID         : PCIe Requester ID for MWr TLPs (default 16'h0200)
//   FIFO_D         : write_data_buffer FIFO depth (default 16)
//   HWM            : write_data_buffer high-watermark (default 12)
//   SYS_BUS_W      : link-layer bus width (default 256)
//   SEQ_WIDTH      : replay-buffer sequence number width (default 12)
//   AXIS_DATA_W    : link-layer AXI stream width (default 512)
// ============================================================

`timescale 1ns/1ps

module stream_tx_path_top #(
    parameter DATA_W         = 512,
    parameter KEEP_W         = DATA_W / 8,
    parameter TUSER_W        = 26,
    parameter SOFF_W         = 24,
    parameter ADDR_W         = 64,
    parameter LEN_W          = 11,
    parameter NUM_STREAMS    = 2,
    parameter MAX_PAYLOAD_DW = 64,
    parameter [15:0] REQ_ID  = 16'h0200,
    parameter FIFO_D         = 16,
    parameter HWM            = 12,
    parameter SYS_BUS_W      = 256,
    parameter SEQ_WIDTH       = 12,
    parameter AXIS_DATA_W    = 512
)(
    input  wire clk,
    input  wire rst_n,

    // -------------------------------------------------------
    // Upstream AXI4-Stream from Engineer A (CDC output)
    // -------------------------------------------------------
    input  wire [DATA_W-1:0]   s_axis_tdata,
    input  wire [KEEP_W-1:0]   s_axis_tkeep,
    input  wire [TUSER_W-1:0]  s_axis_tuser,   // {tlast_flag, session_offset, stream_id}
    input  wire                s_axis_tlast,
    input  wire                s_axis_tvalid,
    output wire                s_axis_tready,

    // -------------------------------------------------------
    // Config-space register bus (from Engineer C)
    // base_addr[stream_id] = IOMMU-translated DMA buffer address
    // -------------------------------------------------------
    input  wire                            cfg_base_addr_wr,
    input  wire [$clog2(NUM_STREAMS)-1:0]  cfg_base_addr_id,
    input  wire [ADDR_W-1:0]              cfg_base_addr_data,
    input  wire                            cfg_base_addr_clr,

    // -------------------------------------------------------
    // PHY TX (to SerDes via data_link_layer_top)
    // -------------------------------------------------------
    output wire                   phy_tx_valid,
    output wire [SYS_BUS_W-1:0]  phy_tx_data,
    output wire                   phy_tx_sof,
    output wire                   phy_tx_eof,

    // -------------------------------------------------------
    // PHY RX (from SerDes — for ACK/NAK replay control)
    // -------------------------------------------------------
    input  wire                   phy_rx_valid,
    input  wire [SYS_BUS_W-1:0]  phy_rx_data,
    input  wire                   phy_rx_sof,
    input  wire                   phy_rx_eof,

    // -------------------------------------------------------
    // Status and error outputs
    // -------------------------------------------------------
    output wire  err_xlat_miss,         // address_translation: base_addr unprogrammed
    output wire  err_addr_ovf,          // address_translation: base+offset overflow
    output wire  stat_split_occurred,   // ar_request_splitter: 4KB split happened
    output wire  err_tkeep_gap,         // write_data_buffer: TKEEP error
    output wire  crc_err,               // link layer CRC error
    output wire  fec_err                // link layer FEC error
);

    // ==================================================================
    // Local wire declarations
    // ==================================================================

    // --- write_data_buffer -> stream_descriptor_gen (data + descriptor) ---
    wire [DATA_W-1:0]  wdb_tdata;
    wire [KEEP_W-1:0]  wdb_tkeep;
    wire [TUSER_W-1:0] wdb_tuser;
    wire               wdb_tlast;
    wire               wdb_tvalid;
    wire               wdb_tready;   // driven by stream_descriptor_gen

    // --- stream_descriptor_gen -> address_translation ---
    wire [ADDR_W-1:0]  sdg_addr;      // always 0 (xlat computes from TUSER)
    wire [LEN_W-1:0]   sdg_len_dw;
    wire [TUSER_W-1:0] sdg_tuser;
    wire               sdg_split_flag; // always 0
    wire [1:0]         sdg_sub_index;  // always 0
    wire               sdg_valid;
    wire               sdg_ready;     // from address_translation.s_ready

    // --- address_translation -> ar_request_splitter ---
    wire [ADDR_W-1:0]  xlat_addr;
    wire [LEN_W-1:0]   xlat_len_dw;
    wire               xlat_stream_id;
    wire               xlat_split_flag;
    wire [1:0]         xlat_sub_index;
    wire               xlat_tlast_flag;
    wire               xlat_valid;
    wire               xlat_ready;    // from ar_request_splitter.s_ready

    // --- ar_request_splitter -> tlp_packetizer descriptor ---
    wire [ADDR_W-1:0]  split_addr;
    wire [LEN_W-1:0]   split_len_dw;
    wire [TUSER_W-1:0] split_tuser;
    wire               split_split_flag;
    wire [1:0]         split_sub_index;
    wire               split_valid;
    wire               split_ready;   // from tlp_packetizer.req_ready

    // --- tlp_packetizer 32-bit TLP DW output ---
    wire               tlp_out_valid;
    wire [31:0]        tlp_out_data;
    wire               tlp_out_sof;
    wire               tlp_out_eof;
    wire               tlp_out_ready;

    // --- Write data side-channel: WDB beat -> 32-bit DW serialiser ---
    // The serialiser converts write_data_buffer's 512-bit beats into the
    // 32-bit DW stream that tlp_packetizer's wdata_* port expects.
    // It is a separate 512->32 shift register; it receives beats from
    // write_data_buffer through a second ready path (wdata_ready).
    wire               wdata_tvalid;
    wire [31:0]        wdata_tdata;
    wire               wdata_tready;  // from tlp_packetizer

    // --- tlp32_to_axis_packer -> data_link_layer_top ---
    wire               pack_tvalid;
    wire [AXIS_DATA_W-1:0] pack_tdata;
    wire               pack_tlast;
    wire               pack_tready;
    wire [1:0]         pack_tready_ch;

    // --- Unused status ---
    wire [$clog2(FIFO_D):0] fifo_occupancy;
    wire fifo_full;
    wire fifo_hwm;
    wire err_burst_interleave_unused;
    wire xlat_busy_unused;
    wire stat_split_nc;

    // ==================================================================
    // 1. write_data_buffer
    //    Accepts 512-bit AXI4-Stream beats from Engineer A's CDC output.
    //    Applies back-pressure, TKEEP validation, and per-channel burst
    //    tracking (Gap fix W1, already applied to the module).
    // ==================================================================
    write_data_buffer #(
        .DATA_W  (DATA_W),
        .KEEP_W  (KEEP_W),
        .TUSER_W (TUSER_W),
        .SOFF_W  (SOFF_W),
        .FIFO_D  (FIFO_D),
        .HWM     (HWM)
    ) u_write_data_buffer (
        .clk                 (clk),
        .rst_n               (rst_n),

        // from Engineer A CDC
        .s_tdata             (s_axis_tdata),
        .s_tkeep             (s_axis_tkeep),
        .s_tvalid            (s_axis_tvalid),
        .s_tlast             (s_axis_tlast),
        .s_tuser             (s_axis_tuser),
        .s_tready            (s_axis_tready),

        // to stream_descriptor_gen and wdata serialiser
        .m_tdata             (wdb_tdata),
        .m_tkeep             (wdb_tkeep),
        .m_tvalid            (wdb_tvalid),
        .m_tlast             (wdb_tlast),
        .m_tuser             (wdb_tuser),
        .m_tready            (wdb_tready),   // see note below *

        // status
        .err_tkeep_gap       (err_tkeep_gap),
        .err_burst_interleave(err_burst_interleave_unused),
        .fifo_occupancy      (fifo_occupancy),
        .fifo_full           (fifo_full),
        .fifo_hwm            (fifo_hwm)
    );

    // * wdb_tready: write_data_buffer has a single m_tready port that
    //   gates the read side of its FIFO.  We need to feed the same beat
    //   to TWO consumers:
    //     (a) stream_descriptor_gen (needs tkeep + tuser for descriptor)
    //     (b) wdata_serialiser (needs tdata for the 32-bit payload stream)
    //
    //   Both consumers must accept the beat in the same cycle.  So
    //   wdb_tready = sdg_ready AND wdata_ser_ready.
    //   stream_descriptor_gen.s_tready = m_ready (from address_translation)
    //   wdata_serialiser is always ready to latch the beat as long as its
    //   internal shift register is empty (see serialiser below).
    //
    //   The actual wdb_tready assignment is at the end of this file
    //   after all sub-module instantiations establish the necessary wires.

    // ==================================================================
    // 2. stream_descriptor_gen
    //    Converts each 512-bit beat into a {addr=0, len_dw, tuser}
    //    descriptor for address_translation.
    // ==================================================================
    stream_descriptor_gen #(
        .DATA_W  (DATA_W),
        .KEEP_W  (KEEP_W),
        .TUSER_W (TUSER_W),
        .LEN_W   (LEN_W),
        .ADDR_W  (ADDR_W)
    ) u_stream_descriptor_gen (
        .clk         (clk),
        .rst_n       (rst_n),

        .s_tdata     (wdb_tdata),
        .s_tkeep     (wdb_tkeep),
        .s_tuser     (wdb_tuser),
        .s_tlast     (wdb_tlast),
        .s_tvalid    (wdb_tvalid),
        .s_tready    (sdg_ready),      // output: driven by address_translation.s_ready

        .m_addr      (sdg_addr),
        .m_len_dw    (sdg_len_dw),
        .m_tuser     (sdg_tuser),
        .m_split_flag(sdg_split_flag),
        .m_sub_index (sdg_sub_index),
        .m_valid     (sdg_valid),
        .m_ready     (xlat_ready)      // input: from address_translation.s_ready
    );

    // Bind wdb_tready: consume the beat from write_data_buffer only when
    // BOTH the descriptor side (sdg_ready = xlat_ready) AND the data
    // serialiser side (wdata_ser_latch_ready) are ready.
    // wdata_ser_latch_ready is declared after the serialiser below.
    // We use a wire so both assignments resolve to the same combinational
    // expression; see the assign at the bottom.

    // ==================================================================
    // 3. address_translation
    //    Computes pcie_addr = base_addr[stream_id] + session_offset.
    //    For split sub-descriptors (s_split_flag=1) uses s_addr directly.
    // ==================================================================
    address_translation #(
        .ADDR_W      (ADDR_W),
        .LEN_W       (LEN_W),
        .TUSER_W     (TUSER_W),
        .SOFF_W      (SOFF_W),
        .NUM_STREAMS (NUM_STREAMS)
    ) u_address_translation (
        .clk               (clk),
        .rst_n             (rst_n),

        // from stream_descriptor_gen
        .s_addr            (sdg_addr),
        .s_len_dw          (sdg_len_dw),
        .s_tuser           (sdg_tuser),
        .s_split_flag      (sdg_split_flag),
        .s_sub_index       (sdg_sub_index),
        .s_valid           (sdg_valid),
        .s_ready           (xlat_ready),      // output: back to stream_descriptor_gen

        // to ar_request_splitter
        .m_pcie_addr       (xlat_addr),
        .m_len_dw          (xlat_len_dw),
        .m_stream_id       (xlat_stream_id),
        .m_split_flag      (xlat_split_flag),
        .m_sub_index       (xlat_sub_index),
        .m_tlast_flag      (xlat_tlast_flag),
        .m_valid           (xlat_valid),
        .m_ready           (xlat_ready_from_splitter),

        // config space registers (base_addr per stream)
        .cfg_base_addr_wr   (cfg_base_addr_wr),
        .cfg_base_addr_id   (cfg_base_addr_id),
        .cfg_base_addr_data (cfg_base_addr_data),
        .cfg_base_addr_clr  (cfg_base_addr_clr),

        // status
        .err_xlat_miss     (err_xlat_miss),
        .err_addr_ovf      (err_addr_ovf),
        .xlat_busy         (xlat_busy_unused)
    );

    // Reconstruct TUSER for ar_request_splitter.
    // address_translation emits {m_tlast_flag, m_stream_id} individually.
    // ar_request_splitter and everything downstream (link_layer_tx) only
    // inspect bit[0]=stream_id and bit[TUSER_W-1]=tlast_flag; session_offset
    // in TUSER is not used past address_translation (the host address is
    // fully resolved in pcie_addr/split_addr).  Zero session_offset.

    // TUSER reconstruction post address_translation.
    // address_translation decomposes TUSER into {tlast_flag, session_offset, stream_id}
    // and does not re-emit session_offset as a separate output (it was consumed to
    // compute pcie_addr).  Downstream of this point (ar_request_splitter ->
    // tlp_packetizer -> link_layer_tx), TUSER is only inspected for:
    //   bit [0]          = stream_id  (link_layer_tx per-channel routing)
    //   bit [TUSER_W-1]  = tlast_flag (marks session boundary)
    // session_offset is no longer needed for any computation (the PCIe host address
    // is now fully resolved in xlat_addr/split_addr and will be placed in the
    // TLP header by tlp_packetizer).  Safe to zero it out.
    wire [TUSER_W-1:0] xlat_tuser_reconstructed;
    assign xlat_tuser_reconstructed = {
        xlat_tlast_flag,            // bit [TUSER_W-1] = tlast_flag
        {(SOFF_W){1'b0}},          // bits [TUSER_W-2:1] = session_offset (zeroed -- not used downstream)
        xlat_stream_id              // bit [0] = stream_id
    };

    // Ready signal from ar_request_splitter back to address_translation
    wire xlat_ready_from_splitter;

    // ==================================================================
    // 4. ar_request_splitter
    //    Detects 4KB page crossings in the translated PCIe address and
    //    splits the descriptor into two sub-descriptors when needed.
    //    sub0 keeps the original addr/len; sub1 gets next_page_addr and
    //    the remainder length.
    //    For split sub1, address_translation will use s_addr directly
    //    (s_split_flag=1 bypass path) — which works because ar_request_splitter
    //    drives sub1_addr = next_page_addr, an already-translated address.
    // ==================================================================

    // Note: ar_request_splitter's output goes to tlp_packetizer's descriptor
    // port (req_*). The split_ready is driven by tlp_packetizer.req_ready.
    // ar_request_splitter also re-emits split sub-descriptors back THROUGH
    // address_translation when split_flag=1.  In this topology, because
    // address_translation runs before the splitter, we do NOT feed the
    // splitter's sub-descriptor output back through address_translation.
    // The splitter pre-computes both sub-addresses from the already-translated
    // input address, so both sub0 and sub1 are already PCIe host addresses.
    // The splitter output goes DIRECTLY to tlp_packetizer.

    ar_request_splitter #(
        .ADDR_W  (ADDR_W),
        .LEN_W   (LEN_W),
        .TUSER_W (TUSER_W),
        .SOFF_W  (SOFF_W)
    ) u_ar_request_splitter (
        .clk                (clk),
        .rst_n              (rst_n),

        // from address_translation
        .s_addr             (xlat_addr),
        .s_len_dw           (xlat_len_dw),
        .s_tuser            (xlat_tuser_reconstructed),
        .s_valid            (xlat_valid),
        .s_ready            (xlat_ready_from_splitter),

        // to tlp_packetizer descriptor interface
        .m_addr             (split_addr),
        .m_len_dw           (split_len_dw),
        .m_tuser            (split_tuser),
        .m_split_flag       (split_split_flag),
        .m_sub_index        (split_sub_index),
        .m_valid            (split_valid),
        .m_ready            (split_ready),

        .stat_split_occurred(stat_split_occurred)
    );

    // ==================================================================
    // 5. Write data serialiser: 512-bit -> 32-bit DW stream
    //
    //    tlp_packetizer's wdata_tdata port is 32 bits.  write_data_buffer
    //    outputs 512-bit beats.  This inline serialiser converts them.
    //
    //    Operation:
    //      - Latches a 512-bit beat from write_data_buffer when wdb_tvalid
    //        and wdata_ser_latch_ready (serialiser is idle / done with
    //        previous beat).
    //      - Emits 16 DW (one per cycle) to wdata_tdata/wdata_tvalid.
    //      - wdata_tready from tlp_packetizer advances the DW counter.
    //      - After 16 DW (or when the full beat is consumed, using the
    //        len_dw from the descriptor to know when to stop), the
    //        serialiser is ready to latch the next beat.
    //
    //    IMPORTANT: the descriptor side (stream_descriptor_gen -> xlat ->
    //    splitter -> packetizer) and the data side (this serialiser) must
    //    stay in lock-step.  write_data_buffer has a single m_tready port,
    //    so both sides consume from the same beat at the same time:
    //      wdb_tready = xlat_ready (descriptor side) AND wdata_ser_latch_ready
    //
    //    The serialiser only latches a new beat when wdb_tready fires
    //    (same cycle the FIFO read happens), so the beat that enters the
    //    descriptor pipeline is the same beat whose data enters the
    //    serialiser.  A beat that generates a split descriptor generates
    //    TWO TLPs from the same payload; the serialiser must emit the
    //    full set of DWs for both sub-TLPs.  ar_request_splitter marks
    //    sub0 with sub_index=0 and len_dw = bytes_before_boundary/4;
    //    sub1 with sub_index=1 and the remainder.  tlp_packetizer
    //    requests exactly chunk_len DWs from wdata_tready for each TLP.
    //    Because the serialiser emits DWs in order, sub0 DWs come first
    //    and sub1 DWs follow, which is exactly what two consecutive
    //    tlp_packetizer TLP transactions consume.
    //
    //    ASSUMPTION: for split TLPs, tlp_packetizer requests sub0 DWs
    //    first (because ar_request_splitter emits sub0 then sub1 in order
    //    and tlp_packetizer accepts them in FIFO order).  This means the
    //    serialiser does NOT need to know about the split -- it just emits
    //    DWs in order and tlp_packetizer's S_DATA state naturally consumes
    //    the right DWs for each sub-TLP.
    // ==================================================================

    localparam DWS_PER_BEAT = DATA_W / 32;    // = 16 for 512-bit
    localparam DW_CNT_W     = $clog2(DWS_PER_BEAT) + 1; // 5 bits to hold 0..16

    reg [DATA_W-1:0]    wdata_shift_reg;
    reg [DW_CNT_W-1:0]  wdata_dw_cnt;       // DWs remaining to emit for current beat
    reg                 wdata_ser_busy;      // 1 = serialiser holds a beat being drained

    wire wdata_ser_latch_ready = !wdata_ser_busy;

    assign wdata_tvalid = wdata_ser_busy;
    assign wdata_tdata  = wdata_shift_reg[31:0];  // always emit lowest 32 bits

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wdata_shift_reg  <= {DATA_W{1'b0}};
            wdata_dw_cnt     <= {DW_CNT_W{1'b0}};
            wdata_ser_busy   <= 1'b0;
        end else begin
            if (wdata_ser_busy) begin
                if (wdata_tready) begin
                    // Advance to next DW
                    wdata_shift_reg <= wdata_shift_reg >> 32;
                    if (wdata_dw_cnt <= {{(DW_CNT_W-1){1'b0}}, 1'b1}) begin
                        // Last DW of this beat consumed
                        wdata_ser_busy <= 1'b0;
                        wdata_dw_cnt   <= {DW_CNT_W{1'b0}};
                    end else begin
                        wdata_dw_cnt <= wdata_dw_cnt - 1'b1;
                    end
                end
            end else begin
                // Latch a new beat when write_data_buffer is read
                // (wdb_tready fires = wdb_tvalid && xlat_ready && wdata_ser_latch_ready)
                if (wdb_tvalid && wdb_tready) begin
                    wdata_shift_reg <= wdb_tdata;
                    // Number of DWs = popcount(tkeep)/4, same as sdg_len_dw,
                    // but popcount is expensive to replicate here.  Instead,
                    // always drain all DWS_PER_BEAT DWs per beat (16 for 512-bit).
                    // tlp_packetizer consumes exactly the right count via
                    // its data_cnt mechanism; excess DWs after the last
                    // meaningful one are silently consumed (tlp_packetizer
                    // stops asserting wdata_tready after chunk_len DWs).
                    // Those excess DWs go nowhere because wdata_tvalid has
                    // no effect when wdata_tready is low.
                    wdata_dw_cnt   <= DWS_PER_BEAT[DW_CNT_W-1:0] - 1'b1; // 15 for 512-bit
                    wdata_ser_busy <= 1'b1;
                end
            end
        end
    end

    // ==================================================================
    // Combined wdb_tready: consume a beat only when BOTH the descriptor
    // side (xlat_ready, back-propagated from address_translation) AND
    // the data serialiser (wdata_ser_latch_ready) are ready.
    // ==================================================================
    assign wdb_tready = xlat_ready && wdata_ser_latch_ready;

    // Note: sdg_ready is also xlat_ready (stream_descriptor_gen just
    // passes m_ready -> s_tready).  wdb_tvalid flows to sdg_valid -> xlat
    // only when wdb_tready is asserted, which requires both sides ready.
    // This guarantees no descriptor is emitted without the corresponding
    // data beat being latched into the serialiser in the same cycle.

    // ==================================================================
    // 6. tlp_packetizer
    //    Builds 4DW MWr TLP headers and concatenates with write data
    //    from the serialiser.  Emits a 32-bit DW stream.
    // ==================================================================

    // Reconstruct the tlp_packetizer descriptor interface from splitter output.
    // split_tuser[0] = stream_id; split_tuser[TUSER_W-1] = tlast_flag.
    // tlp_packetizer uses req_rid (Requester ID) and req_tag for the TLP header.
    // For MWr (posted) requests, tag is irrelevant (PCIe spec: Posted TLPs
    // do not use tags), but we must provide a valid value.  Wire to zero.
    // req_is_write = 1 always for the stream path.

    tlp_packetizer #(
        .MAX_PAYLOAD_DW(MAX_PAYLOAD_DW)
    ) u_tlp_packetizer (
        .clk          (clk),
        .rst_n        (rst_n),

        // descriptor from ar_request_splitter
        .req_valid    (split_valid),
        .req_is_write (1'b1),               // always MWr for stream path
        .req_addr     (split_addr),
        .req_length   (split_len_dw[9:0]),  // packetizer port is 10-bit; LEN_W=11 but max=1023 DW per TLP
        .req_rid      (REQ_ID),
        .req_tag      (8'h00),              // Posted TLPs don't use tags
        .req_ready    (split_ready),        // output: back to ar_request_splitter

        // write data from serialiser
        .wdata_tdata  (wdata_tdata),
        .wdata_tvalid (wdata_tvalid),
        .wdata_tready (wdata_tready),

        // 32-bit TLP DW output
        .tlp_out_valid(tlp_out_valid),
        .tlp_out_data (tlp_out_data),
        .tlp_out_sof  (tlp_out_sof),
        .tlp_out_eof  (tlp_out_eof),
        .tlp_out_ready(tlp_out_ready)
    );

    // ==================================================================
    // 7. tlp32_to_axis_packer
    //    Accumulates the 32-bit TLP DW stream into 512-bit AXIS beats
    //    for data_link_layer_top.
    // ==================================================================
    tlp32_to_axis_packer #(
        .DW_WIDTH   (32),
        .AXIS_WIDTH (AXIS_DATA_W)
    ) u_tlp32_to_axis_packer (
        .clk      (clk),
        .rst_n    (rst_n),

        // narrow input from tlp_packetizer
        .s_tvalid (tlp_out_valid),
        .s_tdata  (tlp_out_data),
        .s_tlast  (tlp_out_eof),
        .s_tready (tlp_out_ready),

        // wide output to data_link_layer_top
        .m_tvalid (pack_tvalid),
        .m_tdata  (pack_tdata),
        .m_tlast  (pack_tlast),
        .m_tready (pack_tready)
    );

    // ==================================================================
    // 8. data_link_layer_top
    //    Reliability (replay buffer), FLIT framing, CRC/FEC, and PHY
    //    interface.  TUSER carries stream_id for per-channel FLIT assembly
    //    in link_layer_tx; the stream path reconstructs it from the
    //    first beat's stream_id.
    //
    //    NOTE: data_link_layer_top's s_tx_axis_tuser needs to carry
    //    stream_id for the per-channel arbitration in link_layer_tx.
    //    We thread it through as the low bit of a TUSER_W-wide signal.
    //    The other bits (session_offset, tlast_flag) are not meaningful
    //    at this stage (they were consumed upstream); we carry stream_id
    //    from the TUSER of the last-accepted split descriptor.
    //
    //    For stream routing, what matters is bit [0] (stream_id).  We
    //    reconstruct from split_tuser: the accepted descriptor's TUSER
    //    has stream_id in bit [0] by the write_data_buffer / address_translation
    //    contract.  Because tlp32_to_axis_packer may span multiple TLPs
    //    into one 512-bit beat, and multiple descriptors may belong to
    //    different channels, we use a registered "current_stream_id" that
    //    captures stream_id at the moment each new TLP starts (tlp_out_sof).
    //    This is consistent with how link_layer_tx expects stream_id:
    //    per-beat, as of the first DW of the TLP's packed beat.
    //
    //    Since the packer collects up to 16 DWs (from possibly multiple
    //    split sub-TLPs of the same original beat, which always have the
    //    same stream_id), the stream_id is stable for the duration of
    //    one packed beat.  For simplicity we hold the stream_id of the
    //    first TLP in the packed beat.
    // ==================================================================

    reg  cur_stream_id;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)
            cur_stream_id <= 1'b0;
        else if (split_valid && split_ready)
            // Capture stream_id when a new descriptor is accepted
            // into tlp_packetizer.
            cur_stream_id <= split_tuser[0];
    end

    // Build a full TUSER_W-wide word for data_link_layer_top.
    // Only stream_id (bit 0) matters downstream (link_layer_tx routing).
    wire [TUSER_W-1:0] dll_tuser;
    assign dll_tuser = {{(TUSER_W-1){1'b0}}, cur_stream_id};

    // pack_tready: combined ready from data_link_layer_top.
    // s_tx_axis_tready from DLL is the combined ready.
    wire dll_tready;
    wire [1:0] dll_tready_ch;
    assign pack_tready = dll_tready;

    // Unused RX stream from DLL (stream_tx_path_top is TX-only; the RX
    // path is handled by pcie_core_top / link_layer_rx separately).
    wire dll_rx_tvalid;
    wire [AXIS_DATA_W-1:0] dll_rx_tdata;
    wire dll_rx_tlast;

    data_link_layer_top #(
        .AXIS_DATA_W(AXIS_DATA_W),
        .SYS_BUS_W  (SYS_BUS_W),
        .SEQ_WIDTH  (SEQ_WIDTH),
        .TUSER_W    (TUSER_W)
    ) u_data_link_layer_top (
        .clk                (clk),
        .rst_n              (rst_n),

        // TX: packed wide beats from tlp32_to_axis_packer
        .s_tx_axis_tvalid   (pack_tvalid),
        .s_tx_axis_tdata    (pack_tdata),
        .s_tx_axis_tlast    (pack_tlast),
        .s_tx_axis_tuser    (dll_tuser),
        .s_tx_axis_tready   (dll_tready),
        .s_tx_axis_tready_ch(dll_tready_ch),

        // RX: not used in this TX-only wrapper; loopback to /dev/null
        .m_rx_axis_tvalid   (dll_rx_tvalid),
        .m_rx_axis_tdata    (dll_rx_tdata),
        .m_rx_axis_tlast    (dll_rx_tlast),
        .m_rx_axis_tready   (1'b1),            // always accept (TX-only top)

        // PHY
        .m_phy_tx_valid     (phy_tx_valid),
        .m_phy_tx_data      (phy_tx_data),
        .m_phy_tx_sof       (phy_tx_sof),
        .m_phy_tx_eof       (phy_tx_eof),

        .s_phy_rx_valid     (phy_rx_valid),
        .s_phy_rx_data      (phy_rx_data),
        .s_phy_rx_sof       (phy_rx_sof),
        .s_phy_rx_eof       (phy_rx_eof),

        .crc_err            (crc_err),
        .fec_err            (fec_err)
    );

`ifdef SIMULATION
    // Verify that the descriptor and data sides never de-sync
    // (both are gated by the same wdb_tready, so they should always
    // latch the same beat in the same cycle).
    reg wdata_latched_this_cycle;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            wdata_latched_this_cycle <= 1'b0;
        end else begin
            // On a wdb_tready pulse, both the descriptor (sdg_valid fires)
            // and the data serialiser (wdata_ser_busy goes high) should
            // advance in the same cycle.
            if (wdb_tvalid && wdb_tready) begin
                if (!wdata_ser_latch_ready)
                    $error("[stream_tx_path_top] wdb_tready fired but wdata serialiser was busy -- de-sync between descriptor and data paths");
            end
        end
    end
`endif

endmodule
