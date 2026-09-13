// ------------------------------------------------------------------
// Session-arm handshake: host writes base_addr + (optional TC), then
// pulses session_arm (W1S). FSM latches into shadow regs one cycle
// later and asserts session_valid for the fabric logic to gate on.
// IDLE -> LATCH -> ACTIVE -> IDLE (on session end or re-arm)
// ------------------------------------------------------------------
module session_arm_ctrl #(
    parameter ADDR_WIDTH = 64,
    parameter TC_WIDTH   = 3,
    parameter ARM_CNT_WIDTH = 8   // saturating debug counter width
)(
    input  wire                   clk,
    input  wire                   rst_n,

    input  wire [ADDR_WIDTH-1:0]  base_addr_in,
    input  wire [TC_WIDTH-1:0]    tc_in,
    input  wire                   session_arm,     // W1S pulse from host reg write
    input  wire                   session_end,     // from fabric: TLAST or overflow abort

    output reg  [ADDR_WIDTH-1:0]  base_addr_latched,
    output reg  [TC_WIDTH-1:0]    tc_latched,
    output reg                    session_valid,

    // NEW: debug/bring-up visibility, readable via CfgRd like your other counters
    output reg [ARM_CNT_WIDTH-1:0] session_arm_while_active_cnt
);

    localparam IDLE   = 2'b00;
    localparam LATCH  = 2'b01;
    localparam ACTIVE = 2'b10;

    reg [1:0] state, next_state;

    always @(posedge clk) begin
        if (!rst_n) state <= IDLE;
        else        state <= next_state;
    end

    always @(*) begin
        next_state = state;
        case (state)
            IDLE:   if (session_arm)  next_state = LATCH;
            LATCH:  next_state = ACTIVE;
            ACTIVE: if (session_end)  next_state = IDLE;
                    // re-arm while ACTIVE is now IGNORED here on purpose --
                    // no transition to LATCH. session_end must retire first.
            default: next_state = IDLE;
        endcase
    end

    // Detect the unsafe condition combinationally for the counter
    wire arm_while_active = (state == ACTIVE) && session_arm;

    always @(posedge clk) begin
        if (!rst_n) begin
            base_addr_latched            <= {ADDR_WIDTH{1'b0}};
            tc_latched                    <= {TC_WIDTH{1'b0}};
            session_valid                 <= 1'b0;
            session_arm_while_active_cnt  <= {ARM_CNT_WIDTH{1'b0}};
        end else begin
            if (state == LATCH) begin
                base_addr_latched <= base_addr_in;
                tc_latched         <= tc_in;
            end
            session_valid <= (next_state == ACTIVE);

            // Saturating counter -- never wraps silently
            if (arm_while_active && !(&session_arm_while_active_cnt))
                session_arm_while_active_cnt <= session_arm_while_active_cnt + 1'b1;
        end
    end
endmodule

//------------------------------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------
// axi4s_ingest_port: per-channel AXI4-Stream ingest with TUSER
// assembly and shared-downstream backpressure.
// One instance per channel (2 total, per work_distribution.md).
// ------------------------------------------------------------------
module axi4s_ingest_port #(
    parameter DATA_WIDTH    = 512,  // 64B beat
    parameter STREAM_ID     = 1,    // compile-time per-channel constant
    parameter OFFSET_WIDTH  = 16,
    parameter SLICE_SIZE    = 16'hFFFF  // compile-time bound (open Q#2)
)(
    input  wire                     clk,
    input  wire                     rst_n,

    // From session_arm_ctrl (this channel's instance)
    input  wire                     session_valid,
    output reg                      session_end,      // TLAST-accepted or overflow abort

    // From overflow module (see #3) -- combinational, same cycle
    input  wire                     overflow_abort,

    // Shared downstream backpressure source (from fifo_hwm, OR'd across channels upstream if needed)
    input  wire                     downstream_full,

    // AXI4-Stream slave side (host -> us)
    input  wire [DATA_WIDTH-1:0]    s_tdata,
    input  wire                     s_tvalid,
    input  wire                     s_tlast,
    output wire                     s_tready,

    // AXI4-Stream master side (us -> interconnect arbiter / Module 5)
    output reg  [DATA_WIDTH-1:0]    m_tdata,
    output reg                      m_tvalid,
    output reg                      m_tlast,
    output reg  [0:0]               m_tuser_stream_id,
    output reg  [OFFSET_WIDTH-1:0]  m_tuser_session_offset,

    output reg  [OFFSET_WIDTH-1:0]  session_offset_dbg  // visibility only
);

    reg [OFFSET_WIDTH-1:0] session_offset;

    // Beat only accepted if: session armed, downstream has room, not overflowing
    wire beat_fire = s_tvalid && s_tready;
    assign s_tready = session_valid && !downstream_full && !overflow_abort;

    always @(posedge clk) begin
        if (!rst_n) begin
            session_offset          <= {OFFSET_WIDTH{1'b0}};
            m_tvalid                <= 1'b0;
            m_tlast                 <= 1'b0;
            m_tdata                 <= {DATA_WIDTH{1'b0}};
            m_tuser_stream_id       <= STREAM_ID[0];
            m_tuser_session_offset  <= {OFFSET_WIDTH{1'b0}};
            session_end             <= 1'b0;
            session_offset_dbg      <= {OFFSET_WIDTH{1'b0}};
        end else begin
            // Default: no end pulse unless set below
            session_end <= 1'b0;

            if (!session_valid) begin
                // Between sessions: offset resets, ready for next LATCH
                session_offset <= {OFFSET_WIDTH{1'b0}};
		session_offset_dbg  <= {OFFSET_WIDTH{1'b0}};
                m_tvalid       <= 1'b0;
            end else if (beat_fire) begin
                m_tdata                <= s_tdata;
                m_tvalid               <= 1'b1;
                m_tlast                <= s_tlast;
                m_tuser_stream_id      <= STREAM_ID[0];
                m_tuser_session_offset <= session_offset;

                session_offset <= session_offset + 1'b1;
                session_offset_dbg <= session_offset + 1'b1;

                if (s_tlast)
                    session_end <= 1'b1;   // clean end -> arm FSM allows next arm
            end else begin
                m_tvalid <= 1'b0;
            end

            if (overflow_abort)
                session_end <= 1'b1;       // abort also clears the way for re-arm
        end
    end

endmodule

//------------------------------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------
// session_overflow_ctrl: bounds-checks session_offset against the
// compile-time SLICE_SIZE and raises overflow_abort one cycle before
// the offset would exceed the bound, so the ingest port can stall
// cleanly instead of accepting an out-of-bounds beat.
// One instance per channel, same as axi4s_ingest_port.
// ------------------------------------------------------------------
module session_overflow_ctrl #(
    parameter OFFSET_WIDTH = 16,
    parameter SLICE_SIZE   = 16'hFFFF   // must match the ingest port's SLICE_SIZE
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     session_valid,
    input  wire [OFFSET_WIDTH-1:0]  session_offset,   // current offset, BEFORE this beat
    input  wire                     beat_fire,         // this cycle's beat is being accepted

    output wire                     overflow_abort,    // combinational, feeds ingest port same cycle
    output reg                      session_overflow_sticky,  // W1C, CfgRd-readable
    input  wire                     overflow_flag_clear       // host W1C strobe
);

    // Abort if accepting this beat would push offset to/past the bound.
    // Checked combinationally against the NEXT offset value, so the
    // ingest port can gate s_tready before the out-of-bounds beat is latched.
    wire would_exceed = session_valid && beat_fire &&
                         (session_offset >= (SLICE_SIZE - 1'b1));

    assign overflow_abort = would_exceed;

    always @(posedge clk) begin
        if (!rst_n) begin
            session_overflow_sticky <= 1'b0;
        end else begin
            if (would_exceed)
                session_overflow_sticky <= 1'b1;       // sticky: latches until host clears
            else if (overflow_flag_clear)
                session_overflow_sticky <= 1'b0;        // W1C from host CfgWr
        end
    end

endmodule

//------------------------------------------------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------
// rst_sync: converts a potentially-async internal reset assertion to
// a synchronous, glitch-free active-low reset for a downstream
// module in the SAME clock domain. Async assert, sync deassert --
// standard 2-FF reset synchronizer.
//
// Per Swaray's note: his module needs rst_n held low >= 2 clock
// cycles minimum. This synchronizer's deassertion edge is inherently
// 2 cycles behind async_rst_n's rise, and assertion is immediate
// (async), so as long as OUR internal reset assertion holds for
// at least 1 cycle, his 2-cycle minimum is satisfied by construction.
// ------------------------------------------------------------------
module rst_sync (
    input  wire clk,
    input  wire async_rst_n,     // from your reset controller, async-capable assert
    output reg  sync_rst_n       // clean, synchronous-deassert reset for Swaray's module
);

    reg stage1_rst_n;

    always @(posedge clk or negedge async_rst_n) begin
        if (!async_rst_n) begin
            stage1_rst_n <= 1'b0;
            sync_rst_n   <= 1'b0;
        end else begin
            stage1_rst_n <= 1'b1;
            sync_rst_n   <= stage1_rst_n;
        end
    end

endmodule

//------------------------------------------------------------------------------------------------------------------------

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

//-----------------------------------------------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------------------------------------------------

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

//------------------------------------------------------------------------------------------------------------------------

`timescale 1ns/1ps
module tb_session_arm_ctrl;

    reg clk = 0;
    reg rst_n;
    reg [63:0] base_addr_in;
    reg [2:0]  tc_in;
    reg        session_arm;
    reg        session_end;
    wire [63:0] base_addr_latched;
    wire [2:0]  tc_latched;
    wire        session_valid;
    wire [7:0]  arm_while_active_cnt;

    integer errors = 0;

    session_arm_ctrl #(.ADDR_WIDTH(64), .TC_WIDTH(3), .ARM_CNT_WIDTH(8)) dut (
        .clk(clk), .rst_n(rst_n),
        .base_addr_in(base_addr_in), .tc_in(tc_in),
        .session_arm(session_arm), .session_end(session_end),
        .base_addr_latched(base_addr_latched), .tc_latched(tc_latched),
        .session_valid(session_valid),
        .session_arm_while_active_cnt(arm_while_active_cnt)
    );

    always #5 clk = ~clk;

    // Sample/settle point: wait for the edge, THEN let NBA updates land
    // before any blocking read or stimulus decision. This is the fix --
    // without the #1, testbench reads race the DUT's non-blocking
    // updates at the same time step.
    task tick;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task check(input cond, input [255:0] msg);
        begin
            if (!cond) begin
                $display("FAIL: %s @ t=%0t", msg, $time);
                errors = errors + 1;
            end else begin
                $display("PASS: %s", msg);
            end
        end
    endtask

    initial begin
        rst_n = 0; base_addr_in = 0; tc_in = 0; session_arm = 0; session_end = 0;
        tick; tick;
        rst_n = 1;
        tick;
        check(session_valid == 0, "reset: session_valid low");

        // ---- Case 1: normal arm -> LATCH -> ACTIVE ----
        base_addr_in = 64'hA000_0000;
        tc_in        = 3'b010;
        session_arm  = 1;
        tick;                    // IDLE -> LATCH lands
        session_arm = 0;
        tick;                    // LATCH -> ACTIVE lands, base_addr/tc latch, session_valid rises
        check(session_valid == 1, "case1: session_valid asserted after arm");
        check(base_addr_latched == 64'hA000_0000, "case1: base_addr latched correctly");
        check(tc_latched == 3'b010, "case1: tc latched correctly");

        // ---- Case 2 (THE BUG FIX): re-arm while ACTIVE must be rejected ----
        base_addr_in = 64'hDEAD_BEEF;
        session_arm  = 1;
        tick;
        session_arm = 0;
        tick;
        check(session_valid == 1, "case2: session_valid stays high through rejected re-arm");
        check(base_addr_latched == 64'hA000_0000, "case2: base_addr UNCHANGED (re-arm ignored)");
        check(arm_while_active_cnt == 8'd1, "case2: arm_while_active_cnt incremented exactly once");

        // ---- Case 3: legitimate session_end, then verify re-arm now works ----
        session_end = 1;
        tick;
        session_end = 0;
        tick;
        check(session_valid == 0, "case3: session_valid drops after session_end");

        base_addr_in = 64'hDEAD_BEEF;
        session_arm  = 1;
        tick;
        session_arm = 0;
        tick;
        check(session_valid == 1, "case3: new session_valid after clean re-arm");
        check(base_addr_latched == 64'hDEAD_BEEF, "case3: new base_addr latched after clean re-arm");
        check(arm_while_active_cnt == 8'd1, "case3: counter unchanged by legitimate re-arm");

        // ---- Case 4: multiple rejected re-arms during one ACTIVE window ----
        session_arm = 1; tick; session_arm = 0; tick;
        session_arm = 1; tick; session_arm = 0; tick;
        check(arm_while_active_cnt == 8'd3, "case4: counter accumulates across multiple rejected re-arms");
        check(base_addr_latched == 64'hDEAD_BEEF, "case4: base_addr still unchanged by rejected re-arms");

        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== %0d TEST(S) FAILED ===", errors);

        $stop;
    end

endmodule

//------------------------------------------------------------------------------------------------------------------------

`timescale 1ns/1ps
module tb_axi4s_ingest_port;

    localparam DATA_WIDTH   = 512;
    localparam STREAM_ID    = 1;
    localparam OFFSET_WIDTH = 16;
    localparam SLICE_SIZE   = 16'hFFFF;

    reg clk = 0;
    reg rst_n;

    reg session_valid;
    wire session_end;

    reg overflow_abort;
    reg downstream_full;

    reg  [DATA_WIDTH-1:0] s_tdata;
    reg                   s_tvalid;
    reg                   s_tlast;
    wire                  s_tready;

    wire [DATA_WIDTH-1:0] m_tdata;
    wire                  m_tvalid;
    wire                  m_tlast;
    wire [0:0]            m_tuser_stream_id;
    wire [OFFSET_WIDTH-1:0] m_tuser_session_offset;
    wire [OFFSET_WIDTH-1:0] session_offset_dbg;

    integer errors = 0;

    axi4s_ingest_port #(
        .DATA_WIDTH(DATA_WIDTH), .STREAM_ID(STREAM_ID),
        .OFFSET_WIDTH(OFFSET_WIDTH), .SLICE_SIZE(SLICE_SIZE)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .session_valid(session_valid), .session_end(session_end),
        .overflow_abort(overflow_abort), .downstream_full(downstream_full),
        .s_tdata(s_tdata), .s_tvalid(s_tvalid), .s_tlast(s_tlast), .s_tready(s_tready),
        .m_tdata(m_tdata), .m_tvalid(m_tvalid), .m_tlast(m_tlast),
        .m_tuser_stream_id(m_tuser_stream_id),
        .m_tuser_session_offset(m_tuser_session_offset),
        .session_offset_dbg(session_offset_dbg)
    );

    always #5 clk = ~clk;

    task tick;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task check(input cond, input [255:0] msg);
        begin
            if (!cond) begin
                $display("FAIL: %s @ t=%0t", msg, $time);
                errors = errors + 1;
            end else begin
                $display("PASS: %s", msg);
            end
        end
    endtask

    // Drive one beat and return after it's accepted (or give up after N cycles)
    task send_beat(input [DATA_WIDTH-1:0] data, input tlast);
        integer guard;
        begin
            s_tdata  = data;
            s_tvalid = 1;
            s_tlast  = tlast;
            guard = 0;
            while (!(s_tvalid && s_tready) && guard < 50) begin
                tick;
                guard = guard + 1;
            end
            tick; // consume the accepted cycle
            s_tvalid = 0;
            s_tlast  = 0;
        end
    endtask

    initial begin
        rst_n = 0; session_valid = 0; overflow_abort = 0; downstream_full = 0;
        s_tdata = 0; s_tvalid = 0; s_tlast = 0;
        tick; tick;
        rst_n = 1;
        tick;

        // ---- Case 2: s_tready low when session_valid=0 ----
        s_tvalid = 1; s_tdata = 64'hAAAA;
        tick;
        check(s_tready == 0, "case2: s_tready low when session not valid");
        s_tvalid = 0;
        tick;

        // ---- Case 1: normal streaming, offset increments, TUSER correct ----
        session_valid = 1;
        tick;
        check(session_offset_dbg == 0, "case1: offset starts at 0 on session start");

        send_beat(512'hDEAD_0001, 0);
        check(m_tvalid == 1, "case1: m_tvalid asserted after beat 0 accepted");
        check(m_tdata == 512'hDEAD_0001, "case1: m_tdata matches beat 0");
        check(m_tuser_stream_id == STREAM_ID[0], "case1: stream_id correct");
        check(m_tuser_session_offset == 16'd0, "case1: TUSER offset==0 for beat 0");

        send_beat(512'hDEAD_0002, 0);
        check(m_tuser_session_offset == 16'd1, "case1: TUSER offset==1 for beat 1");
        check(session_offset_dbg == 16'd2, "case1: internal offset counter now 2");

        // ---- Case 3: s_tready drops when downstream_full, mid-session ----
        downstream_full = 1;
        s_tdata = 512'hFEED; s_tvalid = 1; s_tlast = 0;
        tick;
        check(s_tready == 0, "case3: s_tready low when downstream_full asserted");
        downstream_full = 0;
        s_tvalid = 0;
        tick;

        // ---- Case 4: TLAST beat asserts session_end for exactly one cycle ----
        send_beat(512'hFACE_0003, 1'b1);   // TLAST beat
        check(session_end == 1, "case4: session_end asserted same cycle beat accepted w/ TLAST");
        tick;
        check(session_end == 0, "case4: session_end deasserts one cycle later");

        // ---- Case 6: offset resets to 0 when session_valid drops ----
        session_valid = 0;
        tick;
        check(session_offset_dbg == 16'd0, "case6: offset resets when session_valid drops");

        // ---- Case 5: overflow_abort forces session_end, blocks s_tready ----
        session_valid = 1;
        tick;
        s_tdata = 512'hB00; s_tvalid = 1; s_tlast = 0;
        overflow_abort = 1;
        tick;
        check(s_tready == 0, "case5: s_tready blocked by overflow_abort even though session_valid=1");
        check(session_end == 1, "case5: overflow_abort forces session_end");
        overflow_abort = 0;
        s_tvalid = 0;
        tick;
        check(session_end == 0, "case5: session_end clears after overflow_abort deasserts");

        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== %0d TEST(S) FAILED ===", errors);

        $stop;
    end

endmodule

//------------------------------------------------------------------------------------------------------------------------

`timescale 1ns/1ps
module tb_session_overflow_ctrl;

    localparam OFFSET_WIDTH = 16;
    localparam SLICE_SIZE   = 8;   // small override so boundary is reachable fast

    reg clk = 0;
    reg rst_n;

    reg session_valid;
    reg [OFFSET_WIDTH-1:0] session_offset;
    reg beat_fire;
    reg overflow_flag_clear;

    wire overflow_abort;
    wire session_overflow_sticky;

    integer errors = 0;

    session_overflow_ctrl #(
        .OFFSET_WIDTH(OFFSET_WIDTH), .SLICE_SIZE(SLICE_SIZE)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .session_valid(session_valid),
        .session_offset(session_offset),
        .beat_fire(beat_fire),
        .overflow_abort(overflow_abort),
        .session_overflow_sticky(session_overflow_sticky),
        .overflow_flag_clear(overflow_flag_clear)
    );

    always #5 clk = ~clk;

    task tick;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task check(input cond, input [255:0] msg);
        begin
            if (!cond) begin
                $display("FAIL: %s @ t=%0t", msg, $time);
                errors = errors + 1;
            end else begin
                $display("PASS: %s", msg);
            end
        end
    endtask

    initial begin
        rst_n = 0; session_valid = 0; session_offset = 0;
        beat_fire = 0; overflow_flag_clear = 0;
        tick; tick;
        rst_n = 1;
        tick;

        // ---- Case 1: well below bound, no overflow ----
        session_valid  = 1;
        session_offset = 8'd2;   // SLICE_SIZE-1 == 7, well clear
        beat_fire      = 1;
        tick;
        check(overflow_abort == 0, "case1: no overflow well below bound");
        check(session_overflow_sticky == 0, "case1: sticky stays clear");

        // ---- Case 2: offset == SLICE_SIZE-2 (== 6), beat fires -- should NOT abort ----
        session_offset = SLICE_SIZE - 2;  // 6
        beat_fire      = 1;
        tick;
        check(overflow_abort == 0, "case2: offset==SLICE_SIZE-2 does NOT abort (still legal)");
        check(session_overflow_sticky == 0, "case2: sticky still clear");

        // ---- Case 3: offset == SLICE_SIZE-1 (== 7), beat fires -- SHOULD abort ----
        session_offset = SLICE_SIZE - 1;  // 7
        beat_fire      = 1;
        tick;
        check(overflow_abort == 1, "case3: offset==SLICE_SIZE-1 DOES abort (boundary beat)");
        check(session_overflow_sticky == 1, "case3: sticky latches on overflow");

        // ---- Case 4: sticky stays high after triggering condition clears ----
        session_offset = 8'd0;   // back to a "safe" offset
        beat_fire      = 0;      // no beat firing at all now
        tick;
        tick;
        tick;
        check(overflow_abort == 0, "case4: overflow_abort deasserts once condition clears (combinational)");
        check(session_overflow_sticky == 1, "case4: sticky REMAINS latched across subsequent clean cycles");

        // ---- Case 5: host W1C clears the sticky flag ----
        overflow_flag_clear = 1;
        tick;
        overflow_flag_clear = 0;
        tick;
        check(session_overflow_sticky == 0, "case5: sticky cleared by host W1C");

        // ---- Case 6: overflow_abort only fires when session_valid=1 ----
        session_valid  = 0;
        session_offset = SLICE_SIZE - 1;  // boundary condition on offset...
        beat_fire      = 1;                // ...and a beat claims to fire...
        tick;
        check(overflow_abort == 0, "case6: overflow_abort suppressed when session_valid=0");
        check(session_overflow_sticky == 0, "case6: sticky not set when session invalid");
	
	// ---- Case 7: simultaneous would_exceed + overflow_flag_clear same cycle ----
        // Per DUT priority (if would_exceed / else if clear), set wins over clear.
        session_valid        = 1;
        session_offset       = SLICE_SIZE - 1;  // boundary condition, will assert would_exceed
        beat_fire            = 1;
        overflow_flag_clear  = 1;               // clear asserted the SAME cycle
        tick;
        check(overflow_abort == 1, "case7: overflow_abort still fires despite simultaneous clear");
        check(session_overflow_sticky == 1, "case7: sticky SET wins over clear in same cycle (set has priority)");

        // Follow-up: clear on its own, on a later clean cycle, still works normally
        beat_fire            = 0;
        session_offset       = 8'd0;
        overflow_flag_clear  = 1;
        tick;
        overflow_flag_clear  = 0;
        tick;
        check(session_overflow_sticky == 0, "case7b: clear succeeds once would_exceed condition is gone");

        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== %0d TEST(S) FAILED ===", errors);

        $stop;
    end

endmodule

//------------------------------------------------------------------------------------------------------------------------

`timescale 1ns/1ps
module tb_rst_sync;

    reg clk = 0;
    reg async_rst_n;
    wire sync_rst_n;

    integer errors = 0;

    rst_sync dut (
        .clk(clk),
        .async_rst_n(async_rst_n),
        .sync_rst_n(sync_rst_n)
    );

    always #5 clk = ~clk;

    task check(input cond, input [255:0] msg);
        begin
            if (!cond) begin
                $display("FAIL: %s @ t=%0t", msg, $time);
                errors = errors + 1;
            end else begin
                $display("PASS: %s", msg);
            end
        end
    endtask

    initial begin
        async_rst_n = 1;
        @(posedge clk); #1;
        @(posedge clk); #1;

        // ---- Case 1: async assert takes effect immediately, NOT waiting for a clock edge ----
        // Deassert clk-independence: drop async_rst_n mid-cycle, between edges,
        // and check sync_rst_n reacts within delta-time, not on the next posedge.
        #2;                       // land somewhere mid-cycle, away from any edge
        async_rst_n = 0;
        #1;                       // small settle, well before next posedge
        check(sync_rst_n == 0, "case1: sync_rst_n drops immediately on async assert, no clock edge needed");

        // ---- Case 2: deassertion is synchronized -- takes exactly 2 clock edges ----
        // Re-assert async_rst_n (i.e. release reset) at a known point, then count edges.
        @(negedge clk);           // release reset away from a posedge, to make edge-counting unambiguous
        async_rst_n = 1;

        @(posedge clk); #1;
        check(sync_rst_n == 0, "case2: sync_rst_n STILL low after 1st posedge post-release (stage1 only)");

        @(posedge clk); #1;
        check(sync_rst_n == 1, "case2: sync_rst_n goes high after 2nd posedge post-release (stage2 lands)");

        // ---- Case 3: once high, stays high with no further async_rst_n changes ----
        @(posedge clk); #1;
        check(sync_rst_n == 1, "case3: sync_rst_n remains high on subsequent clean cycles");

        // ---- Case 4: re-assert mid-run, confirm immediate drop again (not a one-shot behavior) ----
        @(negedge clk);
        async_rst_n = 0;
        #1;
        check(sync_rst_n == 0, "case4: re-assertion later in sim still drops sync_rst_n immediately");

        // ---- Case 5: minimum hold -- confirm sync_rst_n takes >=2 registered
        //              clock edges to go high after release, regardless of the
        //              phase at which async_rst_n happens to be released ----
        @(negedge clk);
        async_rst_n = 1;   // release
        begin : edge_count_block
            integer edge_count;
            edge_count = 0;
            while (sync_rst_n == 0) begin
                @(posedge clk); #1;
                edge_count = edge_count + 1;   // count every edge waited on, unconditionally
            end
            check(edge_count >= 2, "case5: sync_rst_n takes >=2 clock edges to deassert after release");
        end

        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== %0d TEST(S) FAILED ===", errors);

        $stop;
    end

endmodule

//------------------------------------------------------------------------------------------------------------------------

`timescale 1ns/1ps
module tb_pcie_tag_manager;

    localparam TAG_WIDTH       = 5;
    localparam MAX_OUTSTANDING = 8;

    reg clk = 0;
    reg rst_n;

    reg tag_alloc_req;
    wire tag_alloc_ready;
    wire [TAG_WIDTH-1:0] tag_alloc_val;

    reg tag_free_req;
    reg [TAG_WIDTH-1:0] tag_free_val;

    integer errors = 0;

    pcie_tag_manager #(
        .TAG_WIDTH(TAG_WIDTH), .MAX_OUTSTANDING(MAX_OUTSTANDING)
    ) dut (
        .clk(clk), .rst_n(rst_n),
        .tag_alloc_req(tag_alloc_req), .tag_alloc_ready(tag_alloc_ready), .tag_alloc_val(tag_alloc_val),
        .tag_free_req(tag_free_req), .tag_free_val(tag_free_val)
    );

    always #5 clk = ~clk;

    task tick;
        begin
            @(posedge clk);
            #1;
        end
    endtask

    task check(input cond, input [255:0] msg);
        begin
            if (!cond) begin
                $display("FAIL: %s @ t=%0t", msg, $time);
                errors = errors + 1;
            end else begin
                $display("PASS: %s", msg);
            end
        end
    endtask

    // Perform one allocation: assert req, wait for the granting edge,
    // capture the granted tag, deassert. Checks belong AFTER this task
    // returns, since tag_alloc_val only becomes valid post-edge.
    task do_alloc(output [TAG_WIDTH-1:0] granted_tag);
        begin
            tag_alloc_req = 1;
            tick;                    // granting edge happens here
            granted_tag = tag_alloc_val;
            tag_alloc_req = 0;
            tick;                    // settle
        end
    endtask

    integer i;
    reg [TAG_WIDTH-1:0] got_tag;

    initial begin
        rst_n = 0; tag_alloc_req = 0; tag_free_req = 0; tag_free_val = 0;
        tick; tick;
        rst_n = 1;
        tick;

        // ---- Case 1: pool starts fully free ----
        check(tag_alloc_ready == 1, "case1: alloc_ready high with full free pool");

        // ---- Case 2: allocate in order, expect lowest-free-tag priority ----
        for (i = 0; i < MAX_OUTSTANDING; i = i + 1) begin
            check(tag_alloc_ready == 1, "case2: alloc_ready high before exhaustion");
            do_alloc(got_tag);
            check(got_tag == i[TAG_WIDTH-1:0], "case2: lowest-free-tag priority allocates in order");
        end

        // ---- Case 3: pool exhausted ----
        check(tag_alloc_ready == 0, "case3: alloc_ready low once all tags allocated");

        // ---- Case 4: freeing a middle tag makes exactly it available ----
        tag_free_req = 1;
        tag_free_val = 5'd3;
        tick;
        tag_free_req = 0;
        tick;
        check(tag_alloc_ready == 1, "case4: alloc_ready rises after a single tag is freed");
        do_alloc(got_tag);
        check(got_tag == 5'd3, "case4: freed tag (3) is the one re-allocated");
        check(tag_alloc_ready == 0, "case4: pool exhausted again after re-consuming the freed tag");

        // ---- Case 5: free a different tag, confirm it's the one granted next ----
        tag_free_req = 1;
        tag_free_val = 5'd7;
        tick;
        tag_free_req = 0;
        tick;
        check(tag_alloc_ready == 1, "case5: alloc_ready rises after freeing tag 7");
        do_alloc(got_tag);
        check(got_tag == 5'd7, "case5: newly freed tag 7 available for allocation");

        // ---- Case 6: same-cycle alloc+free of the SAME tag --
        //              freed tag not reusable until the cycle after the free ----
        check(tag_alloc_ready == 0, "case6 setup: pool exhausted before same-cycle alloc+free test");
        tag_free_req  = 1;
        tag_free_val  = 5'd0;
        tag_alloc_req = 1;
        check(tag_alloc_ready == 0, "case6: alloc_ready still 0 THIS cycle despite simultaneous free");
        tick;                         // free lands; this cycle's alloc attempt used PRE-free free_mask, so no grant
        tag_free_req  = 0;
        // tag_alloc_req is still 1 here -- now that free_mask reflects the freed
        // tag, THIS cycle's request should be granted
        check(tag_alloc_ready == 1, "case6: alloc_ready rises the cycle after the free");
        tick;                         // granting edge for the still-asserted request
        check(tag_alloc_val == 5'd0, "case6: tag 0 is granted on the first opportunity after being freed");
        tag_alloc_req = 0;
        tick;

        if (errors == 0)
            $display("\n=== ALL TESTS PASSED ===");
        else
            $display("\n=== %0d TEST(S) FAILED ===", errors);

        $stop;
    end

endmodule

//------------------------------------------------------------------------------------------------------------------------


























