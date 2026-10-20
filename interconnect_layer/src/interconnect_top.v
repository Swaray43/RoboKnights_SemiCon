//=============================================================================
// interconnect_top.v
// Top-level interconnect layer: two AXI4-Stream ingest ports (SmartNIC,
// ML) -> registered weighted round-robin crossbar -> write-path CDC FIFO
// -> output to Swaray's link layer.
//
// This module implements everything owned by the interconnect layer per
// interconnect_layer_spec.md. It does NOT implement: BAR decode, the
// base_addr/session-armed register file (Kshitij), FLIT framing, real
// address computation, or MSI-X (Swaray). Those are external and this
// module's ports are exactly the contract boundary with them -- see the
// spec doc for the full interface table.
//=============================================================================

module interconnect_top #(
    parameter DATA_WIDTH    = 512,
    parameter KEEP_WIDTH    = DATA_WIDTH/8,
    parameter OFFSET_WIDTH  = 24,
    parameter FIFO_ADDR_WIDTH = 4,      // FIFO depth = 16
    parameter CREDIT_WIDTH  = 8,
    parameter STARVE_CYCLES = 64
)(
    input  wire                     aclk,
    input  wire                     arstn,

    // ---- Channel 0: SmartNIC AXI4-Stream slave in ----
    input  wire [DATA_WIDTH-1:0]    s0_tdata,
    input  wire [KEEP_WIDTH-1:0]    s0_tkeep,
    input  wire                     s0_tlast,
    input  wire                     s0_tvalid,
    output wire                     s0_tready,
    input  wire                     ch0_session_start,   // from Kshitij's config block
    input  wire [OFFSET_WIDTH-1:0]  ch0_session_limit,
    output wire                     ch0_session_active,
    output wire                     ch0_session_overflow,

    // ---- Channel 1: ML AXI4-Stream slave in ----
    input  wire [DATA_WIDTH-1:0]    s1_tdata,
    input  wire [KEEP_WIDTH-1:0]    s1_tkeep,
    input  wire                     s1_tlast,
    input  wire                     s1_tvalid,
    output wire                     s1_tready,
    input  wire                     ch1_session_start,
    input  wire [OFFSET_WIDTH-1:0]  ch1_session_limit,
    output wire                     ch1_session_active,
    output wire                     ch1_session_overflow,

    // ---- Arbiter weights (host-programmable, default 1:1 = 50/50) ----
    input  wire [CREDIT_WIDTH-1:0]  weight0,
    input  wire [CREDIT_WIDTH-1:0]  weight1,

    // ---- PCIe-clock-domain output to Swaray's link layer ----
    // Port shape matches Swaray's stated AXI4-Stream slave contract exactly:
    // s_tdata(512b)/s_tkeep(64b)/s_tvalid/s_tlast/s_tuser/s_tready(from him).
    // TUSER = {tlast_flag, session_offset, stream_id} (OFFSET_WIDTH+2 bits):
    // the link layer's native 26-bit TUSER, emitted here so no remap/shim is
    // needed between the layers (Phase 2 change; ic2link_tuser_remap retired).
    // tlast_flag is a copy of m_tlast; m_tlast is still driven as the normal
    // AXI4-Stream TLAST.
    // session_offset stays per-beat and TLAST stays at true session end --
    // Swaray coalesces into FLITs himself in his own per-stream buffers, so
    // his TUSER-latch-per-burst and interleaving-detector rules apply to
    // HIS downstream hand-off, not to this interface. Confirmed resolved;
    // see ticket_response_and_docs.md PART 1.
    //
    // Ticket G7 (Aditya + Swaray): the layout comment above used to be the
    // ONLY place this order was written down on our side, with the link
    // layer's own files each re-deriving it by hand -- exactly the
    // duplication that caused the Phase 1 misrouting bug ic2link_tuser_remap.v
    // was built to fix. The actual pack below now goes through
    // shared/axis_tuser_defs.vh, the one place the layout is defined;
    // see that file's header for the full writeup. Building this module
    // standalone (outside bridge_top.f, which already carries
    // +incdir+shared) needs `-I <repo_root>/shared` added to the compile.
    input  wire                     pcie_clk,
    input  wire                     pcie_rstn,
    output wire [DATA_WIDTH-1:0]    m_tdata,
    output wire [KEEP_WIDTH-1:0]    m_tkeep,
    output wire [OFFSET_WIDTH+1:0]  m_tuser,            // {tlast_flag, session_offset, stream_id}, width = OFFSET_WIDTH+2
    output wire                     m_tlast,
    output wire                     m_tvalid,
    input  wire                     m_tready
);

    // axis_tuser_defs.vh expects SOFF_W / TUSER_W to already be in scope --
    // alias them onto this module's own OFFSET_WIDTH parameter rather than
    // introducing a second name for the same thing.
    localparam SOFF_W  = OFFSET_WIDTH;
    localparam TUSER_W = OFFSET_WIDTH + 2;         // link-native TUSER width
    `include "axis_tuser_defs.vh"

    // ---- shared downstream backpressure: CDC FIFO almost-full gates both channels ----
    wire fifo_almost_full;

    // ---- channel 0: ingest port ----
    wire req0, grant0, ch0_accepted;
    wire [DATA_WIDTH-1:0]   ch0_data;
    wire [KEEP_WIDTH-1:0]   ch0_keep;
    wire                    ch0_last;
    wire [OFFSET_WIDTH-1:0] ch0_offset;

    axi4s_ingest_port #(
        .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH),
        .OFFSET_WIDTH(OFFSET_WIDTH), .CH_ID(0)
    ) u_ch0 (
        .clk(aclk), .rstn(arstn),
        .s_tdata(s0_tdata), .s_tkeep(s0_tkeep), .s_tlast(s0_tlast), .s_tvalid(s0_tvalid), .s_tready(s0_tready),
        .session_start(ch0_session_start), .session_limit(ch0_session_limit),
        .session_active(ch0_session_active), .session_overflow(ch0_session_overflow),
        .req(req0), .grant(grant0), .ds_backpressure(fifo_almost_full),
        .out_tdata(ch0_data), .out_tkeep(ch0_keep), .out_tlast(ch0_last), .out_session_offset(ch0_offset),
        .accepted(ch0_accepted)
    );

    // ---- channel 1: ingest port ----
    wire req1, grant1, ch1_accepted;
    wire [DATA_WIDTH-1:0]   ch1_data;
    wire [KEEP_WIDTH-1:0]   ch1_keep;
    wire                    ch1_last;
    wire [OFFSET_WIDTH-1:0] ch1_offset;

    axi4s_ingest_port #(
        .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH),
        .OFFSET_WIDTH(OFFSET_WIDTH), .CH_ID(1)
    ) u_ch1 (
        .clk(aclk), .rstn(arstn),
        .s_tdata(s1_tdata), .s_tkeep(s1_tkeep), .s_tlast(s1_tlast), .s_tvalid(s1_tvalid), .s_tready(s1_tready),
        .session_start(ch1_session_start), .session_limit(ch1_session_limit),
        .session_active(ch1_session_active), .session_overflow(ch1_session_overflow),
        .req(req1), .grant(grant1), .ds_backpressure(fifo_almost_full),
        .out_tdata(ch1_data), .out_tkeep(ch1_keep), .out_tlast(ch1_last), .out_session_offset(ch1_offset),
        .accepted(ch1_accepted)
    );

    // ---- arbiter (registered grant, one cycle latency vs req) ----
    rr_arbiter_2ch #(
        .CREDIT_WIDTH(CREDIT_WIDTH), .STARVE_WIDTH(16), .STARVE_CYCLES(STARVE_CYCLES)
    ) u_arb (
        .clk(aclk), .rstn(arstn),
        .req0(req0), .req1(req1),
        .weight0(weight0), .weight1(weight1),
        .grant0(grant0), .grant1(grant1)
    );

    // ---- crossbar mux: mutual exclusion guaranteed by arbiter (tested) ----
    wire [DATA_WIDTH-1:0]   xbar_data  = grant0 ? ch0_data  : ch1_data;
    wire [KEEP_WIDTH-1:0]   xbar_keep  = grant0 ? ch0_keep  : ch1_keep;
    wire                    xbar_last  = grant0 ? ch0_last  : ch1_last;
    wire [OFFSET_WIDTH-1:0] xbar_offset= grant0 ? ch0_offset: ch1_offset;
    wire                    xbar_id    = grant0 ? 1'b0      : 1'b1;
    // Gate on the channel's REAL accepted-beat pulse, not raw grant --
    // grant can be high while ds_backpressure or overflow still blocks the
    // actual AXI4-Stream handshake, in which case the master is legitimately
    // holding the same beat steady and it must not be written again.
    wire                    xbar_valid = (grant0 && ch0_accepted) | (grant1 && ch1_accepted);

    // ---- write-path CDC FIFO ----
    localparam FIFO_WIDTH = DATA_WIDTH + KEEP_WIDTH + 1 /*id*/ + OFFSET_WIDTH + 1 /*tlast*/;

    wire [FIFO_WIDTH-1:0] fifo_wr_data = {xbar_data, xbar_keep, xbar_id, xbar_offset, xbar_last};
    wire fifo_wr_en = xbar_valid;
    wire fifo_full;
    wire [FIFO_WIDTH-1:0] fifo_rd_data;
    wire fifo_empty;

    cdc_fifo_async #(
        .WIDTH(FIFO_WIDTH), .ADDR_WIDTH(FIFO_ADDR_WIDTH), .ALMOST_FULL_PCT(75)
    ) u_fifo (
        .wr_clk(aclk), .wr_rstn(arstn), .wr_en(fifo_wr_en), .wr_data(fifo_wr_data),
        .wr_full(fifo_full), .wr_almost_full(fifo_almost_full),
        .rd_clk(pcie_clk), .rd_rstn(pcie_rstn), .rd_en(m_tready && !fifo_empty), .rd_data(fifo_rd_data),
        .rd_empty(fifo_empty)
    );

    wire m_stream_id_i;
    wire [OFFSET_WIDTH-1:0] m_session_offset_i;
    assign {m_tdata, m_tkeep, m_stream_id_i, m_session_offset_i, m_tlast} = fifo_rd_data;
    assign m_tuser = tuser_pack(m_tlast, m_session_offset_i, m_stream_id_i);   // via shared/axis_tuser_defs.vh (G7)
    assign m_tvalid = !fifo_empty;

endmodule
