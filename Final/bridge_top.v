//=============================================================================
// bridge_top.v
// First integration point: interconnect_layer (Aditya) -> write_data_buffer
// (Swaray, Module 1 of the link layer), each channel gated by its own
// session_arm_ctrl instance (Kshitij).
//
// NOT YET WIRED: Kshitij's actual config-space register file driving
// base_addr_in / tc_in / session_arm per channel -- those are exposed here
// as top-level ports as placeholders. cfg_base_addr_wr/id/data/clr into
// address_translation.v (further downstream, after the TLP packetizer) are
// a separate, later wiring step and not part of this module.
//
// Clock domains: aclk = AXI4-Stream ingest side (host/DMA-source clock).
// pcie_clk = PCIe-side clock, shared by interconnect_top's CDC-FIFO read
// side, both session_arm_ctrl instances, and write_data_buffer (matches
// write_data_buffer's own doc: "upstream -- from Engineer A CDC output",
// i.e. it expects to sit directly after the CDC, no further CDC needed).
//=============================================================================

module bridge_top #(
    parameter DATA_WIDTH   = 512,
    parameter KEEP_WIDTH   = DATA_WIDTH/8,
    parameter OFFSET_WIDTH = 24,
    parameter ADDR_WIDTH   = 64,
    parameter TC_WIDTH     = 3,
    parameter [OFFSET_WIDTH-1:0] CH0_SESSION_LIMIT = {OFFSET_WIDTH{1'b1}},
    parameter [OFFSET_WIDTH-1:0] CH1_SESSION_LIMIT = {OFFSET_WIDTH{1'b1}},
    parameter FIFO_D = 16,
    parameter HWM    = 12
)(
    input  wire                     aclk,
    input  wire                     arstn,
    input  wire                     pcie_clk,
    input  wire                     pcie_rstn,

    // ---- Channel 0: SmartNIC AXI4-Stream slave in ----
    input  wire [DATA_WIDTH-1:0]    s0_tdata,
    input  wire [KEEP_WIDTH-1:0]    s0_tkeep,
    input  wire                     s0_tlast,
    input  wire                     s0_tvalid,
    output wire                     s0_tready,
    output wire                     ch0_session_overflow,

    // ---- Channel 1: ML AXI4-Stream slave in ----
    input  wire [DATA_WIDTH-1:0]    s1_tdata,
    input  wire [KEEP_WIDTH-1:0]    s1_tkeep,
    input  wire                     s1_tlast,
    input  wire                     s1_tvalid,
    output wire                     s1_tready,
    output wire                     ch1_session_overflow,

    // ---- Arbiter weights ----
    input  wire [7:0]               weight0,
    input  wire [7:0]               weight1,

    // ---- Session-arm handshake, per channel (placeholder until Kshitij's
    //      real config-space register file is wired in) ----
    input  wire [ADDR_WIDTH-1:0]    ch0_base_addr_in,
    input  wire [TC_WIDTH-1:0]      ch0_tc_in,
    input  wire                     ch0_session_arm,
    input  wire [ADDR_WIDTH-1:0]    ch1_base_addr_in,
    input  wire [TC_WIDTH-1:0]      ch1_tc_in,
    input  wire                     ch1_session_arm,

    // ---- Final TLP-packetizer-bound output (from write_data_buffer) ----
    output wire [DATA_WIDTH-1:0]    m_tdata,
    output wire [KEEP_WIDTH-1:0]    m_tkeep,
    output wire                     m_tvalid,
    output wire                     m_tlast,
    output wire [OFFSET_WIDTH+1:0]  m_tuser,
    input  wire                     m_tready,

    output wire                     err_tkeep_gap,
    output wire                     err_burst_interleave,
    output wire [$clog2(FIFO_D):0]  fifo_occupancy,
    output wire                     fifo_full,
    output wire                     fifo_hwm
);

    localparam TUSER_W = OFFSET_WIDTH + 2;

    // ---- per-channel session-arm FSMs (Kshitij) ----
    wire ch0_session_valid, ch1_session_valid;
    wire ch0_session_end,   ch1_session_end;
    wire [ADDR_WIDTH-1:0] ch0_base_addr_latched, ch1_base_addr_latched; // unused downstream of this module for now
    wire [TC_WIDTH-1:0]   ch0_tc_latched,        ch1_tc_latched;        // unused downstream of this module for now

    session_arm_ctrl #(.ADDR_WIDTH(ADDR_WIDTH), .TC_WIDTH(TC_WIDTH)) u_arm_ch0 (
        .clk(pcie_clk), .rst_n(pcie_rstn),
        .base_addr_in(ch0_base_addr_in), .tc_in(ch0_tc_in),
        .session_arm(ch0_session_arm), .session_end(ch0_session_end),
        .base_addr_latched(ch0_base_addr_latched), .tc_latched(ch0_tc_latched),
        .session_valid(ch0_session_valid),
        .session_arm_while_active_cnt()  // unused here; wire up if/when CfgRd exposes it
    );

    session_arm_ctrl #(.ADDR_WIDTH(ADDR_WIDTH), .TC_WIDTH(TC_WIDTH)) u_arm_ch1 (
        .clk(pcie_clk), .rst_n(pcie_rstn),
        .base_addr_in(ch1_base_addr_in), .tc_in(ch1_tc_in),
        .session_arm(ch1_session_arm), .session_end(ch1_session_end),
        .base_addr_latched(ch1_base_addr_latched), .tc_latched(ch1_tc_latched),
        .session_valid(ch1_session_valid),
        .session_arm_while_active_cnt()
    );

    // NOTE: session_arm_ctrl runs on pcie_clk above, but the ingest ports
    // it gates run on aclk inside interconnect_top -- session_valid/
    // session_end cross that clock boundary un-synchronized here. This is
    // the same kind of CDC hazard the interconnect layer's own CDC FIFO
    // was built to handle for the data path; session_valid/session_end are
    // slow, host-timescale control signals (one arm per session, not one
    // per beat) so a straightforward 2-flop synchronizer on each direction
    // is enough -- NOT YET ADDED. Flag to Kshitij before this is simulated
    // across two real, non-matched clocks (today's testbenches tie
    // aclk == pcie_clk with zero skew, which hides this).

    // ---- interconnect layer ----
    wire [DATA_WIDTH-1:0]   ic_tdata;
    wire [KEEP_WIDTH-1:0]   ic_tkeep;
    wire [TUSER_W-1:0]      ic_tuser;
    wire                    ic_tlast, ic_tvalid;
    wire                    ic_tready;

    interconnect_top #(
        .DATA_WIDTH(DATA_WIDTH), .KEEP_WIDTH(KEEP_WIDTH), .OFFSET_WIDTH(OFFSET_WIDTH),
        .CH0_SESSION_LIMIT(CH0_SESSION_LIMIT), .CH1_SESSION_LIMIT(CH1_SESSION_LIMIT)
    ) u_interconnect (
        .aclk(aclk), .arstn(arstn),
        .s0_tdata(s0_tdata), .s0_tkeep(s0_tkeep), .s0_tlast(s0_tlast), .s0_tvalid(s0_tvalid), .s0_tready(s0_tready),
        .ch0_session_valid(ch0_session_valid), .ch0_session_end(ch0_session_end),
        .ch0_session_overflow(ch0_session_overflow),
        .s1_tdata(s1_tdata), .s1_tkeep(s1_tkeep), .s1_tlast(s1_tlast), .s1_tvalid(s1_tvalid), .s1_tready(s1_tready),
        .ch1_session_valid(ch1_session_valid), .ch1_session_end(ch1_session_end),
        .ch1_session_overflow(ch1_session_overflow),
        .weight0(weight0), .weight1(weight1),
        .pcie_clk(pcie_clk), .pcie_rstn(pcie_rstn),
        .m_tdata(ic_tdata), .m_tkeep(ic_tkeep), .m_tuser(ic_tuser), .m_tlast(ic_tlast), .m_tvalid(ic_tvalid),
        .m_tready(ic_tready)
    );

    // ---- Swaray's write_data_buffer (Module 1 of the link layer) ----
    write_data_buffer #(
        .DATA_W(DATA_WIDTH), .KEEP_W(KEEP_WIDTH), .TUSER_W(TUSER_W), .SOFF_W(OFFSET_WIDTH),
        .FIFO_D(FIFO_D), .HWM(HWM)
    ) u_write_data_buffer (
        .clk(pcie_clk), .rst_n(pcie_rstn),
        .s_tdata(ic_tdata), .s_tkeep(ic_tkeep), .s_tvalid(ic_tvalid), .s_tlast(ic_tlast), .s_tuser(ic_tuser),
        .s_tready(ic_tready),
        .m_tdata(m_tdata), .m_tkeep(m_tkeep), .m_tvalid(m_tvalid), .m_tlast(m_tlast), .m_tuser(m_tuser),
        .m_tready(m_tready),
        .err_tkeep_gap(err_tkeep_gap), .err_burst_interleave(err_burst_interleave),
        .fifo_occupancy(fifo_occupancy), .fifo_full(fifo_full), .fifo_hwm(fifo_hwm)
    );

endmodule
