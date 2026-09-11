`timescale 1ns/1ps

module tb_pcie_rx_engine;

    parameter TAG_WIDTH      = 10;
    parameter MAX_DW_PER_REQ = 16;
    parameter PCIE_ADDR_W    = 64;
    parameter AXI_ADDR_W     = 32;

    reg                     clk;
    reg                     rst_n;

    reg                     tlp_rx_valid;
    reg                     tlp_rx_sof;
    reg                     tlp_rx_eof;
    reg [31:0]              tlp_rx_data;
    reg [4:0]               tlp_type;
    reg [9:0]               tlp_length_dw;
    reg [TAG_WIDTH-1:0]     tlp_tag;
    reg [6:0]               tlp_lower_addr;
    reg [2:0]               tlp_cpl_status;
    reg [PCIE_ADDR_W-1:0]   tlp_addr;

    reg                     req_alloc_valid;
    reg [TAG_WIDTH-1:0]     req_alloc_tag;
    reg [9:0]               req_alloc_len;
    wire                    req_alloc_ready;

    wire                    m_axis_cpl_tvalid;
    wire [31:0]             m_axis_cpl_tdata;
    wire                    m_axis_cpl_tlast;
    wire [3:0]              m_axis_cpl_tkeep;
    wire                    m_axis_cpl_tuser;
    reg                     m_axis_cpl_tready;

    wire                    m_axi_awvalid;
    reg                     m_axi_awready; // Added driver
    wire [AXI_ADDR_W-1:0]   m_axi_awaddr;
    wire                    m_axi_awerr;

    reg [PCIE_ADDR_W-1:0]   cfg_window_base;
    reg [PCIE_ADDR_W-1:0]   cfg_window_mask;
    reg [AXI_ADDR_W-1:0]    cfg_axi_base;

    pcie_rx_engine #(
        .TAG_WIDTH(TAG_WIDTH),
        .MAX_DW_PER_REQ(MAX_DW_PER_REQ),
        .PCIE_ADDR_W(PCIE_ADDR_W),
        .AXI_ADDR_W(AXI_ADDR_W)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .tlp_rx_valid(tlp_rx_valid),
        .tlp_rx_sof(tlp_rx_sof),
        .tlp_rx_eof(tlp_rx_eof),
        .tlp_rx_data(tlp_rx_data),
        .tlp_type(tlp_type),
        .tlp_length_dw(tlp_length_dw),
        .tlp_tag(tlp_tag),
        .tlp_lower_addr(tlp_lower_addr),
        .tlp_cpl_status(tlp_cpl_status),
        .tlp_addr(tlp_addr),
        .req_alloc_valid(req_alloc_valid),
        .req_alloc_tag(req_alloc_tag),
        .req_alloc_len(req_alloc_len),
        .req_alloc_ready(req_alloc_ready),
        .m_axis_cpl_tvalid(m_axis_cpl_tvalid),
        .m_axis_cpl_tdata(m_axis_cpl_tdata),
        .m_axis_cpl_tlast(m_axis_cpl_tlast),
        .m_axis_cpl_tkeep(m_axis_cpl_tkeep),
        .m_axis_cpl_tuser(m_axis_cpl_tuser),
        .m_axis_cpl_tready(m_axis_cpl_tready),
        .m_axi_awvalid(m_axi_awvalid),
        .m_axi_awready(m_axi_awready), // Connected
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awerr(m_axi_awerr),
        .cfg_window_base(cfg_window_base),
        .cfg_window_mask(cfg_window_mask),
        .cfg_axi_base(cfg_axi_base)
    );

    always #5 clk = ~clk;

    always @(posedge clk) begin
        if (m_axi_awvalid && m_axi_awready) begin
            $display("[%8.3f ns] MWr AWTRANSLATE: Local AXI Address = 0x%08h | UR/ERR = %0b",
                     $time, m_axi_awaddr, m_axi_awerr);
        end
    end

    // ... [Rest of testbench remains identical to previous version, initializing m_axi_awready = 1'b1 in initial block] ...
    initial begin
        clk = 0;
        rst_n = 0;
        m_axi_awready = 1; // Default AXI Ready High
        m_axis_cpl_tready = 1;
        // [Proceed with previously defined stimuli]
    end
endmodule