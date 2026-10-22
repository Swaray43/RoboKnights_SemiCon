`timescale 1ns/1ps
// ============================================================
// tb_pcie_rx_engine -- S3 (rank 3)
//
// The previous stimulus block was replaced by a placeholder
// comment; only clock/reset remained and the bench hung (no
// $finish). Rewritten with real stimulus:
//   1. MWr inside the configured translation window
//   2. MWr outside the window (translation_err expected)
//   3. CplD for a tag the reorder buffer actually allocated
//   4. A malformed TLP (unrecognised Type field)
//   5. G4 (P2-5): a genuine MRd (Type 0_0000, Fmt has-data=0) must NOT
//      be misrouted through the address translator as a write
//   6. F5/F6: a Message (Type 1_0000) is not decoded as an MWr
//   7a S12/F6: CplDLk (Type 0_1011, with data) for an allocated tag drains
//      exactly like a CplD. Reverting F6 (is_cpld matching only 0_1010)
//      makes 7a fail: the TLP is flagged malformed and no data drains.
//   7b S12/F6: CplLk (Type 0_1011, no data) is recognised (not malformed)
//      and produces no AXI write and no completion data.
//   8. S13: an MWr into the exclusion window (BAR0) is not a translation
//      miss (no error, no XLAT_MISS count); a real miss still counts.
// and a RESULT: PASS/FAIL line.
// ============================================================
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
    reg                     tlp_has_data;   // G4

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
    reg                     m_axi_awready;
    wire [AXI_ADDR_W-1:0]   m_axi_awaddr;
    wire                    m_axi_awerr;

    reg [PCIE_ADDR_W-1:0]   cfg_window_base;
    reg [PCIE_ADDR_W-1:0]   cfg_window_mask;
    reg [AXI_ADDR_W-1:0]    cfg_axi_base;
    reg                     cfg_excl_en;      // S13
    reg [PCIE_ADDR_W-1:0]   cfg_excl_base, cfg_excl_mask;
    wire [7:0]              xlat_miss_cnt;
    reg                     xlat_miss_clr;
    integer                 cap_before;

    wire                    malformed_tlp_err;

    integer errors;

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
        .tlp_addr(tlp_addr), .tlp_hdr_done(1'b0),
        .tlp_has_data(tlp_has_data),
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
        .m_axi_awready(m_axi_awready),
        .m_axi_awaddr(m_axi_awaddr),
        .m_axi_awerr(m_axi_awerr),
        .cfg_window_base(cfg_window_base),
        .cfg_window_mask(cfg_window_mask),
        .cfg_axi_base(cfg_axi_base),
        .cfg_excl_en(cfg_excl_en), .cfg_excl_base(cfg_excl_base), .cfg_excl_mask(cfg_excl_mask),
        .err_xlat_miss_cnt(xlat_miss_cnt), .err_xlat_miss_clr(xlat_miss_clr),
        .cfg_req_valid(),
        .cfg_req_type1(),
        .cfg_req_tag(),
        .cfg_req_len_dw(),
        .cfg_req_addr(),
        .malformed_tlp_err(malformed_tlp_err)
    );

    always #5 clk = ~clk;

    task check_eq(input [63:0] got, input [63:0] exp, input [255:0] label);
        begin
            if (got !== exp) begin
                $display("FAIL %0s: got=0x%0h expected=0x%0h", label, got, exp);
                errors = errors + 1;
            end
        end
    endtask

    // -------- capture completion stream --------
    reg [31:0] cap_data [0:31];
    reg        cap_last [0:31];
    integer    cap_cnt;

    always @(posedge clk) begin
        if (m_axis_cpl_tvalid && m_axis_cpl_tready) begin
            cap_data[cap_cnt] = m_axis_cpl_tdata;
            cap_last[cap_cnt] = m_axis_cpl_tlast;
            cap_cnt = cap_cnt + 1;
        end
    end

    initial begin
        clk = 0; rst_n = 0;
        tlp_rx_valid = 0; tlp_rx_sof = 0; tlp_rx_eof = 0; tlp_rx_data = 0;
        tlp_type = 0; tlp_length_dw = 0; tlp_tag = 0; tlp_lower_addr = 0;
        tlp_cpl_status = 0; tlp_addr = 0; tlp_has_data = 0;
        req_alloc_valid = 0; req_alloc_tag = 0; req_alloc_len = 0;
        m_axi_awready = 1;
        m_axis_cpl_tready = 1;
        errors = 0; cap_cnt = 0;

        cfg_window_base = 64'h0000_0000_2000_0000;
        cfg_window_mask = 64'hFFFF_FFFF_F000_0000; // 256MB window
        cfg_axi_base    = 32'h1000_0000;
        cfg_excl_en     = 1'b1;                    // S13: BAR0 at 0x8000_0000, 64 KB
        cfg_excl_base   = 64'h0000_0000_8000_0000;
        cfg_excl_mask   = 64'hFFFF_FFFF_FFFF_0000;
        xlat_miss_clr   = 1'b0;

        @(negedge clk); @(negedge clk);
        rst_n = 1;
        @(negedge clk);

        // ============================================================
        // Test 1: MWr inside the configured window
        // ============================================================
        // axi_addr_valid is a single-cycle pulse the cycle after SOF
        // is sampled (it clears again the cycle after that, since
        // m_axi_awready is held high here) -- check it right at the
        // negedge following that one posedge, before it clears.
        @(negedge clk);
        tlp_type       = 5'b00000;              // MWr (Type 0, has_data=1)
        tlp_has_data   = 1;
        tlp_addr       = 64'h0000_0000_2000_1004;
        tlp_rx_valid   = 1; tlp_rx_sof = 1; tlp_rx_eof = 1;
        tlp_rx_data    = 32'hCAFE_0001;
        @(posedge clk); // SOF sampled here -> AWVALID becomes 1 after this edge
        @(negedge clk);
        tlp_rx_valid = 0; tlp_rx_sof = 0; tlp_rx_eof = 0;
        check_eq(m_axi_awvalid, 1, "T1: MWr inside window -> AWVALID");
        check_eq(m_axi_awaddr, 32'h1000_1004, "T1: translated AXI address");
        check_eq(m_axi_awerr, 0, "T1: no translation error inside window");
        @(negedge clk);

        // ============================================================
        // Test 2: MWr outside the configured window
        // ============================================================
        @(negedge clk);
        tlp_type     = 5'b00000; // MWr
        tlp_has_data = 1;
        tlp_addr     = 64'h0000_0000_3000_1004; // different upper bits -> miss
        tlp_rx_valid = 1; tlp_rx_sof = 1; tlp_rx_eof = 1;
        tlp_rx_data  = 32'hBAD0_0000;
        @(posedge clk); // SOF sampled here -> translation_err becomes 1 after this edge
        @(negedge clk);
        tlp_rx_valid = 0; tlp_rx_sof = 0; tlp_rx_eof = 0;
        check_eq(m_axi_awerr, 1, "T2: MWr outside window -> translation_err");
        check_eq(m_axi_awvalid, 0, "T2: AWVALID not asserted on a miss");
        @(negedge clk);

        // ============================================================
        // Test 3: CplD for an allocated tag (2 DW payload)
        // ============================================================
        @(negedge clk);
        req_alloc_valid = 1; req_alloc_tag = 10'h020; req_alloc_len = 10'd2;
        @(posedge clk);
        while (!req_alloc_ready) @(posedge clk);
        @(negedge clk);
        req_alloc_valid = 0;

        tlp_type       = 5'b01010; // CplD
        tlp_tag        = 10'h020;
        tlp_lower_addr = 7'd0;     // DW offset 0
        tlp_cpl_status = 3'b000;
        tlp_rx_valid   = 1; tlp_rx_sof = 1; tlp_rx_eof = 0;
        tlp_rx_data    = 32'h1111_1111;
        @(negedge clk);
        tlp_rx_sof = 0; tlp_rx_eof = 1;
        tlp_rx_data = 32'h2222_2222;
        @(negedge clk);
        tlp_rx_valid = 0; tlp_rx_eof = 0;
        repeat (4) @(negedge clk);

        check_eq(cap_cnt, 2, "T3: 2-DW completion drained");
        check_eq(cap_data[0], 32'h1111_1111, "T3: completion DW0");
        check_eq(cap_last[0], 0, "T3: completion DW0 not tlast");
        check_eq(cap_data[1], 32'h2222_2222, "T3: completion DW1");
        check_eq(cap_last[1], 1, "T3: completion DW1 tlast");

        // ============================================================
        // Test 4: malformed TLP (unrecognised Type field)
        // ============================================================
        @(negedge clk);
        tlp_type     = 5'b11111; // not MWr/CplD/Cfg
        tlp_rx_valid = 1; tlp_rx_sof = 1; tlp_rx_eof = 1;
        tlp_rx_data  = 32'hDEAD_BEEF;
        @(posedge clk);
        check_eq(malformed_tlp_err, 1, "T4: malformed TLP flagged");
        @(negedge clk);
        tlp_rx_valid = 0; tlp_rx_sof = 0; tlp_rx_eof = 0;
        @(posedge clk);
        check_eq(malformed_tlp_err, 0, "T4: malformed flag is a one-cycle pulse");

        // ============================================================
        // Test 5 (G4/P2-5): genuine MRd (Type 0_0000, has_data=0) inside
        // the configured window must NOT produce an AXI write -- it used
        // to be indistinguishable from MWr and decode a bogus write.
        // ============================================================
        @(negedge clk);
        tlp_type     = 5'b00000; // MRd: same Type as MWr, no data
        tlp_has_data = 0;
        tlp_addr     = 64'h0000_0000_2000_2004; // inside the window
        tlp_rx_valid = 1; tlp_rx_sof = 1; tlp_rx_eof = 1;
        tlp_rx_data  = 32'h0000_0000;
        @(posedge clk);
        check_eq(malformed_tlp_err, 0, "T5: MRd is not malformed");
        @(negedge clk);
        tlp_rx_valid = 0; tlp_rx_sof = 0; tlp_rx_eof = 0;
        check_eq(m_axi_awvalid, 0, "T5: MRd must not drive AWVALID (no misrouted write)");
        check_eq(m_axi_awerr, 0, "T5: MRd does not raise a translation error");
        @(posedge clk);

        // ============================================================
        // Test 6 (F5/F6): Message routed to RC (Type 1_0000) -- used to be
        // decoded as an MWr. No AXI write, not malformed.
        // ============================================================
        @(negedge clk);
        tlp_type     = 5'b10000; tlp_has_data = 0;
        tlp_addr     = 64'h0000_0000_2000_100C;
        tlp_rx_valid = 1; tlp_rx_sof = 1; tlp_rx_eof = 1;
        @(posedge clk);
        check_eq(malformed_tlp_err, 0, "T6: Msg is not malformed");
        @(negedge clk);
        tlp_rx_valid = 0; tlp_rx_sof = 0; tlp_rx_eof = 0;
        check_eq(m_axi_awvalid, 0, "T6: Msg does not issue an AXI write");
        @(posedge clk);

        // ============================================================
        // Test 7a (S12/F6): CplDLk (Type 0_1011, with data) for an
        // allocated tag -- must drain exactly like a CplD.
        // ============================================================
        @(negedge clk);
        cap_before = cap_cnt;
        req_alloc_valid = 1; req_alloc_tag = 10'h021; req_alloc_len = 10'd2;
        @(posedge clk);
        while (!req_alloc_ready) @(posedge clk);
        @(negedge clk);
        req_alloc_valid = 0;
        tlp_type       = 5'b01011; // CplDLk
        tlp_has_data   = 1;
        tlp_tag        = 10'h021;
        tlp_lower_addr = 7'd0;
        tlp_cpl_status = 3'b000;
        tlp_rx_valid   = 1; tlp_rx_sof = 1; tlp_rx_eof = 0;
        tlp_rx_data    = 32'h3333_3333;
        @(posedge clk);
        check_eq(malformed_tlp_err, 0, "T7a: CplDLk is not malformed");
        @(negedge clk);
        tlp_rx_sof = 0; tlp_rx_eof = 1;
        tlp_rx_data = 32'h4444_4444;
        @(negedge clk);
        tlp_rx_valid = 0; tlp_rx_eof = 0;
        repeat (4) @(negedge clk);
        check_eq(cap_cnt - cap_before, 2, "T7a: CplDLk 2 DW drained");
        check_eq(cap_data[cap_before],     32'h3333_3333, "T7a: CplDLk DW0");
        check_eq(cap_last[cap_before],     0,             "T7a: CplDLk DW0 not tlast");
        check_eq(cap_data[cap_before + 1], 32'h4444_4444, "T7a: CplDLk DW1");
        check_eq(cap_last[cap_before + 1], 1,             "T7a: CplDLk DW1 tlast");

        // ============================================================
        // Test 7b (S12/F6): CplLk (Type 0_1011, no data) -- recognised,
        // no AXI write, no completion data.
        // ============================================================
        @(negedge clk);
        cap_before = cap_cnt;
        tlp_type     = 5'b01011; tlp_has_data = 0;
        tlp_tag      = 10'h022;   // not allocated: nothing may drain
        tlp_rx_valid = 1; tlp_rx_sof = 1; tlp_rx_eof = 1;
        tlp_rx_data  = 32'h0000_0000;
        @(posedge clk);
        check_eq(malformed_tlp_err, 0, "T7b: CplLk is not malformed");
        @(negedge clk);
        tlp_rx_valid = 0; tlp_rx_sof = 0; tlp_rx_eof = 0;
        check_eq(m_axi_awvalid, 0, "T7b: CplLk no AXI write");
        repeat (4) @(negedge clk);
        check_eq(cap_cnt - cap_before, 0, "T7b: CplLk drains no data");

        // ============================================================
        // Test 8 (S13): MWr into BAR0 (exclusion window) is not a miss;
        // an MWr outside every window still is.
        // ============================================================
        @(negedge clk); xlat_miss_clr = 1; @(negedge clk); xlat_miss_clr = 0;
        check_eq(xlat_miss_cnt, 0, "T8: XLAT_MISS cleared");
        tlp_type     = 5'b00000; tlp_has_data = 1;
        tlp_addr     = 64'h0000_0000_8000_0008;   // BAR0 + 0x08
        tlp_rx_valid = 1; tlp_rx_sof = 1; tlp_rx_eof = 1;
        tlp_rx_data  = 32'h0000_0302;
        @(posedge clk);
        @(negedge clk);
        tlp_rx_valid = 0; tlp_rx_sof = 0; tlp_rx_eof = 0;
        check_eq(m_axi_awerr, 0,   "T8: BAR0 MWr no xlat error");
        check_eq(m_axi_awvalid, 0, "T8: BAR0 MWr no DMA write");
        @(negedge clk);
        check_eq(xlat_miss_cnt, 0, "T8: BAR0 MWr not counted");
        tlp_addr     = 64'h0000_0000_3000_0000;   // outside both windows
        tlp_rx_valid = 1; tlp_rx_sof = 1; tlp_rx_eof = 1;
        @(posedge clk);
        @(negedge clk);
        tlp_rx_valid = 0; tlp_rx_sof = 0; tlp_rx_eof = 0;
        check_eq(m_axi_awerr, 1, "T8: real miss -> xlat error");
        @(negedge clk);
        check_eq(xlat_miss_cnt, 1, "T8: real miss still counted");

        if (errors == 0) $display("RESULT: PASS");
        else              $display("RESULT: FAIL (%0d errors)", errors);
        $finish;
    end

    initial begin
        #20000;
        $display("[ERROR] Watchdog timeout");
        $display("RESULT: FAIL (watchdog)");
        $finish;
    end

endmodule