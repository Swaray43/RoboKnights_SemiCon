// =============================================================================
// tb_pcie_config_space / tb_msix_controller
// =============================================================================

module tb_pcie_config_space();
reg clk, rst_n;
reg cfg_wr_en, cfg_rd_en;
reg [11:0] cfg_addr;
reg [31:0] cfg_wdata;
wire [31:0] cfg_rdata;
wire cfg_cpl_valid;
reg link_training_bit;
reg [5:0] link_width;
reg [3:0] link_speed;
wire [31:0] bar0, bar1, bar2, bar3, bar4, bar5;
wire [2:0] mps, mrrs;
wire mem_en, bm_en, msix_en, msix_fmask;
integer errors;

pcie_config_space #(
    .BAR0_SIZE(32'h0001_0000) // 64KB
) dut (
    .clk(clk), .rst_n(rst_n),
    .cfg_wr_en(cfg_wr_en), .cfg_rd_en(cfg_rd_en), .cfg_addr(cfg_addr), .cfg_wdata(cfg_wdata),
    .cfg_rdata(cfg_rdata), .cfg_cpl_valid(cfg_cpl_valid),
    .link_training_bit(link_training_bit), .link_width(link_width), .link_speed(link_speed),
    .bar_addr_0(bar0), .bar_addr_1(bar1), .bar_addr_2(bar2), .bar_addr_3(bar3), .bar_addr_4(bar4), .bar_addr_5(bar5),
    .max_payload_size(mps), .max_read_req_size(mrrs),
    .mem_space_enable(mem_en), .bus_master_enable(bm_en),
    .msix_enable(msix_en), .msix_func_mask(msix_fmask)
);

initial clk = 0;
always #5 clk = ~clk;

task cfg_write(input [11:0] addr, input [31:0] data);
    begin
        @(negedge clk);
        cfg_addr = addr; cfg_wdata = data; cfg_wr_en = 1;
        @(negedge clk);
        cfg_wr_en = 0;
    end
endtask

task cfg_read(input [11:0] addr);
    begin
        @(negedge clk);
        cfg_addr = addr; cfg_rd_en = 1;
        @(negedge clk);
        cfg_rd_en = 0;
    end
endtask

initial begin
    errors = 0;
    rst_n = 0; cfg_wr_en = 0; cfg_rd_en = 0; cfg_addr = 0; cfg_wdata = 0;
    link_training_bit = 0; link_width = 6'd0; link_speed = 4'd0;
    repeat (3) @(negedge clk);
    rst_n = 1;
    @(negedge clk);

    // ---- Vendor/Device ID readback ----
    cfg_read(12'h000);
    if (cfg_rdata !== 32'hC0DEFACE) begin
        $display("FAIL: Vendor/Device ID = %h, expected C0DEFACE", cfg_rdata);
        errors = errors + 1;
    end else $display("PASS: Vendor/Device ID readback correct");

    if (cfg_cpl_valid !== 1'b1) begin
        $display("FAIL: cfg_cpl_valid did not pulse on CfgRd");
        errors = errors + 1;
    end else $display("PASS: cfg_cpl_valid pulsed on CfgRd");

    // ---- BAR0 size probe: write all-1s, size should reveal as ~(64KB-1) in low bits ----
    cfg_write(12'h010, 32'hFFFF_FFFF);
    cfg_read(12'h010);
    if (cfg_rdata !== 32'hFFFF_0000) begin
        $display("FAIL: BAR0 size probe = %h, expected FFFF0000 (64KB size mask)", cfg_rdata);
        errors = errors + 1;
    end else $display("PASS: BAR0 size probe correctly reveals 64KB size");

    // ---- BAR0 real base address write (must respect size mask, low 16 bits force 0) ----
    cfg_write(12'h010, 32'h1234_5678);
    cfg_read(12'h010);
    if (cfg_rdata !== 32'h1234_0000) begin
        $display("FAIL: BAR0 base = %h, expected 12340000 (low bits masked by size)", cfg_rdata);
        errors = errors + 1;
    end else $display("PASS: BAR0 base address correctly masked to size boundary");

    if (bar0 !== 32'h1234_0000) begin
        $display("FAIL: bar_addr_0 output = %h, expected 12340000", bar0);
        errors = errors + 1;
    end else $display("PASS: bar_addr_0 output matches config space BAR0");

    // ---- BAR1 (unimplemented, size=0): must stay hardwired 0 regardless of writes ----
    cfg_write(12'h014, 32'hFFFF_FFFF);
    cfg_read(12'h014);
    if (cfg_rdata !== 32'h0) begin
        $display("FAIL: unimplemented BAR1 = %h, expected 0", cfg_rdata);
        errors = errors + 1;
    end else $display("PASS: unimplemented BAR1 correctly hardwired to 0");

    // ---- Device Control: MPS/MRRS write and readback ----
    cfg_write(12'h048, (32'd5 << 12) | (32'd3 << 5)); // MRRS=101 @[14:12], MPS=011 @[7:5]
    cfg_read(12'h048);
    if (mps !== 3'b011 || mrrs !== 3'b101) begin
        $display("FAIL: Device Control MPS=%b MRRS=%b, expected MPS=011 MRRS=101", mps, mrrs);
        errors = errors + 1;
    end else $display("PASS: Device Control MPS/MRRS written and exposed correctly");

    // ---- Command register: mem space + bus master enable ----
    cfg_write(12'h004, 32'h0000_0003);
    @(negedge clk);
    if (mem_en !== 1'b1 || bm_en !== 1'b1) begin
        $display("FAIL: mem_space_enable=%b bus_master_enable=%b, expected both 1", mem_en, bm_en);
        errors = errors + 1;
    end else $display("PASS: Command register mem/bus-master enable bits correct");

    // ---- MSI-X Message Control: enable + function mask ----
    cfg_write(12'h050, {1'b1, 1'b1, 30'b0}); // msix_enable=1, func_mask=1
    @(negedge clk);
    if (msix_en !== 1'b1 || msix_fmask !== 1'b1) begin
        $display("FAIL: msix_enable=%b msix_func_mask=%b, expected both 1", msix_en, msix_fmask);
        errors = errors + 1;
    end else $display("PASS: MSI-X Message Control enable/function-mask correct");

    cfg_read(12'h050);
    if (cfg_rdata[7:0] !== 8'h11) begin
        $display("FAIL: MSI-X Capability ID = %h, expected 11", cfg_rdata[7:0]);
        errors = errors + 1;
    end else $display("PASS: MSI-X Capability ID correct");

    // ---- Capabilities pointer chain: 0x34 -> 0x40 (PCIe cap, next=0x50) -> 0x50 (MSI-X cap, terminal) ----
    cfg_read(12'h034);
    if (cfg_rdata[7:0] !== 8'h40) begin
        $display("FAIL: Capabilities pointer = %h, expected 40", cfg_rdata[7:0]);
        errors = errors + 1;
    end else $display("PASS: Capabilities pointer correctly points to 0x40");

    cfg_read(12'h040);
    if (cfg_rdata[7:0] !== 8'h10 || cfg_rdata[15:8] !== 8'h50) begin
        $display("FAIL: PCIe cap header = %h, expected id=10 next=50", cfg_rdata[15:0]);
        errors = errors + 1;
    end else $display("PASS: PCIe capability header id/next-ptr correct, chains to MSI-X cap");

    // ---- Link Status readback ----
    link_training_bit = 1; link_width = 6'd4; link_speed = 4'd6;
    @(negedge clk);
    cfg_read(12'h05C);
    if (cfg_rdata[0] !== 1'b1 || cfg_rdata[7:2] !== 6'd4 || cfg_rdata[11:8] !== 4'd6) begin
        $display("FAIL: Link Status = %h, expected training=1 width=4 speed=6", cfg_rdata);
        errors = errors + 1;
    end else $display("PASS: Link Status correctly reflects live training/width/speed inputs");

    // ---- CfgWr must generate Cpl (no data), i.e. cfg_cpl_valid pulses, cfg_rdata unaffected ----
    cfg_write(12'h004, 32'h0);
    if (cfg_cpl_valid !== 1'b1) begin
        $display("FAIL: cfg_cpl_valid did not pulse on CfgWr");
        errors = errors + 1;
    end else $display("PASS: cfg_cpl_valid pulsed on CfgWr (Cpl, no data)");

    if (errors == 0) $display("ALL PASS: pcie_config_space");
    else $display("TOTAL FAILURES: %0d", errors);
    $stop;
end
endmodule


module tb_msix_controller();
reg clk, rst_n;
reg tbl_wr_en;
reg [2:0] tbl_wr_vec;
reg [63:0] tbl_wr_addr;
reg [31:0] tbl_wr_data;
reg tbl_wr_mask;
reg tbl_rd_en;
reg [2:0] tbl_rd_vec;
wire [63:0] tbl_rd_addr;
wire [31:0] tbl_rd_data;
wire tbl_rd_mask;
wire [7:0] pba;
reg dma_complete_pulse;
reg [2:0] dma_complete_vector;
wire mwr_valid;
wire [63:0] mwr_addr;
wire [31:0] mwr_data;
integer errors;

msix_controller #(.NUM_VECTORS(8)) dut (
    .clk(clk), .rst_n(rst_n),
    .tbl_wr_en(tbl_wr_en), .tbl_wr_vec(tbl_wr_vec), .tbl_wr_addr(tbl_wr_addr),
    .tbl_wr_data(tbl_wr_data), .tbl_wr_mask(tbl_wr_mask),
    .tbl_rd_en(tbl_rd_en), .tbl_rd_vec(tbl_rd_vec),
    .tbl_rd_addr(tbl_rd_addr), .tbl_rd_data(tbl_rd_data), .tbl_rd_mask(tbl_rd_mask),
    .pba(pba),
    .dma_complete_pulse(dma_complete_pulse), .dma_complete_vector(dma_complete_vector),
    .mwr_valid(mwr_valid), .mwr_addr(mwr_addr), .mwr_data(mwr_data)
);

initial clk = 0;
always #5 clk = ~clk;

initial begin
    errors = 0;
    rst_n = 0; tbl_wr_en = 0; tbl_rd_en = 0; dma_complete_pulse = 0;
    tbl_wr_vec = 0; tbl_wr_addr = 0; tbl_wr_data = 0; tbl_wr_mask = 0;
    tbl_rd_vec = 0; dma_complete_vector = 0;
    repeat (3) @(negedge clk);
    rst_n = 1;
    @(negedge clk);

    // ---- Default-masked reset state: no spurious interrupts before host programs table ----
    dma_complete_pulse = 1; dma_complete_vector = 3'd2;
    @(negedge clk);
    dma_complete_pulse = 0;
    @(negedge clk);
    if (mwr_valid !== 1'b0) begin
        $display("FAIL: mwr_valid fired for an unprogrammed (default-masked) vector");
        errors = errors + 1;
    end else $display("PASS: unprogrammed vector correctly stayed masked, no spurious MWr");

    if (pba[2] !== 1'b1) begin
        $display("FAIL: PBA bit 2 not set despite masked interrupt firing");
        errors = errors + 1;
    end else $display("PASS: PBA bit correctly set for masked vector");

    // ---- Program vector 3 unmasked, fire completion, expect immediate MWr ----
    tbl_wr_en = 1; tbl_wr_vec = 3'd3; tbl_wr_addr = 64'hA000_0000_1000; tbl_wr_data = 32'hCAFEBABE; tbl_wr_mask = 1'b0;
    @(negedge clk);
    tbl_wr_en = 0;
    @(negedge clk);

    dma_complete_pulse = 1; dma_complete_vector = 3'd3;
    @(negedge clk);
    dma_complete_pulse = 0;

    if (mwr_valid !== 1'b1 || mwr_addr !== 64'hA000_0000_1000 || mwr_data !== 32'hCAFEBABE) begin
        $display("FAIL: unmasked vector did not fire correct MWr. valid=%b addr=%h data=%h", mwr_valid, mwr_addr, mwr_data);
        errors = errors + 1;
    end else $display("PASS: unmasked vector correctly issued 1-DW MWr with table's addr/data");

    @(negedge clk);
    if (mwr_valid !== 1'b0) begin
        $display("FAIL: mwr_valid did not deassert after one cycle (pulse behavior expected)");
        errors = errors + 1;
    end else $display("PASS: mwr_valid correctly returned to pulse-only behavior");

    if (pba[3] !== 1'b0) begin
        $display("FAIL: PBA bit 3 incorrectly set for an unmasked vector that fired successfully");
        errors = errors + 1;
    end else $display("PASS: PBA bit correctly stayed clear for unmasked vector");

    // ---- Table readback ----
    tbl_rd_en = 1; tbl_rd_vec = 3'd3;
    @(negedge clk);
    tbl_rd_en = 0;
    @(negedge clk);
    if (tbl_rd_addr !== 64'hA000_0000_1000 || tbl_rd_data !== 32'hCAFEBABE || tbl_rd_mask !== 1'b0) begin
        $display("FAIL: table readback mismatch. addr=%h data=%h mask=%b", tbl_rd_addr, tbl_rd_data, tbl_rd_mask);
        errors = errors + 1;
    end else $display("PASS: table entry readback matches what was written");

    if (errors == 0) $display("ALL PASS: msix_controller");
    else $display("TOTAL FAILURES: %0d", errors);
    $stop;
end
endmodule
