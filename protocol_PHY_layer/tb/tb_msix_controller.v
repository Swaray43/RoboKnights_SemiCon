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
    .mwr_valid(mwr_valid), .mwr_ready(1'b1), .mwr_addr(mwr_addr), .mwr_data(mwr_data)
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
