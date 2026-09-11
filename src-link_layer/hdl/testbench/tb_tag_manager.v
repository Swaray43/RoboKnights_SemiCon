`timescale 1ns/1ps

module tb_tag_manager;

    // Scaled down parameters for simulation speed
    localparam N = 4; 
    localparam TIMEOUT = 20;

    // Signals
    reg clk;
    reg rst_n;
    
    // Upstream Allocation
    reg s_valid;
    reg s_is_non_posted;
    wire s_tready;
    wire [9:0] m_tag;
    wire m_tag_valid;
    
    // Downstream Completion
    reg cpl_valid;
    reg [9:0] cpl_tag;
    
    // UR Generation
    wire ur_valid;
    wire [9:0] ur_tag;

    // DUT Instantiation
    tag_manager #(
        .N(N),
        .TIMEOUT_CYCLES(TIMEOUT)
    ) dut (
        .clk(clk),
        .rst_n(rst_n),
        .s_valid(s_valid),
        .s_is_non_posted(s_is_non_posted),
        .s_tready(s_tready),
        .m_tag(m_tag),
        .m_tag_valid(m_tag_valid),
        .cpl_valid(cpl_valid),
        .cpl_tag(cpl_tag),
        .ur_valid(ur_valid),
        .ur_tag(ur_tag)
    );

    // Clock generator (100MHz)
    always #5 clk = ~clk;

    initial begin
        // 0. Initialize
        clk = 0;
        rst_n = 0;
        s_valid = 0;
        s_is_non_posted = 0;
        cpl_valid = 0;
        cpl_tag = 0;

        // Apply Reset
        #20;
        rst_n = 1;
        #20;

        // 1. Basic Allocation & Completion
        @(posedge clk);
        s_valid = 1;
        s_is_non_posted = 1;
        @(posedge clk);
        s_valid = 0;
        
        #30; // Simulate fabric latency
        
        @(posedge clk);
        cpl_valid = 1;
        cpl_tag = 0; // Return tag 0
        @(posedge clk);
        cpl_valid = 0;

        // 2. Pool Exhaustion Test
        #20;
        @(posedge clk);
        s_valid = 1;
        s_is_non_posted = 1;
        // Hold valid high until tready drops (pool empty)
        wait(s_tready == 0); 
        @(posedge clk);
        s_valid = 0;

        // 3. Timeout & UR Generation Verification
        // Leave the 4 allocated tags uncompleted. 
        // Wait for the TIMEOUT threshold to expire.
        #(TIMEOUT * 10 + 100);

        // 4. Posted Request Bypass
        // Ensure m_tag_valid does not assert for MWr
        @(posedge clk);
        s_valid = 1;
        s_is_non_posted = 0;
        @(posedge clk);
        s_valid = 0;

        #50;
        $display("Simulation Complete.");
        $finish;
    end
    
    // Event Monitor
    always @(posedge clk) begin
        if (m_tag_valid) $display("[%0t] ALLOCATED: Tag %0d", $time, m_tag);
        if (cpl_valid)   $display("[%0t] COMPLETED: Tag %0d", $time, cpl_tag);
        if (ur_valid)    $display("[%0t] TIMEOUT UR: Tag %0d forcefully freed", $time, ur_tag);
    end

endmodule