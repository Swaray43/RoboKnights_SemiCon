`timescale 1ns/1ps

module tb_axi_backpressure;

    reg clk;
    reg rst_n;

    // Allocation
    reg        req_valid;
    reg [9:0]  req_tag;
    reg [9:0]  req_len_dw;
    wire       req_ready;

    // Ingress
    reg        rx_valid, rx_sof, rx_dvalid, rx_last_dw;
    reg [9:0]  rx_offset;
    reg [9:0]  rx_tag;
    reg [31:0] rx_data;
    
    // Egress AXI-S
    wire        m_axis_tvalid;
    wire [31:0] m_axis_tdata;
    wire        m_axis_tlast;
    wire [3:0]  m_axis_tkeep;
    wire        m_axis_tuser;
    reg         m_axis_tready;

    axi_pcie_reorder_buffer dut (
        .clk(clk), .rst_n(rst_n),
        .req_valid(req_valid), .req_tag(req_tag), .req_len_dw(req_len_dw), .req_ready(req_ready),
        .rx_cpld_valid(rx_valid), .rx_cpld_sof(rx_sof), .rx_cpld_offset(rx_offset),
        .rx_cpld_tag(rx_tag), .rx_cpld_data(rx_data), .rx_cpld_dvalid(rx_dvalid),
        .rx_cpld_status(3'b0), .rx_cpld_last_dw(rx_last_dw),
        .m_axis_tvalid(m_axis_tvalid), .m_axis_tdata(m_axis_tdata),
        .m_axis_tlast(m_axis_tlast), .m_axis_tkeep(m_axis_tkeep),
        .m_axis_tuser(m_axis_tuser), .m_axis_tready(m_axis_tready)
    );

    always #5 clk = ~clk;

    // AXI Protocol Monitor
    always @(posedge clk) begin
        if (m_axis_tvalid) begin
            if (m_axis_tready) begin
                $display("[%8.3f ns] AXI OUT ACCEPTED : Data=0x%08h | TLAST=%b", $time, m_axis_tdata, m_axis_tlast);
            end else begin
                $display("[%8.3f ns] AXI OUT STALLED  : Data=0x%08h | TLAST=%b (Waiting for TREADY)", $time, m_axis_tdata, m_axis_tlast);
            end
        end
    end

    initial begin
        clk = 0; rst_n = 0;
        req_valid = 0; req_tag = 0; req_len_dw = 0;
        rx_valid = 0; rx_sof = 0; rx_offset = 0; rx_tag = 0; rx_data = 0; rx_dvalid = 0; rx_last_dw = 0;
        m_axis_tready = 1; // Initially ready

        #20 rst_n = 1; #20;

        $display("\n--- TEST: AXI Backpressure (Egress Stall) ---");
        
        // 1. Allocate a 4-DW Request (Tag 0x020)
        @(posedge clk);
        req_valid <= 1; req_tag <= 10'h020; req_len_dw <= 10'd4;
        @(posedge clk) req_valid <= 0;

        #30;
        // 2. Inject all 4 DWs in order (simulate normal reception)
        $display("[%8.3f ns] RX: Injecting 4 DWs...", $time);
        @(posedge clk);
        rx_valid <= 1; rx_sof <= 1; rx_offset <= 10'd0; rx_tag <= 10'h020; rx_dvalid <= 1; rx_data <= 32'hDDDD_0000;
        @(posedge clk) rx_sof <= 0; rx_data <= 32'hDDDD_1111;
        @(posedge clk) rx_data <= 32'hDDDD_2222;
        @(posedge clk) rx_data <= 32'hDDDD_3333; rx_last_dw <= 1;
        @(posedge clk) rx_valid <= 0; rx_dvalid <= 0; rx_last_dw <= 0;

        // 3. Play with m_axis_tready during output
        // Wait for the first DW to appear on the AXI bus
        wait(m_axis_tvalid == 1'b1);
        
        @(posedge clk); 
        // Drop TREADY just as DW 1 is about to be presented
        $display("[%8.3f ns] TB: Dropping TREADY to 0 (Stall downstream)...", $time);
        m_axis_tready <= 0;
        
        // Hold stall for 4 clock cycles
        #40; 
        
        $display("[%8.3f ns] TB: Raising TREADY to 1 (Resume downstream)...", $time);
        m_axis_tready <= 1;

        // Wait a couple cycles, then stall again right before the LAST beat
        @(posedge clk);
        @(posedge clk);
        $display("[%8.3f ns] TB: Dropping TREADY to 0 (Stall on TLAST)...", $time);
        m_axis_tready <= 0;
        
        #20;
        $display("[%8.3f ns] TB: Raising TREADY to 1 (Resume)...", $time);
        m_axis_tready <= 1;

        #50;
        $display("\n--- SIMULATION COMPLETE ---");
        $finish;
    end
endmodule