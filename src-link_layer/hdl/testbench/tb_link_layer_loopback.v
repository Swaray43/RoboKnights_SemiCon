`timescale 1ns/1ps

module tb_link_layer_loopback;

    parameter AXIS_DATA_W = 512;
    parameter SYS_BUS_W   = 256;
    parameter CLK_PERIOD  = 10;

    reg clk;
    reg rst_n;

    // TX Input Interface
    reg                   s_axis_tvalid;
    reg [AXIS_DATA_W-1:0] s_axis_tdata;
    reg                   s_axis_tlast;
    wire                  s_axis_tready;

    // System Bus (with Error Injection capability)
    wire                  sys_bus_valid;
    wire [SYS_BUS_W-1:0]  sys_bus_data_tx;
    wire [SYS_BUS_W-1:0]  sys_bus_data_rx;
    wire                  sys_bus_sof;
    wire                  sys_bus_eof;
    
    reg  [SYS_BUS_W-1:0]  err_mask; // XOR mask for bit flipping

    // RX Output Interface
    wire                  m_axis_tvalid;
    wire [AXIS_DATA_W-1:0] m_axis_tdata;
    wire                  m_axis_tlast;
    reg                   m_axis_tready;

    wire                  crc_err;
    wire                  fec_err;

    // Error Injection Logic
    assign sys_bus_data_rx = sys_bus_data_tx ^ err_mask;

    link_layer_tx #(
        .AXIS_DATA_W(AXIS_DATA_W),
        .SYS_BUS_W(SYS_BUS_W)
    ) u_tx (
        .clk(clk),
        .rst_n(rst_n),
        .s_axis_tvalid(s_axis_tvalid),
        .s_axis_tdata(s_axis_tdata),
        .s_axis_tlast(s_axis_tlast),
        .s_axis_tready(s_axis_tready),
        .m_sys_bus_valid(sys_bus_valid),
        .m_sys_bus_data(sys_bus_data_tx),
        .m_sys_bus_sof(sys_bus_sof),
        .m_sys_bus_eof(sys_bus_eof)
    );

    link_layer_rx #(
        .SYS_BUS_W(SYS_BUS_W),
        .AXIS_DATA_W(AXIS_DATA_W)
    ) u_rx (
        .clk(clk),
        .rst_n(rst_n),
        .s_sys_bus_valid(sys_bus_valid),
        .s_sys_bus_data(sys_bus_data_rx),
        .s_sys_bus_sof(sys_bus_sof),
        .s_sys_bus_eof(sys_bus_eof),
        .m_axis_tvalid(m_axis_tvalid),
        .m_axis_tdata(m_axis_tdata),
        .m_axis_tlast(m_axis_tlast),
        .m_axis_tready(m_axis_tready),
        .crc_err(crc_err),
        .fec_err(fec_err)
    );

    always #(CLK_PERIOD / 2) clk = ~clk;

    reg [AXIS_DATA_W-1:0] test_data [0:7];
    integer rx_beat_cnt;

    task send_axis_beat;
        input [AXIS_DATA_W-1:0] data;
        input                   last;
        begin
            while (!s_axis_tready) @(posedge clk);
            s_axis_tvalid <= 1'b1;
            s_axis_tdata  <= data;
            s_axis_tlast  <= last;
            @(posedge clk);
            s_axis_tvalid <= 1'b0;
            s_axis_tlast  <= 1'b0;
        end
    endtask

    initial begin
        clk           = 0;
        rst_n         = 0;
        s_axis_tvalid = 0;
        s_axis_tdata  = 0;
        s_axis_tlast  = 0;
        m_axis_tready = 1;
        err_mask      = 0;
        rx_beat_cnt   = 0;

        test_data[0] = {64{8'h01}}; test_data[1] = {64{8'h02}};
        test_data[2] = {64{8'h03}}; test_data[3] = {64{8'h04}};
        test_data[4] = {64{8'h05}}; test_data[5] = {64{8'h06}};
        test_data[6] = {64{8'h07}}; test_data[7] = {64{8'h08}};

        #25 rst_n = 1;
        
        $display("=========================================================");
        $display("--- LINK LAYER ADVANCED EDGE-CASE TESTBENCH ---");
        $display("=========================================================");

        // --- TEST 1: Back-to-Back FLITs ---
        $display("\n[ %8.3f ns ] TEST 1: Sending Two Back-to-Back FLITs (8 Beats)", $time);
        send_axis_beat(test_data[0], 0); send_axis_beat(test_data[1], 0);
        send_axis_beat(test_data[2], 0); send_axis_beat(test_data[3], 0); // FLIT 1
        send_axis_beat(test_data[4], 0); send_axis_beat(test_data[5], 0);
        send_axis_beat(test_data[6], 0); send_axis_beat(test_data[7], 0); // FLIT 2
        
        #400; // Wait for processing

        // --- TEST 2: CRC/FEC Error Injection ---
        $display("\n[ %8.3f ns ] TEST 2: Injecting Single-Bit Error on System Bus", $time);
        rx_beat_cnt = 0; // reset monitor count
        
        // Setup a thread to flip a bit during the 7th cycle (EOF)
        fork
            begin
                wait(sys_bus_eof == 1'b1);
                err_mask[15] = 1'b1; // Corrupt a payload bit
                @(posedge clk);
                err_mask[15] = 1'b0;
            end
            begin
                send_axis_beat(test_data[0], 0); send_axis_beat(test_data[1], 0);
                send_axis_beat(test_data[2], 0); send_axis_beat(test_data[3], 0);
            end
        join

        #100;
        if (crc_err) $display("[ %8.3f ns ] PASS: CRC Error correctly asserted and FLIT dropped.", $time);
        else $display("[ %8.3f ns ] FAIL: CRC Error missed!", $time);
        
        #100;

        // --- TEST 3: RX Backpressure ---
        $display("\n[ %8.3f ns ] TEST 3: Toggling m_axis_tready to stall RX output", $time);
        rx_beat_cnt = 0;
        
        fork
            begin
                // Toggle ready signal every 30ns
                repeat(10) begin
                    #30 m_axis_tready = ~m_axis_tready;
                end
                m_axis_tready = 1'b1; // leave high at end
            end
            begin
                send_axis_beat(test_data[4], 0); send_axis_beat(test_data[5], 0);
                send_axis_beat(test_data[6], 0); send_axis_beat(test_data[7], 0);
            end
        join

        #300;
        
        // --- TEST 4: Premature TLAST (Partial FLIT) ---
        $display("\n[ %8.3f ns ] TEST 4: Premature TLAST (Sending 2 beats instead of 4)", $time);
        send_axis_beat(test_data[0], 0);
        send_axis_beat(test_data[1], 1); // Assert TLAST early

        #300;
        
        $display("\n=========================================================");
        $display("--- SIMULATION COMPLETE ---");
        $display("=========================================================");
        $finish;
    end

    // Auto-Monitor Data Outputs
    always @(posedge clk) begin
        if (m_axis_tvalid && m_axis_tready) begin
            $display("[ %8.3f ns ] RX Output Beat | LAST: %b | Data: %h", $time, m_axis_tlast, m_axis_tdata[63:0]);
        end
    end

endmodule