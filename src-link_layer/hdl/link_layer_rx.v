`timescale 1ns/1ps

module link_layer_rx #(
    parameter SYS_BUS_W   = 256,
    parameter AXIS_DATA_W = 512,
    parameter FLIT_BYTES  = 256,
    parameter FLIT_BITS   = FLIT_BYTES * 8 // 2048 bits total
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // Interface from Protocol-to-PHY Layer (System-Synchronous 256-bit Bus)
    input  wire                  s_sys_bus_valid,
    input  wire [SYS_BUS_W-1:0]  s_sys_bus_data,
    input  wire                  s_sys_bus_sof,
    input  wire                  s_sys_bus_eof,

    // Interface to Interconnect Layer (AXI4-Stream 512-bit)
    output reg                   m_axis_tvalid,
    output reg  [AXIS_DATA_W-1:0] m_axis_tdata,
    output reg                   m_axis_tlast,
    input  wire                  m_axis_tready,

    // Status Signals
    output reg                   crc_err,
    output reg                   fec_err
);

    localparam PAYLOAD_BITS = 2008;

    reg [1791:0]           rx_payload_buf;
    reg [2:0]              rx_cycle_cnt;
    reg                    rx_active;

    reg [PAYLOAD_BITS-1:0] payload_to_process;
    reg                    process_flit;
    reg [1:0]              tx_beat_cnt;

    // CRC32 Calculation
    function [31:0] calc_crc32;
        input [PAYLOAD_BITS-1:0] data;
        integer i;
        reg [31:0] crc;
        begin
            crc = 32'hFFFFFFFF;
            for (i = 0; i < PAYLOAD_BITS; i = i + 1) begin
                if ((crc[31] ^ data[i]) == 1'b1)
                    crc = (crc << 1) ^ 32'h04C11DB7;
                else
                    crc = crc << 1;
            end
            calc_crc32 = ~crc;
        end
    endfunction

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_cycle_cnt       <= 3'd0;
            rx_active          <= 1'b0;
            process_flit       <= 1'b0;
            rx_payload_buf     <= 1792'd0;
            payload_to_process <= {PAYLOAD_BITS{1'b0}};
            crc_err            <= 1'b0;
            fec_err            <= 1'b0;
        end else begin
            process_flit <= 1'b0;

            if (s_sys_bus_valid) begin
                if (s_sys_bus_sof || rx_active) begin
                    rx_active <= 1'b1;
                    case (rx_cycle_cnt)
                        3'd0: rx_payload_buf[255:0]    <= s_sys_bus_data;
                        3'd1: rx_payload_buf[511:256]  <= s_sys_bus_data;
                        3'd2: rx_payload_buf[767:512]  <= s_sys_bus_data;
                        3'd3: rx_payload_buf[1023:768] <= s_sys_bus_data;
                        3'd4: rx_payload_buf[1279:1024]<= s_sys_bus_data;
                        3'd5: rx_payload_buf[1535:1280]<= s_sys_bus_data;
                        3'd6: rx_payload_buf[1791:1536]<= s_sys_bus_data;
                        3'd7: begin
                            rx_active    <= 1'b0;
                            rx_cycle_cnt <= 3'd0;

                            // Reconstruct clean 2008-bit payload
                            payload_to_process <= {s_sys_bus_data[215:0], rx_payload_buf[1791:0]};
                            process_flit       <= 1'b1;

                            // Check CRC against embedded value in s_sys_bus_data[247:216]
                            crc_err <= (calc_crc32({s_sys_bus_data[215:0], rx_payload_buf[1791:0]}) != s_sys_bus_data[247:216]);
                        end
                    endcase

                    if (rx_cycle_cnt != 3'd7) begin
                        rx_cycle_cnt <= rx_cycle_cnt + 1'b1;
                    end
                end
            end
        end
    end

    // Stream verified beats out to Interconnect Layer
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            m_axis_tvalid <= 1'b0;
            m_axis_tdata  <= {AXIS_DATA_W{1'b0}};
            m_axis_tlast  <= 1'b0;
            tx_beat_cnt   <= 2'd0;
        end else begin
            if (process_flit && !crc_err) begin
                m_axis_tvalid <= 1'b1;
                m_axis_tdata  <= payload_to_process[511:0];
                m_axis_tlast  <= 1'b0;
                tx_beat_cnt   <= 2'd1;
            end else if (m_axis_tvalid && m_axis_tready) begin
                case (tx_beat_cnt)
                    2'd1: begin
                        m_axis_tdata <= payload_to_process[1023:512];
                        tx_beat_cnt  <= 2'd2;
                    end
                    2'd2: begin
                        m_axis_tdata <= payload_to_process[1535:1024];
                        tx_beat_cnt  <= 2'd3;
                    end
                    2'd3: begin
                        m_axis_tdata <= {40'd0, payload_to_process[2007:1536]};
                        m_axis_tlast <= 1'b1;
                        tx_beat_cnt  <= 2'd0;
                    end
                    2'd0: begin
                        m_axis_tvalid <= 1'b0;
                        m_axis_tlast  <= 1'b0;
                    end
                endcase
            end
        end
    end

endmodule