`timescale 1ns/1ps

module link_layer_tx #(
    parameter AXIS_DATA_W  = 512,
    parameter SYS_BUS_W    = 256,
    parameter FLIT_BYTES   = 256,
    parameter FLIT_BITS    = FLIT_BYTES * 8 // 2048 bits total
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // Interconnect Layer Interface (AXI4-Stream 512-bit)
    input  wire                  s_axis_tvalid,
    input  wire [AXIS_DATA_W-1:0] s_axis_tdata,
    input  wire                  s_axis_tlast,
    output wire                  s_axis_tready,

    // Protocol-to-PHY Layer Interface (System-Synchronous 256-bit Bus)
    output reg                   m_sys_bus_valid,
    output reg  [SYS_BUS_W-1:0]  m_sys_bus_data,
    output reg                   m_sys_bus_sof,
    output reg                   m_sys_bus_eof
);

    localparam BEATS_PER_FLIT  = 4;
    localparam PAYLOAD_BITS    = 2008; // 2048 - 32(CRC32) - 8(FEC) = 2008 bits (251 Bytes)

    reg [AXIS_DATA_W-1:0] flit_buf [0:BEATS_PER_FLIT-1];
    reg [1:0]             beat_cnt;
    reg                   tx_busy;
    reg [2:0]             out_cnt;

    reg [PAYLOAD_BITS-1:0] assembled_payload;
    wire [31:0]            crc32_val;
    wire [7:0]             fec_code;

    assign s_axis_tready = !tx_busy;

    // CRC32 Generator over 2008-bit Payload
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

    // FEC Parity Generator over 2008-bit Payload
    function [7:0] calc_fec;
        input [PAYLOAD_BITS-1:0] d;
        begin
            calc_fec[0] = ^(d & 2008'h5555555555555555);
            calc_fec[1] = ^(d & 2008'hAAAAAAAAAAAAAAAA);
            calc_fec[2] = ^(d & 2008'hCCCCCCCCCCCCCCCC);
            calc_fec[3] = ^(d & 2008'hFFFFFFFF00000000);
            calc_fec[4] = ^d[511:0];
            calc_fec[5] = ^d[1023:512];
            calc_fec[6] = ^d[1535:1024];
            calc_fec[7] = ^d[2007:1536];
        end
    endfunction

    assign crc32_val = calc_crc32(assembled_payload);
    assign fec_code  = calc_fec(assembled_payload);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            beat_cnt          <= 2'd0;
            tx_busy           <= 1'b0;
            out_cnt           <= 3'd0;
            m_sys_bus_valid   <= 1'b0;
            m_sys_bus_data    <= {SYS_BUS_W{1'b0}};
            m_sys_bus_sof     <= 1'b0;
            m_sys_bus_eof     <= 1'b0;
            assembled_payload <= {PAYLOAD_BITS{1'b0}};
        end else begin
            m_sys_bus_valid <= 1'b0;
            m_sys_bus_sof   <= 1'b0;
            m_sys_bus_eof   <= 1'b0;

            if (!tx_busy) begin
                if (s_axis_tvalid) begin
                    flit_buf[beat_cnt] <= s_axis_tdata;
                    beat_cnt <= beat_cnt + 1'b1;

                    if (beat_cnt == 2'd3 || s_axis_tlast) begin
                        assembled_payload <= {s_axis_tdata[471:0], flit_buf[2], flit_buf[1], flit_buf[0]};
                        tx_busy  <= 1'b1;
                        beat_cnt <= 2'd0;
                        out_cnt  <= 3'd0;
                    end
                end
            end else begin
                m_sys_bus_valid <= 1'b1;
                m_sys_bus_sof   <= (out_cnt == 3'd0);
                m_sys_bus_eof   <= (out_cnt == 3'd7);

                case (out_cnt)
                    3'd0: m_sys_bus_data <= assembled_payload[255:0];
                    3'd1: m_sys_bus_data <= assembled_payload[511:256];
                    3'd2: m_sys_bus_data <= assembled_payload[767:512];
                    3'd3: m_sys_bus_data <= assembled_payload[1023:768];
                    3'd4: m_sys_bus_data <= assembled_payload[1279:1024];
                    3'd5: m_sys_bus_data <= assembled_payload[1535:1280];
                    3'd6: m_sys_bus_data <= assembled_payload[1791:1536];
                    3'd7: begin
                        // [215:0]   : Final 216 bits payload
                        // [247:216] : 32-bit CRC32
                        // [255:248] : 8-bit FEC
                        m_sys_bus_data <= {fec_code, crc32_val, assembled_payload[2007:1792]};
                        tx_busy <= 1'b0;
                    end
                endcase
                out_cnt <= out_cnt + 1'b1;
            end
        end
    end

endmodule