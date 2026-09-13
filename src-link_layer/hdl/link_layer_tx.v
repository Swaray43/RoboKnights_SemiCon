`timescale 1ns/1ps

// ============================================================
// Module : link_layer_tx
// Layer  : Link layer (Engineer B)
//
// Change (interleave support 2025):
//   Fully independent per-channel FLIT assembly state.
//   See module comment in write_data_buffer.v for context.
//
//   Each channel (0=SmartNIC, 1=ML) owns:
//     flit_buf_ch[CH][0..3]  four 512-bit beat slots
//     beat_cnt_ch[CH]        beats buffered so far
//     ch_busy[CH]            FLIT sealed, awaiting serialisation
//     payload_ch[CH]         assembled 2008-bit payload
//
//   Output arbitration: round-robin between two sealed channels.
//
//   Per-channel back-pressure: s_axis_tready_ch[CH] de-asserts
//   while ch_busy[CH]=1. The combined s_axis_tready is high when
//   the presented channel (s_axis_tuser[0]) is ready.
//
// LT2 fix retained: partial FLITs zero-fill unwritten slots.
// LT4 fix retained: out_cnt does not overflow past 7.
// ============================================================

module link_layer_tx #(
    parameter AXIS_DATA_W  = 512,
    parameter SYS_BUS_W    = 256,
    parameter FLIT_BYTES   = 256,
    parameter FLIT_BITS    = FLIT_BYTES * 8,
    parameter TUSER_W      = 26,
    parameter SOFF_W       = 24
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // Upstream AXI4-Stream 512-bit
    // tuser[0]        = stream_id  (0=SmartNIC, 1=ML)
    // tuser[SOFF_W:1] = session_offset
    // tuser[TUSER_W-1]= tlast_flag
    input  wire                    s_axis_tvalid,
    input  wire [AXIS_DATA_W-1:0]  s_axis_tdata,
    input  wire                    s_axis_tlast,
    input  wire [TUSER_W-1:0]      s_axis_tuser,
    output wire                    s_axis_tready,      // high when presented channel is ready
    output wire [1:0]              s_axis_tready_ch,   // per-channel ready

    // Downstream System-Synchronous 256-bit bus to PHY
    output reg                     m_sys_bus_valid,
    output reg  [SYS_BUS_W-1:0]    m_sys_bus_data,
    output reg                     m_sys_bus_sof,
    output reg                     m_sys_bus_eof
);

    localparam BEATS_PER_FLIT = 4;
    localparam PAYLOAD_BITS   = 2008;
    localparam NUM_CHANNELS   = 2;

    wire ch_sel = s_axis_tuser[0];

    // ------------------------------------------------------------------
    // Per-channel FLIT assembly state
    // ------------------------------------------------------------------
    reg [AXIS_DATA_W-1:0] flit_buf_ch [0:NUM_CHANNELS-1][0:BEATS_PER_FLIT-1];
    reg [1:0]             beat_cnt_ch  [0:NUM_CHANNELS-1];
    reg                   ch_busy      [0:NUM_CHANNELS-1];
    reg [PAYLOAD_BITS-1:0] payload_ch  [0:NUM_CHANNELS-1];

    // ------------------------------------------------------------------
    // CRC32
    // ------------------------------------------------------------------
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

    // ------------------------------------------------------------------
    // FEC parity
    // ------------------------------------------------------------------
    function [7:0] calc_fec;
        input [PAYLOAD_BITS-1:0] d;
        begin
            calc_fec[0] = ^d[250:0];
            calc_fec[1] = ^d[501:251];
            calc_fec[2] = ^d[752:502];
            calc_fec[3] = ^d[1003:753];
            calc_fec[4] = ^d[1254:1004];
            calc_fec[5] = ^d[1505:1255];
            calc_fec[6] = ^d[1756:1506];
            calc_fec[7] = ^d[2007:1757];
        end
    endfunction

    // ------------------------------------------------------------------
    // Per-channel back-pressure
    // ------------------------------------------------------------------
    assign s_axis_tready_ch[0] = !ch_busy[0];
    assign s_axis_tready_ch[1] = !ch_busy[1];
    assign s_axis_tready       =  s_axis_tready_ch[ch_sel];

    // ------------------------------------------------------------------
    // Output serialiser state
    // ------------------------------------------------------------------
    reg [2:0]              out_cnt;
    reg                    out_busy;
    reg                    out_ch;
    reg [PAYLOAD_BITS-1:0] out_payload;

    wire [31:0] out_crc = calc_crc32(out_payload);
    wire [7:0]  out_fec = calc_fec(out_payload);

    // ------------------------------------------------------------------
    // Round-robin arbiter
    // ------------------------------------------------------------------
    reg rr_last;

    wire ch0_rdy = ch_busy[0];
    wire ch1_rdy = ch_busy[1];
    wire any_rdy = ch0_rdy || ch1_rdy;

    wire next_ch = (ch0_rdy && ch1_rdy) ? !rr_last :
                   ch1_rdy              ?  1'b1     :
                                           1'b0;

    // ------------------------------------------------------------------
    // Per-channel FLIT assembly
    // ------------------------------------------------------------------
    integer c;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            for (c = 0; c < NUM_CHANNELS; c = c + 1) begin
                beat_cnt_ch[c]    <= 2'd0;
                ch_busy[c]        <= 1'b0;
                payload_ch[c]     <= {PAYLOAD_BITS{1'b0}};
                flit_buf_ch[c][0] <= {AXIS_DATA_W{1'b0}};
                flit_buf_ch[c][1] <= {AXIS_DATA_W{1'b0}};
                flit_buf_ch[c][2] <= {AXIS_DATA_W{1'b0}};
                flit_buf_ch[c][3] <= {AXIS_DATA_W{1'b0}};
            end
        end else begin
            // Accept beat into the correct channel
            if (s_axis_tvalid && s_axis_tready_ch[ch_sel]) begin
                flit_buf_ch[ch_sel][beat_cnt_ch[ch_sel]] <= s_axis_tdata;

                if (beat_cnt_ch[ch_sel] == 2'd3 || s_axis_tlast) begin
                    // Seal FLIT with zero-fill for unwritten slots (LT2 fix)
                    begin : seal
                        reg [AXIS_DATA_W-1:0] b0, b1, b2;
                        b0 = (beat_cnt_ch[ch_sel] >= 2'd1) ? flit_buf_ch[ch_sel][0] : {AXIS_DATA_W{1'b0}};
                        b1 = (beat_cnt_ch[ch_sel] >= 2'd2) ? flit_buf_ch[ch_sel][1] : {AXIS_DATA_W{1'b0}};
                        b2 = (beat_cnt_ch[ch_sel] >= 2'd3) ? flit_buf_ch[ch_sel][2] : {AXIS_DATA_W{1'b0}};
                        payload_ch[ch_sel] <= {s_axis_tdata[471:0], b2, b1, b0};
                    end
                    ch_busy[ch_sel]     <= 1'b1;
                    beat_cnt_ch[ch_sel] <= 2'd0;
                end else begin
                    beat_cnt_ch[ch_sel] <= beat_cnt_ch[ch_sel] + 1'b1;
                end
            end

            // Release winning channel when serialiser picks it up
            if (!out_busy && any_rdy)
                ch_busy[next_ch] <= 1'b0;
        end
    end

    // ------------------------------------------------------------------
    // Output serialisation FSM
    // ------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            out_busy        <= 1'b0;
            out_ch          <= 1'b0;
            out_cnt         <= 3'd0;
            rr_last         <= 1'b0;
            out_payload     <= {PAYLOAD_BITS{1'b0}};
            m_sys_bus_valid <= 1'b0;
            m_sys_bus_data  <= {SYS_BUS_W{1'b0}};
            m_sys_bus_sof   <= 1'b0;
            m_sys_bus_eof   <= 1'b0;
        end else begin
            m_sys_bus_valid <= 1'b0;
            m_sys_bus_sof   <= 1'b0;
            m_sys_bus_eof   <= 1'b0;

            if (!out_busy) begin
                if (any_rdy) begin
                    out_ch      <= next_ch;
                    out_payload <= payload_ch[next_ch];
                    rr_last     <= next_ch;
                    out_busy    <= 1'b1;
                    out_cnt     <= 3'd0;
                end
            end else begin
                m_sys_bus_valid <= 1'b1;
                m_sys_bus_sof   <= (out_cnt == 3'd0);
                m_sys_bus_eof   <= (out_cnt == 3'd7);

                case (out_cnt)
                    3'd0: m_sys_bus_data <= out_payload[255:0];
                    3'd1: m_sys_bus_data <= out_payload[511:256];
                    3'd2: m_sys_bus_data <= out_payload[767:512];
                    3'd3: m_sys_bus_data <= out_payload[1023:768];
                    3'd4: m_sys_bus_data <= out_payload[1279:1024];
                    3'd5: m_sys_bus_data <= out_payload[1535:1280];
                    3'd6: m_sys_bus_data <= out_payload[1791:1536];
                    3'd7: begin
                        // [215:0]=payload tail [247:216]=CRC32 [255:248]=FEC
                        m_sys_bus_data <= {out_fec, out_crc, out_payload[2007:1792]};
                        out_busy <= 1'b0;
                    end
                endcase

                // LT4 fix: explicit reset, no overflow past 7
                if (out_cnt != 3'd7)
                    out_cnt <= out_cnt + 1'b1;
                else
                    out_cnt <= 3'd0;
            end
        end
    end

`ifdef SIMULATION
always @(posedge clk) begin
    if (s_axis_tvalid && ch_busy[ch_sel] && s_axis_tready_ch[ch_sel])
        $error("[link_layer_tx] Beat accepted on ch%0d while ch_busy=1", ch_sel);
end
`endif

endmodule