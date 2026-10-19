`timescale 1ns/1ps

// ============================================================
// Module : tlp_packetizer
// Layer  : Link layer (Engineer B -- Module 5 of 9)
//
// Change (bridge review follow-up, "P5"):
//   DW0 was built from 31 bits ({1'b0, Fmt, Type, 1'b0, TC, 4'b0, Attr,
//   2'b0, Length}), so Verilog zero-extended it and Fmt landed in
//   DW0[30:28] instead of DW0[31:29]; Type, TC etc. were all shifted down
//   by one bit. Any receiver decoding the header (including the new
//   rx_tlp_delimiter, which sizes TLPs from Fmt/Length) saw garbage.
//   DW0 is now the PCIe layout, exactly 32 bits:
//     [31:29] Fmt  [28:24] Type  [23] T9  [22:20] TC  [19] T8  [18] Attr2
//     [17] LN  [16] TH  [15] TD  [14] EP  [13:12] Attr  [11:10] AT
//     [9:0] Length
// ============================================================

module tlp_packetizer #(
    parameter MAX_PAYLOAD_DW = 10'd64
)(
    input  wire        clk,
    input  wire        rst_n,

    // Descriptor interface (from ar_request_splitter)
    input  wire        req_valid,
    input  wire        req_is_write,
    input  wire [63:0] req_addr,
    input  wire [9:0]  req_length,
    input  wire [15:0] req_rid,
    input  wire [7:0]  req_tag,
    output reg         req_ready,

    // Write data stream -- MWr only
    input  wire [31:0] wdata_tdata,
    input  wire        wdata_tvalid,
    output reg         wdata_tready,

    // Outbound TLP word stream
    output reg         tlp_out_valid,
    output reg  [31:0] tlp_out_data,
    output reg         tlp_out_sof,
    output reg         tlp_out_eof,
    input  wire        tlp_out_ready
);

    // TLP format constants
    localparam FMT_4DW_NODATA = 3'b001;
    localparam FMT_4DW_DATA   = 3'b011;
    localparam TYPE_MEM_REQ   = 5'b00000;

    // FSM states
    localparam S_IDLE = 3'd0;
    localparam S_EVAL = 3'd1;
    localparam S_HDR0 = 3'd2;
    localparam S_HDR1 = 3'd3;
    localparam S_HDR2 = 3'd4;
    localparam S_HDR3 = 3'd5;
    localparam S_DATA = 3'd6;
    localparam S_NEXT = 3'd7;

    reg [2:0]  state;
    reg [63:0] curr_addr;
    reg [9:0]  rem_length;
    reg [9:0]  chunk_len;
    reg        curr_is_write;
    reg [7:0]  curr_tag;
    reg [15:0] curr_rid;
    reg [9:0]  data_cnt;

    // PCIe spec: LastBE = 4'b0000 when Length == 1 DW
    wire [3:0] last_be  = (chunk_len == 10'd1) ? 4'b0000 : 4'b1111;
    wire [2:0] tlp_fmt  = curr_is_write ? FMT_4DW_DATA : FMT_4DW_NODATA;

    // FirstBE for non-DW-aligned start addresses
    function [3:0] first_be_from_offset;
        input [1:0] byte_offset;
        begin
            case (byte_offset)
                2'b00: first_be_from_offset = 4'b1111;
                2'b01: first_be_from_offset = 4'b1110;
                2'b10: first_be_from_offset = 4'b1100;
                2'b11: first_be_from_offset = 4'b1000;
            endcase
        end
    endfunction

    wire [3:0] first_be = first_be_from_offset(curr_addr[1:0]);

    // DW0, PCIe bit layout (32 bits exactly -- see header note)
    wire [31:0] hdr_dw0 = {tlp_fmt,        // [31:29] Fmt
                           TYPE_MEM_REQ,   // [28:24] Type
                           1'b0,           // [23]    T9
                           3'b000,         // [22:20] TC
                           1'b0,           // [19]    T8
                           1'b0,           // [18]    Attr[2]
                           1'b0,           // [17]    LN
                           1'b0,           // [16]    TH
                           1'b0,           // [15]    TD
                           1'b0,           // [14]    EP
                           2'b00,          // [13:12] Attr[1:0]
                           2'b00,          // [11:10] AT
                           chunk_len};     // [9:0]   Length

    // -------------------------------------------------------
    // Combinational output mux
    // -------------------------------------------------------
    always @(*) begin
        tlp_out_valid = 1'b0;
        tlp_out_data  = 32'd0;
        tlp_out_sof   = 1'b0;
        tlp_out_eof   = 1'b0;
        wdata_tready  = 1'b0;

        case (state)
            S_HDR0: begin
                tlp_out_valid = 1'b1;
                tlp_out_sof   = 1'b1;
                tlp_out_data  = hdr_dw0;
            end
            S_HDR1: begin
                tlp_out_valid = 1'b1;
                // DW1: {ReqID[15:0], Tag[7:0], LastBE[3:0], FirstBE[3:0]}
                tlp_out_data  = {curr_rid, curr_tag, last_be, first_be};
            end
            S_HDR2: begin
                tlp_out_valid = 1'b1;
                tlp_out_data  = curr_addr[63:32];
            end
            S_HDR3: begin
                tlp_out_valid = 1'b1;
                tlp_out_eof   = !curr_is_write;
                tlp_out_data  = {curr_addr[31:2], 2'b00};
            end
            S_DATA: begin
                tlp_out_valid = wdata_tvalid;
                tlp_out_eof   = (data_cnt == 10'd1) && wdata_tvalid;
                tlp_out_data  = wdata_tdata;
                wdata_tready  = tlp_out_ready;
            end
            default: ;
        endcase
    end

    // -------------------------------------------------------
    // FSM
    // -------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state         <= S_IDLE;
            req_ready     <= 1'b1;
            curr_addr     <= 64'd0;
            rem_length    <= 10'd0;
            chunk_len     <= 10'd0;
            curr_is_write <= 1'b0;
            curr_tag      <= 8'd0;
            curr_rid      <= 16'd0;
            data_cnt      <= 10'd0;
        end else begin
            case (state)
                S_IDLE: begin
                    req_ready <= 1'b1;
                    if (req_valid && req_ready) begin
                        curr_addr    <= req_addr;
                        rem_length   <= req_length;
                        curr_is_write<= req_is_write;
                        curr_tag     <= req_tag;
                        curr_rid     <= req_rid;
                        req_ready    <= 1'b0;
                        state        <= S_EVAL;
                    end
                end

                S_EVAL: begin
                    chunk_len <= (rem_length > MAX_PAYLOAD_DW) ?
                                  MAX_PAYLOAD_DW : rem_length;
                    state <= S_HDR0;
                end

                S_HDR0: if (tlp_out_ready) state <= S_HDR1;
                S_HDR1: if (tlp_out_ready) state <= S_HDR2;
                S_HDR2: if (tlp_out_ready) state <= S_HDR3;

                S_HDR3: begin
                    if (tlp_out_ready) begin
                        if (curr_is_write) begin
                            data_cnt <= chunk_len;
                            state    <= S_DATA;
                        end else begin
                            state <= S_NEXT;
                        end
                    end
                end

                S_DATA: begin
                    if (wdata_tvalid && tlp_out_ready) begin
                        data_cnt <= data_cnt - 10'd1;
                        if (data_cnt == 10'd1)
                            state <= S_NEXT;
                    end
                end

                S_NEXT: begin
                    rem_length <= rem_length - chunk_len;
                    curr_addr  <= curr_addr + {chunk_len, 2'b00};
                    if (rem_length == chunk_len) begin
                        req_ready <= 1'b1;
                        state     <= S_IDLE;
                    end else begin
                        state <= S_EVAL;
                    end
                end

                default: state <= S_IDLE;
            endcase
        end
    end

`ifdef SIMULATION
    always @(posedge clk) begin
        if (tlp_out_sof && !tlp_out_valid)
            $error("[tlp_packetizer] SOF without valid");
        if (tlp_out_eof && !tlp_out_valid)
            $error("[tlp_packetizer] EOF without valid");
        if (wdata_tready && state !== S_DATA)
            $error("[tlp_packetizer] wdata_tready outside S_DATA");
    end
`endif

endmodule
