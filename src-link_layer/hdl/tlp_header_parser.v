`timescale 1ns/1ps

// ============================================================
// Gap fix H1 (this session):
//   parsed_tag was assigned only rx_tlp_data[15:8] (8 bits) at
//   header phase 1, zero-extended into the 10-bit parsed_tag reg.
//   When Extended Tag Field is enabled, PCIe TLPs carry a 10-bit
//   tag: the low 8 bits sit in DW1[15:8] as before, but bits 8 and
//   9 (T8/T9) live in DW0 -- bit 19 (T8) and bit 23 (T9) -- not in
//   DW1 at all. Those two bits are now latched at header phase 0
//   (same cycle DW0 is captured) and combined with DW1[15:8] when
//   parsed_tag is written at phase 1.
// ============================================================

module tlp_header_parser #(
    parameter TAG_WIDTH   = 10,
    parameter PCIE_ADDR_W = 64
)(
    input  wire                    clk,
    input  wire                    rst_n,

    // Raw input from FLIT Framer (RX decapsulation)
    input  wire                    rx_tlp_valid,
    input  wire [31:0]             rx_tlp_data,
    input  wire                    rx_tlp_sof,
    input  wire                    rx_tlp_eof,
    
    // Parsed outputs to RX Engine
    output reg                     parsed_rx_valid,
    output reg                     parsed_rx_sof,
    output reg                     parsed_rx_eof,
    output reg  [31:0]             parsed_rx_data,
    
    output reg  [4:0]              parsed_type,
    output reg  [9:0]              parsed_length_dw,
    output reg  [TAG_WIDTH-1:0]    parsed_tag,
    output reg  [6:0]              parsed_lower_addr,
    output reg  [2:0]              parsed_cpl_status,
    output reg  [PCIE_ADDR_W-1:0]  parsed_addr
);

    reg [1:0] header_phase;
    reg       is_4dw_header;

    // Gap fix H1: Tag[9] (DW0 bit 23) and Tag[8] (DW0 bit 19),
    // latched at phase 0 alongside the rest of DW0.
    reg [1:0] tag_ext;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parsed_rx_valid   <= 0;
            parsed_rx_sof     <= 0;
            parsed_rx_eof     <= 0;
            parsed_rx_data    <= 0;
            parsed_type       <= 0;
            parsed_length_dw  <= 0;
            parsed_tag        <= 0;
            parsed_lower_addr <= 0;
            parsed_cpl_status <= 0;
            parsed_addr       <= 0;
            header_phase      <= 0;
            is_4dw_header     <= 0;
            tag_ext           <= 0;
        end else begin
            parsed_rx_valid <= rx_tlp_valid;
            parsed_rx_sof   <= rx_tlp_sof;
            parsed_rx_eof   <= rx_tlp_eof;
            parsed_rx_data  <= rx_tlp_data;
            
            if (rx_tlp_valid) begin
                if (rx_tlp_sof) begin
                    header_phase     <= 2'd1;
                    parsed_type      <= rx_tlp_data[28:24];
                    is_4dw_header    <= rx_tlp_data[29]; // Format bit indicates 4DW vs 3DW
                    parsed_length_dw <= rx_tlp_data[9:0];
                    tag_ext          <= {rx_tlp_data[23], rx_tlp_data[19]}; // T9, T8
                end else if (header_phase == 2'd1) begin
                    header_phase      <= 2'd2;
                    parsed_tag        <= {tag_ext, rx_tlp_data[15:8]}; // {T9,T8,Tag[7:0]} -> 10-bit tag
                    parsed_cpl_status <= rx_tlp_data[15:13];
                    parsed_lower_addr <= rx_tlp_data[6:0];
                    if (!is_4dw_header) begin
                        parsed_addr <= {32'd0, rx_tlp_data[31:2], 2'b00};
                    end else begin
                        parsed_addr[63:32] <= rx_tlp_data;
                    end
                end else if (header_phase == 2'd2) begin
                    if (is_4dw_header) begin
                        parsed_addr[31:0] <= {rx_tlp_data[31:2], 2'b00};
                    end
                    header_phase <= 2'd3; // Payload phase
                end
            end else begin
                header_phase <= 0;
            end
        end
    end
endmodule