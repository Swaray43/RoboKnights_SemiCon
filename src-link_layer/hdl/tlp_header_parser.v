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
// ------------------------------------------------------------
// Fix P1 (Phase 2 review): address/tag DW indexing + header-done strobe
//   PCIe request header: DW0 = Fmt/Type/Len, DW1 = ReqID/Tag/BE,
//   DW2 = Addr (3DW) or Addr[63:32] (4DW), DW3 = Addr[31:0] (4DW).
//   The old code took the address from DW1/DW2 (one DW early), so every
//   MWr decoded to a garbage address. Completions carry Tag and Lower
//   Address in DW2, not DW1 -- also fixed.
//   parsed_hdr_done pulses (aligned with the other parsed_* outputs) on
//   the cycle parsed_addr becomes valid: last header DW of the TLP.
// ------------------------------------------------------------
module tlp_header_parser #(
    parameter TAG_WIDTH   = 10,
    parameter PCIE_ADDR_W = 64
)(
    input  wire                    clk,
    input  wire                    rst_n,
    // Raw input from the RX unpacker
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
    output reg  [PCIE_ADDR_W-1:0]  parsed_addr,
    output reg                     parsed_hdr_done
);
    reg [2:0] header_phase;      // 1 = next is DW1, 2 = DW2, 3 = DW3, 4 = payload
    reg       is_4dw_header;
    reg       is_cpl;
    reg [1:0] tag_ext;           // {T9, T8} from DW0 bits 23 / 19

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
            parsed_hdr_done   <= 0;
            header_phase      <= 0;
            is_4dw_header     <= 0;
            is_cpl            <= 0;
            tag_ext           <= 0;
        end else begin
            parsed_rx_valid <= rx_tlp_valid;
            parsed_rx_sof   <= rx_tlp_sof;
            parsed_rx_eof   <= rx_tlp_eof;
            parsed_rx_data  <= rx_tlp_data;
            parsed_hdr_done <= 1'b0;
            if (rx_tlp_valid) begin
                if (rx_tlp_sof) begin                       // DW0
                    header_phase     <= 3'd1;
                    parsed_type      <= rx_tlp_data[28:24];
                    is_4dw_header    <= rx_tlp_data[29];
                    is_cpl           <= (rx_tlp_data[28:24] == 5'b01010);
                    parsed_length_dw <= rx_tlp_data[9:0];
                    tag_ext          <= {rx_tlp_data[23], rx_tlp_data[19]};
                end else if (header_phase == 3'd1) begin    // DW1
                    header_phase      <= 3'd2;
                    parsed_cpl_status <= rx_tlp_data[15:13];
                    if (!is_cpl) parsed_tag <= {tag_ext, rx_tlp_data[15:8]};
                end else if (header_phase == 3'd2) begin    // DW2
                    if (is_cpl) begin
                        parsed_tag        <= {tag_ext, rx_tlp_data[15:8]};
                        parsed_lower_addr <= rx_tlp_data[6:0];
                        parsed_hdr_done   <= 1'b1;
                        header_phase      <= 3'd4;
                    end else if (!is_4dw_header) begin
                        parsed_addr       <= {{(PCIE_ADDR_W-32){1'b0}}, rx_tlp_data[31:2], 2'b00};
                        parsed_lower_addr <= {rx_tlp_data[6:2], 2'b00};
                        parsed_hdr_done   <= 1'b1;
                        header_phase      <= 3'd4;
                    end else begin
                        parsed_addr[63:32] <= rx_tlp_data;
                        header_phase       <= 3'd3;
                    end
                end else if (header_phase == 3'd3) begin    // DW3 (4DW request)
                    parsed_addr[31:0] <= {rx_tlp_data[31:2], 2'b00};
                    parsed_lower_addr <= {rx_tlp_data[6:2], 2'b00};
                    parsed_hdr_done   <= 1'b1;
                    header_phase      <= 3'd4;
                end
                if (rx_tlp_eof && !rx_tlp_sof) header_phase <= 3'd0;
            end
            // no valid: hold header_phase (a gap between beats must not
            // restart the header count; the next SOF always does)
        end
    end
endmodule
