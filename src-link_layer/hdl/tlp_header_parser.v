`timescale 1ns/1ps

// ============================================================
// Gap fix H1 (earlier session):
//   parsed_tag was assigned only rx_tlp_data[15:8] (8 bits) at
//   header phase 1, zero-extended into the 10-bit parsed_tag reg.
//   When Extended Tag Field is enabled, PCIe TLPs carry a 10-bit
//   tag: the low 8 bits sit in DW1[15:8] as before, but bits 8 and
//   9 (T8/T9) live in DW0 -- bit 19 (T8) and bit 23 (T9) -- not in
//   DW1 at all. Those two bits are now latched at header phase 0
//   (same cycle DW0 is captured) and combined with DW1[15:8] when
//   parsed_tag is written at phase 1.
//
// Gap fix H2 (this session, found while building link_layer_top):
//   A 4DW-format header is 4 DWs long (DW0 hdr, DW1 ReqID/Tag/BE,
//   DW2 addr[63:32], DW3 addr[31:2]) -- matching what both
//   tlp_packetizer.v and tx_engine.v actually emit. The old FSM only
//   had ONE phase between SOF and "payload" (header_phase 1 -> 2 ->
//   payload), i.e. budget for just 2 post-SOF words. It used word1
//   (ReqID/Tag/BE) as if it were address data (parsed_addr[63:32]
//   for 4DW, or the whole 32-bit address for 3DW) and then used
//   word2 (the TX side's real addr[63:32]) as parsed_addr[31:0] --
//   both header words after SOF were captured into the wrong
//   fields, and for 4DW headers the real DW3 (addr[31:2]) was never
//   consumed at all; it fell straight into the payload phase and
//   got parsed as the first payload DW. Any 64-bit-addressed MWr
//   decoded to a corrupted address and a payload shifted by one DW.
//
//   Fixed with a proper 3-state header walk after SOF: phase 1
//   captures tag/cpl_status/lower_addr from word1 (ReqID/Tag/BE --
//   this part was already correct) and no longer touches
//   parsed_addr; phase 2 takes word2 as addr[63:32] (4DW) or as the
//   complete 32-bit address plus the phase-3 skip (3DW); phase 3
//   (4DW only) takes word3 as addr[31:2]. Only then does the walk
//   return to idle/payload.
//
// Gap fix H3 (this session, also found while building link_layer_top):
//   parsed_rx_valid/sof/eof/data used to be a straight 1-cycle-delayed
//   passthrough of EVERY input word, header DWs included, with
//   parsed_rx_sof mirroring the TLP's own rx_tlp_sof (the header's
//   first word). But pcie_rx_engine (see its header comment and
//   tb_pcie_rx_engine.v's stimulus) expects tlp_rx_data to carry
//   PAYLOAD words only, with tlp_rx_sof marking the first payload
//   word -- and tlp_type/tlp_tag/tlp_addr/etc. already stable and
//   held for the whole payload burst, since that's the only way
//   pcie_address_translator's `tlp_rx_valid && tlp_rx_sof && is_mwr`
//   trigger samples a valid, fully-latched tlp_addr. Fed the old
//   passthrough, every header DW -- including the bogus one this
//   fix's own H2 part would have produced -- would have gone out as
//   if it were payload, and the real sof/metadata timing would never
//   line up with what pcie_rx_engine actually samples.
//
//   Fixed by gating parsed_rx_valid/data to the post-header portion
//   of the TLP only (header_phase==0 && in_payload), with a one-shot
//   payload_pending_sof flag raised the cycle the header walk
//   finishes so the first real payload word -- and only that word --
//   carries parsed_rx_sof.
//
// Fix P2-5 / formal F5 (Phase 2, adapted to this file's H1/H2/H3
// structure -- this file has no parsed_hdr_done output and no is_cpl
// register, so this is a re-derivation of the upstream fix against
// this baseline, not a literal reapplication of that diff):
//   F5: the parser exported Type (DW0[28:24]) but not Fmt, so
//       downstream could not tell a Memory Read (Fmt 000/001, Type
//       0_0000) from a Memory Write (Fmt 010/011, same Type):
//       pcie_rx_engine decoded every MRd as an MWr. New output
//       parsed_has_data = Fmt[1] (DW0 bit 30), latched at SOF
//       alongside parsed_type.
//   F6 (CplLk/CplDLk misrouted) does not apply to this file as
//       written -- this version has no parser-side is_cpl latch;
//       that classification lives in pcie_rx_engine.v's is_cpld,
//       fixed there instead (see pcie_rx_engine.v).
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
    output reg  [PCIE_ADDR_W-1:0]  parsed_addr,
    output reg                     parsed_has_data    // P2-5/F5: Fmt[1] (DW0[30]) -- MWr carries data, MRd doesn't
);

    reg [1:0] header_phase;
    reg       is_4dw_header;
    reg       in_payload;           // header walk for this TLP is done
    reg       payload_pending_sof;  // next payload word gets parsed_rx_sof

    // Gap fix H1: Tag[9] (DW0 bit 23) and Tag[8] (DW0 bit 19),
    // latched at phase 0 alongside the rest of DW0.
    reg [1:0] tag_ext;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            parsed_rx_valid     <= 0;
            parsed_rx_sof       <= 0;
            parsed_rx_eof       <= 0;
            parsed_rx_data      <= 0;
            parsed_type         <= 0;
            parsed_length_dw    <= 0;
            parsed_tag          <= 0;
            parsed_lower_addr   <= 0;
            parsed_cpl_status   <= 0;
            parsed_addr         <= 0;
            parsed_has_data     <= 0;
            header_phase        <= 0;
            is_4dw_header       <= 0;
            tag_ext             <= 0;
            in_payload          <= 0;
            payload_pending_sof <= 0;
        end else begin
            // Default: no payload word out this cycle unless the
            // in_payload branch below says otherwise.
            parsed_rx_valid <= 1'b0;
            parsed_rx_sof   <= 1'b0;
            parsed_rx_eof   <= 1'b0;

            if (rx_tlp_valid) begin
                if (rx_tlp_sof) begin
                    header_phase     <= 2'd1;
                    in_payload       <= 1'b0;
                    parsed_type      <= rx_tlp_data[28:24];
                    is_4dw_header    <= rx_tlp_data[29]; // Format bit indicates 4DW vs 3DW
                    parsed_has_data  <= rx_tlp_data[30]; // P2-5/F5: Fmt[1] -- MWr(1) vs MRd(0)
                    parsed_length_dw <= rx_tlp_data[9:0];
                    tag_ext          <= {rx_tlp_data[23], rx_tlp_data[19]}; // T9, T8
                end else if (header_phase == 2'd1) begin
                    // word1: ReqID[31:16] / Tag[15:8] / LastBE[7:4] / FirstBE[3:0]
                    // -- never an address word, for either 3DW or 4DW format.
                    parsed_tag        <= {tag_ext, rx_tlp_data[15:8]}; // {T9,T8,Tag[7:0]} -> 10-bit tag
                    parsed_cpl_status <= rx_tlp_data[15:13];
                    parsed_lower_addr <= rx_tlp_data[6:0];
                    header_phase      <= 2'd2;
                end else if (header_phase == 2'd2) begin
                    if (is_4dw_header) begin
                        // word2: addr[63:32]; word3 (addr[31:2]) still to come.
                        parsed_addr[63:32] <= rx_tlp_data;
                        header_phase       <= 2'd3;
                    end else begin
                        // 3DW: word2 is the complete 32-bit address; header done.
                        parsed_addr         <= {32'd0, rx_tlp_data[31:2], 2'b00};
                        header_phase        <= 2'd0;
                        in_payload          <= 1'b1;
                        payload_pending_sof <= 1'b1;
                    end
                end else if (header_phase == 2'd3) begin
                    // word3 (4DW only): addr[31:2]; header done.
                    parsed_addr[31:0]   <= {rx_tlp_data[31:2], 2'b00};
                    header_phase        <= 2'd0;
                    in_payload          <= 1'b1;
                    payload_pending_sof <= 1'b1;
                end else if (in_payload) begin
                    // Genuine payload word (header_phase idle, header
                    // walk for this TLP already finished) -- forward
                    // it, with sof on the first such word only.
                    parsed_rx_valid     <= 1'b1;
                    parsed_rx_sof       <= payload_pending_sof;
                    parsed_rx_eof       <= rx_tlp_eof;
                    parsed_rx_data      <= rx_tlp_data;
                    payload_pending_sof <= 1'b0;
                end

                if (rx_tlp_eof) begin
                    in_payload <= 1'b0;
                    if (header_phase == 2'd0) header_phase <= 2'd0; // stays idle
                end
            end else begin
                header_phase <= 0;
            end
        end
    end
endmodule