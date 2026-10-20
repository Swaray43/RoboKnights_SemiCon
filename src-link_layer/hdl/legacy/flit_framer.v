`timescale 1ns/1ps

/*
 * Module: flit_framer
 *
 *
 * Flit layout (MSB-first), unchanged in spirit from the original 128-bit
 * framing, generalized to any width:
 *   [FLIT_WIDTH-1        -: 8]         sync marker
 *   [FLIT_WIDTH-9        -: 3]         reserved
 *   [FLIT_WIDTH-12]                    sof
 *   [FLIT_WIDTH-13]                    eof
 *   [EOFPOS_MSB:EOFPOS_LSB]            eof DW-position index (EOF_POS_W bits)
 *   [.. down to PAYLOAD_W]             reserved header padding
 *   [PAYLOAD_W-1:0]                    payload (NUM_PAYLOAD_DW x 32-bit DWs,
 *                                       DW0 at the MSB end of the payload
 *                                       field down to the last DW at the LSB
 *                                       end; unused trailing DW slots zero)
 *
 * NOTE: FEC_WIDTH must be large enough for fec_encoder_decoder to cover
 * FLIT_WIDTH bits (2^(FEC_WIDTH-1) >= FLIT_WIDTH) -- e.g. FLIT_WIDTH=2048
 * needs FEC_WIDTH >= 12. See fec_encoder_decoder.v.
 *
 * To use this as the FLIT-level framing/FEC sub-block of a wider link
 * layer, instantiate with FLIT_WIDTH set to that layer's real flit width
 * and an adequately sized FEC_WIDTH.
 */
module flit_framer #(
    parameter FLIT_WIDTH = 128,
    parameter FEC_WIDTH  = 8
)(
    input  wire                  clk,
    input  wire                  rst_n,

    // TX Streaming TLP Interface
    input  wire                  tx_tlp_valid,
    input  wire [31:0]           tx_tlp_data,
    input  wire                  tx_tlp_sof,
    input  wire                  tx_tlp_eof,
    output wire                  tx_tlp_ready,

    // Outbound Framed FLIT Output (to Physical Link)
    output reg                   m_flit_tx_valid,
    output reg  [FLIT_WIDTH-1:0] m_flit_tx_data,
    output reg  [FEC_WIDTH-1:0]  m_flit_tx_fec,

    // Inbound Framed FLIT Input (from Physical Link)
    input  wire                  s_flit_rx_valid,
    input  wire [FLIT_WIDTH-1:0] s_flit_rx_data,
    input  wire [FEC_WIDTH-1:0]  s_flit_rx_fec,

    // Decapsulated Streaming TLP Interface
    output reg                   rx_tlp_valid,
    output reg  [31:0]           rx_tlp_data,
    output reg                   rx_tlp_sof,
    output reg                   rx_tlp_eof,
    output wire                  rx_flit_err
);

    // ------------------------------------------------------------------
    // Generic header/payload layout derived from FLIT_WIDTH
    // ------------------------------------------------------------------
    localparam integer HDR_W          = 32;                 // 1 DW of header
    localparam integer PAYLOAD_W      = FLIT_WIDTH - HDR_W;
    localparam integer NUM_PAYLOAD_DW = PAYLOAD_W / 32;
    localparam integer WORD_CNT_W     = (NUM_PAYLOAD_DW <= 1) ? 1 : $clog2(NUM_PAYLOAD_DW);
    localparam integer EOF_POS_W      = WORD_CNT_W;
    localparam integer RESERVED_W     = HDR_W - 8 - 3 - 1 - 1 - EOF_POS_W;

    localparam integer SYNC_LSB   = FLIT_WIDTH - 8;
    localparam integer RSVD1_LSB  = SYNC_LSB - 3;
    localparam integer SOF_BIT    = RSVD1_LSB - 1;
    localparam integer EOF_BIT    = SOF_BIT - 1;
    localparam integer EOFPOS_MSB = EOF_BIT - 1;
    localparam integer EOFPOS_LSB = EOFPOS_MSB - EOF_POS_W + 1;

    // synthesis translate_off
    initial begin
        if ((FLIT_WIDTH % 32) != 0 || FLIT_WIDTH < 64) begin
            $display("ERROR: flit_framer: FLIT_WIDTH=%0d must be a multiple of 32 and >= 64 (32-bit header + >=1 payload DW)", FLIT_WIDTH);
            $finish;
        end
        if (RESERVED_W < 0) begin
            $display("ERROR: flit_framer: FLIT_WIDTH=%0d leaves no room for the fixed header fields (sync/reserved/sof/eof/eof_pos)", FLIT_WIDTH);
            $finish;
        end
    end
    // synthesis translate_on

    assign tx_tlp_ready = 1'b1;

    // Internal Registers for TX Accumulation
    // NUM_PAYLOAD_DW-1 buffered DWs; the final DW of a flit is taken
    // directly from tx_tlp_data at the moment the flit is finalized.
    reg [31:0] tx_buf [0:(NUM_PAYLOAD_DW > 1 ? NUM_PAYLOAD_DW-2 : 0)];
    reg [WORD_CNT_W-1:0] tx_word_cnt;
    reg                  has_sof;

    // Intermediate Signals for FEC Submodule
    reg                   fec_tx_en;
    reg  [FLIT_WIDTH-1:0] fec_tx_in;
    wire [FLIT_WIDTH-1:0] fec_encoded_data;
    wire [FEC_WIDTH-1:0]  fec_encoded_code;

    wire [FLIT_WIDTH-1:0] fec_rx_corr_data;
    wire                  fec_rx_uncorrectable;
    wire                  fec_rx_corrected;

    // SEC-DED FEC Protection Unit Instance
    fec_encoder_decoder #(
        .DATA_WIDTH(FLIT_WIDTH),
        .FEC_WIDTH(FEC_WIDTH)
    ) fec_unit (
        .clk(clk),
        .rst_n(rst_n),
        .tx_val(fec_tx_en),
        .tx_data(fec_tx_in),
        .tx_fec_data(fec_encoded_data),
        .tx_fec_code(fec_encoded_code),
        .rx_val(s_flit_rx_valid),
        .rx_fec_data(s_flit_rx_data),
        .rx_fec_code(s_flit_rx_fec),
        .rx_corr_data(fec_rx_corr_data),
        .rx_uncorrectable_err(fec_rx_uncorrectable),
        .rx_corrected_err(fec_rx_corrected)
    );

    assign rx_flit_err = fec_rx_uncorrectable;

    // ------------------------------------------------------------------
    // Generic payload assembly: place the buffered DWs (indices
    // 0..tx_word_cnt-1) and the incoming DW (at index tx_word_cnt) from
    // the MSB end of the payload field downward; any remaining trailing
    // DW slots stay zero. This is the width-generic replacement for the
    // old fixed {DW0,DW1,DW2} concatenation.
    // ------------------------------------------------------------------
    reg [PAYLOAD_W-1:0] payload_next;
    integer j;
    always @(*) begin
        payload_next = {PAYLOAD_W{1'b0}};
        for (j = 0; j < NUM_PAYLOAD_DW; j = j + 1) begin
            if (j < tx_word_cnt)
                payload_next[(NUM_PAYLOAD_DW-1-j)*32 +: 32] = tx_buf[j];
            else if (j == tx_word_cnt)
                payload_next[(NUM_PAYLOAD_DW-1-j)*32 +: 32] = tx_tlp_data;
            // else: trailing unused DW slot, stays zero
        end
    end

    // TX FLIT Framing Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_word_cnt     <= {WORD_CNT_W{1'b0}};
            has_sof         <= 1'b0;
            fec_tx_en       <= 1'b0;
            fec_tx_in       <= {FLIT_WIDTH{1'b0}};
            m_flit_tx_valid <= 1'b0;
            m_flit_tx_data  <= {FLIT_WIDTH{1'b0}};
            m_flit_tx_fec   <= {FEC_WIDTH{1'b0}};
        end else begin
            fec_tx_en <= 1'b0;

            if (tx_tlp_valid && tx_tlp_ready) begin
                if (tx_tlp_sof) has_sof <= 1'b1;

                if (tx_tlp_eof || (tx_word_cnt == NUM_PAYLOAD_DW-1)) begin
                    // Flit boundary: either the TLP ends here, or the
                    // payload is full and must be flushed regardless.
                    fec_tx_en <= 1'b1;
                    fec_tx_in <= {
                        8'h5A,                     // Sync Marker
                        3'b000,                     // Reserved
                        has_sof | tx_tlp_sof,       // SOF
                        tx_tlp_eof,                 // EOF
                        tx_word_cnt,                // EOF/last-DW position index
                        {RESERVED_W{1'b0}},          // Reserved header padding
                        payload_next                // Packed payload DWs
                    };
                    tx_word_cnt <= {WORD_CNT_W{1'b0}};
                    has_sof     <= 1'b0;
                end else begin
                    tx_buf[tx_word_cnt] <= tx_tlp_data;
                    tx_word_cnt         <= tx_word_cnt + 1'b1;
                end
            end

            // Drive Encoded Output Pipeline
            m_flit_tx_valid <= fec_tx_en;
            m_flit_tx_data  <= fec_encoded_data;
            m_flit_tx_fec   <= fec_encoded_code;
        end
    end

    // ------------------------------------------------------------------
    // RX FLIT Deframing Logic (generic width)
    // ------------------------------------------------------------------
    reg [WORD_CNT_W-1:0] rx_word_cnt;
    reg [FLIT_WIDTH-1:0] rx_flit_reg;
    reg                  rx_active;

    wire rx_hdr_eof                       = rx_flit_reg[EOF_BIT];
    wire [EOF_POS_W-1:0] rx_hdr_eof_pos   = rx_flit_reg[EOFPOS_MSB:EOFPOS_LSB];
    wire rx_is_eof_word                   = rx_hdr_eof && (rx_hdr_eof_pos == rx_word_cnt);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_tlp_valid <= 1'b0;
            rx_tlp_data  <= 32'd0;
            rx_tlp_sof   <= 1'b0;
            rx_tlp_eof   <= 1'b0;
            rx_word_cnt  <= {WORD_CNT_W{1'b0}};
            rx_active    <= 1'b0;
            rx_flit_reg  <= {FLIT_WIDTH{1'b0}};
        end else begin
            if (s_flit_rx_valid && !fec_rx_uncorrectable) begin
                rx_flit_reg <= fec_rx_corr_data;
                rx_active   <= 1'b1;
                rx_word_cnt <= {WORD_CNT_W{1'b0}};
            end

            if (rx_active) begin
                rx_tlp_valid <= 1'b1;
                rx_tlp_data  <= rx_flit_reg[(NUM_PAYLOAD_DW-1-rx_word_cnt)*32 +: 32];
                rx_tlp_sof   <= (rx_word_cnt == {WORD_CNT_W{1'b0}}) ? rx_flit_reg[SOF_BIT] : 1'b0;
                rx_tlp_eof   <= rx_is_eof_word;

                if (rx_is_eof_word || (rx_word_cnt == NUM_PAYLOAD_DW-1)) begin
                    // Either the signaled EOF landed on this DW, or this is
                    // simply the last DW slot in the flit (a full flit that
                    // continues the TLP with no EOF yet) -- either way this
                    // flit is done; the DW is still valid data either way.
                    rx_active <= 1'b0;
                end else begin
                    rx_word_cnt <= rx_word_cnt + 1'b1;
                end
            end else begin
                rx_tlp_valid <= 1'b0;
                rx_tlp_sof   <= 1'b0;
                rx_tlp_eof   <= 1'b0;
            end
        end
    end

endmodule