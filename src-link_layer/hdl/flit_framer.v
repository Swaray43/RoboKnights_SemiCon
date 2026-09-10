`timescale 1ns/1ps

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

    assign tx_tlp_ready = 1'b1;

    // Internal Registers for TX Accumulation
    reg [31:0] tx_buf [0:1];
    reg [1:0]  tx_word_cnt;
    reg        has_sof;

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

    // TX FLIT Framing Logic
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            tx_word_cnt     <= 2'd0;
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

                if (tx_word_cnt == 2'd0) begin
                    if (tx_tlp_eof) begin
                        // Single-DW TLP Packet Boundary
                        fec_tx_en <= 1'b1;
                        fec_tx_in <= {
                            8'h5A,                // Sync Marker [127:120]
                            3'b000,               // Reserved [119:117]
                            has_sof | tx_tlp_sof, // SOF [116]
                            1'b1,                 // EOF [115]
                            2'd0,                 // EOF Position Index (DW0) [114:113]
                            17'd0,                // Reserved Header Bits [112:96]
                            tx_tlp_data,          // DW0 Payload [95:64]
                            32'd0,                // DW1 Padding [63:32]
                            32'd0                 // DW2 Padding [31:0]
                        };
                        tx_word_cnt <= 2'd0;
                        has_sof     <= 1'b0;
                    end else begin
                        tx_buf[0]   <= tx_tlp_data;
                        tx_word_cnt <= 2'd1;
                    end
                end else if (tx_word_cnt == 2'd1) begin
                    if (tx_tlp_eof) begin
                        // 2-DW TLP Packet Boundary
                        fec_tx_en <= 1'b1;
                        fec_tx_in <= {
                            8'h5A,
                            3'b000,
                            has_sof | tx_tlp_sof,
                            1'b1,
                            2'd1,                 // EOF Position Index (DW1) [114:113]
                            17'd0,
                            tx_buf[0],            // DW0 Payload [95:64]
                            tx_tlp_data,          // DW1 Payload [63:32]
                            32'd0                 // DW2 Padding [31:0]
                        };
                        tx_word_cnt <= 2'd0;
                        has_sof     <= 1'b0;
                    end else begin
                        tx_buf[1]   <= tx_tlp_data;
                        tx_word_cnt <= 2'd2;
                    end
                end else begin
                    // Full 3-DW FLIT Frame Boundary
                    fec_tx_en <= 1'b1;
                    fec_tx_in <= {
                        8'h5A,
                        3'b000,
                        has_sof | tx_tlp_sof,
                        tx_tlp_eof,
                        2'd2,                     // EOF Position Index (DW2) [114:113]
                        17'd0,
                        tx_buf[0],                // DW0 Payload [95:64]
                        tx_buf[1],                // DW1 Payload [63:32]
                        tx_tlp_data               // DW2 Payload [31:0]
                    };
                    tx_word_cnt <= 2'd0;
                    has_sof     <= 1'b0;
                end
            end

            // Drive Encoded Output Pipeline
            m_flit_tx_valid <= fec_tx_en;
            m_flit_tx_data  <= fec_encoded_data;
            m_flit_tx_fec   <= fec_encoded_code;
        end
    end

    // RX FLIT Deframing Logic
    reg [1:0]   rx_word_cnt;
    reg [127:0] rx_flit_reg;
    reg         rx_active;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            rx_tlp_valid <= 1'b0;
            rx_tlp_data  <= 32'd0;
            rx_tlp_sof   <= 1'b0;
            rx_tlp_eof   <= 1'b0;
            rx_word_cnt  <= 2'd0;
            rx_active    <= 1'b0;
            rx_flit_reg  <= 128'd0;
        end else begin
            if (s_flit_rx_valid && !fec_rx_uncorrectable) begin
                rx_flit_reg <= fec_rx_corr_data;
                rx_active   <= 1'b1;
                rx_word_cnt <= 2'd0;
            end

            if (rx_active) begin
                case (rx_word_cnt)
                    2'd0: begin
                        rx_tlp_valid <= 1'b1;
                        rx_tlp_data  <= rx_flit_reg[95:64];
                        rx_tlp_sof   <= rx_flit_reg[116];
                        rx_tlp_eof   <= rx_flit_reg[115] && (rx_flit_reg[114:113] == 2'd0);
                        
                        if (rx_flit_reg[115] && (rx_flit_reg[114:113] == 2'd0)) begin
                            rx_active <= 1'b0;
                        end else begin
                            rx_word_cnt <= 2'd1;
                        end
                    end

                    2'd1: begin
                        rx_tlp_valid <= 1'b1;
                        rx_tlp_data  <= rx_flit_reg[63:32];
                        rx_tlp_sof   <= 1'b0;
                        rx_tlp_eof   <= rx_flit_reg[115] && (rx_flit_reg[114:113] == 2'd1);

                        if (rx_flit_reg[115] && (rx_flit_reg[114:113] == 2'd1)) begin
                            rx_active <= 1'b0;
                        end else begin
                            rx_word_cnt <= 2'd2;
                        end
                    end

                    2'd2: begin
                        rx_tlp_valid <= 1'b1;
                        rx_tlp_data  <= rx_flit_reg[31:0];
                        rx_tlp_sof   <= 1'b0;
                        rx_tlp_eof   <= rx_flit_reg[115] && (rx_flit_reg[114:113] == 2'd2);
                        rx_active    <= 1'b0;
                    end

                    default: begin
                        rx_tlp_valid <= 1'b0;
                        rx_active    <= 1'b0;
                    end
                endcase
            end else begin
                rx_tlp_valid <= 1'b0;
                rx_tlp_sof   <= 1'b0;
                rx_tlp_eof   <= 1'b0;
            end
        end
    end

endmodule