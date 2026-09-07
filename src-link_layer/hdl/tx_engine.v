`timescale 1ns/1ps

module tx_engine #(
    parameter MAX_PAYLOAD_SIZE_DW = 4,
    parameter [15:0] REQ_ID       = 16'h0200
)(
    input  wire        clk,
    input  wire        rst_n,

    // Posted Request Interface
    input  wire        p_req_valid,
    output reg         p_req_ready,
    input  wire [63:0] p_req_addr,
    input  wire [9:0]  p_req_len,
    input  wire [9:0]  p_req_tag,
    input  wire [3:0]  p_req_fbe,
    input  wire [3:0]  p_req_lbe,
    input  wire [31:0] p_s_tdata,
    input  wire        p_s_tvalid,
    output reg         p_s_tready,

    // Non-Posted Request Interface
    input  wire        np_req_valid,
    output reg         np_req_ready,
    input  wire [63:0] np_req_addr,
    input  wire [9:0]  np_req_len,
    input  wire [9:0]  np_req_tag,
    input  wire [3:0]  np_req_fbe,
    input  wire [3:0]  np_req_lbe,

    // UpdateFC Interface
    input  wire        update_fc_valid,
    input  wire [1:0]  update_fc_type,
    input  wire [7:0]  update_fc_hdr_limit,
    input  wire [11:0] update_fc_data_limit,

    // Outbound TLP Stream
    output reg  [31:0] m_tdata,
    output reg         m_tvalid,
    input  wire        m_tready,
    output reg         m_tlast
);

    // FSM States
    localparam STATE_IDLE = 3'd0;
    localparam STATE_HDR0 = 3'd1;
    localparam STATE_HDR1 = 3'd2;
    localparam STATE_HDR2 = 3'd3;
    localparam STATE_HDR3 = 3'd4;
    localparam STATE_DATA = 3'd5;
    localparam STATE_DONE = 3'd6;

    reg [2:0] state;

    // Credit Management Registers
    reg [7:0]  p_hdr_limit;
    reg [11:0] p_data_limit;
    reg [7:0]  p_hdr_consumed;
    reg [11:0] p_data_consumed;

    reg [7:0]  np_hdr_limit;
    reg [11:0] np_data_limit;
    reg [7:0]  np_hdr_consumed;
    reg [11:0] np_data_consumed;

    // Internal Registers for Active Packet
    reg        is_posted;
    reg [63:0] active_addr;
    reg [9:0]  active_len;
    reg [9:0]  active_tag;
    reg [3:0]  active_fbe;
    reg [3:0]  active_lbe;
    reg [9:0]  data_cnt;

    // Address width check: True if top 32 bits are non-zero (4DW Header)
    wire is_64bit = (active_addr[63:32] != 32'd0);

    // ------------------------------------------------------------------------
    // Credit Update Accounting
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            p_hdr_limit   <= 8'd0;
            p_data_limit  <= 12'd0;
            np_hdr_limit  <= 8'd0;
            np_data_limit <= 12'd0;
        end else if (update_fc_valid) begin
            if (update_fc_type == 2'b00) begin
                p_hdr_limit  <= update_fc_hdr_limit;
                p_data_limit <= update_fc_data_limit;
            end else if (update_fc_type == 2'b01) begin
                np_hdr_limit  <= update_fc_hdr_limit;
                np_data_limit <= update_fc_data_limit;
            end
        end
    end

    // ------------------------------------------------------------------------
    // PCIe Credit Calculations (1 Data Credit = 4 DW)
    // ------------------------------------------------------------------------
    wire [11:0] p_req_data_credits = (p_req_len + 10'd3) >> 2;

    wire [7:0]  p_hdr_diff  = p_hdr_limit - p_hdr_consumed;
    wire [11:0] p_data_diff = p_data_limit - p_data_consumed;
    wire [7:0]  np_hdr_diff = np_hdr_limit - np_hdr_consumed;

    wire p_has_hdr_credit  = (p_hdr_diff  > 8'd0)                 && (p_hdr_diff  < 8'd128);
    wire p_has_data_credit = (p_data_diff >= p_req_data_credits)  && (p_data_diff < 12'd2048);
    wire np_has_hdr_credit = (np_hdr_diff > 8'd0)                 && (np_hdr_diff < 8'd128);

    wire p_has_credits  = p_has_hdr_credit && p_has_data_credit;
    wire np_has_credits = np_has_hdr_credit;

    // Ordering Rules
    wire can_start_p  = p_req_valid && p_has_credits;
    wire can_start_np = np_req_valid && np_has_credits && !p_req_valid;

    // ------------------------------------------------------------------------
    // Sequential State Machine
    // ------------------------------------------------------------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state            <= STATE_IDLE;
            p_hdr_consumed   <= 8'd0;
            p_data_consumed  <= 12'd0;
            np_hdr_consumed  <= 8'd0;
            np_data_consumed <= 12'd0;
            is_posted        <= 1'b0;
            active_addr      <= 64'd0;
            active_len       <= 10'd0;
            active_tag       <= 10'd0;
            active_fbe       <= 4'd0;
            active_lbe       <= 4'd0;
            data_cnt         <= 10'd0;
            p_req_ready      <= 1'b0;
            np_req_ready     <= 1'b0;
        end else begin
            p_req_ready  <= 1'b0;
            np_req_ready <= 1'b0;

            case (state)
                STATE_IDLE: begin
                    if (can_start_p) begin
                        is_posted       <= 1'b1;
                        active_addr     <= p_req_addr;
                        active_len      <= p_req_len;
                        active_tag      <= p_req_tag;
                        active_fbe      <= p_req_fbe;
                        active_lbe      <= p_req_lbe;
                        p_hdr_consumed  <= p_hdr_consumed + 8'd1;
                        p_data_consumed <= p_data_consumed + ((p_req_len + 10'd3) >> 2);
                        state           <= STATE_HDR0;
                    end else if (can_start_np) begin
                        is_posted       <= 1'b0;
                        active_addr     <= np_req_addr;
                        active_len      <= np_req_len;
                        active_tag      <= np_req_tag;
                        active_fbe      <= np_req_fbe;
                        active_lbe      <= np_req_lbe;
                        np_hdr_consumed <= np_hdr_consumed + 8'd1;
                        state           <= STATE_HDR0;
                    end
                end

                STATE_HDR0: begin
                    if (m_tready) state <= STATE_HDR1;
                end

                STATE_HDR1: begin
                    if (m_tready) begin
                        if (is_64bit)
                            state <= STATE_HDR2;
                        else
                            state <= STATE_HDR3;
                    end
                end

                STATE_HDR2: begin
                    if (m_tready) state <= STATE_HDR3;
                end

                STATE_HDR3: begin
                    if (m_tready) begin
                        if (!is_posted) begin
                            np_req_ready <= 1'b1;
                            state        <= STATE_DONE;
                        end else begin
                            data_cnt     <= active_len;
                            state        <= STATE_DATA;
                        end
                    end
                end

                STATE_DATA: begin
                    if (p_s_tvalid && m_tready) begin
                        data_cnt <= data_cnt - 10'd1;
                        if (data_cnt == 10'd1) begin
                            p_req_ready <= 1'b1;
                            state       <= STATE_DONE;
                        end
                    end
                end

                STATE_DONE: begin
                    if (is_posted) begin
                        if (!p_req_valid) state <= STATE_IDLE;
                    end else begin
                        if (!np_req_valid) state <= STATE_IDLE;
                    end
                end

                default: state <= STATE_IDLE;
            endcase
        end
    end

    // ------------------------------------------------------------------------
    // Outbound Stream Formatting Logic
    // ------------------------------------------------------------------------
    wire [2:0] tlp_fmt = is_posted ? (is_64bit ? 3'b011 : 3'b010)
                                   : (is_64bit ? 3'b001 : 3'b000);

    always @(*) begin
        m_tvalid   = 1'b0;
        m_tdata    = 32'd0;
        m_tlast    = 1'b0;
        p_s_tready = 1'b0;

        case (state)
            STATE_HDR0: begin
                m_tvalid = 1'b1;
                m_tdata  = {1'b0, tlp_fmt, 5'b00000, 1'b0, 3'b000, 1'b0, active_tag[9], 2'b00, 1'b0, 1'b0, 2'b00, 1'b0, active_tag[8], active_len};
                m_tlast  = 1'b0;
            end

            STATE_HDR1: begin
                m_tvalid = 1'b1;
                m_tdata  = {REQ_ID, active_tag[7:0], active_lbe, active_fbe};
                m_tlast  = 1'b0;
            end

            STATE_HDR2: begin
                m_tvalid = 1'b1;
                m_tdata  = active_addr[63:32];
                m_tlast  = 1'b0;
            end

            STATE_HDR3: begin
                m_tvalid = 1'b1;
                m_tdata  = active_addr[31:0];
                m_tlast  = !is_posted;
            end

            STATE_DATA: begin
                m_tvalid   = p_s_tvalid;
                m_tdata    = p_s_tdata;
                p_s_tready = m_tready;
                m_tlast    = (data_cnt == 10'd1);
            end

            default: begin
                m_tvalid   = 1'b0;
                m_tdata    = 32'd0;
                m_tlast    = 1'b0;
                p_s_tready = 1'b0;
            end
        endcase
    end

endmodule