`timescale 1ns/1ps

/*
 * Module: replay_buffer
 * Function: 
 *  1. Appends a 12-bit Sequence Number to outgoing FLITs.
 *  2. Stores transmitted FLITs in a local circular buffer.
 *  3. Purges buffered FLITs when an ACK sequence number is received.
 *  4. Rewinds the transmit pointer to re-send FLITs when a NAK is received.
 */
module replay_buffer #(
    parameter FLIT_WIDTH = 2048,
    parameter SEQ_WIDTH  = 12,
    parameter BUF_DEPTH  = 32
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // Interface from Transaction Layer (TX Engine)
    input  wire                   s_valid,
    input  wire [FLIT_WIDTH-1:0]  s_data,
    output wire                   s_ready,

    // Interface to Link Layer TX (Physical Framing)
    output reg                    m_valid,
    output reg  [FLIT_WIDTH-1:0]  m_data,
    input  wire                   m_ready,

    // Interface from DLLP Parser (ACK/NAK feedback)
    input  wire                   rx_ack_valid,
    input  wire [SEQ_WIDTH-1:0]   rx_ack_seq,
    input  wire                   rx_nak_valid,
    input  wire [SEQ_WIDTH-1:0]   rx_nak_seq
);

    localparam ADDR_W = $clog2(BUF_DEPTH);

    // Circular Buffer Memory
    reg [FLIT_WIDTH-1:0] buffer [0:BUF_DEPTH-1];
    
    // Pointers
    reg [ADDR_W-1:0]     head_ptr;    // Write pointer (new TLPs)
    reg [ADDR_W-1:0]     tail_ptr;    // Commit pointer (ACK'd TLPs)
    reg [ADDR_W-1:0]     tx_ptr;      // Read pointer (currently transmitting)
    
    reg [SEQ_WIDTH-1:0]  next_seq_no;
    
    wire is_full  = ((head_ptr + 1'b1) % BUF_DEPTH) == tail_ptr;
    wire is_empty = (tx_ptr == head_ptr);

    assign s_ready = !is_full && !rx_nak_valid; // Stall upstream on NAK or full

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head_ptr    <= 0;
            tail_ptr    <= 0;
            tx_ptr      <= 0;
            next_seq_no <= 0;
            m_valid     <= 0;
            m_data      <= 0;
        end else begin
            // 1. Handle ACK: Advance tail pointer to clear stored FLITs
            if (rx_ack_valid) begin
                tail_ptr <= (rx_ack_seq + 1'b1) % BUF_DEPTH;
            end

            // 2. Handle NAK: Rewind TX pointer to the NAK'd sequence
            if (rx_nak_valid) begin
                tx_ptr <= rx_nak_seq % BUF_DEPTH;
                m_valid <= 1'b0; // Flush current output
            end 
            // 3. Normal Transmission / Write
            else begin
                // Accept new data from upstream
                if (s_valid && s_ready) begin
                    // Embed sequence number into FLIT (Assuming top 12 bits for simplicity)
                    buffer[head_ptr] <= {next_seq_no, s_data[FLIT_WIDTH-SEQ_WIDTH-1:0]};
                    head_ptr         <= (head_ptr + 1'b1) % BUF_DEPTH;
                    next_seq_no      <= next_seq_no + 1'b1;
                end

                // Transmit pending data to downstream
                if (!is_empty && (m_ready || !m_valid)) begin
                    m_valid <= 1'b1;
                    m_data  <= buffer[tx_ptr];
                    tx_ptr  <= (tx_ptr + 1'b1) % BUF_DEPTH;
                end else if (m_valid && m_ready) begin
                    m_valid <= 1'b0; // De-assert if no new data
                end
            end
        end
    end
endmodule