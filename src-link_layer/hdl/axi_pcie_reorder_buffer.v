`timescale 1ns/1ps

module axi_pcie_reorder_buffer #(
    parameter NUM_SLOTS      = 8,
    parameter TAG_WIDTH      = 10,  // Extended tag support
    parameter MAX_DW_PER_REQ = 16,  // Max Read Request Size (in DW)
    parameter TIMEOUT_CYCLES = 64
)(
    input  wire                   clk,
    input  wire                   rst_n,

    // Request Allocation Interface
    input  wire                   req_valid,
    input  wire [TAG_WIDTH-1:0]   req_tag,
    input  wire [9:0]             req_len_dw,
    output wire                   req_ready,

    // RX CplD Ingress Interface
    input  wire                   rx_cpld_valid,
    input  wire                   rx_cpld_sof,      // High on first DW of CplD TLP
    input  wire [9:0]             rx_cpld_offset,   // DW offset of fragment
    input  wire [TAG_WIDTH-1:0]   rx_cpld_tag,
    input  wire [31:0]            rx_cpld_data,
    input  wire                   rx_cpld_dvalid,
    input  wire [2:0]             rx_cpld_status,
    input  wire                   rx_cpld_last_dw,

    // Outbound AXI4-Stream Interface
    output reg                    m_axis_tvalid,
    output reg  [31:0]            m_axis_tdata,
    output reg                    m_axis_tlast,
    output reg  [3:0]             m_axis_tkeep,
    output reg                    m_axis_tuser,     // 1 = UR/Completer Abort Error
    input  wire                   m_axis_tready
);

    localparam SLOT_BITS = $clog2(NUM_SLOTS);

    // Slot tracking tables
    reg [TAG_WIDTH-1:0] slot_tag    [0:NUM_SLOTS-1];
    reg [9:0]           slot_len    [0:NUM_SLOTS-1];
    reg [9:0]           slot_rx_cnt [0:NUM_SLOTS-1]; 
    reg                 slot_valid  [0:NUM_SLOTS-1];
    reg                 slot_done   [0:NUM_SLOTS-1];
    
    // Internal Memory
    reg [31:0]          buffer_data [0:NUM_SLOTS-1][0:MAX_DW_PER_REQ-1];

    reg [SLOT_BITS-1:0] head_ptr;
    reg [SLOT_BITS-1:0] tail_ptr;
    reg [SLOT_BITS:0]   occupancy;

    reg [9:0]           read_dw_cnt;
    reg [31:0]          timeout_timer;
    reg                 drain_state;
    reg [9:0]           current_rx_offset;

    wire alloc_en  = req_valid && req_ready;
    wire retire_en = drain_state && m_axis_tvalid && m_axis_tready && m_axis_tlast;

    assign req_ready = (occupancy < NUM_SLOTS);

    // Combinatorial CAM Match for Ingress Data
    reg [SLOT_BITS-1:0] match_idx;
    reg                 match_found;
    integer i;

    always @(*) begin
        match_found = 1'b0;
        match_idx   = 0;
        for (i = 0; i < NUM_SLOTS; i = i + 1) begin
            if (slot_valid[i] && (slot_tag[i] == rx_cpld_tag)) begin
                match_found = 1'b1;
                match_idx   = i[SLOT_BITS-1:0];
            end
        end
    end

    // Immediate offset calculation on SOF cycle
    reg [9:0] write_dw_offset;
    always @(*) begin
        if (rx_cpld_sof) begin
            write_dw_offset = rx_cpld_offset;
        end else begin
            write_dw_offset = current_rx_offset;
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head_ptr          <= 0;
            tail_ptr          <= 0;
            occupancy         <= 0;
            read_dw_cnt       <= 0;
            timeout_timer     <= 0;
            drain_state       <= 0;
            current_rx_offset <= 0;
            
            m_axis_tvalid     <= 0;
            m_axis_tdata      <= 0;
            m_axis_tlast      <= 0;
            m_axis_tkeep      <= 4'hf;
            m_axis_tuser      <= 0;
            
            for (i = 0; i < NUM_SLOTS; i = i + 1) begin
                slot_valid[i]  <= 0;
                slot_rx_cnt[i] <= 0;
                slot_done[i]   <= 0;
            end
        end else begin
            
            // 1. Request Allocation
            if (alloc_en) begin
                slot_valid[tail_ptr]  <= 1'b1;
                slot_tag[tail_ptr]    <= req_tag;
                slot_len[tail_ptr]    <= req_len_dw;
                slot_rx_cnt[tail_ptr] <= 0;
                slot_done[tail_ptr]   <= 0;
                tail_ptr              <= tail_ptr + 1;
            end

            // 2. RX Data Ingress (Immediate Offset Calculation)
            if (rx_cpld_valid && match_found) begin
                if (rx_cpld_dvalid) begin
                    if (write_dw_offset < MAX_DW_PER_REQ) begin
                        buffer_data[match_idx][write_dw_offset] <= rx_cpld_data;
                        current_rx_offset <= write_dw_offset + 1;
                        
                        slot_rx_cnt[match_idx] <= slot_rx_cnt[match_idx] + 1;
                        
                        if (slot_rx_cnt[match_idx] + 1 >= slot_len[match_idx]) begin
                            slot_done[match_idx] <= 1'b1;
                        end
                    end
                end
            end

            // 3. Occupancy Tracking
            if (alloc_en && retire_en) occupancy <= occupancy;
            else if (alloc_en)         occupancy <= occupancy + 1;
            else if (retire_en)        occupancy <= occupancy - 1;

            // 4. AXI4-Stream Egress
            if (!drain_state) begin
                m_axis_tvalid <= 0;
                m_axis_tuser  <= 0;
                m_axis_tlast  <= 0;

                if (slot_valid[head_ptr]) begin
                    if (slot_done[head_ptr]) begin
                        drain_state   <= 1;
                        read_dw_cnt   <= 0;
                        timeout_timer <= 0;
                        
                        m_axis_tvalid <= 1;
                        m_axis_tdata  <= buffer_data[head_ptr][0];
                        m_axis_tlast  <= (slot_len[head_ptr] == 1);
                        m_axis_tkeep  <= 4'hf;
                        m_axis_tuser  <= 0;
                    end else begin
                        timeout_timer <= timeout_timer + 1;
                        if (timeout_timer >= TIMEOUT_CYCLES - 1) begin
                            drain_state   <= 1;
                            read_dw_cnt   <= 0;
                            m_axis_tvalid <= 1;
                            m_axis_tdata  <= 32'hDEAD_DEAD; 
                            m_axis_tlast  <= 1; 
                            m_axis_tuser  <= 1; 
                        end
                    end
                end
            end else begin
                if (m_axis_tvalid && m_axis_tready) begin
                    if (m_axis_tlast) begin
                        slot_valid[head_ptr] <= 0;
                        head_ptr             <= head_ptr + 1;
                        drain_state          <= 0;
                        m_axis_tvalid        <= 0;
                        m_axis_tlast         <= 0;
                        m_axis_tuser         <= 0;
                        timeout_timer        <= 0;
                    end else begin
                        read_dw_cnt   <= read_dw_cnt + 1;
                        m_axis_tdata  <= buffer_data[head_ptr][read_dw_cnt + 1];
                        m_axis_tlast  <= ((read_dw_cnt + 1) == (slot_len[head_ptr] - 1));
                    end
                end
            end
        end
    end
endmodule