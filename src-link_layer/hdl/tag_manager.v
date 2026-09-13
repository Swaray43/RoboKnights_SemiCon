`timescale 1ns/1ps

/*
 * Module: tag_manager
 * Function: 
 *  1. Allocates unique Tags for Non-Posted requests (MRd).
 *  2. Recycles Tags back into a free pool upon Completion (CplD).
 *  3. Tracks timeouts for outstanding requests and escalates to Unsupported Request (UR).
 *  4. Single-cycle arbitration ensures FIFO pointers are never corrupted by simultaneous updates.
 */
module tag_manager #(
    parameter N = 256, // Selectable at build: 32, 256, or 1024
    parameter TIMEOUT_CYCLES = 10000
)(
    input  wire          clk,
    input  wire          rst_n,

    // Upstream Allocation Interface
    input  wire          s_valid,
    input  wire          s_is_non_posted, // 1 for MRd/CfgRd
    output wire          s_tready,
    output reg  [9:0]    m_tag,
    output reg           m_tag_valid,

    // Downstream Completion Interface (CplD return)
    input  wire          cpl_valid,
    input  wire [9:0]    cpl_tag,

    // UR Generation Interface (Upstream)
    output reg           ur_valid,
    output reg  [9:0]    ur_tag
);

    localparam TAG_W = 10;
    localparam PTR_W = $clog2(N);

    // Free pool FIFO
    reg [TAG_W-1:0] free_pool [0:N-1];
    reg [PTR_W:0]   head, tail;
    wire            pool_empty = (head == tail);
    wire [TAG_W-1:0] next_free_tag = free_pool[head[PTR_W-1:0]];

    // Active tag tracking
    reg active_tags [0:N-1];
    reg [$clog2(TIMEOUT_CYCLES)-1:0] timers [0:N-1];
    
    integer i;
    reg timeout_handled;

    // Gate new requests when empty
    assign s_tready = s_is_non_posted ? ~pool_empty : 1'b1;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            head <= 0;
            tail <= N[PTR_W:0];
            m_tag_valid <= 0;
            ur_valid <= 0;
            for (i = 0; i < N; i = i + 1) begin
                free_pool[i] <= i;
                active_tags[i] <= 0;
                timers[i] <= 0;
            end
        end else begin
            m_tag_valid <= 0;
            ur_valid <= 0;
            timeout_handled = 0;

            // 1. Tag Allocation
            if (s_valid && s_tready && s_is_non_posted) begin
                m_tag <= next_free_tag;
                m_tag_valid <= 1;
                active_tags[next_free_tag] <= 1;
                timers[next_free_tag] <= TIMEOUT_CYCLES;
                head <= head + 1;
            end

            // 2. Recycling Arbitration (Completion vs. Timeout)
            if (cpl_valid && active_tags[cpl_tag]) begin
                // Priority A: Incoming Completion
                active_tags[cpl_tag] <= 0;
                free_pool[tail[PTR_W-1:0]] <= cpl_tag;
                tail <= tail + 1;
                
                // Decrement timers, but prevent any from hitting 0/recycling this cycle
                for (i = 0; i < N; i = i + 1) begin
                    if (active_tags[i] && timers[i] > 0 && i != cpl_tag) begin
                        if (timers[i] == 1) timers[i] <= 1; // Stall timeout
                        else                timers[i] <= timers[i] - 1;
                    end
                end
            end else begin
                // Priority B: Local Timeouts (Process max 1 per cycle)
                for (i = 0; i < N; i = i + 1) begin
                    if (active_tags[i] && timers[i] > 0) begin
                        if (timers[i] == 1 && !timeout_handled) begin
                            // Timeout hit: trigger UR and recycle
                            active_tags[i] <= 0;
                            ur_valid <= 1;
                            ur_tag <= i;
                            free_pool[tail[PTR_W-1:0]] <= i;
                            tail <= tail + 1;
                            timeout_handled = 1;
                            timers[i] <= 0;
                        end else if (timers[i] == 1 && timeout_handled) begin
                            // Wait for next cycle if FIFO tail is busy
                            timers[i] <= 1; 
                        end else begin
                            timers[i] <= timers[i] - 1;
                        end
                    end
                end
            end
        end
    end
endmodule