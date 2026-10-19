// ------------------------------------------------------------------
// rst_sync: converts a potentially-async internal reset assertion to
// a synchronous, glitch-free active-low reset for a downstream
// module in the SAME clock domain. Async assert, sync deassert --
// standard 2-FF reset synchronizer.
//
// Per Swaray's note: his module needs rst_n held low >= 2 clock
// cycles minimum. This synchronizer's deassertion edge is inherently
// 2 cycles behind async_rst_n's rise, and assertion is immediate
// (async), so as long as OUR internal reset assertion holds for
// at least 1 cycle, his 2-cycle minimum is satisfied by construction.
// ------------------------------------------------------------------
module rst_sync (
    input  wire clk,
    input  wire async_rst_n,     // from your reset controller, async-capable assert
    output reg  sync_rst_n       // clean, synchronous-deassert reset for Swaray's module
);

    reg stage1_rst_n;

    always @(posedge clk or negedge async_rst_n) begin
        if (!async_rst_n) begin
            stage1_rst_n <= 1'b0;
            sync_rst_n   <= 1'b0;
        end else begin
            stage1_rst_n <= 1'b1;
            sync_rst_n   <= stage1_rst_n;
        end
    end

endmodule
