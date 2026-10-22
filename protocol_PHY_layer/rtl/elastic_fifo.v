`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// TX elastic deskew buffer (fixed-depth model). Per-lane FIFO + shared release
// gate. NOTE ON NAMING: this is "four-lane buffering with a common release
// threshold," not a true UI-level skew measurement - it never timestamps or
// compares arrival time between lanes, only fill count. True deskew would
// need a sequence-marker mechanism; deferred given project scope.
// -----------------------------------------------------------------------------
module elastic_fifo #(
parameter DEPTH = 8,
parameter PTR_WIDTH = 3
)(
input  wire       clk,
input  wire       rst_n,
input  wire       wr_en,
input  wire [7:0] wr_data,
input  wire       rd_en,
output reg  [7:0] rd_data,
output wire [PTR_WIDTH:0] fill_count,
output wire        empty,
output wire        full
);

reg [7:0] mem [0:DEPTH-1];
reg [PTR_WIDTH-1:0] wr_ptr, rd_ptr;
reg [PTR_WIDTH:0]   count;

assign fill_count = count;
assign empty = (count == 0);
assign full  = (count == DEPTH);

// OPT-5: the array and its registered read port sit in a block WITHOUT reset,
// so synthesis can map them to RAM (an array under async reset is built from
// flip-flops). rd_data is only consumed one clock after a read, so its reset
// value was never used.
always @(posedge clk) begin
  if (wr_en && !full)  mem[wr_ptr] <= wr_data;
  if (rd_en && !empty) rd_data     <= mem[rd_ptr];
end

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    wr_ptr  <= 0;
    rd_ptr  <= 0;
    count   <= 0;
  end else begin
    if (wr_en && !full)  wr_ptr <= wr_ptr + 1'b1;
    if (rd_en && !empty) rd_ptr <= rd_ptr + 1'b1;
    case ({wr_en && !full, rd_en && !empty})
      2'b10: count <= count + 1'b1;
      2'b01: count <= count - 1'b1;
      default: count <= count;
    endcase
  end
end
endmodule
