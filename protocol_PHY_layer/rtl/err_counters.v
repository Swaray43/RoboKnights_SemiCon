`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// err_counters - Step 4 item 6: sticky error counters for the driver, in BAR0.
//
// Reset by PERST# only (not by hot reset / link down), so a driver can read
// how many link-downs or hot resets happened. 16-bit saturating; writing any
// value to a counter clears it.
//
//   BAR0 0x040  CRC-64 failures (after FEC)        link layer RX
//   BAR0 0x044  FEC corrected Flits                 link layer RX
//   BAR0 0x048  FEC uncorrectable Flits             link layer RX
//   BAR0 0x04C  replays (NAK or timeout)            link layer TX
//   BAR0 0x050  NAKs sent                           link layer RX
//   BAR0 0x054  malformed inbound TLPs              RX decode
//   BAR0 0x058  link downs (L0 left, K6)            LTSSM
//   BAR0 0x05C  hot resets                          reset controller
//   BAR0 0x060  session overflows ch0               interconnect (synchronised)
//   BAR0 0x064  session overflows ch1
// -----------------------------------------------------------------------------
module err_counters #(
    parameter N = 10
)(
    input  wire          clk,
    input  wire          perst_n,
    input  wire [N-1:0]  evt,           // 1-clock pulses, clk domain
    input  wire          wr_en,
    input  wire          rd_en,
    input  wire [3:0]    idx,           // counter index (0 = BAR0 0x040)
    output reg  [31:0]   rdata,
    output reg           rd_valid
);
reg [15:0] cnt [0:N-1];
integer i;
always @(posedge clk or negedge perst_n) begin
  if (!perst_n) begin
    for (i = 0; i < N; i = i + 1) cnt[i] <= 16'd0;
    rdata <= 32'd0; rd_valid <= 1'b0;
  end else begin
    rd_valid <= rd_en;
    if (rd_en) rdata <= (idx < N) ? {16'd0, cnt[idx]} : 32'd0;
    for (i = 0; i < N; i = i + 1) begin
      if (wr_en && idx == i)              cnt[i] <= 16'd0;
      else if (evt[i] && cnt[i] != 16'hFFFF) cnt[i] <= cnt[i] + 16'd1;
    end
  end
end
endmodule
