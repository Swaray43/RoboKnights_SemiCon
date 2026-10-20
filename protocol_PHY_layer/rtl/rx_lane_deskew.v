// =============================================================================
// rx_lane_deskew - RX lane-to-lane deskew using the TS1 COM symbol (#1)
//
// A real channel delays each lane by a different amount, so the same TX byte
// reaches the 4 lane_rx blocks at different times. Every lane carries COM
// (0xBC) at byte 0 of each TS1 ordered set, at the same point in the stream,
// so COM is the common marker:
//
//   1. While not locked, each lane ignores bytes until it sees COM, then starts
//      filling its own FIFO ("armed"). The COM is the FIFO's first entry.
//   2. When all 4 lanes are armed, every FIFO head is the SAME COM, so the
//      lanes are aligned: lock, and from then on pop one byte from all 4 lanes
//      together whenever all 4 have one. Output is in lockstep, skew removed.
//   3. If the first-armed lane collects more than MAX_SKEW bytes before the
//      last lane arms, the lanes must have armed on different ordered sets
//      (e.g. the channel came up mid-set). Clear everything and retry on the
//      next COMs.
//
// Tolerates up to MAX_SKEW bytes (= 4*MAX_SKEW clk) between the earliest and
// latest lane. MAX_SKEW must be < 16 (one TS1 ordered set) and < DEPTH.
//
// LIMIT (inherent to any marker-based deskew): COM repeats every 16 bytes, so
// a lane that is late by more than 8 bytes looks exactly like one that is
// early by (16 - late) bytes and is aligned one ordered set off. Keep total
// channel skew under 8 bytes (32 clk); real PCIe lane-skew limits are far
// below an ordered set. A longer, non-repeating training pattern would be
// needed to detect larger skew.
// Locking only happens while align_en is high (training); after that the
// alignment is frozen and 0xBC inside Flit data is harmless.
// =============================================================================
module rx_lane_deskew #(
    parameter DEPTH    = 16,   // per-lane FIFO depth in bytes (power of 2)
    parameter MAX_SKEW = 8     // max lane-to-lane skew in bytes
)(
    input  wire        clk,
    input  wire        rst_n,
    input  wire        align_en,

    input  wire [31:0] in_bytes,       // lane i = [8*i +: 8]
    input  wire [3:0]  in_valid,

    output reg  [31:0] out_bytes,
    output reg         out_valid,      // all 4 lanes, same cycle
    output reg         locked,
    output reg         overflow        // sticky: a lane FIFO overflowed after lock (skew grew)
);

localparam [7:0] COM = 8'hBC;
localparam AW = (DEPTH <= 4) ? 2 : (DEPTH <= 8) ? 3 : (DEPTH <= 16) ? 4 : (DEPTH <= 32) ? 5 : 6;

reg [7:0]  mem0 [0:DEPTH-1];
reg [7:0]  mem1 [0:DEPTH-1];
reg [7:0]  mem2 [0:DEPTH-1];
reg [7:0]  mem3 [0:DEPTH-1];
reg [AW-1:0] wp0, wp1, wp2, wp3, rp0, rp1, rp2, rp3;
reg [AW:0]   cnt0, cnt1, cnt2, cnt3;
reg [3:0]    armed;

wire [7:0] b0 = in_bytes[7:0];
wire [7:0] b1 = in_bytes[15:8];
wire [7:0] b2 = in_bytes[23:16];
wire [7:0] b3 = in_bytes[31:24];

// a lane writes if it is already armed, or is arming on this COM byte
wire [3:0] arm_now = {4{!locked && align_en}} & in_valid &
                     {b3 == COM && !armed[3], b2 == COM && !armed[2],
                      b1 == COM && !armed[1], b0 == COM && !armed[0]};
wire [3:0] wr = in_valid & (armed | arm_now);

wire all_nonempty = (cnt0 != 0) && (cnt1 != 0) && (cnt2 != 0) && (cnt3 != 0);
wire pop          = locked && all_nonempty;

// retry if the earliest lane has run too far ahead before all lanes armed
wire too_far = !locked && ((cnt0 > MAX_SKEW) || (cnt1 > MAX_SKEW) ||
                           (cnt2 > MAX_SKEW) || (cnt3 > MAX_SKEW));

wire [3:0] full = {cnt3 == DEPTH, cnt2 == DEPTH, cnt1 == DEPTH, cnt0 == DEPTH};

always @(posedge clk or negedge rst_n) begin
  if (!rst_n) begin
    wp0 <= 0; wp1 <= 0; wp2 <= 0; wp3 <= 0;
    rp0 <= 0; rp1 <= 0; rp2 <= 0; rp3 <= 0;
    cnt0 <= 0; cnt1 <= 0; cnt2 <= 0; cnt3 <= 0;
    armed     <= 4'b0000;
    locked    <= 1'b0;
    overflow  <= 1'b0;
    out_valid <= 1'b0;
    out_bytes <= 32'd0;
  end else if (too_far) begin
    // armed on different ordered sets: drop everything and re-arm on next COMs
    wp0 <= 0; wp1 <= 0; wp2 <= 0; wp3 <= 0;
    rp0 <= 0; rp1 <= 0; rp2 <= 0; rp3 <= 0;
    cnt0 <= 0; cnt1 <= 0; cnt2 <= 0; cnt3 <= 0;
    armed     <= 4'b0000;
    out_valid <= 1'b0;
  end else begin
    armed <= armed | arm_now;
    if (!locked && ((armed | arm_now) == 4'b1111) && align_en)
      locked <= 1'b1;   // all heads are the same COM from the next cycle on

    if (wr[0] && !full[0]) begin mem0[wp0] <= b0; wp0 <= wp0 + 1'b1; end
    if (wr[1] && !full[1]) begin mem1[wp1] <= b1; wp1 <= wp1 + 1'b1; end
    if (wr[2] && !full[2]) begin mem2[wp2] <= b2; wp2 <= wp2 + 1'b1; end
    if (wr[3] && !full[3]) begin mem3[wp3] <= b3; wp3 <= wp3 + 1'b1; end
    if (locked && |(wr & full)) overflow <= 1'b1;

    out_valid <= pop;
    if (pop) begin
      out_bytes <= {mem3[rp3], mem2[rp2], mem1[rp1], mem0[rp0]};
      rp0 <= rp0 + 1'b1; rp1 <= rp1 + 1'b1; rp2 <= rp2 + 1'b1; rp3 <= rp3 + 1'b1;
    end

    cnt0 <= cnt0 + (wr[0] && !full[0]) - pop;
    cnt1 <= cnt1 + (wr[1] && !full[1]) - pop;
    cnt2 <= cnt2 + (wr[2] && !full[2]) - pop;
    cnt3 <= cnt3 + (wr[3] && !full[3]) - pop;
  end
end

endmodule
