// LEGACY (I2): companion of legacy_axi4s_ingest_port (same 16-bit offset
// scheme). The interconnect layer does its own limit check. Not on the build
// file list; kept for reference only.
// ------------------------------------------------------------------
// legacy_session_overflow_ctrl: bounds-checks session_offset against the
// compile-time SLICE_SIZE and raises overflow_abort one cycle before
// the offset would exceed the bound, so the ingest port can stall
// cleanly instead of accepting an out-of-bounds beat.
// One instance per channel, same as legacy_axi4s_ingest_port.
// ------------------------------------------------------------------
module legacy_session_overflow_ctrl #(
    parameter OFFSET_WIDTH = 16,
    parameter SLICE_SIZE   = 16'hFFFF   // must match the ingest port's SLICE_SIZE
)(
    input  wire                     clk,
    input  wire                     rst_n,

    input  wire                     session_valid,
    input  wire [OFFSET_WIDTH-1:0]  session_offset,   // current offset, BEFORE this beat
    input  wire                     beat_fire,         // this cycle's beat is being accepted

    output wire                     overflow_abort,    // combinational, feeds ingest port same cycle
    output reg                      session_overflow_sticky,  // W1C, CfgRd-readable
    input  wire                     overflow_flag_clear       // host W1C strobe
);

    // Abort if accepting this beat would push offset to/past the bound.
    // Checked combinationally against the NEXT offset value, so the
    // ingest port can gate s_tready before the out-of-bounds beat is latched.
    wire would_exceed = session_valid && beat_fire &&
                         (session_offset >= (SLICE_SIZE - 1'b1));

    assign overflow_abort = would_exceed;

    always @(posedge clk) begin
        if (!rst_n) begin
            session_overflow_sticky <= 1'b0;
        end else begin
            if (would_exceed)
                session_overflow_sticky <= 1'b1;       // sticky: latches until host clears
            else if (overflow_flag_clear)
                session_overflow_sticky <= 1'b0;        // W1C from host CfgWr
        end
    end

endmodule
