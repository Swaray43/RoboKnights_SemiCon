`timescale 1ns/1ps
// ============================================================
// flit_crc64_parts -- CRC-64/ECMA-182 of the 1936-bit Flit body as
// explicit XOR trees (Step 4 Wave C).
//
// The CRC is linear in the data: crc = ~(K0 ^ XOR_i d[i] * V_i), where V_i is
// the 64-bit effect of body bit i. Here every output bit o is computed as
// ^(d & MASK_o) over each 256-bit chunk separately: 8 independent trees of at
// most 256 inputs per output bit (about 3 LUT6 levels), registered by the
// caller, then combined (8 inputs per bit) by flit_crc64_combine.
// The masks are elaboration-time constants computed from the polynomial, so
// there is no LFSR chain for synthesis to unroll (Vivado kept the unrolled
// chain of flit_crc64 deep: 51 logic levels in the Wave A+B run).
// ============================================================
module flit_crc64_parts (
    input  wire [1935:0] d,
    output wire [511:0]  parts       // {part7, ..., part0}, 64 bits each
);
    localparam N = 1936;
    localparam [63:0] P = 64'h42F0E1EBA9EA3693;

    // bit i of the result = output bit o of V_i (effect of body bit i)
    function [N-1:0] crc_mask;
        input integer o;
        integer i;
        reg [63:0] x;
        begin
            x = P;                                  // effect of the last body bit
            for (i = N - 1; i >= 0; i = i - 1) begin
                crc_mask[i] = x[o];
                x = x[63] ? ((x << 1) ^ P) : (x << 1);   // one more step for earlier bits
            end
        end
    endfunction

    genvar o, k;
    generate for (o = 0; o < 64; o = o + 1) begin : g_bit
        localparam [N-1:0] M = crc_mask(o);
        for (k = 0; k < 8; k = k + 1) begin : g_chunk
            localparam LO  = 256 * k;
            localparam LEN = (k == 7) ? (N - 1792) : 256;
            assign parts[64*k + o] = ^(d[LO +: LEN] & M[LO +: LEN]);
        end
    end endgenerate
endmodule
