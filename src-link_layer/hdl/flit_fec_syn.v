`timescale 1ns/1ps
// ============================================================
// flit_fec_syn -- the 3 Hamming syndromes + overall parities of the Flit FEC
// (fl_cw_syn in flit_defs.vh) as explicit XOR trees (Step 4 Wave C).
// Output {sp2, sp1, sp0}, 11 bits each = {parity, syndrome[9:0]}.
// Syndrome bit b of interleave j = XOR of the data bits whose Hamming
// position has bit b set; the positions are elaboration-time constants.
// ============================================================
module flit_fec_syn (
    input  wire [1999:0] d,
    output wire [32:0]   sp
);
    localparam N = 2000, NB = 250;

    function is_pow2; input [10:0] p; begin is_pow2 = (p != 0) && ((p & (p - 11'd1)) == 0); end endfunction

    // mask of data bits for interleave j: bit = 1 if the bit's position has bit b set
    // (b = 10 selects every data bit of the interleave: overall parity)
    function [N-1:0] fec_mask;
        input integer j;
        input integer b;
        integer bb, k;
        reg [10:0] p;
        begin
            fec_mask = {N{1'b0}};
            p = 11'd2;
            for (bb = 0; bb < NB; bb = bb + 1)
                if ((bb % 3) == j)
                    for (k = 0; k < 8; k = k + 1) begin
                        p = p + 11'd1;
                        if (is_pow2(p)) p = p + 11'd1;
                        fec_mask[bb*8 + k] = (b == 10) ? 1'b1 : p[b];
                    end
        end
    endfunction

    genvar j, b;
    generate for (j = 0; j < 3; j = j + 1) begin : g_il
        for (b = 0; b < 11; b = b + 1) begin : g_b
            localparam [N-1:0] M = fec_mask(j, b);
            assign sp[11*j + b] = ^(d & M);
        end
    end endgenerate
endmodule
