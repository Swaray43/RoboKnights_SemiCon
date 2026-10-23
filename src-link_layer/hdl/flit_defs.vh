// ============================================================
// flit_defs.vh  --  shared 256-byte Flit layout + CRC64 + FEC
// `include this INSIDE a module body (link_layer_tx / link_layer_rx).
//
// Flit layout (fixes review items L6, L8; matches PCIe 6.0 byte budget)
//
//   byte  0 .. 235   TLP area     236 B = 59 DW   flit[1887:0]
//   byte 236 .. 241  DLP           6 B            flit[1935:1888]
//   byte 242 .. 249  CRC-64        8 B            flit[1999:1936]
//   byte 250 .. 255  FEC           6 B            flit[2047:2000]
//
//   Byte n of the Flit is flit[8n+7:8n]; on the 256-bit bus word k
//   carries flit[256k+255:256k], so byte 0 goes out first.
//
//   TLP area: TLPs are packed back-to-back as DWs (DW i = flit[32i+31:32i]).
//   A TLP may straddle two Flits. All-zero DWs are NOP TLPs (Flit-mode
//   NOP) and are used as padding; the receiver skips them.
//
//   DLP (48 bits, flit[1935:1888]):
//     [11:0]  Flit sequence number (valid when kind == 0)
//     [13:12] kind : 0 = payload Flit (sequenced, held in replay buffer)
//                    1 = control-only Flit (carries ACK/NAK, not sequenced)
//     [14]    ack_valid    } piggy-backed ACK/NAK for the reverse
//     [15]    nak_valid    } direction (the far end's replay buffer)
//     [27:16] ack/nak sequence number = last Flit received in order
//     [28]    fc_dllp      (G2) control Flit only: TLP area DW0/DW1/DW2
//                          carry Flow-Control DLLPs for P / NP / Cpl
//     [47:29] reserved (0)
//
//   FC DLLP (one per DW, PCIe DLLP byte order, DW bit 31 = byte 0 bit 7):
//     [31:24] type  InitFC1 0100_0vvv / InitFC2 1100_0vvv / UpdateFC 1000_0vvv
//                   with [5:4] = 00 P, 01 NP, 10 Cpl and vvv = VC (0 only)
//     [23:22] HdrScale (0)   [21:14] HdrFC
//     [13:12] DataScale (0)  [11:0]  DataFC      HdrFC/DataFC 0 = infinite
//   Control Flits are not sequenced or replayed: a lost FC Flit is covered
//   by the InitFC resend timer in data_link_layer_top.
//
//   CRC-64: CRC-64/ECMA-182 (poly 0x42F0E1EBA9EA3693, init and xorout
//   all-ones) over flit[1935:0] (TLP area + DLP).
//
//   FEC: 3-way byte-interleaved SEC-DED over flit[1999:0] (bytes 0..249,
//   i.e. everything except the FEC bytes). Codeword j holds bytes
//   j, j+3, j+6 ... (84/83/83 bytes = up to 672 data bits) and has
//   10 Hamming check bits + 1 overall parity bit = 11 bits.
//     flit[2010:2000] = codeword 0 {overall, check[9:0]}
//     flit[2021:2011] = codeword 1
//     flit[2032:2022] = codeword 2
//     flit[2047:2033] = reserved (0)
//   Corrects 1 bit error in each codeword (so up to 3 per Flit if they
//   fall in different interleaves), detects 2 in the same codeword.
//   Same 6-byte budget and 3-way interleave as PCIe 6.0; PCIe uses
//   Reed-Solomon symbols, this uses binary SEC-DED.
// ============================================================

localparam FLIT_W       = 2048;
localparam FL_TLP_DW    = 59;
localparam FL_TLP_W     = FL_TLP_DW * 32;   // 1888
localparam FL_DLP_LO    = 1888;
localparam FL_DLP_W     = 48;
localparam FL_BODY_W    = 1936;             // TLP area + DLP (CRC coverage)
localparam FL_CRC_LO    = 1936;
localparam FL_PROT_W    = 2000;             // body + CRC (FEC coverage)
localparam FL_FEC_LO    = 2000;
localparam FL_FEC_BYTES = 250;              // bytes covered by FEC
localparam FEC_CW_BITS  = 11;

localparam [1:0] FLIT_KIND_PAYLOAD = 2'd0;
localparam [1:0] FLIT_KIND_CONTROL = 2'd1;
localparam       FL_DLP_FC_BIT     = 28;

// ---------------- CRC-64/ECMA-182 ----------------
function [63:0] flit_crc64;
    input [FL_BODY_W-1:0] d;
    integer i;
    reg [63:0] c;
    begin
        c = {64{1'b1}};
        for (i = 0; i < FL_BODY_W; i = i + 1) begin
            if (c[63] ^ d[i])
                c = (c << 1) ^ 64'h42F0E1EBA9EA3693;
            else
                c = c << 1;
        end
        flit_crc64 = ~c;
    end
endfunction

// ---------------- CRC-64 split for pipelining (Step 4 Wave C) ----------------
// The CRC is affine in the data: crc = ~(K0 ^ XOR_k part_k), where part_k is
// the linear contribution of one 256-bit chunk (computed from a zero state and
// advanced through the rest of the body as zeros) and K0 is the all-ones init
// advanced through FL_BODY_W zero bits. Each part is an independent XOR tree,
// so the parts can be registered and combined in the next stage.
// Loop bounds are constants (Yosys/LibreLane); lo/len must be literals.
function [63:0] flit_crc64_part;
    input [255:0] ch;
    input integer lo;          // first body bit of the chunk
    input integer len;         // bits in the chunk (<= 256)
    integer i;
    reg [63:0] c;
    begin
        c = 64'd0;
        for (i = 0; i < 256; i = i + 1)
            if (i < len) c = (c[63] ^ ch[i]) ? ((c << 1) ^ 64'h42F0E1EBA9EA3693) : (c << 1);
        for (i = 0; i < FL_BODY_W; i = i + 1)
            if (i >= lo + len) c = c[63] ? ((c << 1) ^ 64'h42F0E1EBA9EA3693) : (c << 1);
        flit_crc64_part = c;
    end
endfunction

function [63:0] flit_crc64_k0;
    input dummy;
    integer i;
    reg [63:0] c;
    begin
        c = {64{1'b1}};
        for (i = 0; i < FL_BODY_W; i = i + 1)
            c = c[63] ? ((c << 1) ^ 64'h42F0E1EBA9EA3693) : (c << 1);
        flit_crc64_k0 = c;
    end
endfunction

// all 8 parts of a body, {part7, ..., part0}
function [511:0] flit_crc64_parts;
    input [FL_BODY_W-1:0] d;
    begin
        flit_crc64_parts = {
            flit_crc64_part({112'd0, d[1935:1792]}, 1792, 144),
            flit_crc64_part(d[1536 +: 256], 1536, 256),
            flit_crc64_part(d[1280 +: 256], 1280, 256),
            flit_crc64_part(d[1024 +: 256], 1024, 256),
            flit_crc64_part(d[ 768 +: 256],  768, 256),
            flit_crc64_part(d[ 512 +: 256],  512, 256),
            flit_crc64_part(d[ 256 +: 256],  256, 256),
            flit_crc64_part(d[   0 +: 256],    0, 256)};
    end
endfunction

function [63:0] flit_crc64_combine;
    input [511:0] parts;
    integer k;
    reg [63:0] x;
    begin
        x = flit_crc64_k0(1'b0);
        for (k = 0; k < 8; k = k + 1) x = x ^ parts[64*k +: 64];
        flit_crc64_combine = ~x;
    end
endfunction

// ---------------- helpers ----------------
function fl_is_pow2;
    input [10:0] p;
    begin
        fl_is_pow2 = (p != 0) && ((p & (p - 11'd1)) == 0);
    end
endfunction

// Hamming syndrome + overall parity of one interleave, data bits only.
// Returns {parity, syndrome[9:0]}. Data bit k of the codeword sits at the
// k-th position >= 3 that is not a power of two (positions 1,2,4..512 are
// the check bits), so the syndrome of a single data-bit error is its
// position, never zero, never a power of two.
function [10:0] fl_cw_syn;
    input [FL_PROT_W-1:0] d;
    input integer         j;          // interleave 0..2
    integer b, k;
    reg [10:0] p;
    reg [9:0]  s;
    reg        par;
    begin
        s = 10'd0; par = 1'b0; p = 11'd2;
        for (b = j; b < FL_FEC_BYTES; b = b + 3) begin
            for (k = 0; k < 8; k = k + 1) begin
                p = p + 11'd1;
                if (fl_is_pow2(p)) p = p + 11'd1;   // "if" not "while": p starts at 3, never two powers of 2 in a row past 2; Yosys rejects while here
                if (d[b*8+k]) begin
                    s   = s ^ p[9:0];
                    par = ~par;
                end
            end
        end
        fl_cw_syn = {par, s};
    end
endfunction

// Encoder: 48-bit FEC field for flit[1999:0]
function [47:0] flit_fec_encode;
    input [FL_PROT_W-1:0] d;
    integer j;
    reg [10:0] sp;
    reg [47:0] f;
    begin
        f = 48'd0;
        for (j = 0; j < 3; j = j + 1) begin
            sp = fl_cw_syn(d, j);
            // check bits = data syndrome (makes total syndrome zero);
            // overall bit gives even parity over data + check bits.
            f[j*FEC_CW_BITS +: FEC_CW_BITS] = {sp[10] ^ (^sp[9:0]), sp[9:0]};
        end
        flit_fec_encode = f;
    end
endfunction

// Decoder: returns {uncorrectable, corrected, fixed_data[1999:0]}
function [FL_PROT_W+1:0] flit_fec_decode;
    input [FL_PROT_W-1:0] d;
    input [47:0]          f;
    integer j, b, k;
    reg [10:0] sp, p;
    reg [9:0]  s, c_rx;
    reg        o, ovr_rx;
    reg [FL_PROT_W-1:0] fixed;
    reg        unc, cor, found;
    begin
        fixed = d; unc = 1'b0; cor = 1'b0; found = 1'b0;
        for (j = 0; j < 3; j = j + 1) begin
            c_rx   = f[j*FEC_CW_BITS +: 10];
            ovr_rx = f[j*FEC_CW_BITS + 10];
            sp     = fl_cw_syn(d, j);
            s      = sp[9:0] ^ c_rx;
            o      = sp[10] ^ (^c_rx) ^ ovr_rx;
            if (s == 10'd0 && !o) begin
                // clean
            end else if (o) begin
                if (s == 10'd0 || fl_is_pow2({1'b0, s})) begin
                    // error was in a check bit or the overall bit;
                    // data is already correct.
                    cor = 1'b1;
                end else begin
                    // single data-bit error at position s: find and flip it
                    p = 11'd2;
                    found = 1'b0;
                    for (b = j; b < FL_FEC_BYTES; b = b + 3) begin
                        for (k = 0; k < 8; k = k + 1) begin
                            p = p + 11'd1;
                            if (fl_is_pow2(p)) p = p + 11'd1;   // "if" not "while": p starts at 3, never two powers of 2 in a row past 2; Yosys rejects while here
                            if (p[9:0] == s && p[10] == 1'b0) begin
                                fixed[b*8+k] = ~fixed[b*8+k];
                                found = 1'b1;
                            end
                        end
                    end
                    // s beyond the last used position => cannot be a
                    // single error after all
                    if (found) cor = 1'b1;
                    else       unc = 1'b1;
                end
            end else begin
                unc = 1'b1;   // s != 0, even parity: double error
            end
        end
        flit_fec_decode = {unc, cor, fixed};
    end
endfunction
