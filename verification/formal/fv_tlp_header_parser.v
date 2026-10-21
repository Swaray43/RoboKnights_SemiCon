// Formal harness for src-link_layer/hdl/tlp_header_parser.v (Swaray's RTL, unmodified).
//
// Environment: a well-formed stream of 32-bit DWs from the RX unpacker.
//   - every TLP starts with rx_tlp_sof; no beat outside a TLP; no SOF inside one
//   - a TLP has at least its header (3 DW, or 4 DW when Fmt[0] = DW0[29] = 1);
//     after that, EOF may come on any DW (payload length is free)
//   - valid may drop between any two DWs (gaps)
//   - completions (Type 0_1010 Cpl/CplD and 0_1011 CplLk/CplDLk) are always 3DW
//   - all data bits are free
//
// Properties (outputs are registered, so they are checked one clock after the beat):
//   T1  parsed_hdr_done pulses EXACTLY when the last header DW of a TLP was
//       received: DW2 for a 3DW request or a completion, DW3 for a 4DW request.
//       So: exactly once per TLP, never on payload, never outside a TLP.
//   T2  with hdr_done on a request: parsed_addr = {32'b0, DW2[31:2], 2'b00}
//       (3DW) or {DW2, DW3[31:2], 2'b00} (4DW)
//   T3  with hdr_done: parsed_type = DW0[28:24], parsed_length_dw = DW0[9:0]
//   T4  with hdr_done: parsed_tag = {DW0[23], DW0[19], Tag[7:0]}, with Tag[7:0]
//       from DW1[15:8] (request) or DW2[15:8] (completion)
//   T5  parsed_rx_* is the input delayed by exactly one clock
//   T6  with hdr_done: parsed_has_data = Fmt[1] = DW0[30], so downstream can
//       tell MWr from MRd (ticket P2-5; before the fix the parser had no such
//       output and every MRd looked like an MWr)
//   CplLk (0_1011) is in the environment: since the F6 fix it takes the
//   completion path (tag from DW2), and T1/T4 hold for it too.
module fv_tlp_header_parser (
    input wire        clk,
    input wire        v,        // rx_tlp_valid
    input wire [31:0] d,        // rx_tlp_data
    input wire        s,        // rx_tlp_sof
    input wire        e         // rx_tlp_eof
);
reg rst_n = 1'b0;
always @(posedge clk) rst_n <= 1'b1;

wire        o_valid, o_sof, o_eof, o_done;
wire [31:0] o_data;
wire [4:0]  o_type;
wire [9:0]  o_len;
wire [9:0]  o_tag;
wire [6:0]  o_lower;
wire [2:0]  o_cst;
wire [63:0] o_addr;
wire        o_hasdata;

tlp_header_parser #(.TAG_WIDTH(10), .PCIE_ADDR_W(64)) dut (
    .clk(clk), .rst_n(rst_n),
    .rx_tlp_valid(v), .rx_tlp_data(d), .rx_tlp_sof(s), .rx_tlp_eof(e),
    .parsed_rx_valid(o_valid), .parsed_rx_sof(o_sof), .parsed_rx_eof(o_eof),
    .parsed_rx_data(o_data), .parsed_type(o_type), .parsed_length_dw(o_len),
    .parsed_tag(o_tag), .parsed_lower_addr(o_lower), .parsed_cpl_status(o_cst),
    .parsed_addr(o_addr), .parsed_hdr_done(o_done), .parsed_has_data(o_hasdata));

// ---------------- environment shadow ----------------
reg        in_tlp = 1'b0;
reg [2:0]  pos    = 3'd0;     // index of the NEXT DW of the current TLP (saturates at 7)
reg [31:0] dw0 = 0, dw1 = 0, dw2 = 0;
wire       is4  = dw0[29];
wire       cpl  = (dw0[28:24] == 5'b01010) || (dw0[28:24] == 5'b01011);
wire [2:0] hdr_last = (cpl || !is4) ? 3'd2 : 3'd3;   // index of the last header DW

wire sof_cpl = (d[28:24] == 5'b01010) || (d[28:24] == 5'b01011);

always @(*) begin
    if (!rst_n)              assume(!v);
    if (!v)                  assume(!s && !e);
    if (v && !in_tlp)        assume(s);                   // nothing outside a TLP
    if (v && in_tlp)         assume(!s);                  // no SOF inside a TLP
    if (v && e)              assume(in_tlp && !s && pos >= hdr_last);   // at least the header
    if (v && s && sof_cpl)   assume(d[29] == 1'b0);       // completions are 3DW
end

// expected outputs for THIS beat (seen on the outputs next clock)
wire        beat_in   = v && in_tlp && !s;
wire        exp_done  = beat_in && (pos == hdr_last);
wire [63:0] exp_addr  = is4 ? {dw2, d[31:2], 2'b00} : {32'd0, d[31:2], 2'b00};
wire [9:0]  exp_tag   = {dw0[23], dw0[19], cpl ? d[15:8] : dw1[15:8]};

reg        done_q = 0, cpl_q = 0, hasdata_q = 0;
reg [63:0] addr_q = 0;
reg [9:0]  tag_q = 0, len_q = 0;
reg [4:0]  type_q = 0;
reg        v_q = 0, s_q = 0, e_q = 0;
reg [31:0] d_q = 0;

always @(posedge clk) begin
    v_q <= v; s_q <= s; e_q <= e; d_q <= d;
    done_q <= rst_n && exp_done;
    if (exp_done) begin
        cpl_q <= cpl; addr_q <= exp_addr; tag_q <= exp_tag;
        type_q <= dw0[28:24]; len_q <= dw0[9:0]; hasdata_q <= dw0[30];
    end
    if (rst_n && v) begin
        if (s) begin
            in_tlp <= 1'b1; pos <= 3'd1; dw0 <= d;
        end else if (in_tlp) begin
            if (pos == 3'd1) dw1 <= d;
            if (pos == 3'd2) dw2 <= d;
            if (pos != 3'd7) pos <= pos + 3'd1;
            if (e) begin in_tlp <= 1'b0; pos <= 3'd0; end
        end
    end
end

reg past_ok = 1'b0;
always @(posedge clk) past_ok <= rst_n;

always @(*) begin
    if (past_ok) begin
        assert(o_done == done_q);                                    // T1
        if (o_done && !cpl_q) assert(o_addr == addr_q);              // T2
        if (o_done) assert(o_type == type_q && o_len == len_q);      // T3
        if (o_done) assert(o_tag == tag_q);                          // T4
        assert(o_valid == v_q && o_sof == s_q && o_eof == e_q);      // T5
        if (v_q) assert(o_data == d_q);
        if (o_done) assert(o_hasdata == hasdata_q);                  // T6
    end
end

// ---------------- reachability ----------------
always @(posedge clk) if (past_ok) begin
    cover(o_done && !cpl_q && type_q == 5'b00000 && hasdata_q && addr_q[63:32] == 0);   // 3DW MWr
    cover(o_done && !cpl_q && addr_q[63:32] != 0);                                       // 4DW request
    cover(o_done && cpl_q);                                                              // completion
    cover(o_done && $past(o_done, 3));                                                   // back-to-back TLPs
    cover(o_done && !$past(v, 2));                                                      // gap just before the last header DW
end
endmodule
