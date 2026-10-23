`timescale 1ns/1ps
// ============================================================
// Module : tlp_tx_wide
// Layer  : Link layer -- Step 4 items 1 + 2 (512-bit TLP path, beat merging)
//
// Replaces the 512->32 serialiser, address_translation (inline here),
// ar_request_splitter, the 32-bit tlp_packetizer and tlp32_to_axis_packer
// on the stream TX path. Everything runs on 512-bit beats; a TLP leaves as
// ceil((4 + len) / 16) beats, one per clock.
//
//   S0  input register: translate (base[stream] + session offset, or the
//       absolute MSI-X address), drop + flag a beat whose base is not
//       programmed (err_xlat_miss) or whose address overflows (err_addr_ovf).
//   S1  merge unit: places the beat's DWs into an accumulator (one TLP being
//       built). A TLP is closed ("flushed") when
//         - it reaches MAX_PAYLOAD_DW,
//         - it reaches a 4 KB boundary (the beat is split there; the rest
//           starts the next TLP at the boundary: stat_split_occurred),
//         - the beat carried TLAST (end of session) or is an MSI-X,
//         - the next beat is not contiguous (other stream / other address),
//         - the input has been idle for MERGE_IDLE clocks.
//       MERGE_EN = 0 closes every TLP after one beat (old behaviour: one MWr
//       per beat, two if it crosses 4 KB).
//   S2  formatter: 4-DW MWr header (same fields as tlp_packetizer) + payload,
//       sent as 512-bit beats with TKEEP. A TLP starts only once the Posted
//       credit for it is granted (credit_manager, combinational grant).
//
// Order is strictly preserved: beats -> TLPs in arrival order, so an MSI-X
// (which closes the open TLP first) always follows the data it reports.
// ============================================================
module tlp_tx_wide #(
    parameter DATA_W         = 512,
    parameter ADDR_W         = 64,
    parameter SOFF_W         = 24,
    parameter MAX_PAYLOAD_DW = 64,        // power of two, 16..256
    parameter MERGE_EN       = 1,
    parameter MERGE_IDLE     = 8,
    parameter [15:0] REQ_ID  = 16'h0200
)(
    input  wire                 clk,
    input  wire                 rst_n,

    // beat source (WDB beat or MSI-X), valid/ready
    input  wire                 s_valid,
    output wire                 s_ready,
    input  wire [DATA_W-1:0]    s_data,
    input  wire [4:0]           s_ndw,         // 1..16 valid DWs (low DWs of the beat)
    input  wire                 s_sid,
    input  wire [SOFF_W-1:0]    s_offset,      // session offset (bytes)
    input  wire                 s_tlast,
    input  wire                 s_msix,
    input  wire [ADDR_W-1:0]    s_msix_addr,

    // per-stream base address registers (same port as address_translation)
    input  wire                 cfg_base_addr_wr,
    input  wire                 cfg_base_addr_id,
    input  wire [ADDR_W-1:0]    cfg_base_addr_data,
    input  wire                 cfg_base_addr_clr,

    // Posted credit (credit_manager): grant is combinational on valid
    output wire                 alloc_valid,
    output wire [9:0]           alloc_dw_len,
    input  wire                 alloc_grant,

    // TLP beats out
    output wire                 m_valid,
    output wire [DATA_W-1:0]    m_data,
    output wire [DATA_W/8-1:0]  m_keep,
    output wire                 m_last,
    input  wire                 m_ready,

    output reg                  err_xlat_miss,
    output reg                  err_addr_ovf,
    output reg                  stat_split_occurred,
    output wire                 busy
);

    localparam BDW   = DATA_W / 32;                 // 16 DWs per beat
    localparam MPS   = MAX_PAYLOAD_DW;
    localparam ACC_W = MPS * 32;
    localparam LW    = $clog2(MPS + 1);             // acc length width
    localparam TOT_DW = MPS + 4;
    localparam NBEAT  = (TOT_DW + BDW - 1) / BDW;   // beats per max TLP
    localparam BI_W   = (NBEAT > 1) ? $clog2(NBEAT) : 1;

    // ------------------------------------------------------------
    // S0: translate
    // ------------------------------------------------------------
    reg [ADDR_W-1:0] base_r [0:1];
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            base_r[0] <= {ADDR_W{1'b0}};
            base_r[1] <= {ADDR_W{1'b0}};
        end else if (cfg_base_addr_clr) begin
            base_r[0] <= {ADDR_W{1'b0}};
            base_r[1] <= {ADDR_W{1'b0}};
        end else if (cfg_base_addr_wr) begin
            base_r[cfg_base_addr_id] <= cfg_base_addr_data;
        end
    end

    wire [ADDR_W:0]  sum      = {1'b0, base_r[s_sid]} + {{(ADDR_W+1-SOFF_W){1'b0}}, s_offset};
    wire             unprog   = (base_r[s_sid] == {ADDR_W{1'b0}});

    reg              b_v;          // S1 holds a beat
    reg [DATA_W-1:0] b_data;
    reg [4:0]        b_ndw;
    reg [4:0]        b_c;          // DWs of the beat already placed
    reg [ADDR_W-1:0] b_addr;       // address of DW 0 of the beat
    reg              b_sid, b_tlast, b_msix;
    reg [ADDR_W-1:0] b_paddr;      // b_addr + 4*b_c, kept incrementally (timing)
    reg [4:0]        b_left;       // b_ndw - b_c
    reg [10:0]       b_topage;     // DWs from b_paddr to the next 4 KB boundary (1..1024)
    wire             b_done;       // the beat finishes this cycle

    assign s_ready = !b_v || b_done;
    wire   s_fire  = s_valid && s_ready;
    wire   s_bad   = !s_msix && (unprog || sum[ADDR_W]);

    // ------------------------------------------------------------
    // S1: merge unit
    // ------------------------------------------------------------
    reg              a_open;       // accumulator holds DWs
    reg              a_term;       // accumulator is complete: flush before anything else
    reg [ACC_W-1:0]  a_data;
    reg [LW-1:0]     a_len;
    reg [ADDR_W-1:0] a_addr;
    reg [ADDR_W-1:0] a_eaddr;      // a_addr + 4*a_len, kept incrementally (timing)
    reg [10:0]       a_room;       // MPS - a_len
    reg              a_sid, a_msix;
    reg [3:0]        idle_cnt;

    // E: closed TLP waiting for the formatter
    reg              e_v;
    reg [ACC_W-1:0]  e_data;
    reg [LW-1:0]     e_len;
    reg [ADDR_W-1:0] e_addr;
    wire             e_take;       // formatter takes E this cycle
    wire             e_free = !e_v || e_take;

    wire [ADDR_W-1:0] p_addr   = b_paddr;                          // address of the next DW to place (registered)
    wire [ADDR_W-1:0] a_end    = a_eaddr;                          // address after the accumulator's last DW (registered)
    wire              contig   = a_open && !a_term && !a_msix && !b_msix &&
                                 (b_sid == a_sid) && (a_end == p_addr);
    wire              appendable = (MERGE_EN != 0) && contig;
    wire              idle_out = a_open && !b_v && (idle_cnt >= MERGE_IDLE - 1);
    wire              need_flush = a_open && (a_term || (b_v && !appendable) || idle_out);
    wire              flush    = need_flush && e_free;

    // the beat can place a piece this cycle into a fresh or the open accumulator
    wire              into_new = !a_open || flush;
    wire              piece_go = b_v && (into_new || appendable) && !(a_open && a_term && !flush);
    wire [LW-1:0]     dst_len  = into_new ? {LW{1'b0}} : a_len;
    // Timing: every size below comes from registers (b_left, b_topage, a_room);
    // the piece for a fresh accumulator and for an append are formed in
    // parallel and only selected by into_new at the end.
    wire [4:0]        left     = b_left;
    wire [10:0]       to_page  = b_topage;                          // DWs to the 4 KB boundary
    function [10:0] min3; input [10:0] x, y, z; reg [10:0] m; begin
        m = (x < y) ? x : y; min3 = (m < z) ? m : z; end endfunction
    wire [10:0]       pn       = min3({6'd0, left}, to_page, MPS);
    wire [10:0]       pa       = min3({6'd0, left}, to_page, a_room);
    wire [10:0]       piece    = into_new ? pn : pa;
    wire [10:0]       room_mps = into_new ? MPS : a_room;
    wire              pcut_n   = (pn == to_page) && (to_page < MPS);
    wire              pcut_a   = (pa == to_page) && (to_page < a_room);
    wire              page_cut = into_new ? pcut_n : pcut_a;            // closes at a 4 KB boundary
    wire              mps_cut  = into_new ? (pn == MPS) : (pa == a_room);
    assign            b_done   = piece_go && (piece[4:0] == left);
    wire              new_term = (MERGE_EN == 0) || page_cut || mps_cut || b_msix ||
                                 (b_done && b_tlast);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            b_v <= 1'b0; b_data <= {DATA_W{1'b0}}; b_ndw <= 5'd0; b_c <= 5'd0;
            b_left <= 5'd0; b_topage <= 11'd1024;
            b_addr <= {ADDR_W{1'b0}}; b_paddr <= {ADDR_W{1'b0}}; b_sid <= 1'b0; b_tlast <= 1'b0; b_msix <= 1'b0;
            err_xlat_miss <= 1'b0; err_addr_ovf <= 1'b0;
        end else begin
            err_xlat_miss <= s_fire && !s_msix && unprog;
            err_addr_ovf  <= s_fire && !s_msix && !unprog && sum[ADDR_W];
            if (s_fire && !s_bad && s_ndw != 5'd0) begin
                b_v     <= 1'b1;
                b_data  <= s_data;
                b_ndw   <= s_ndw;
                b_c     <= 5'd0;
                b_addr  <= s_msix ? s_msix_addr : sum[ADDR_W-1:0];
                b_paddr <= s_msix ? s_msix_addr : sum[ADDR_W-1:0];
                b_left  <= s_ndw;
                b_topage <= 11'd1024 - {1'b0, (s_msix ? s_msix_addr[11:2] : sum[11:2])};
                b_sid   <= s_sid;
                b_tlast <= s_tlast;
                b_msix  <= s_msix;
            end else if (b_done) begin
                b_v <= 1'b0;
            end else if (b_v && piece_go) begin
                b_c     <= b_c + piece[4:0];
                b_paddr <= b_paddr + {piece[4:0], 2'b00};
                b_left  <= b_left - piece[4:0];
                b_topage <= (b_topage == piece) ? 11'd1024 : b_topage - piece;
            end
        end
    end

    // beat DWs [b_c .. b_c+piece-1] -> accumulator DWs [dst_len ..].
    // Per accumulator DW: a 16:1 pick from the beat (select from registers
    // b_c / a_len only) and a write enable from the piece range.
    wire [6:0]        d_app    = {2'd0, b_c} - {{(7-LW){1'b0}}, a_len};   // beat DW = acc DW + d
    reg  [ACC_W-1:0]  a_next;
    integer q;
    reg [3:0] sel_n, sel_a;
    always @(*) begin
        for (q = 0; q < MPS; q = q + 1) begin
            sel_n = q + b_c;                              // fresh accumulator: acc DW q <- beat DW q + b_c
            sel_a = q + d_app;                            // append: acc DW q <- beat DW q - a_len + b_c
            if (into_new)
                a_next[q*32 +: 32] = (q < pn) ? b_data[sel_n*32 +: 32] : 32'd0;
            else
                a_next[q*32 +: 32] = (q >= a_len && q < a_len + pa) ? b_data[sel_a*32 +: 32] : a_data[q*32 +: 32];
        end
    end

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            a_open <= 1'b0; a_term <= 1'b0; a_data <= {ACC_W{1'b0}}; a_len <= {LW{1'b0}};
            a_room <= MPS;
            a_addr <= {ADDR_W{1'b0}}; a_eaddr <= {ADDR_W{1'b0}}; a_sid <= 1'b0; a_msix <= 1'b0; idle_cnt <= 4'd0;
            e_v <= 1'b0; e_data <= {ACC_W{1'b0}}; e_len <= {LW{1'b0}}; e_addr <= {ADDR_W{1'b0}};
            stat_split_occurred <= 1'b0;
        end else begin
            stat_split_occurred <= 1'b0;
            if (e_take) e_v <= 1'b0;

            if (flush) begin
                e_v    <= 1'b1;
                e_data <= a_data;
                e_len  <= a_len;
                e_addr <= a_addr;
                a_open <= 1'b0;
                a_term <= 1'b0;
                a_data <= {ACC_W{1'b0}};
                a_len  <= {LW{1'b0}};
            end

            if (piece_go) begin
                a_open <= 1'b1;
                a_term <= new_term;
                a_data <= a_next;
                a_room <= room_mps - piece;
                a_len  <= dst_len + piece[LW-1:0];
                a_eaddr <= p_addr + {piece[4:0], 2'b00};
                if (into_new) begin
                    a_addr <= p_addr;
                    a_sid  <= b_sid;
                    a_msix <= b_msix;
                end
                if (page_cut && piece[4:0] != left) stat_split_occurred <= 1'b1;
            end

            if (b_v || !a_open) idle_cnt <= 4'd0;
            else if (idle_cnt != 4'hF) idle_cnt <= idle_cnt + 4'd1;
        end
    end

    // ------------------------------------------------------------
    // S2: formatter
    // ------------------------------------------------------------
    reg                    f_v;
    reg [NBEAT*DATA_W-1:0] f_tlp;      // header + payload (zero padded to whole beats)
    reg [LW+1:0]           f_tot;      // total DWs
    reg [BI_W-1:0]         f_bi;       // beat index

    wire [LW+1:0] beat_lo   = {f_bi, 4'd0};
    wire          f_lastb   = (f_tot - beat_lo) <= BDW;
    wire          f_adv     = f_v && m_ready;
    wire          f_free    = !f_v || (f_adv && f_lastb);
    assign alloc_valid      = e_v && f_free;
    assign alloc_dw_len     = e_len10;
    assign e_take           = alloc_valid && alloc_grant;

    wire [9:0]  e_len10 = e_len;
    wire [31:0] h0 = {3'b011, 5'b00000, 1'b0, 3'b000, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0, 1'b0,
                      2'b00, 2'b00, e_len10};
    wire [31:0] h1 = {REQ_ID, 8'h00, ((e_len == 1) ? 4'b0000 : 4'b1111), 4'b1111};
    wire [31:0] h2 = e_addr[63:32];
    wire [31:0] h3 = {e_addr[31:2], 2'b00};

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            f_v <= 1'b0; f_tlp <= {NBEAT*DATA_W{1'b0}}; f_tot <= 0; f_bi <= {BI_W{1'b0}};
        end else begin
            if (f_adv) begin
                if (f_lastb) f_v <= 1'b0;
                else         f_bi <= f_bi + 1'b1;
            end
            if (e_take) begin
                f_v   <= 1'b1;
                f_tlp <= {{(NBEAT*DATA_W-TOT_DW*32){1'b0}}, e_data, h3, h2, h1, h0};
                f_tot <= e_len + 4;
                f_bi  <= {BI_W{1'b0}};
            end
        end
    end

    wire [LW+1:0] nleft = f_tot - beat_lo;
    reg  [DATA_W/8-1:0] keep_c;
    integer kk;
    always @(*) begin
        keep_c = {DATA_W/8{1'b0}};
        for (kk = 0; kk < BDW; kk = kk + 1)
            if (kk < nleft) keep_c[kk*4 +: 4] = 4'hF;
    end

    assign m_valid = f_v;
    assign m_data  = f_tlp[f_bi*DATA_W +: DATA_W];
    assign m_keep  = keep_c;
    assign m_last  = f_lastb;
    assign busy    = b_v || a_open || e_v || f_v;

endmodule
