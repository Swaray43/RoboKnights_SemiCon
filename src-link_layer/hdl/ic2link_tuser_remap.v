`timescale 1ns/1ps

// ============================================================
// Module : ic2link_tuser_remap
// Seam A fix (bridge review): interconnect -> link layer TUSER order
//
//   interconnect_top m_tuser[24:0] = {stream_id, session_offset[23:0]}
//   link layer s_axis_tuser[25:0]  = {tlast_flag, session_offset[23:0], stream_id}
//
// Wired straight across, every beat was routed as SmartNIC (offset bit 0,
// almost always 0, became stream_id) and its address came out at half the
// real offset (plus 8 MB for ML beats). Put this between the two in the
// bridge top. Pure wiring, no logic.
// ============================================================

module ic2link_tuser_remap #(
    parameter SOFF_W = 24
)(
    input  wire [SOFF_W:0]   ic_tuser,     // {stream_id, session_offset}
    input  wire              ic_tlast,
    output wire [SOFF_W+1:0] link_tuser    // {tlast_flag, session_offset, stream_id}
);
    assign link_tuser = {ic_tlast, ic_tuser[SOFF_W-1:0], ic_tuser[SOFF_W]};
endmodule