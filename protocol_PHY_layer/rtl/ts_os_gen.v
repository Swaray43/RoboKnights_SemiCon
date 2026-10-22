`timescale 1ns/1ps
// -----------------------------------------------------------------------------
// ts_os_gen - byte k of a TS1 or TS2 ordered set for each of the 4 TX lanes.
// (Replaces ts1_os_gen; adds TS2 for the Configuration state of pcie_ltssm.)
//
// Same 16-byte format ts_ordered_set_detector expects:
//   byte 0      COM (0xBC)
//   byte 1      link number (0x00)
//   byte 2      lane number = physical TX lane index (0,1,2,3)
//   bytes 3-9   0x00
//   bytes 10-15 identifier: TS1 = 0x4A, TS2 = 0x45
// A crossed channel shows up at the far end as lane numbers 3,2,1,0.
// -----------------------------------------------------------------------------
module ts_os_gen (
input  wire [3:0] idx,
input  wire       ts2,        // 0 = TS1, 1 = TS2
output wire [7:0] byte_l0,
output wire [7:0] byte_l1,
output wire [7:0] byte_l2,
output wire [7:0] byte_l3
);

localparam [7:0] COM    = 8'hBC;
localparam [7:0] TS1_ID = 8'h4A;
localparam [7:0] TS2_ID = 8'h45;

wire [7:0] id = ts2 ? TS2_ID : TS1_ID;

function [7:0] ts_byte;
  input [3:0] k;
  input [7:0] lane;
  input [7:0] ident;
  begin
    if (k == 4'd0)       ts_byte = COM;
    else if (k == 4'd2)  ts_byte = lane;
    else if (k >= 4'd10) ts_byte = ident;
    else                 ts_byte = 8'h00;
  end
endfunction

assign byte_l0 = ts_byte(idx, 8'd0, id);
assign byte_l1 = ts_byte(idx, 8'd1, id);
assign byte_l2 = ts_byte(idx, 8'd2, id);
assign byte_l3 = ts_byte(idx, 8'd3, id);
endmodule
