// ============================================================
// axis_tuser_defs.vh -- single source of truth for the AXI4-Stream
// TUSER layout crossing the interconnect_top <-> link_layer seam
// (ticket G7, Aditya + Swaray).
//
// `include this INSIDE a module body (same convention as flit_defs.vh),
// AFTER the module has declared its own SOFF_W and TUSER_W parameters
// (every module that already touches TUSER has these two parameters --
// this file does not introduce new ones, it just stops each module from
// re-deriving the slice math and the field order by hand).
//
// Layout (link-native order, MSB to LSB) -- UNCHANGED from what's live
// today in interconnect_top.v / write_data_buffer.v / address_translation.v:
//
//   TUSER[TUSER_W-1]            = tlast_flag     (1 bit, mirrors TLAST)
//   TUSER[SOFF_W : 1]           = session_offset (SOFF_W bits)
//   TUSER[0]                   = stream_id       (1 bit: 0=SmartNIC, 1=ML)
//
//   TUSER_W must equal SOFF_W + 2.
//
// -- Why this file exists (ticket G7) --
// This exact layout used to live in ONE place: a shim module,
// ic2link_tuser_remap.v, that translated interconnect_top's original
// {stream_id, session_offset} order into the link layer's
// {tlast_flag, session_offset, stream_id} order. Phase 2 retired that
// shim and instead made interconnect_top emit the link-native order
// directly -- which meant the layout stopped living in one place and
// started living as a hand-copied comment + hand-written bit-slice in
// every module that touches it: interconnect_top.v (packs it),
// write_data_buffer.v and address_translation.v (unpack it), plus
// width-only copies in ar_request_splitter.v, stream_descriptor_gen.v,
// link_phy_bridge_top.v, link_layer_top.v, and several testbenches'
// own `make_tuser` helpers. Nothing enforced that all of these agreed,
// which is exactly the bug class that the original shim existed to
// prevent (see ic2link_tuser_remap.v's own header: before it existed,
// every beat was silently misrouted as stream 0 at half its real
// offset). Phase 2 removed the shim but kept the same hand-duplication
// risk, just moved it from "the shim's one file" to "every file."
//
// The fix: every module that packs or unpacks TUSER now `includes this
// file and calls tuser_pack / tuser_stream_id / tuser_session_offset /
// tuser_tlast_flag instead of writing its own concatenation or slice.
// Changing the layout (wider session_offset, a multi-bit stream_id for
// a 3rd stream, a reordered field) means editing ONLY this file; every
// includer picks up the new slice positions automatically, and the
// $fatal width checks already present in write_data_buffer.v /
// address_translation.v / link_phy_bridge_top.v (TUSER_W == SOFF_W+2)
// keep catching anyone who still hardcodes TUSER_W=26 locally instead
// of deriving it.
//
// Build note: this file lives in shared/, not inside either layer's own
// directory, since both sides `include it. Compiling anything that
// `includes it needs an include path to shared/ --
// bridge_top.f already carries +incdir+shared for the full-bridge
// build; src-link_layer/hdl/run_all_link.py's per-bench -I flag has
// been extended the same way. A standalone interconnect-layer build
// needs the same `-I <repo_root>/shared` (or equivalent `+incdir+`)
// added to its command line.
// ============================================================

// synthesis translate_off
initial if (TUSER_W != SOFF_W + 2)
    $fatal(1, "axis_tuser_defs.vh: TUSER_W(%0d) must equal SOFF_W(%0d)+2 = %0d -- these two parameters disagree in this instance",
           TUSER_W, SOFF_W, SOFF_W + 2);
// synthesis translate_on

localparam TUSER_SID_BIT   = 0;                     // stream_id bit position
localparam TUSER_SOFF_LO   = TUSER_SID_BIT + 1;     // = 1
localparam TUSER_SOFF_HI   = TUSER_SOFF_LO + SOFF_W - 1; // = SOFF_W
localparam TUSER_TLAST_BIT = TUSER_SOFF_HI + 1;     // = SOFF_W+1 = TUSER_W-1

// ---- pack: build a TUSER word from its three fields ----
function [TUSER_W-1:0] tuser_pack;
    input                  tlast_flag;
    input [SOFF_W-1:0]     session_offset;
    input                  stream_id;
    begin
        tuser_pack = {tlast_flag, session_offset, stream_id};
    end
endfunction

// ---- unpack: pull one field back out of a TUSER word ----
function tuser_stream_id;
    input [TUSER_W-1:0] tuser;
    begin
        tuser_stream_id = tuser[TUSER_SID_BIT];
    end
endfunction

function [SOFF_W-1:0] tuser_session_offset;
    input [TUSER_W-1:0] tuser;
    begin
        tuser_session_offset = tuser[TUSER_SOFF_HI:TUSER_SOFF_LO];
    end
endfunction

function tuser_tlast_flag;
    input [TUSER_W-1:0] tuser;
    begin
        tuser_tlast_flag = tuser[TUSER_TLAST_BIT];
    end
endfunction
