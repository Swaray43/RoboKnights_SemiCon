# Legacy link-layer RTL (not in any build)

| File | Why parked |
|---|---|
| `pcie_core_top.v` | Old request-API top with the pre-Flit DLL. Replaced by `link_layer_top.v`. |
| `DLLP_parser.v`, `flit_framer.v`, `fec_encoder_decoder.v` | Pre-Flit-mode DLL blocks; ACK/NAK, framing and FEC now live in `link_layer_tx/rx.v` + `flit_defs.vh`. |
| `ic2link_tuser_remap.v` | TUSER reorder shim. Retired in Phase 2: `interconnect_top` now emits the link's native `{tlast_flag, session_offset, stream_id}`. |
