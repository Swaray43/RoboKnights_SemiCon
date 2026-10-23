# src-link_layer/hdl/files.f -- link-layer RTL + unit testbenches (read by run_all_link.py)
#   rtl <path>                      -- RTL source, always compiled
#   tb  <name> <bench.v> [dep...]   -- testbench; deps = extra RTL only that bench needs
# Paths relative to src-link_layer/hdl.

# ---- link-layer RTL ----
rtl flit_defs.vh
rtl tlp_packetizer.v
rtl tlp_tx_wide.v
rtl replay_buffer.v
rtl link_layer_tx.v
rtl link_layer_rx.v
rtl data_link_layer_top.v
rtl flit_payload_builder.v
rtl rx_tlp_delimiter.v
rtl write_data_buffer.v
rtl stream_tx_path_top.v
rtl ar_request_splitter.v
rtl tlp32_to_axis_packetizer.v
rtl address_translation.v
rtl stream_descriptor_gen.v
rtl cfg_base_addr_bridge.v
rtl tlp_header_parser.v
rtl pcie_rx_engine.v
rtl axi_pcie_reorder_buffer.v
rtl pcie_address_translator.v
rtl credit_manager.v
rtl tx_engine.v
rtl tag_manager.v
rtl link_layer_top.v

# ---- unit testbenches (each prints RESULT: PASS/FAIL) ----
tb tlp_packetizer          testbench/tb_tlp_packetizer.v
tb tlp_packetizer_mps      testbench/tb_tlp_packetizer_mps.v
tb replay_buffer           testbench/tb_replay_buffer.v
tb pcie_rx_engine          testbench/tb_pcie_rx_engine.v
tb link_layer_loopback     testbench/tb_link_layer_loopback.v
tb credit_manager          testbench/tb_credit_manager.v
tb credit_gate_proof       testbench/tb_credit_gate_proof.v
tb axi_pcie_reorder_buffer testbench/tb_axi_pcie_reorder_buffer.v
tb tag_manager             testbench/tb_tag_manager.v
tb tx_engine               testbench/tb_tx_engine.v
tb cfg_base_addr_bridge    testbench/tb_cfg_base_addr_bridge.v
tb stream_path             testbench/tb_stream_path.v
tb address_translation     testbench/tb_address_translation.v
tb ar_request_splitter     testbench/tb_ar_request_splitter.v
tb link_layer_top          testbench/tb_link_layer_top.v
tb link_phy_bridge_top     testbench/tb_link_phy_bridge_top.v link_phy_bridge_top.v PHY:pcie_phy_top.v PHY:pcie_ltssm.v PHY:ts_os_gen.v PHY:rx_lane_deskew.v PHY:ts_ordered_set_detector.v PHY:lane_reversal_detector.v PHY:lane_reversal_latch.v PHY:pcie_reset_controller.v PHY:pcie_phy_tx.v PHY:pcie_phy_rx.v PHY:pcie_phy_tx_w.v PHY:pcie_phy_rx_w.v PHY:ts_os_detector_w.v PHY:pcie_config_space.v PHY:bar0_dma_regs.v PHY:session_arm_ctrl.v PHY:pulse_sync.v PHY:msix_controller.v PHY:tx_elastic_deskew.v PHY:elastic_fifo.v PHY:err_counters.v PHY:flit_lane_striper.v PHY:flit_lane_destriper.v PHY:flit_sof_strip.v PHY:lane_tx.v PHY:lane_rx.v PHY:lane_swap_mux.v PHY:byte_to_symbol.v PHY:symbol_to_byte.v PHY:gray_enc.v PHY:gray_dec.v PHY:pam4_map.v PHY:pam4_demap.v PHY:rst_sync.v
# G1/G3 end to end: host MWr through the link lands in BAR0 (TC reg), XLAT_MISS counts and clears
tb g1_host_bar0 testbench/tb_g1_host_bar0.v link_phy_bridge_top.v PHY:pcie_phy_top.v PHY:pcie_ltssm.v PHY:ts_os_gen.v PHY:rx_lane_deskew.v PHY:ts_ordered_set_detector.v PHY:lane_reversal_detector.v PHY:lane_reversal_latch.v PHY:pcie_reset_controller.v PHY:pcie_phy_tx.v PHY:pcie_phy_rx.v PHY:pcie_phy_tx_w.v PHY:pcie_phy_rx_w.v PHY:ts_os_detector_w.v PHY:pcie_config_space.v PHY:bar0_dma_regs.v PHY:session_arm_ctrl.v PHY:pulse_sync.v PHY:msix_controller.v PHY:tx_elastic_deskew.v PHY:elastic_fifo.v PHY:err_counters.v PHY:flit_lane_striper.v PHY:flit_lane_destriper.v PHY:flit_sof_strip.v PHY:lane_tx.v PHY:lane_rx.v PHY:lane_swap_mux.v PHY:byte_to_symbol.v PHY:symbol_to_byte.v PHY:gray_enc.v PHY:gray_dec.v PHY:pam4_map.v PHY:pam4_demap.v PHY:rst_sync.v
