// files.f - the ONE RTL file list for the Protocol-to-PHY layer (P5).
// One module per file. Paths are relative to this folder.
// Testbenches add tb/models/*.v (simulation-only models) and their own tb file.
// Not listed on purpose: legacy/ (superseded, see legacy/*.v headers) and
// formal/ (proof harnesses).

rtl/axi_mrd_generator.v
rtl/bar0_dma_regs.v
rtl/byte_to_symbol.v
rtl/elastic_fifo.v
rtl/fifo_scoreboard.v
rtl/flit_lane_destriper.v
rtl/flit_lane_striper.v
rtl/flit_sof_strip.v
rtl/gray_dec.v
rtl/gray_enc.v
rtl/lane_reversal_detector.v
rtl/lane_reversal_latch.v
rtl/lane_swap_mux.v
rtl/mrd_completion_handler.v
rtl/mrd_outstanding_table.v
rtl/msix_controller.v
rtl/pam4_demap.v
rtl/pam4_map.v
rtl/pcie_config_space.v
rtl/pcie_reset_controller.v
rtl/pcie_tag_manager.v
rtl/perf_counters.v
rtl/pulse_sync.v
rtl/rst_sync.v
rtl/session_arm_ctrl.v
rtl/symbol_to_byte.v
rtl/ts_ordered_set_detector.v
rtl/tx_elastic_deskew.v
rtl/pcie_phy_tx.v
rtl/pcie_phy_rx.v
rtl/lane_tx.v
rtl/lane_rx.v
rtl/pcie_phy_top.v
