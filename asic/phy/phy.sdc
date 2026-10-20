# Period comes from CLOCK_PERIOD in config.yaml, so the two never disagree.
set period $::env(CLOCK_PERIOD)
create_clock -name clk  -period $period [get_ports clk]
create_clock -name aclk -period $period [get_ports aclk]
# Unrelated clocks. Crossings: pulse_sync (session start/end), 2-flop sync (overflow).
set_clock_groups -asynchronous -group [get_clocks clk] -group [get_clocks aclk]
set io [expr $period * 0.2]
set_input_delay  $io -clock aclk [get_ports {session_end_aclk* session_overflow_aclk* aclk_rst_n}]
set_output_delay $io -clock aclk [get_ports {session_start_aclk*}]
# arb_weight_ch0/1 and session_limit are written in the clk domain and read on aclk WITHOUT a
# synchroniser: they are quasi-static (host writes them before arming, never during a session).
set_false_path -to [get_ports {arb_weight_ch0* arb_weight_ch1* session_limit*}]
# perst_n is asynchronous (PCIe connector)
set_false_path -from [get_ports perst_n]
set_input_delay  $io -clock clk [get_ports {ts_* hot_reset_request flr_request updatefc_received cfg_wr_en cfg_rd_en cfg_addr* cfg_wdata* tbl_* dma_complete_* mwr_ready phy_tx_data* phy_tx_valid phy_tx_sof phy_tx_eof rx_level* bar0_* session_done*}]
set_output_delay $io -clock clk [get_ports {link_trained lane_reversed_locked datapath_rst_n axi_rst_n flr_active cfg_rdata* cfg_cpl_valid tbl_rd_* pba* mwr_valid mwr_addr* mwr_data* phy_tx_ready tx_level* phy_rx_* phy_tx_idle phy_framing_err phy_lane_err phy_ts_sending ltssm_state* phy_deskew_locked bar0_rdata* bar0_rd_valid cfg_base_addr_* cfg_tc_* session_valid*}]
set_clock_uncertainty 0.25 [all_clocks]
# PDK-neutral load/driver: LibreLane provides these per PDK (sky130A, gf180mcuD, ihp-sg13g2).
if {[info exists ::env(OUTPUT_CAP_LOAD)]} { set_load [expr $::env(OUTPUT_CAP_LOAD) / 1000.0] [all_outputs] }
if {[info exists ::env(SYNTH_DRIVING_CELL)]} {
    set dc [split $::env(SYNTH_DRIVING_CELL) "/"]
    set_driving_cell -lib_cell [lindex $dc 0] -pin [lindex $dc 1] [all_inputs]
}
