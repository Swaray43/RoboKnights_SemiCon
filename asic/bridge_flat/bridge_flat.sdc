# Period comes from CLOCK_PERIOD in config.yaml, so the two never disagree.
set period $::env(CLOCK_PERIOD)
create_clock -name clk  -period $period [get_ports clk]
create_clock -name aclk -period $period [get_ports aclk]
set_clock_groups -asynchronous -group [get_clocks clk] -group [get_clocks aclk]
set io [expr $period * 0.2]
set_input_delay  $io -clock aclk [get_ports {s0_* s1_*}]
set_output_delay $io -clock aclk [get_ports {s0_tready s1_tready session_active* session_overflow*}]
set_false_path -from [get_ports perst_n]
set_input_delay  $io -clock clk  [get_ports {m_rx_axis_tready ts_* hot_reset_request flr_request updatefc_received cfg_wr_en cfg_rd_en cfg_addr* cfg_wdata* bar0_wr_en bar0_rd_en bar0_addr* bar0_wdata* tbl_* rx_level*}]
set_output_delay $io -clock clk  [get_ports {m_rx_axis_tvalid m_rx_axis_tdata* m_rx_axis_tkeep* m_rx_axis_tlast link_trained datapath_rst_n axi_rst_n cfg_rdata* cfg_cpl_valid bar0_rdata* bar0_rd_valid tx_level* err_* crc_err fec_err phy_*}]
set_clock_uncertainty 0.25 [all_clocks]
# PDK-neutral load/driver: LibreLane provides these per PDK (sky130A, gf180mcuD, ihp-sg13g2).
if {[info exists ::env(OUTPUT_CAP_LOAD)]} { set_load [expr $::env(OUTPUT_CAP_LOAD) / 1000.0] [all_outputs] }
if {[info exists ::env(SYNTH_DRIVING_CELL)]} {
    set dc [split $::env(SYNTH_DRIVING_CELL) "/"]
    set_driving_cell -lib_cell [lindex $dc 0] -pin [lindex $dc 1] [all_inputs]
}
