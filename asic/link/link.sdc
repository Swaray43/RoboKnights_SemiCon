# Period comes from CLOCK_PERIOD in config.yaml, so the two never disagree.
set period $::env(CLOCK_PERIOD)
create_clock -name clk -period $period [get_ports clk]
set io [expr $period * 0.2]
set_input_delay  $io -clock clk [get_ports {s_axis_* cfg_base_addr_* msix_req_valid msix_req_addr* msix_req_data* phy_tx_ready phy_rx_* m_axi_awready cpl_tready rst_n}]
# Decode window/AXI base: quasi-static (set at config time, never during traffic)
set_false_path -from [get_ports {cfg_window_base* cfg_window_mask* cfg_axi_base*}]
set_output_delay $io -clock clk [all_outputs]
set_clock_uncertainty 0.25 [all_clocks]
# PDK-neutral load/driver: LibreLane provides these per PDK (sky130A, gf180mcuD, ihp-sg13g2).
if {[info exists ::env(OUTPUT_CAP_LOAD)]} { set_load [expr $::env(OUTPUT_CAP_LOAD) / 1000.0] [all_outputs] }
if {[info exists ::env(SYNTH_DRIVING_CELL)]} {
    set dc [split $::env(SYNTH_DRIVING_CELL) "/"]
    set_driving_cell -lib_cell [lindex $dc 0] -pin [lindex $dc 1] [all_inputs]
}
