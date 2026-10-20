# Period comes from CLOCK_PERIOD in config.yaml, so the two never disagree.
set period $::env(CLOCK_PERIOD)
create_clock -name aclk     -period $period [get_ports aclk]
create_clock -name pcie_clk -period $period [get_ports pcie_clk]
# Unrelated clocks. Every crossing is the Gray-pointer CDC FIFO (2-flop synchronised pointers).
set_clock_groups -asynchronous -group [get_clocks aclk] -group [get_clocks pcie_clk]

# Block I/O budget: 20% of the period each side (refine when integrating).
set io [expr $period * 0.2]
# aclk side: both AXI4-Stream slaves, session control, weights
set_input_delay  $io -clock aclk     [get_ports {s0_* s1_* ch0_session_start ch0_session_limit* ch1_session_start ch1_session_limit* weight0* weight1* arstn}]
set_output_delay $io -clock aclk     [get_ports {s0_tready s1_tready ch0_session_active ch0_session_overflow ch1_session_active ch1_session_overflow}]
# pcie_clk side: FIFO read port
set_input_delay  $io -clock pcie_clk [get_ports {m_tready pcie_rstn}]
set_output_delay $io -clock pcie_clk [get_ports {m_tdata* m_tkeep* m_tuser* m_tlast m_tvalid}]
set_clock_uncertainty 0.25 [all_clocks]
# PDK-neutral load/driver: LibreLane provides these per PDK (sky130A, gf180mcuD, ihp-sg13g2).
if {[info exists ::env(OUTPUT_CAP_LOAD)]} { set_load [expr $::env(OUTPUT_CAP_LOAD) / 1000.0] [all_outputs] }
if {[info exists ::env(SYNTH_DRIVING_CELL)]} {
    set dc [split $::env(SYNTH_DRIVING_CELL) "/"]
    set_driving_cell -lib_cell [lindex $dc 0] -pin [lindex $dc 1] [all_inputs]
}
