# interconnect_top on SKY130 (LibreLane). Both clocks 15 ns (66 MHz); they are
# unrelated (async CDC FIFO between them). X.1 removed the TKEEP -> TREADY path,
# so aclk no longer needs 30 ns.
create_clock -name aclk     -period 15.0 [get_ports aclk]
create_clock -name pcie_clk -period 15.0 [get_ports pcie_clk]
set_clock_groups -asynchronous -group [get_clocks aclk] -group [get_clocks pcie_clk]

set_input_delay  2.0 -clock aclk     [get_ports {s0_tdata* s0_tkeep* s0_tlast s0_tvalid s1_tdata* s1_tkeep* s1_tlast s1_tvalid}]
set_input_delay  2.0 -clock aclk     [get_ports {ch0_session_start ch0_session_limit* ch1_session_start ch1_session_limit* weight0* weight1*}]
set_input_delay  2.0 -clock pcie_clk [get_ports {m_tready}]

set_output_delay 2.0 -clock aclk     [get_ports {s0_tready s1_tready ch0_session_active ch0_session_overflow ch1_session_active ch1_session_overflow session_end*}]
set_output_delay 2.0 -clock pcie_clk [get_ports {m_tdata* m_tkeep* m_tvalid m_tlast m_tuser*}]

set_false_path -from [get_ports {arstn pcie_rstn}]
