# bridge_top.sdc -- clocks for the full bridge (Quartus / Timing Analyzer)
create_clock -name clk  -period 4.000 [get_ports clk]
create_clock -name aclk -period 5.400 [get_ports aclk]
set_clock_groups -asynchronous -group [get_clocks clk] -group [get_clocks aclk]
derive_clock_uncertainty
