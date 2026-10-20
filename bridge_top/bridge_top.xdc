## bridge_top.xdc -- clocks for the full bridge (Vivado)
## clk  : pcie_clk shared by interconnect read side, link layer and PHY (250 MHz target)
## aclk : fabric AXI4-Stream clock (<= 250 MHz; 185 MHz used in simulation)
create_clock -name clk  -period 4.000 [get_ports clk]
create_clock -name aclk -period 5.400 [get_ports aclk]

## The two clocks are unrelated. Every crossing goes through cdc_fifo_async
## (Gray pointers), pulse_sync, rst_sync or a 2-flop level synchroniser.
set_clock_groups -asynchronous -group [get_clocks clk] -group [get_clocks aclk]


## No pin assignments: this is an RTL/timing build. tx_level/rx_level would go to
## SerDes IP (GTY/F-Tile), which is outside this project.
