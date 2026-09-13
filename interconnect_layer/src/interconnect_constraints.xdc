## interconnect_constraints.xdc
## Vivado constraints for the interconnect layer (axi4s_ingest_port,
## rr_arbiter_2ch, interconnect_top, cdc_fifo_async).
##
## Fill in actual clock periods/pin locations for your target board/part --
## the values below are placeholders matching the project's stated targets
## (AXI-side <=250 MHz; PCIe6 link clock TBD by Swaray's block).

## ---- Clocks ----
create_clock -name aclk     -period 4.000  [get_ports aclk]      ;# 250 MHz AXI-side target
create_clock -name pcie_clk -period 3.000  [get_ports pcie_clk]  ;# placeholder -- set to actual PCIe6 link clock

## Mark the two clocks as asynchronous to each other -- required so the
## tool doesn't attempt (and fail) to time-close a direct path between
## them; the ONLY sanctioned crossing is through cdc_fifo_async's
## gray-code synchronizers below.
set_clock_groups -asynchronous \
    -group [get_clocks aclk] \
    -group [get_clocks pcie_clk]

## ---- CDC multicycle paths: gray-code pointer synchronizers ----
## Each 2-flop synchronizer chain is inherently a multi-cycle-safe crossing
## by construction (gray code guarantees only one bit changes per
## increment), but without an explicit constraint, STA will either flag a
## false single-cycle violation across these paths or -- worse -- not know
## to allow the extra settling cycle at all. Set 2-cycle multicycle paths
## on both synchronizer chains, in both directions.

## write pointer (aclk domain) -> synchronized into pcie_clk domain
set_multicycle_path -setup -end   2 -from [get_cells */u_fifo/wr_ptr_gray_reg*] -to [get_cells */u_fifo/wr_ptr_gray_rd1_reg*]
set_multicycle_path -hold  -end   1 -from [get_cells */u_fifo/wr_ptr_gray_reg*] -to [get_cells */u_fifo/wr_ptr_gray_rd1_reg*]

## read pointer (pcie_clk domain) -> synchronized into aclk domain
set_multicycle_path -setup -end   2 -from [get_cells */u_fifo/rd_ptr_gray_reg*] -to [get_cells */u_fifo/rd_ptr_gray_wr1_reg*]
set_multicycle_path -hold  -end   1 -from [get_cells */u_fifo/rd_ptr_gray_reg*] -to [get_cells */u_fifo/rd_ptr_gray_wr1_reg*]

## ---- High-fanout control signal buffering ----
## stream_select / grant0 / grant1 fan out to every per-channel register in
## the ingest ports and the crossbar mux -- flagged in the arbiter's design
## review as needing automated buffer-tree insertion. Vivado handles this
## by default via normal fanout-driven buffering; no explicit constraint
## needed unless post-synthesis reports show excessive fanout on these
## nets specifically -- if so, add:
## set_max_fanout 16 [get_nets */grant0] [get_nets */grant1]

## ---- I/O delays ----
## Fill in once the actual board-level timing budget from Swaray's/
## Kshitij's blocks is known -- placeholders only.
# set_input_delay  -clock aclk     0.500 [get_ports {s0_t* s1_t*}]
# set_output_delay -clock pcie_clk 0.500 [get_ports {m_t*}]
