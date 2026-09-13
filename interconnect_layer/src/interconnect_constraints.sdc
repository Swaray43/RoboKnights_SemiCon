# interconnect_constraints.sdc
# SDC constraints for the OpenLane/OpenROAD flow (OpenSTA), mirroring
# interconnect_constraints.xdc for the Vivado flow. SDC syntax is close to
# but not identical to XDC -- keep both files in sync when either changes.

# ---- Clocks ----
create_clock -name aclk     -period 4.000 [get_ports aclk]
create_clock -name pcie_clk -period 3.000 [get_ports pcie_clk]

set_clock_groups -asynchronous \
    -group [get_clocks aclk] \
    -group [get_clocks pcie_clk]

# ---- CDC multicycle paths ----
set_multicycle_path -setup -end 2 -from [get_pins */u_fifo/wr_ptr_gray_reg*/Q] -to [get_pins */u_fifo/wr_ptr_gray_rd1_reg*/D]
set_multicycle_path -hold  -end 1 -from [get_pins */u_fifo/wr_ptr_gray_reg*/Q] -to [get_pins */u_fifo/wr_ptr_gray_rd1_reg*/D]

set_multicycle_path -setup -end 2 -from [get_pins */u_fifo/rd_ptr_gray_reg*/Q] -to [get_pins */u_fifo/rd_ptr_gray_wr1_reg*/D]
set_multicycle_path -hold  -end 1 -from [get_pins */u_fifo/rd_ptr_gray_reg*/Q] -to [get_pins */u_fifo/rd_ptr_gray_wr1_reg*/D]

# ---- I/O delays (fill in once board/interface timing is known) ----
# set_input_delay  -clock aclk     0.500 [get_ports {s0_t* s1_t*}]
# set_output_delay -clock pcie_clk 0.500 [get_ports {m_t*}]
