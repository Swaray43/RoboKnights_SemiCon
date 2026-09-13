SIM ?= icarus
TOPLEVEL_LANG ?= verilog

# Source files - add pcie_phy_datapath.v/pcie_tag_man.v here once the PHY
# datapath is folded into pcie_link_top in a later pass.
VERILOG_SOURCES += $(PWD)/../pcie_phy_datapath.v
VERILOG_SOURCES += $(PWD)/../pcie_tag_man.v
VERILOG_SOURCES += $(PWD)/../pcie_config_msix.v
VERILOG_SOURCES += $(PWD)/pcie_link_top.v

TOPLEVEL = pcie_link_top
MODULE   = test_pcie_link_top

ifeq ($(SIM),verilator)
    EXTRA_ARGS += --trace --trace-structs
endif

COCOTB_HDL_TIMEUNIT = 1ns
COCOTB_HDL_TIMEPRECISION = 1ns

include $(shell cocotb-config --makefiles)/Makefile.sim
