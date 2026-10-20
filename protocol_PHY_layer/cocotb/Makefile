SIM ?= icarus
TOPLEVEL_LANG ?= verilog

# Sources come from the single file list (../files.f), one module per file.
VERILOG_SOURCES += $(addprefix $(PWD)/../,$(shell grep -v '^//' ../files.f | grep -v '^\s*$$'))

TOPLEVEL = pcie_phy_top
COCOTB_TOPLEVEL = pcie_phy_top
MODULE   = test_pcie_phy_top_base,test_pcie_phy_top
COCOTB_TEST_MODULES = test_pcie_phy_top_base,test_pcie_phy_top

ifeq ($(SIM),verilator)
    EXTRA_ARGS += --trace --trace-structs
endif

COCOTB_HDL_TIMEUNIT = 1ns
COCOTB_HDL_TIMEPRECISION = 1ps

include $(shell cocotb-config --makefiles)/Makefile.sim
