import sys
import os

sys.path.insert(0, os.path.dirname(__file__))

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import FallingEdge, ClockCycles

from models.rc_bfm import RootComplexBFM


@cocotb.test()
async def test_reset_holds_config_and_msix_quiet(dut):
    """Config space and MSI-X must sit quietly in reset under PERST#."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    bfm = RootComplexBFM(dut)

    dut.perst_n.value = 0
    dut.ts_byte_valid.value = 0
    dut.ts_byte_l0.value = 0
    dut.ts_byte_l1.value = 0
    dut.ts_byte_l2.value = 0
    dut.ts_byte_l3.value = 0
    dut.hot_reset_request.value = 0
    dut.flr_request.value = 0
    dut.updatefc_received.value = 0
    dut.cfg_wr_en.value = 0
    dut.cfg_rd_en.value = 0
    dut.cfg_addr.value = 0
    dut.cfg_wdata.value = 0
    dut.tbl_wr_en.value = 0
    dut.tbl_rd_en.value = 0
    dut.dma_complete_pulse.value = 0

    await ClockCycles(dut.clk, 3)

    dut.cfg_addr.value = 0x000
    dut.cfg_rd_en.value = 1
    await FallingEdge(dut.clk)
    dut.cfg_rd_en.value = 0
    assert int(dut.cfg_rdata.value) == 0, "config space produced non-zero read under PERST#"
    assert int(dut.pba.value) == 0, "MSI-X PBA non-zero under PERST#"


@cocotb.test()
async def test_link_up_config_space_live(dut):
    """Bring the link up through the real training chain, then confirm
    config space is live and Link Status reflects the real link_trained
    signal - not a testbench stand-in."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    bfm = RootComplexBFM(dut)

    await bfm.reset()
    await bfm.train_link()
    await bfm.release_axi_reset()

    vendor_device = await bfm.cfg_read(0x000)
    assert vendor_device == 0xC0DEFACE, f"Vendor/Device ID = {vendor_device:#x}, expected 0xC0DEFACE"

    link_status = await bfm.cfg_read(0x05C)
    assert (link_status & 0x1) == 1, "Link Status training bit not set from the real link_trained signal"


@cocotb.test()
async def test_bar_sizing_and_writes(dut):
    """BAR0 (64KB, implemented) must reveal its size on the standard PCI
    size-probe; BAR1 (unimplemented) must stay hardwired zero."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    bfm = RootComplexBFM(dut)

    await bfm.reset()
    await bfm.train_link()
    await bfm.release_axi_reset()

    await bfm.cfg_write(0x010, 0xFFFFFFFF)
    size_probe = await bfm.cfg_read(0x010)
    assert size_probe == 0xFFFF0000, f"BAR0 size probe = {size_probe:#x}, expected 0xFFFF0000"

    await bfm.cfg_write(0x010, 0x12345678)
    base = await bfm.cfg_read(0x010)
    assert base == 0x12340000, f"BAR0 base = {base:#x}, expected 0x12340000 (masked to size boundary)"

    await bfm.cfg_write(0x014, 0xFFFFFFFF)
    bar1 = await bfm.cfg_read(0x014)
    assert bar1 == 0, f"unimplemented BAR1 = {bar1:#x}, expected 0"


@cocotb.test()
async def test_msix_masked_and_unmasked(dut):
    """A masked vector must set its PBA bit instead of firing; an unmasked
    vector must issue the 1-DW MWr with the table's address/data."""
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    bfm = RootComplexBFM(dut)

    await bfm.reset()
    await bfm.train_link()
    await bfm.release_axi_reset()

    # Vector 2 left at its default-masked reset state - fire and expect PBA, no MWr
    await bfm.fire_dma_complete(2)
    await FallingEdge(dut.clk)
    assert int(dut.mwr_valid.value) == 0, "mwr_valid fired for a masked vector"
    assert (int(dut.pba.value) >> 2) & 1 == 1, "PBA bit not set for masked vector"

    # Vector 3 programmed unmasked - fire and expect immediate MWr
    await bfm.program_msix_vector(3, 0xA0000000_1000, 0xCAFEBABE, masked=False)
    await bfm.fire_dma_complete(3)
    assert int(dut.mwr_valid.value) == 1, "mwr_valid did not fire for unmasked vector"
    assert int(dut.mwr_addr.value) == 0xA0000000_1000, "mwr_addr mismatch"
    assert int(dut.mwr_data.value) == 0xCAFEBABE, "mwr_data mismatch"
