"""
cocotb tests for the new pcie_phy_top features: BAR0 DMA registers, session
start crossing to aclk, MSI-X on session_done, and the PAM4 datapath with the
SerDes channel modelled in Python (loopback lives in the testbench, not RTL).
The datapath tests train the link over that channel (TS1 sent by TX, read off
the received lanes), not through the ts_byte_* injection port.

The four original tests in test_pcie_phy_top_base.py run against the same top.
"""

import os
import random
import sys

sys.path.insert(0, os.path.dirname(__file__))

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, RisingEdge

from models.rc_bfm import RootComplexBFM


async def _bring_up(dut, lane_nums=(0, 1, 2, 3)):
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    cocotb.start_soon(Clock(dut.aclk, 7, unit="ns").start())
    bfm = RootComplexBFM(dut)
    await bfm.reset()
    await bfm.train_link(lane_nums)
    await bfm.release_axi_reset()
    await bfm.enable_function()   # Memory Space + Bus Master + MSI-X Enable
    return bfm


@cocotb.test()
async def test_bar0_session_arm_and_start_pulse(dut):
    """BAR0 base/limit/weights reach their ports; arming ch1 latches its base,
    raises session_valid[1] and sends exactly one start pulse on aclk."""
    bfm = await _bring_up(dut)

    starts = {"ch0": 0, "ch1": 0}

    async def count_starts():
        while True:
            await RisingEdge(dut.aclk)
            v = int(dut.session_start_aclk.value)
            starts["ch0"] += v & 1
            starts["ch1"] += (v >> 1) & 1

    cocotb.start_soon(count_starts())

    base = 0x0000_00C0_FFEE_0000
    await bfm.program_session(1, base, limit=0x40000, weights=(2, 7))
    await ClockCycles(dut.clk, 8)

    assert int(dut.session_valid.value) == 0b10, "only ch1 should be active"
    assert int(dut.cfg_base_addr_ch1.value) == base, "ch1 base address not latched from BAR0"
    assert int(dut.session_limit.value) == 0x40000
    assert int(dut.arb_weight_ch0.value) == 2 and int(dut.arb_weight_ch1.value) == 7
    assert starts == {"ch0": 0, "ch1": 1}, f"expected one ch1 start pulse on aclk, got {starts}"
    assert await bfm.bar0_read(0x2C) == (base >> 32), "BASE_CH1_HI readback"


@cocotb.test()
async def test_session_done_raises_msix(dut):
    """session_done on ch0 sends the MWr programmed for ch0's BAR0 vector,
    and the message waits for mwr_ready."""
    bfm = await _bring_up(dut)
    await bfm.bar0_write(0x10, (5 << 8) | 4)          # ch0 -> vector 4, ch1 -> vector 5
    await bfm.program_msix_vector(4, 0xFEE0_0000_0000_0040, 0x1234, masked=False)

    dut.mwr_ready.value = 0
    await FallingEdge(dut.clk)
    dut.session_done.value = 0b01
    await FallingEdge(dut.clk)
    dut.session_done.value = 0
    await ClockCycles(dut.clk, 4)
    await FallingEdge(dut.clk)
    assert int(dut.mwr_valid.value) == 1, "MWr should be held while mwr_ready is low"
    assert int(dut.mwr_addr.value) == 0xFEE0_0000_0000_0040
    assert int(dut.mwr_data.value) == 0x1234
    dut.mwr_ready.value = 1
    await FallingEdge(dut.clk)
    assert int(dut.mwr_valid.value) == 0, "MWr should retire once accepted"


async def _datapath_roundtrip(dut, cross_lanes, n_flits):
    # Channel first, then train over it: no ts_byte_* injection here.
    cocotb.start_soon(Clock(dut.clk, 10, unit="ns").start())
    cocotb.start_soon(Clock(dut.aclk, 7, unit="ns").start())
    bfm = RootComplexBFM(dut)
    await bfm.reset()
    cocotb.start_soon(bfm.serdes_loopback(cross_lanes=cross_lanes))
    await bfm.wait_link_up_over_lanes()
    await bfm.release_axi_reset()

    rng = random.Random(1234)
    sent = []
    for _ in range(n_flits):
        sent.append([rng.getrandbits(256) for _ in range(8)])
    sent[0][3] = int("FB" * 32, 16)                     # SOF-looking payload

    got = []

    async def collect():
        while True:
            await RisingEdge(dut.clk)
            if int(dut.phy_rx_valid.value):
                got.append((int(dut.phy_rx_data.value), int(dut.phy_rx_sof.value),
                            int(dut.phy_rx_eof.value)))

    cocotb.start_soon(collect())
    for flit in sent:
        await bfm.send_flit(flit)

    for _ in range(400 * n_flits):
        if len(got) >= 8 * n_flits:
            break
        await RisingEdge(dut.clk)

    expected = [(w, int(i == 0), int(i == 7)) for flit in sent for i, w in enumerate(flit)]
    assert len(got) == len(expected), f"got {len(got)} words, expected {len(expected)}"
    for k, (g, e) in enumerate(zip(got, expected)):
        assert g == e, f"word {k} mismatch (sof/eof or data)"
    assert int(dut.phy_framing_err.value) == 0
    assert int(dut.phy_lane_err.value) == 0


@cocotb.test()
async def test_datapath_loopback(dut):
    """Train over the lanes, then 3 Flits through tx_level -> Python channel
    -> rx_level, bit-exact."""
    await _datapath_roundtrip(dut, cross_lanes=False, n_flits=3)
    assert int(dut.lane_reversed_locked.value) == 0


@cocotb.test()
async def test_datapath_lane_reversed(dut):
    """Same, with the channel crossing lanes: training must detect the
    reversal from the TS1 lane numbers alone."""
    await _datapath_roundtrip(dut, cross_lanes=True, n_flits=2)
    assert int(dut.lane_reversed_locked.value) == 1


@cocotb.test()
async def test_msix_table_through_bar0(dut):
    """Config 0x54 points at BAR0+0x1000; a vector programmed there with
    DWORD writes fires correctly, and nothing happens with MSI-X disabled."""
    bfm = await _bring_up(dut)
    assert await bfm.cfg_read(0x054) == 0x1000, "MSI-X table offset/BIR"
    assert await bfm.cfg_read(0x058) == 0x1800, "MSI-X PBA offset/BIR"
    base = 0x1000 + 16 * 3
    await bfm.bar0_write(base + 0, 0xFEE0_0040)
    await bfm.bar0_write(base + 4, 0x0)
    await bfm.bar0_write(base + 8, 0xABCD)
    await bfm.bar0_write(base + 12, 0)            # unmask
    await bfm.fire_dma_complete(3)
    assert int(dut.mwr_valid.value) == 1 and int(dut.mwr_data.value) == 0xABCD
    await FallingEdge(dut.clk)
    await bfm.cfg_write(0x050, 0)                 # MSI-X off
    await bfm.fire_dma_complete(3)
    await FallingEdge(dut.clk)
    assert int(dut.mwr_valid.value) == 0 and int(dut.pba.value) == 0, "MSI-X disabled must drop the event"
