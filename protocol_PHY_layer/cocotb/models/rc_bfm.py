"""
Root-complex BFM for the PCIe 6 FLIT-mode link.

Scope for this first pass: CfgRd/CfgWr generation (per work_distribution.md's
BFM bullet) plus the TS1/TS2 training-sequence send used to bring the link up
before config space is reachable. MRd/MWr generation is deferred until the
AXI-master/tag-manager scope question is settled with the team - adding it now
would silently re-absorb scope that's still an open conversation.
"""

import cocotb
from cocotb.triggers import RisingEdge, FallingEdge, ClockCycles


class RootComplexBFM:
    def __init__(self, dut):
        self.dut = dut

    async def reset(self, cycles=5):
        """Drive PERST# for `cycles` clock edges, matching the reset
        controller's >=2-cycle-low requirement with margin, then release."""
        self.dut.perst_n.value = 0
        self.dut.ts_byte_valid.value = 0
        self.dut.ts_byte_l0.value = 0
        self.dut.ts_byte_l1.value = 0
        self.dut.ts_byte_l2.value = 0
        self.dut.ts_byte_l3.value = 0
        self.dut.hot_reset_request.value = 0
        self.dut.flr_request.value = 0
        self.dut.updatefc_received.value = 0
        self.dut.cfg_wr_en.value = 0
        self.dut.cfg_rd_en.value = 0
        self.dut.cfg_addr.value = 0
        self.dut.cfg_wdata.value = 0
        self.dut.tbl_wr_en.value = 0
        self.dut.tbl_rd_en.value = 0
        self.dut.dma_complete_pulse.value = 0
        # ---- inputs added with pcie_phy_top (were not on pcie_link_top) ----
        self.dut.mwr_ready.value = 1
        self.dut.phy_tx_valid.value = 0
        self.dut.phy_tx_sof.value = 0
        self.dut.phy_tx_eof.value = 0
        self.dut.phy_tx_data.value = 0
        self.dut.rx_level.value = 0
        self.dut.rx_level_valid.value = 0
        self.dut.bar0_wr_en.value = 0
        self.dut.bar0_rd_en.value = 0
        self.dut.bar0_addr.value = 0
        self.dut.bar0_wdata.value = 0
        self.dut.session_done.value = 0
        self.dut.session_end_aclk.value = 0
        self.dut.session_overflow_aclk.value = 0
        self.dut.aclk_rst_n.value = 0
        for _ in range(cycles):
            await FallingEdge(self.dut.clk)
        self.dut.perst_n.value = 1
        self.dut.aclk_rst_n.value = 1
        await FallingEdge(self.dut.clk)

    async def _send_ts1_all_lanes(self, lane_nums=(0, 1, 2, 3)):
        """Port of the Verilog send_ts1_all_lanes task: COM, link#=0x00 x2,
        lane# 0..3, PAD to byte 9, then 6 bytes of rate-identifier (0x4A)."""
        self.dut.ts_byte_valid.value = 1
        self.dut.ts_byte_l0.value = 0xBC
        self.dut.ts_byte_l1.value = 0xBC
        self.dut.ts_byte_l2.value = 0xBC
        self.dut.ts_byte_l3.value = 0xBC
        await FallingEdge(self.dut.clk)

        for lane in ("ts_byte_l0", "ts_byte_l1", "ts_byte_l2", "ts_byte_l3"):
            getattr(self.dut, lane).value = 0x00
        await FallingEdge(self.dut.clk)

        self.dut.ts_byte_l0.value = lane_nums[0]
        self.dut.ts_byte_l1.value = lane_nums[1]
        self.dut.ts_byte_l2.value = lane_nums[2]
        self.dut.ts_byte_l3.value = lane_nums[3]
        await FallingEdge(self.dut.clk)

        for _ in range(7):  # bytes 3..9
            for lane in ("ts_byte_l0", "ts_byte_l1", "ts_byte_l2", "ts_byte_l3"):
                getattr(self.dut, lane).value = 0x00
            await FallingEdge(self.dut.clk)

        for _ in range(6):  # bytes 10..15
            for lane in ("ts_byte_l0", "ts_byte_l1", "ts_byte_l2", "ts_byte_l3"):
                getattr(self.dut, lane).value = 0x4A
            await FallingEdge(self.dut.clk)

        self.dut.ts_byte_valid.value = 0

    async def _wait_negedges(self, n):
        for _ in range(n):
            await FallingEdge(self.dut.clk)

    async def train_link(self, lane_nums=(0, 1, 2, 3)):
        """Send two TS1 ordered sets (matching the Verilog testbenches'
        margin) and return once link_trained asserts. lane_nums (3,2,1,0)
        models a lane-reversed link."""
        await self._send_ts1_all_lanes(lane_nums)
        await self._send_ts1_all_lanes(lane_nums)
        await self._wait_negedges(3)
        assert self.dut.link_trained.value == 1, "link_trained did not assert after training sequence"

    async def release_axi_reset(self):
        """Pulse updatefc_received once link_state has settled to LINK_UP,
        matching the margin needed in the Verilog integration testbench.
        Stays FallingEdge-aligned throughout (not ClockCycles, which lands
        just after a rising edge and would leave no posedge for the pulse
        to actually be sampled on)."""
        await self._wait_negedges(3)
        self.dut.updatefc_received.value = 1
        await FallingEdge(self.dut.clk)
        self.dut.updatefc_received.value = 0
        await FallingEdge(self.dut.clk)
        assert self.dut.axi_rst_n.value == 1, "axi_rst_n did not release"

    async def cfg_write(self, addr, data):
        await FallingEdge(self.dut.clk)
        self.dut.cfg_addr.value = addr
        self.dut.cfg_wdata.value = data
        self.dut.cfg_wr_en.value = 1
        await FallingEdge(self.dut.clk)
        self.dut.cfg_wr_en.value = 0

    async def cfg_read(self, addr):
        await FallingEdge(self.dut.clk)
        self.dut.cfg_addr.value = addr
        self.dut.cfg_rd_en.value = 1
        await FallingEdge(self.dut.clk)
        self.dut.cfg_rd_en.value = 0
        return int(self.dut.cfg_rdata.value)

    async def program_msix_vector(self, vec, addr, data, masked=False):
        await FallingEdge(self.dut.clk)
        self.dut.tbl_wr_vec.value = vec
        self.dut.tbl_wr_addr.value = addr
        self.dut.tbl_wr_data.value = data
        self.dut.tbl_wr_mask.value = 1 if masked else 0
        self.dut.tbl_wr_en.value = 1
        await FallingEdge(self.dut.clk)
        self.dut.tbl_wr_en.value = 0

    async def fire_dma_complete(self, vec):
        await FallingEdge(self.dut.clk)
        self.dut.dma_complete_vector.value = vec
        self.dut.dma_complete_pulse.value = 1
        await FallingEdge(self.dut.clk)
        self.dut.dma_complete_pulse.value = 0

    # ---- BAR0 DMA registers (pcie_phy_top) ----
    async def bar0_write(self, addr, data):
        await FallingEdge(self.dut.clk)
        self.dut.bar0_addr.value = addr
        self.dut.bar0_wdata.value = data
        self.dut.bar0_wr_en.value = 1
        await FallingEdge(self.dut.clk)
        self.dut.bar0_wr_en.value = 0

    async def bar0_read(self, addr):
        await FallingEdge(self.dut.clk)
        self.dut.bar0_addr.value = addr
        self.dut.bar0_rd_en.value = 1
        await FallingEdge(self.dut.clk)
        self.dut.bar0_rd_en.value = 0
        return int(self.dut.bar0_rdata.value)

    async def program_session(self, ch, base_addr, limit=None, weights=None):
        """Write a channel's 64-bit base address (and optionally the limit and
        arbiter weights), then set its arm bit."""
        lo = 0x20 + 8 * ch
        await self.bar0_write(lo, base_addr & 0xFFFFFFFF)
        await self.bar0_write(lo + 4, (base_addr >> 32) & 0xFFFFFFFF)
        if limit is not None:
            await self.bar0_write(0x0C, limit)
        if weights is not None:
            await self.bar0_write(0x08, (weights[1] << 8) | weights[0])
        await self.bar0_write(0x00, 1 << ch)

    # ---- Datapath helpers ----
    async def serdes_loopback(self, cross_lanes=False):
        """Model the channel in Python: copy tx_level -> rx_level every clock
        (one register stage). Start with cocotb.start_soon()."""
        while True:
            await RisingEdge(self.dut.clk)
            lv = int(self.dut.tx_level.value)
            vv = int(self.dut.tx_level_valid.value)
            if cross_lanes:
                lv = sum(((lv >> (3 * i)) & 7) << (3 * (3 - i)) for i in range(4))
                vv = sum(((vv >> i) & 1) << (3 - i) for i in range(4))
            self.dut.rx_level.value = lv
            self.dut.rx_level_valid.value = vv

    async def send_flit(self, words):
        """Push one Flit (8 x 256-bit ints) honouring phy_tx_ready."""
        for i, w in enumerate(words):
            self.dut.phy_tx_data.value = w
            self.dut.phy_tx_sof.value = int(i == 0)
            self.dut.phy_tx_eof.value = int(i == len(words) - 1)
            self.dut.phy_tx_valid.value = 1
            while True:
                await RisingEdge(self.dut.clk)
                if int(self.dut.phy_tx_ready.value):
                    break
            await FallingEdge(self.dut.clk)
        self.dut.phy_tx_valid.value = 0
        self.dut.phy_tx_sof.value = 0
        self.dut.phy_tx_eof.value = 0
