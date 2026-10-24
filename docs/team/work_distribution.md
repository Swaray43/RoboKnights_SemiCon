## Aditya — Interconnect layer (AXI host interfaces + crossbar + CDC)

**Goal: Accept all traffic from the FPGA fabric and deliver a clean, clock-crossed 512-bit AXI4-Stream to Swaray's link layer input.**

**AXI4 slave interface**
- Implement AW, W, and B channel FSMs with full handshake (AWVALID/AWREADY, WVALID/WREADY, BVALID/BREADY)
- Implement AWREADY de-assertion for back-pressure when downstream buffer is full
- Latch burst descriptor (AWADDR, AWLEN, AWSIZE, AWBURST, AWID) into a descriptor FIFO
- Handle WLAST detection to mark end of burst
- Generate BRESP (OKAY / SLVERR) back to the AXI master once TLP is acknowledged

**AXI4 master interface**
- Implement AR channel FSM for read address acceptance
- Drive R channel with RDATA, RRESP, RLAST, RID on completion return
- Implement ARREADY de-assertion when completion reorder buffer (Swaray) signals full
- Handle ARID tracking so read responses are returned to the correct originator

**AXI4-Stream interface (SmartNIC / ML path)**
- Implement TDATA (512-bit), TVALID, TREADY, TKEEP, TUSER handshake
- Handle partial beats using TKEEP byte-enable masking
- Support TUSER sideband for error flags and stream metadata
- Implement TREADY back-pressure from downstream CDC FIFO

**Central crossbar**
- Build a 512-bit wide N-to-1 crossbar mux (slave IF + master IF + stream IF as inputs)
- Implement port enable logic to select the active upstream source per cycle
- Ensure no two sources can drive the crossbar simultaneously (priority or time-sliced)

**Dynamic arbiters**
- Implement round-robin arbiter across slave, master, and stream ports
- Add starvation guard: any port waiting more than N cycles gets elevated priority
- Expose a programmable weight register so the stream path can be given higher bandwidth share for ML workloads
- Arbitration decision must be registered (no combinational path into the crossbar mux)

**CDC FIFOs — write/stream path**
- Implement async FIFO (depth=16) for the write/stream path crossing from AXI clock to PCIe link clock
- Use gray-code pointer synchronisation with 2-FF synchroniser on both read and write pointer
- Set FIFO width to 512 bits (matching AXI4-Stream width)
- Add FIFO full flag routed back as TREADY de-assertion to the stream interface
- Add FIFO empty flag to stall the downstream link layer

**CDC FIFOs — read/CplD path**
- Implement async FIFO (depth=16) for completion data crossing from PCIe link clock back to AXI clock
- Same gray-code sync scheme as write FIFO
- Route FIFO output to AXI4 master R channel (RDATA, RRESP, RLAST)

**Timing and constraints**
- Write XDC multi-cycle path constraints for all gray-code pointer crossings
- Verify setup/hold margins on both FIFO pointers at target frequency (AXI ≤250 MHz, PCIe link clk target)
- Run timing closure in Vivado and confirm no unconstrained paths cross the clock boundary

**Verification**
- Write an AXI master BFM that drives random burst sequences (fixed, incrementing, wrapping bursts)
- Write a FIFO monitor that checks gray-code pointer correctness cycle by cycle
- Simulate back-pressure scenarios: downstream FIFO full, partial TKEEP beats, concurrent slave + stream traffic
- Confirm BRESP is returned only after the burst descriptor exits the CDC FIFO successfully

**Interface contract deliverable to Swaray**
- AXI4-Stream bus: TDATA (512-bit), TVALID, TREADY, TKEEP (64-bit), TUSER (sideband with burst descriptor fields: addr64, len_dw, attr, TC, reqtype)
- TREADY back-pressure from B must stall the crossbar output within one cycle
- Document the TUSER bit-field mapping in a shared interface spec before B starts integration

---

## Swaray — Link layer (TLP formation + FLIT framing + FEC/CRC + flow control)

**Goal: Receive the 512-bit stream from Aaditya, form correct PCIe TLPs, pack them into 256AXI4-Stream bus: TDATA (512-bit), TVALID, TREADY, TKEEP (64-bit), TUSER
TUSER carries the burst descriptor: {addr64, len_dw, attr, TC, reqtype}
I control TREADY — de-asserting it stalls A completely-byte FLITs with CRC, and deliver a 256-bit system-synchronous FLIT bus to Kshitij.**

**Write data buffer**
- Implement AW+W channel merge buffer (depth = MAX_PAYLOAD_SIZE × 4 entries)
- Assert back-pressure (TREADY de-assertion to A) when buffer occupancy exceeds the high-watermark
- Expose a configurable high-watermark register (default: 75% full)

**AR request splitter**
- Detect any read request whose address + length crosses a 4 KB page boundary
- Split such requests into two sub-requests at the boundary, each with its own address and adjusted length
- Pass non-crossing requests through unmodified

**Address translation**
- Implement BAR decoder: compare incoming AXI address against up to 6 BAR base/limit register pairs
- On a BAR hit, output the translated PCIe bus address (BAR base offset + AXI offset)
- On a miss, flag as UR (Unsupported Request) and return SLVERR to Aaditya's B channel
- Implement IOMMU pass-through: accept a host-programmed translation table (bus address → physical address) and apply it for DMA writes

**Tag manager**
- Implement a tag allocator with parametric pool size N (N = 32, 256, or 1024 selectable at build time)
- Allocate a 10-bit extended tag for every Non-Posted request (MRd, CfgRd)
- Implement tag recycling: on CplD return, free the tag back to the pool
- Implement configurable per-tag timeout: if no CplD arrives within T cycles, flag the tag as timed out and generate a UR completion upstream
- Gate new MRd requests when the tag pool is empty (de-assert TREADY to A)

**TLP packetizer**
- Build MWr64 TLP headers (4 DW): Fmt, Type, Length, RequesterID, Tag, LastBE, FirstBE, Addr[63:2]
- Build MRd64 TLP headers similarly
- Respect MAX_PAYLOAD_SIZE parameter: split any burst larger than MPS into multiple TLPs, each with an incremented address offset
- Assert TLAST at the end of each TLP payload
- Handle FirstBE and LastBE correctly for non-DW-aligned start/end addresses

**CplD reorder buffer**
- Implement per-tag completion storage: allocate a buffer slot per outstanding tag
- Accept out-of-order CplD TLPs from Kshitij's RX engine and store by tag
- Reassemble multi-CplD sequences (when a single MRd generates more than one completion)
- Release completed data in original request order to Aaditya's CDC read FIFO
- Implement configurable timeout per tag slot; escalate timed-out slots as UR completions

**Credit manager**
- Track Posted Header Credits (PHC), Posted Data Credits (PDC), Non-Posted Header Credits (NPHC), Non-Posted Data Credits (NPDC), and Completion Credits (CC) independently
- Decrement credits on every TLP transmission
- Replenish credits on receipt of UpdateFC DLLPs from Kshitij
- De-assert TREADY to Aaditya when any relevant credit counter reaches zero
- Implement infinite credit detection (credit value = 0x7F / 0xFFF means unlimited)

**PCIe TX engine**
- Arbitrate between Posted (MWr), Non-Posted (MRd, CfgRd), and Completion TLP queues
- Enforce PCIe ordering rules: Posted must not bypass older Non-Posted from the same requester
- Implement MSI-X write: on interrupt request from Kshitij's MSI-X controller, inject a 1-DW MWr TLP to the MSI-X table address
- Serialize selected TLP onto the output stream toward the FLIT framer

**PCIe RX engine**
- Receive incoming TLP stream from Kshitij's FLIT deframer
- Parse TLP header: extract Type, RequesterID, Tag, Address, Length
- Route CplD TLPs to the CplD reorder buffer by tag
- Route CfgRd/CfgWr TLPs to Kshitij's config space block
- Route MWr (from host to FPGA) to Aaditya's AXI master interface
- Flag malformed TLPs (bad CRC, reserved fields set) as error completions

**FLIT framer**
- Pack serialised TLP stream into fixed 256-byte FLITs per PCIe 6 spec
- Pad short TLPs to fill the FLIT boundary with null bytes
- Append 6-byte CRC per FLIT
- Maintain a FLIT sequence number for ACK/NAK tracking
- Implement retry buffer: hold each transmitted FLIT until ACK received from Kshitij; retransmit on NAK

**FEC block**
- Apply forward error correction encoding to each FLIT before handing to Kshitij
- Implement the FEC scheme specified by PCIe 6 (Reed-Solomon or equivalent)
- On the RX side, apply FEC decoding to incoming FLITs before passing to the deframer
- Flag uncorrectable errors to the error reporting logic

**Verification**
- Write a TLP checker that validates every generated TLP header field against the PCIe 6 base spec
- Write a credit accounting model and assert that no TLP is ever sent without sufficient credits
- Simulate OOO completion scenarios: send MRd bursts, return CplD in reverse order, verify in-order delivery to A
- Simulate tag exhaustion: fill all tag slots, confirm TREADY de-assertion, confirm recovery after CplD returns
- Simulate NAK retransmit: drop a FLIT at the C interface, confirm retry buffer replays it

**Interface contract deliverable to Kshitij**
- System-synchronous 256-bit bus carrying packed FLITs with FLIT sequence number sideband
- Upstream from C: UpdateFC DLLP values (PHC, PDC, NPHC, NPDC, CC replenishment counts) and ACK/NAK per FLIT sequence number
- Document FLIT bus signal list and UpdateFC encoding in a shared interface spec before C starts integration

---

## Kshitij — Protocol-to-PHY layer (lane striping + PAM4 + sideband + verification infra)

**Goal: Accept the 256-bit FLIT bus from Swaray, stripe it across 4 PCIe lanes with Gray coding and PAM4 mapping, drive the analog SerDes, and own all sideband controllers and the full verification stack.**

**PCIe TX engine (physical side)**
- Accept the 256-bit FLIT bus from Swaray
- Distribute FLIT bytes across 4 lanes in byte-interleaved round-robin order (lane 0 gets bytes 0,4,8…; lane 1 gets 1,5,9… etc.)
- Implement lane deskew at startup: absorb per-lane skew using elastic buffers up to ±N UI
- Assert lane-valid only after deskew lock on all 4 lanes

**PCIe RX engine (physical side)**
- Receive 4-lane symbol streams from the analog SerDes
- Realign lanes: detect lane reversal and lane-to-lane skew, reorder to canonical order
- Strip 128b/130b (or FLIT-mode) framing tokens
- Reconstruct the 256-bit FLIT bus and pass upstream to Swaray's FLIT deframer

**Gray coding**
- Implement 2-bit Gray encoder on the TX path: for each PAM4 symbol, encode the 2 data bits as Gray code before mapping to PAM4 levels
- Implement 2-bit Gray decoder on the RX path: decode received Gray-coded bits back to binary
- Verify encoder/decoder are exact inverses across all 4 input combinations

**PAM4 mapper**
- Map each 2-bit Gray-coded symbol to one of four voltage levels: {0,1,2,3} → {−3,−1,+1,+3} (normalised)
- Implement the inverse de-mapper on RX: threshold the received analog level into 2 bits
- Expose a per-lane pre-emphasis register for equalization tuning

**Soft PHY wrapper**
- Wrap the TX and RX datapaths into a single PHY interface block targeting Vivado
- Constrain all internal paths to ≤250 MHz
- Write XDC timing constraints for the 256-bit parallel interface to/from Swaray
- Target output: 4-lane parallel symbol bus at the analog PHY boundary (the handoff to the external multi-lane SerDes)
- Document all I/O port names and widths for the SerDes integration team

**Reset controller**
- Monitor PERST# pin: on assertion, immediately reset all PCIe datapath logic
- Implement hot-reset sequence: on receipt of a hot-reset TS1/TS2 ordered set, reset the link and re-enter link training
- Implement FLR (Function Level Reset): on CfgWr to the FLR bit, reset the function's state while keeping the link up
- Implement link-down detection: if the link goes down unexpectedly, assert AXI reset to Aaditya's interfaces and hold until link re-trains
- Implement AXI reset sequencing: deassert AXI reset only after link training completes and the first UpdateFC DLLP is received

**Config space block**
- Implement the PCIe Type 0 configuration space header (64 bytes)
- Implement up to 6 BAR registers: size and base address, writable by the host during BAR sizing at boot
- Implement Device Control register: Max Payload Size, Max Read Request Size, enable bits
- Implement Link Status register: current link speed, link width, link training bit
- Respond to CfgRd TLPs from Swaray's RX engine with the correct register value
- Respond to CfgWr TLPs by updating the relevant register and generating a Cpl (no data) response

**MSI-X controller**
- Implement the MSI-X vector table (up to 2048 entries): each entry holds a 64-bit message address and 32-bit message data
- Implement the MSI-X PBA (Pending Bit Array): set the pending bit for any interrupt that fires while the vector is masked
- On DMA completion signal from Swaray's TX engine, look up the vector table entry and issue a 1-DW MWr TLP to Swaray for injection
- Implement per-vector masking: if the vector is masked, set the PBA bit instead of sending the MWr
- Implement the MSI-X Capability structure in the config space block (Capability ID, Message Control, Table BIR/Offset, PBA BIR/Offset)

**Verification stack — testbench infrastructure**
- Set up the cocotb project structure: Makefile, simulator targets for Verilator and Icarus, waveform dump config
- Write a PCIe 6 FLIT-mode root complex BFM in cocotb: capable of sending CfgRd, MRd, MWr TLPs and receiving completions
- Write a PCIe link layer monitor: checks FLIT sequence numbers, ACK/NAK protocol, UpdateFC DLLP timing
- Write SVA assertions for: credit never goes negative, tag never double-allocated, PERST# reset sequence timing, FLIT CRC correctness
- Integrate SymbiYosys for formal proof of the reset sequencing FSM and the Gray code encoder/decoder

**Performance counters and scoreboard**
- Implement the following 32-bit saturating counters in RTL, readable via CfgRd: TLP TX count, TLP RX count, credit stall cycles (per credit type), tag utilisation (peak and current), reorder buffer occupancy (peak and current), FLIT NAK count, FEC corrected error count, FEC uncorrectable error count
- Implement a coverage model in cocotb: track which TLP types, credit types, and error scenarios have been exercised
- Write a scoreboard that checks every MWr payload sent by Aaditya's BFM arrives byte-for-byte at the simulated root complex

**Timing sign-off — integration gate**
- Run full-design timing analysis in Vivado after all three layers are integrated
- Confirm no setup violations on the 256-bit A→B and B→C interface buses
- Confirm all CDC paths (Aaditya's gray-code FIFOs) have valid multi-cycle constraints
- Sign-off report is the final integration checkpoint — nothing goes to tape-out without it